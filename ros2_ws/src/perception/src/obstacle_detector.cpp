// obstacle_detector: /scan (sensor_msgs/LaserScan) ->
//   /obstacles (geometry_msgs/PoseArray)   cluster centroids, frame `map`, one message per scan
//   /costmap   (nav_msgs/OccupancyGrid)    known-pose log-odds occupancy grid, frame `map`,
//                                           0 free / 100 occupied / -1 unknown (NOT inflated)
// Poses come from TF (map -> laser at the scan stamp); this is mapping with given poses, not SLAM.
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "perception/scan_processing.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

namespace perception
{

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector()
  : Node("obstacle_detector")
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    scan_resolution_deg_ = declare_parameter<double>("scan_resolution_deg", 2.0);
    cluster_gap_m_ = declare_parameter<double>("cluster_gap_m", 0.15);
    min_cluster_points_ = declare_parameter<int>("min_cluster_points", 3);
    const double costmap_rate = declare_parameter<double>("costmap_publish_rate_hz", 2.0);

    OccupancyMapper::Params mp;
    mp.resolution = declare_parameter<double>("map_resolution", 0.1);
    const double w_m = declare_parameter<double>("map_width_m", 14.0);
    const double h_m = declare_parameter<double>("map_height_m", 14.0);
    mp.width = static_cast<int>(std::lround(w_m / mp.resolution));
    mp.height = static_cast<int>(std::lround(h_m / mp.resolution));
    mp.origin_x = declare_parameter<double>("map_origin_x", -7.0);
    mp.origin_y = declare_parameter<double>("map_origin_y", -7.0);
    mapper_ = std::make_unique<OccupancyMapper>(mp);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    // tf2_ros::MessageFilter waits for transforms asynchronously: the buffer needs a timer interface
    tf_buffer_->setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface()));
    // Default listener (own node + thread): marks the buffer as served by a dedicated thread, which
    // tf2_ros::MessageFilter requires. (Shows up as a hidden transform_listener_impl_* node.)
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    obstacles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("obstacles", 10);
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", rclcpp::QoS(1));

    scan_sub_.subscribe(this, "scan", rmw_qos_profile_sensor_data);
    tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      scan_sub_, *tf_buffer_, global_frame_, 20, get_node_logging_interface(),
      get_node_clock_interface());
    tf_filter_->registerCallback(&ObstacleDetector::on_scan, this);

    reset_srv_ = create_service<std_srvs::srv::Empty>(
      "reset_costmap",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>) {
        std::lock_guard<std::mutex> lock(mutex_);
        mapper_->reset();
        RCLCPP_INFO(get_logger(), "costmap reset");
      });

    // ROS-clock timer: the period is in simulation seconds, not wall seconds
    costmap_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / costmap_rate), [this]() {publish_costmap();});

    RCLCPP_INFO(
      get_logger(), "obstacle_detector up: scan resampled to %.1f deg, %dx%d cells @ %.2f m, frame %s",
      scan_resolution_deg_, mp.width, mp.height, mp.resolution, global_frame_.c_str());
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(global_frame_, scan->header.frame_id, scan->header.stamp);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "no TF %s -> %s at scan stamp: %s",
        global_frame_.c_str(), scan->header.frame_id.c_str(), ex.what());
      return;
    }
    const double tx = tf.transform.translation.x;
    const double ty = tf.transform.translation.y;
    const auto & q = tf.transform.rotation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    const double res = scan_resolution_deg_ * M_PI / 180.0;
    const DownsampledScan ds = downsample_scan(
      scan->ranges, scan->angle_min, scan->angle_increment, res, scan->range_min, scan->range_max);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      mapper_->integrate(tx, ty, yaw, ds, scan->range_min, scan->range_max);
    }

    const auto clusters =
      cluster_scan(ds, scan->range_min, scan->range_max, cluster_gap_m_, min_cluster_points_);
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = scan->header.stamp;
    msg.header.frame_id = global_frame_;
    const double c = std::cos(yaw), s = std::sin(yaw);
    for (const auto & cl : clusters) {
      geometry_msgs::msg::Pose p;
      p.position.x = tx + c * cl.centroid.x - s * cl.centroid.y;
      p.position.y = ty + s * cl.centroid.x + c * cl.centroid.y;
      p.orientation.w = 1.0;
      msg.poses.push_back(p);
    }
    obstacles_pub_->publish(msg);
  }

  void publish_costmap()
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = global_frame_;
    const auto & p = mapper_->params();
    grid.info.resolution = static_cast<float>(p.resolution);
    grid.info.width = p.width;
    grid.info.height = p.height;
    grid.info.origin.position.x = p.origin_x;
    grid.info.origin.position.y = p.origin_y;
    grid.info.origin.orientation.w = 1.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      grid.data = mapper_->occupancy();
    }
    costmap_pub_->publish(grid);
  }

  std::string global_frame_;
  double scan_resolution_deg_;
  double cluster_gap_m_;
  int min_cluster_points_;

  std::mutex mutex_;
  std::unique_ptr<OccupancyMapper> mapper_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tf_filter_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr costmap_timer_;
};

}  // namespace perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception::ObstacleDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
