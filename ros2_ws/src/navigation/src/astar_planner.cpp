// astar_planner: /costmap (OccupancyGrid) + /goal_pose (PoseStamped) -> /plan (nav_msgs/Path)
//
// The map is built online from the lidar, so the plan is recomputed at `replan_rate_hz` from the
// robot's current pose (TF map -> base_link). If the goal is inside an obstacle or unreachable the
// node publishes an EMPTY path and logs a warning - it never returns a straight line.
// /goal_cancel (std_msgs/Empty) clears the active goal; the planner then publishes empty paths,
// which makes the controller stop.
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation/astar.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace navigation
{

class AStarPlannerNode : public rclcpp::Node
{
public:
  AStarPlannerNode()
  : Node("astar_planner")
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    AStarParams p;
    p.inflation_radius = declare_parameter<double>("inflation_radius", 0.30);
    p.robot_radius = declare_parameter<double>("robot_radius", 0.16);
    p.occupied_threshold = declare_parameter<int>("occupied_threshold", 65);
    p.unknown_is_free = declare_parameter<bool>("unknown_is_free", true);
    p.smooth = declare_parameter<bool>("smooth_path", true);
    p.output_spacing = declare_parameter<double>("output_spacing", 0.10);
    planner_ = std::make_unique<AStarPlanner>(p);
    goal_reached_distance_ = declare_parameter<double>("goal_reached_distance", 0.20);
    const double rate = declare_parameter<double>("replan_rate_hz", 2.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    plan_pub_ = create_publisher<nav_msgs::msg::Path>("plan", 10);
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "costmap", rclcpp::QoS(1),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {on_costmap(std::move(m));});
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr g) {on_goal(std::move(g));});
    cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
      "goal_cancel", 10, [this](std_msgs::msg::Empty::ConstSharedPtr) {
        have_goal_ = false;
        RCLCPP_INFO(get_logger(), "goal cancelled");
      });

    // ROS-clock timer: the period is in simulation seconds, not wall seconds
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this]() {on_timer();});
    RCLCPP_INFO(
      get_logger(), "astar_planner up: inflation %.2f m, unknown_is_free=%d, replan %.1f Hz",
      p.inflation_radius, p.unknown_is_free, rate);
  }

private:
  void on_costmap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr m)
  {
    GridMap g;
    g.width = static_cast<int>(m->info.width);
    g.height = static_cast<int>(m->info.height);
    g.resolution = m->info.resolution;
    g.origin_x = m->info.origin.position.x;
    g.origin_y = m->info.origin.position.y;
    g.data.assign(m->data.begin(), m->data.end());
    map_ = std::move(g);
    have_map_ = true;
  }

  void on_goal(geometry_msgs::msg::PoseStamped::ConstSharedPtr g)
  {
    if (g->header.frame_id != global_frame_ && !g->header.frame_id.empty()) {
      RCLCPP_WARN(
        get_logger(), "goal frame '%s' != '%s'; goal ignored", g->header.frame_id.c_str(),
        global_frame_.c_str());
      return;
    }
    goal_ = Point{g->pose.position.x, g->pose.position.y};
    have_goal_ = true;
    RCLCPP_INFO(get_logger(), "new goal (%.2f, %.2f)", goal_.x, goal_.y);
  }

  void publish_path(const std::vector<Point> & pts)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = global_frame_;
    for (const auto & p : pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    plan_pub_->publish(path);
  }

  void on_timer()
  {
    if (!have_goal_) {
      publish_path({});
      return;
    }
    if (!have_map_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for /costmap");
      publish_path({});
      return;
    }
    Point start;
    try {
      const auto tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
      start = Point{tf.transform.translation.x, tf.transform.translation.y};
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no TF %s -> %s: %s", global_frame_.c_str(),
        robot_frame_.c_str(), ex.what());
      publish_path({});
      return;
    }
    if (std::hypot(goal_.x - start.x, goal_.y - start.y) < goal_reached_distance_) {
      RCLCPP_INFO(get_logger(), "goal reached (within %.2f m)", goal_reached_distance_);
      have_goal_ = false;
      publish_path({});
      return;
    }

    const PlanResult res = planner_->plan(map_, start, goal_);
    if (res.status != PlanStatus::OK) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "no plan to (%.2f, %.2f): %s -> publishing empty path",
        goal_.x, goal_.y, to_string(res.status));
      publish_path({});
      return;
    }
    publish_path(res.path);
  }

  std::string global_frame_;
  std::string robot_frame_;
  double goal_reached_distance_{0.2};
  std::unique_ptr<AStarPlanner> planner_;
  GridMap map_;
  bool have_map_{false};
  Point goal_;
  bool have_goal_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation::AStarPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
