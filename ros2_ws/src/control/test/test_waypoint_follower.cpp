// Node-in-the-loop controller test.
//
// The REAL control::WaypointFollower node runs against a simulated plant (a differential-drive
// kinematic model with first-order velocity lag and acceleration limits, i.e. what the Gazebo
// diff-drive plugin approximates). The plant is a stand-in for Gazebo, not Gazebo itself - the
// Gazebo-in-the-loop version of this check is `ros2 run evaluation controller_check`.
//
// Scenario: straight path along the x axis, robot starts 0.5 m off-axis. The cross-track error
// must be below 0.1 m within 5 s and the commanded velocities must never exceed the limits.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "control/waypoint_follower.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation/pure_pursuit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

namespace
{
constexpr double kVMax = 0.5;
constexpr double kWMax = 1.2;

class Plant : public rclcpp::Node
{
public:
  Plant(double x, double y, double yaw)
  : Node("test_plant"), x_(x), y_(y), yaw_(yaw)
  {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        v_cmd_ = m->linear.x;
        w_cmd_ = m->angular.z;
        max_v_seen_ = std::max(max_v_seen_, std::fabs(m->linear.x));
        max_w_seen_ = std::max(max_w_seen_, std::fabs(m->angular.z));
      });
    path_pub_ = create_publisher<nav_msgs::msg::Path>("plan", 10);

    static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.rotation.w = 1.0;
    static_tf_->sendTransform(tf);

    for (double px = -1.0; px <= 12.0 + 1e-9; px += 0.1) {
      path_.push_back({px, 0.0});
    }
    t0_ = std::chrono::steady_clock::now();
    plant_timer_ = create_wall_timer(10ms, [this]() {step(0.01);});
    odom_timer_ = create_wall_timer(20ms, [this]() {publish_odom();});
    plan_timer_ = create_wall_timer(500ms, [this]() {publish_plan();});
  }

  struct Sample {double t; double err;};
  std::vector<Sample> samples() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_;
  }
  double max_v() const {std::lock_guard<std::mutex> l(mutex_); return max_v_seen_;}
  double max_w() const {std::lock_guard<std::mutex> l(mutex_); return max_w_seen_;}

private:
  void step(double dt)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // first-order lag (tau = 0.15 s) with acceleration limits
    const double tau = 0.15;
    const double dv = std::max(-1.0 * dt, std::min(1.0 * dt, (v_cmd_ - v_) * dt / tau));
    const double dw = std::max(-3.0 * dt, std::min(3.0 * dt, (w_cmd_ - w_) * dt / tau));
    v_ += dv;
    w_ += dw;
    x_ += v_ * std::cos(yaw_) * dt;
    y_ += v_ * std::sin(yaw_) * dt;
    yaw_ += w_ * dt;
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
    samples_.push_back({t, navigation::PurePursuit::cross_track_error(path_, x_, y_)});
  }

  void publish_odom()
  {
    nav_msgs::msg::Odometry o;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      o.pose.pose.position.x = x_;
      o.pose.pose.position.y = y_;
      o.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
      o.pose.pose.orientation.w = std::cos(yaw_ / 2.0);
      o.twist.twist.linear.x = v_;
      o.twist.twist.angular.z = w_;
    }
    o.header.stamp = now();
    o.header.frame_id = "odom";
    o.child_frame_id = "base_link";
    odom_pub_->publish(o);
  }

  void publish_plan()
  {
    nav_msgs::msg::Path p;
    p.header.stamp = now();
    p.header.frame_id = "map";
    for (const auto & q : path_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = p.header;
      ps.pose.position.x = q.x;
      ps.pose.position.y = q.y;
      ps.pose.orientation.w = 1.0;
      p.poses.push_back(ps);
    }
    path_pub_->publish(p);
  }

  mutable std::mutex mutex_;
  double x_, y_, yaw_;
  double v_{0.0}, w_{0.0}, v_cmd_{0.0}, w_cmd_{0.0};
  double max_v_seen_{0.0}, max_w_seen_{0.0};
  std::vector<navigation::Point> path_;
  std::vector<Sample> samples_;
  std::chrono::steady_clock::time_point t0_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::TimerBase::SharedPtr plant_timer_, odom_timer_, plan_timer_;
};
}  // namespace

TEST(WaypointFollower, OffAxisStartConvergesBelow10cmWithin5sAndRespectsLimits)
{
  rclcpp::init(0, nullptr);
  {
    auto follower = std::make_shared<control::WaypointFollower>();
    auto plant = std::make_shared<Plant>(0.0, 0.5, 0.0);  // 0.5 m off the path, heading along it
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(follower);
    exec.add_node(plant);
    std::atomic<bool> stop{false};
    std::thread spinner([&]() {
        while (!stop && rclcpp::ok()) {
          exec.spin_some(5ms);
          std::this_thread::sleep_for(1ms);
        }
      });

    std::this_thread::sleep_for(5300ms);
    stop = true;
    spinner.join();

    const auto samples = plant->samples();
    ASSERT_GT(samples.size(), 400u) << "plant did not run";

    // first time after which the error stays below 0.1 m
    double settle_t = -1.0;
    for (size_t i = samples.size(); i-- > 0;) {
      if (samples[i].err >= 0.1) {
        settle_t = (i + 1 < samples.size()) ? samples[i + 1].t : 1e9;
        break;
      }
      settle_t = 0.0;
    }
    double err_at_5s = samples.back().err;
    for (const auto & s : samples) {
      if (s.t <= 5.0) {err_at_5s = s.err;}
    }
    std::printf(
      "[ INFO ] cross-track error: start %.3f m, at 5.0 s %.3f m, stays below 0.1 m from t = %.2f s; "
      "max |v| = %.3f, max |w| = %.3f\n",
      samples.front().err, err_at_5s, settle_t, plant->max_v(), plant->max_w());

    EXPECT_LT(err_at_5s, 0.1);
    EXPECT_LE(settle_t, 5.0);
    EXPECT_LE(plant->max_v(), kVMax + 1e-9);
    EXPECT_LE(plant->max_w(), kWMax + 1e-9);
    EXPECT_GT(plant->max_v(), 0.1) << "controller never commanded any motion";
  }
  rclcpp::shutdown();
}
