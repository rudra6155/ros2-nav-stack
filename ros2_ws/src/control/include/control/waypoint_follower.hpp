// waypoint_follower: /plan (nav_msgs/Path) + /odom (nav_msgs/Odometry) -> /cmd_vel (geometry_msgs/Twist)
// via pure pursuit (navigation::PurePursuit). Velocity limits are parameters (loaded from
// robot_description/config/robot_limits.yaml in the launch files) and are enforced on every command.
//
// Safety behaviour: the robot is commanded to stop (zero Twist, published every cycle) when
//   - there is no plan, or the plan is empty (planner could not reach the goal), or
//   - the last plan is older than `plan_timeout_s` (planner dead/stalled), or
//   - no odometry has been received yet.
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation/pure_pursuit.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace control
{

class WaypointFollower : public rclcpp::Node
{
public:
  explicit WaypointFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("waypoint_follower", options)
  {
    navigation::PurePursuitParams p;
    p.lookahead_distance = declare_parameter<double>("lookahead_distance", 0.6);
    p.max_linear_velocity = declare_parameter<double>("max_linear_velocity", 0.5);
    p.max_angular_velocity = declare_parameter<double>("max_angular_velocity", 1.2);
    p.min_linear_velocity = declare_parameter<double>("min_linear_velocity", 0.08);
    p.goal_tolerance = declare_parameter<double>("goal_tolerance", 0.10);
    p.slowdown_distance = declare_parameter<double>("slowdown_distance", 0.8);
    p.rotate_in_place_angle = declare_parameter<double>("rotate_in_place_angle", 0.9);
    p.heading_gain = declare_parameter<double>("heading_gain", 1.5);
    pp_.set_params(p);
    plan_timeout_s_ = declare_parameter<double>("plan_timeout_s", 3.0);
    const double rate = declare_parameter<double>("control_rate_hz", 20.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "plan", 10, [this](nav_msgs::msg::Path::ConstSharedPtr m) {on_plan(std::move(m));});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_odom(std::move(m));});
    // ROS-clock timer: the period is in simulation seconds, not wall seconds
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this]() {on_timer();});

    RCLCPP_INFO(
      get_logger(), "waypoint_follower up: lookahead %.2f m, limits v<=%.2f m/s w<=%.2f rad/s",
      p.lookahead_distance, p.max_linear_velocity, p.max_angular_velocity);
  }

private:
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr m)
  {
    odom_frame_ = m->header.frame_id;
    const auto & q = m->pose.pose.orientation;
    pose_.x = m->pose.pose.position.x;
    pose_.y = m->pose.pose.position.y;
    pose_.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_odom_ = true;
    if (pending_plan_) {
      auto plan = pending_plan_;
      pending_plan_.reset();
      on_plan(plan);
    }
  }

  void on_plan(nav_msgs::msg::Path::ConstSharedPtr m)
  {
    if (m->poses.empty()) {
      pp_.set_path({});
      plan_nonempty_ = false;
      return;
    }
    if (!have_odom_) {
      pending_plan_ = m;  // transform into the odom frame once we know it
      return;
    }
    // Express the path in the odometry frame (the pose we control is in that frame).
    double tx = 0.0, ty = 0.0, c = 1.0, s = 0.0;
    const std::string & plan_frame = m->header.frame_id;
    if (!plan_frame.empty() && plan_frame != odom_frame_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(odom_frame_, plan_frame, tf2::TimePointZero);
        const auto & q = tf.transform.rotation;
        const double yaw =
          std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        c = std::cos(yaw);
        s = std::sin(yaw);
        tx = tf.transform.translation.x;
        ty = tf.transform.translation.y;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "cannot transform plan %s -> %s: %s; stopping",
          plan_frame.c_str(), odom_frame_.c_str(), ex.what());
        pp_.set_path({});
        plan_nonempty_ = false;
        return;
      }
    }
    std::vector<navigation::Point> pts;
    pts.reserve(m->poses.size());
    for (const auto & ps : m->poses) {
      const double x = ps.pose.position.x, y = ps.pose.position.y;
      pts.push_back({tx + c * x - s * y, ty + s * x + c * y});
    }
    pp_.set_path(std::move(pts));
    plan_nonempty_ = true;
    last_plan_time_ = now();
  }

  void publish(double v, double w)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = v;
    t.angular.z = w;
    cmd_pub_->publish(t);
  }

  void on_timer()
  {
    if (!have_odom_ || !plan_nonempty_ || !pp_.has_path()) {
      publish(0.0, 0.0);
      return;
    }
    if ((now() - last_plan_time_).seconds() > plan_timeout_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "plan is stale (> %.1f s); stopping", plan_timeout_s_);
      publish(0.0, 0.0);
      return;
    }
    const auto cmd = pp_.compute(pose_);
    const auto & prm = pp_.params();
    // the controller already respects the limits; clamp again so no code path can exceed them
    const double v = std::min(prm.max_linear_velocity, std::max(0.0, cmd.linear));
    const double w = std::min(prm.max_angular_velocity, std::max(-prm.max_angular_velocity, cmd.angular));
    publish(v, w);
  }

  navigation::PurePursuit pp_;
  navigation::Pose2D pose_;
  bool have_odom_{false};
  std::string odom_frame_{"odom"};
  bool plan_nonempty_{false};
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
  double plan_timeout_s_{3.0};
  nav_msgs::msg::Path::ConstSharedPtr pending_plan_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace control
