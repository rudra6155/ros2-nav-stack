// controller_check: Gazebo-in-the-loop version of the controller contract.
//
//   "given a straight path and a robot starting 0.5 m off-axis, the controller drives the
//    cross-track error below 0.1 m within 5 s in simulation"
//
// Preconditions (scripts/check_controller_gazebo.sh sets them up): the `empty` world is running
// headless with the robot spawned, the waypoint_follower node is running (NO planner, so nothing
// else publishes /plan), everything on sim time.
//
// The check teleports the robot 0.5 m off a straight path along the x axis, publishes that path
// as /plan (re-published at 2 Hz like the planner would), and measures the true cross-track error
// from the ground-truth odometry for `--duration` sim seconds. It also records the largest |v| and
// |w| actually commanded on /cmd_vel and compares them with the configured limits.
// Result: results/controller_check.json; exit code 0 = pass.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigation/pure_pursuit.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ControllerCheck : public rclcpp::Node
{
public:
  ControllerCheck()
  : Node("controller_check", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}))
  {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 50, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_odom(*m);});
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 50, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        std::lock_guard<std::mutex> l(mutex_);
        if (measuring_) {
          max_v_ = std::max(max_v_, std::fabs(m->linear.x));
          max_w_ = std::max(max_w_, std::fabs(m->angular.z));
        }
      });
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("plan", 10);
    set_state_ = create_client<gazebo_msgs::srv::SetEntityState>("set_entity_state");
  }

  bool wait_ready(int timeout_s)
  {
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < timeout_s) {
      if (now().nanoseconds() > 0 && odom_count_ > 5 && set_state_->service_is_ready() &&
        count_subscribers("plan") >= 1 && count_publishers("cmd_vel") >= 1)
      {
        return true;
      }
      std::this_thread::sleep_for(200ms);
    }
    return false;
  }

  bool teleport(double x, double y, double yaw)
  {
    auto req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    req->state.name = "diffbot";
    req->state.pose.position.x = x;
    req->state.pose.position.y = y;
    req->state.pose.position.z = 0.06;
    req->state.pose.orientation.z = std::sin(yaw / 2.0);
    req->state.pose.orientation.w = std::cos(yaw / 2.0);
    req->state.reference_frame = "world";
    auto prom = std::make_shared<std::promise<bool>>();
    auto fut = prom->get_future();
    set_state_->async_send_request(
      req, [prom](rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedFuture f) {
        prom->set_value(f.get()->success);
      });
    if (fut.wait_for(10s) != std::future_status::ready || !fut.get()) {return false;}
    for (int i = 0; i < 100; ++i) {  // wait for the odometry to show the new pose at rest
      {
        std::lock_guard<std::mutex> l(mutex_);
        if (std::hypot(x_ - x, y_ - y) < 0.05 && speed_ < 0.05) {return true;}
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  void publish_plan(const std::vector<navigation::Point> & pts)
  {
    nav_msgs::msg::Path p;
    p.header.frame_id = "map";
    p.header.stamp = now();
    for (const auto & q : pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = p.header;
      ps.pose.position.x = q.x;
      ps.pose.position.y = q.y;
      ps.pose.orientation.w = 1.0;
      p.poses.push_back(ps);
    }
    plan_pub_->publish(p);
  }

  struct Sample {double t; double err;};

  std::vector<Sample> run(const std::vector<navigation::Point> & path, double duration_s)
  {
    {
      std::lock_guard<std::mutex> l(mutex_);
      path_ = path;
      samples_.clear();
      max_v_ = max_w_ = 0.0;
      measuring_ = true;
      t0_ = last_t_;
    }
    publish_plan(path);
    double last_pub = now().seconds();
    const double start = last_pub;
    while (now().seconds() - start < duration_s + 0.2) {
      std::this_thread::sleep_for(20ms);
      if (now().seconds() - last_pub >= 0.5) {
        publish_plan(path);
        last_pub = now().seconds();
      }
    }
    std::lock_guard<std::mutex> l(mutex_);
    measuring_ = false;
    return samples_;
  }

  double max_v() {std::lock_guard<std::mutex> l(mutex_); return max_v_;}
  double max_w() {std::lock_guard<std::mutex> l(mutex_); return max_w_;}

private:
  void on_odom(const nav_msgs::msg::Odometry & m)
  {
    std::lock_guard<std::mutex> l(mutex_);
    ++odom_count_;
    x_ = m.pose.pose.position.x;
    y_ = m.pose.pose.position.y;
    speed_ = std::hypot(m.twist.twist.linear.x, m.twist.twist.linear.y);
    last_t_ = rclcpp::Time(m.header.stamp).seconds();
    if (measuring_) {
      samples_.push_back({last_t_ - t0_, navigation::PurePursuit::cross_track_error(path_, x_, y_)});
    }
  }

  std::mutex mutex_;
  std::atomic<int> odom_count_{0};
  double x_{0}, y_{0}, speed_{0}, last_t_{0}, t0_{0};
  double max_v_{0}, max_w_{0};
  bool measuring_{false};
  std::vector<navigation::Point> path_;
  std::vector<Sample> samples_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string results_dir = "/workspace/results";
  double offset = 0.5, duration = 5.0, threshold = 0.1, vmax = 0.5, wmax = 1.2;
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  for (size_t i = 1; i + 1 < args.size(); i += 2) {
    if (args[i] == "--results-dir") {results_dir = args[i + 1];}
    else if (args[i] == "--offset") {offset = std::stod(args[i + 1]);}
    else if (args[i] == "--duration") {duration = std::stod(args[i + 1]);}
    else if (args[i] == "--threshold") {threshold = std::stod(args[i + 1]);}
    else if (args[i] == "--vmax") {vmax = std::stod(args[i + 1]);}
    else if (args[i] == "--wmax") {wmax = std::stod(args[i + 1]);}
  }

  auto node = std::make_shared<ControllerCheck>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread spinner([&exec]() {exec.spin();});

  int rc = 1;
  if (!node->wait_ready(60)) {
    std::cerr << "controller_check: stack not ready (need /odom, set_entity_state, a /plan subscriber)\n";
  } else {
    // straight path along the x axis at y = 0; the robot starts `offset` m beside it
    const double x0 = -3.0;
    std::vector<navigation::Point> path;
    for (double x = x0; x <= 5.0 + 1e-9; x += 0.1) {path.push_back({x, 0.0});}
    if (!node->teleport(x0, offset, 0.0)) {
      std::cerr << "controller_check: could not place the robot\n";
    } else {
      const auto samples = node->run(path, duration);
      // settle time: first time after which the error stays below the threshold
      double settle = -1.0;
      for (size_t i = samples.size(); i-- > 0;) {
        if (samples[i].err >= threshold) {settle = (i + 1 < samples.size()) ? samples[i + 1].t : 1e9; break;}
        settle = 0.0;
      }
      double err_end = samples.empty() ? std::nan("") : samples.back().err;
      for (const auto & s : samples) {if (s.t <= duration) {err_end = s.err;}}
      const double v = node->max_v(), w = node->max_w();
      const bool ok = !samples.empty() && err_end < threshold && settle <= duration &&
        v <= vmax + 1e-6 && w <= wmax + 1e-6 && v > 0.1;
      std::ofstream out(results_dir + "/controller_check.json");
      out << "{\n  \"start_offset_m\": " << offset << ",\n  \"duration_s\": " << duration
          << ",\n  \"threshold_m\": " << threshold << ",\n  \"initial_error_m\": "
          << (samples.empty() ? std::nan("") : samples.front().err)
          << ",\n  \"error_at_duration_m\": " << err_end << ",\n  \"settle_time_s\": " << settle
          << ",\n  \"max_commanded_linear_mps\": " << v << ",\n  \"max_commanded_angular_radps\": " << w
          << ",\n  \"limit_linear_mps\": " << vmax << ",\n  \"limit_angular_radps\": " << wmax
          << ",\n  \"samples\": " << samples.size() << ",\n  \"passed\": " << (ok ? "true" : "false") << "\n}\n";
      std::printf(
        "controller_check: offset %.2f m -> error at %.1f s = %.3f m (threshold %.2f), settled below it at %.2f s, "
        "max |v| %.3f (limit %.2f), max |w| %.3f (limit %.2f) -> %s\n",
        offset, duration, err_end, threshold, settle, v, vmax, w, wmax, ok ? "PASS" : "FAIL");
      rc = ok ? 0 : 1;
    }
  }
  node->publish_plan({});
  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return rc;
}
