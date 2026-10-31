// Pure pursuit path tracker written for this project (ROS-free core).
//
// Given a polyline path and the robot pose, it picks the point `lookahead_distance` further along
// the path than the robot's closest point and steers on the circular arc towards it:
//     curvature = 2 * y_l / L^2     (y_l: lateral offset of the lookahead point in the robot frame)
// Both velocity components are clamped to the configured limits, and when the angular limit
// binds, the linear velocity is reduced so that the commanded curvature is preserved.
#pragma once

#include <cstddef>
#include <vector>

#include "navigation/astar.hpp"

namespace navigation
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct PurePursuitParams
{
  double lookahead_distance{0.6};   // m
  double max_linear_velocity{0.5};  // m/s    (clamp)
  double max_angular_velocity{1.2}; // rad/s  (clamp)
  double min_linear_velocity{0.08}; // m/s    creep speed on the final approach
  double goal_tolerance{0.10};      // m      stop when the final path point is this close
  double slowdown_distance{0.8};    // m      start slowing down this far from the goal
  double rotate_in_place_angle{0.9};// rad    heading error above which the robot turns on the spot
  double heading_gain{1.5};         // 1/s    proportional gain for turning on the spot
};

struct VelocityCommand
{
  double linear{0.0};
  double angular{0.0};
  bool goal_reached{false};
  double cross_track_error{0.0};    // m, distance from the robot to the path
  Point lookahead_point;
};

class PurePursuit
{
public:
  explicit PurePursuit(const PurePursuitParams & params = PurePursuitParams());

  void set_params(const PurePursuitParams & params) {params_ = params;}
  const PurePursuitParams & params() const {return params_;}

  /// Replace the path; the progress along the path restarts.
  void set_path(std::vector<Point> path);
  bool has_path() const {return !path_.empty();}

  /// Velocity command for the given pose. Zero command if there is no path.
  VelocityCommand compute(const Pose2D & pose);

  /// Euclidean distance from (x, y) to the polyline (0 for an empty path).
  static double cross_track_error(const std::vector<Point> & path, double x, double y);

private:
  PurePursuitParams params_;
  std::vector<Point> path_;
  std::vector<double> cumulative_;  // arclength at each vertex
  size_t segment_{0};               // progress: index of the current segment (monotonic)
  bool fresh_{true};
};

}  // namespace navigation
