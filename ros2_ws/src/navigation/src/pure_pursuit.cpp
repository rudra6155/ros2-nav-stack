#include "navigation/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace navigation
{

namespace
{
double clamp(double v, double lo, double hi) {return std::min(hi, std::max(lo, v));}

// Projection of p onto segment [a, b]: returns the parameter t in [0, 1] and the squared distance.
std::pair<double, double> project(Point a, Point b, double px, double py)
{
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > 1e-12) {
    t = clamp(((px - a.x) * dx + (py - a.y) * dy) / len2, 0.0, 1.0);
  }
  const double qx = a.x + t * dx - px, qy = a.y + t * dy - py;
  return {t, qx * qx + qy * qy};
}
}  // namespace

PurePursuit::PurePursuit(const PurePursuitParams & params)
: params_(params)
{
}

void PurePursuit::set_path(std::vector<Point> path)
{
  path_ = std::move(path);
  cumulative_.assign(path_.size(), 0.0);
  for (size_t i = 1; i < path_.size(); ++i) {
    cumulative_[i] = cumulative_[i - 1] +
      std::hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y);
  }
  segment_ = 0;
  fresh_ = true;
}

double PurePursuit::cross_track_error(const std::vector<Point> & path, double x, double y)
{
  if (path.empty()) {
    return 0.0;
  }
  if (path.size() == 1) {
    return std::hypot(path[0].x - x, path[0].y - y);
  }
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    best = std::min(best, project(path[i], path[i + 1], x, y).second);
  }
  return std::sqrt(best);
}

VelocityCommand PurePursuit::compute(const Pose2D & pose)
{
  VelocityCommand cmd;
  if (path_.empty()) {
    return cmd;
  }

  const Point goal = path_.back();
  const double dist_goal = std::hypot(goal.x - pose.x, goal.y - pose.y);
  cmd.lookahead_point = goal;
  if (dist_goal < params_.goal_tolerance) {
    cmd.goal_reached = true;
    cmd.cross_track_error = cross_track_error(path_, pose.x, pose.y);
    return cmd;
  }

  // 1. closest point on the path (searching forward from the current segment)
  double s_closest = 0.0;
  double best_d2 = std::numeric_limits<double>::infinity();
  size_t best_seg = segment_;
  if (path_.size() == 1) {
    best_d2 = (path_[0].x - pose.x) * (path_[0].x - pose.x) +
      (path_[0].y - pose.y) * (path_[0].y - pose.y);
    s_closest = 0.0;
    best_seg = 0;
  } else {
    const size_t first = fresh_ ? 0 : segment_;
    const double window = 3.0;  // m of path searched ahead of the current progress
    double s_ref = fresh_ ? 0.0 : cumulative_[first];
    for (size_t i = first; i + 1 < path_.size(); ++i) {
      if (!fresh_ && cumulative_[i] - s_ref > window) {
        break;
      }
      const auto pr = project(path_[i], path_[i + 1], pose.x, pose.y);
      if (pr.second < best_d2 - 1e-12) {
        best_d2 = pr.second;
        best_seg = i;
        s_closest = cumulative_[i] + pr.first * (cumulative_[i + 1] - cumulative_[i]);
      }
    }
    segment_ = best_seg;
    fresh_ = false;
  }
  cmd.cross_track_error = std::sqrt(best_d2);

  // 2. lookahead point: `lookahead_distance` further along the path (or the goal if shorter)
  Point target = goal;
  if (path_.size() > 1) {
    const double s_target = std::min(s_closest + params_.lookahead_distance, cumulative_.back());
    size_t i = best_seg;
    while (i + 1 < path_.size() - 1 && cumulative_[i + 1] < s_target) {
      ++i;
    }
    const double seg_len = cumulative_[i + 1] - cumulative_[i];
    const double t = seg_len > 1e-9 ? clamp((s_target - cumulative_[i]) / seg_len, 0.0, 1.0) : 1.0;
    target = Point{path_[i].x + t * (path_[i + 1].x - path_[i].x),
      path_[i].y + t * (path_[i + 1].y - path_[i].y)};
  }
  cmd.lookahead_point = target;

  // 3. lookahead point in the robot frame
  const double dx = target.x - pose.x, dy = target.y - pose.y;
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  const double lx = c * dx + s * dy;
  const double ly = -s * dx + c * dy;
  const double ld2 = std::max(lx * lx + ly * ly, 1e-6);
  const double alpha = std::atan2(ly, lx);

  const double vmax = params_.max_linear_velocity;
  const double wmax = params_.max_angular_velocity;

  if (std::fabs(alpha) > params_.rotate_in_place_angle) {
    // facing away from the path: turn on the spot first
    double w = params_.heading_gain * alpha;
    const double w_min = 0.25;
    if (std::fabs(w) < w_min) {
      w = std::copysign(w_min, alpha);
    }
    cmd.linear = 0.0;
    cmd.angular = clamp(w, -wmax, wmax);
    return cmd;
  }

  const double kappa = 2.0 * ly / ld2;
  double v = vmax * std::max(0.3, std::cos(alpha));       // slow down when not aligned
  const double slow = vmax * dist_goal / std::max(params_.slowdown_distance, 1e-3);
  v = std::min(v, std::max(params_.min_linear_velocity, slow));  // slow down near the goal
  v = clamp(v, 0.0, vmax);
  double w = v * kappa;
  if (std::fabs(w) > wmax) {
    v *= wmax / std::fabs(w);  // keep the commanded curvature when the angular clamp binds
    w = std::copysign(wmax, w);
  }
  cmd.linear = v;
  cmd.angular = w;
  return cmd;
}

}  // namespace navigation
