// ROS-free core of the perception pipeline (unit-testable without a ROS graph):
//   1. fixed-resolution downsampling of a LaserScan
//   2. clustering of scan hits into obstacles (centroids)
//   3. known-pose log-odds occupancy mapping (this is NOT SLAM: poses are given)
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <cstddef>
#include <vector>

namespace perception
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

/// Scan resampled to a fixed angular grid: ranges[k] is along angle angle_min + k*angle_increment.
///   finite value in [range_min, range_max): a hit
///   +inf : the beam saw nothing out to range_max (free space)
///   NaN  : no usable measurement in the bin
struct DownsampledScan
{
  double angle_min{-M_PI};
  double angle_increment{0.0};
  std::vector<float> ranges;
};

struct Cluster
{
  std::vector<Point2> points;
  Point2 centroid;
};

inline double wrap_to_pi(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

/// Resample a (full-circle) scan to bins of `target_resolution` rad on the grid
/// theta_k = -pi + k * target_resolution, k = 0..N-1, N = round(2*pi / target_resolution).
/// Each input beam goes to its nearest bin; a bin keeps the closest valid hit (conservative).
/// Priority inside a bin: valid hit (min) > "nothing seen" (+inf) > unusable (NaN).
inline DownsampledScan downsample_scan(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double target_resolution, double range_min, double range_max)
{
  DownsampledScan out;
  const int n_bins = std::max(1, static_cast<int>(std::lround(2.0 * M_PI / target_resolution)));
  out.angle_min = -M_PI;
  out.angle_increment = 2.0 * M_PI / n_bins;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  out.ranges.assign(n_bins, nan);

  for (size_t i = 0; i < ranges.size(); ++i) {
    const double angle = wrap_to_pi(angle_min + static_cast<double>(i) * angle_increment);
    int k = static_cast<int>(std::lround((angle - out.angle_min) / out.angle_increment));
    k = ((k % n_bins) + n_bins) % n_bins;
    const float r = ranges[i];
    float & bin = out.ranges[k];
    if (std::isfinite(r) && r >= range_min && r < range_max) {
      // valid hit: keep the closest one
      if (!std::isfinite(bin) || r < bin) {
        bin = r;
      }
    } else if ((std::isinf(r) && r > 0.0f) || (std::isfinite(r) && r >= range_max)) {
      // nothing seen out to max range: only upgrade an unusable bin
      if (std::isnan(bin)) {
        bin = inf;
      }
    }
  }
  return out;
}

inline bool is_hit(float r, double range_min, double range_max)
{
  return std::isfinite(r) && r >= range_min && r < range_max;
}

/// Group consecutive hit beams into clusters. Two neighbouring beams belong to the same
/// cluster when their points are closer than  gap_m + 1.5 * resolution * max(r1, r2)
/// (the beam spacing grows linearly with range). The scan wraps around at +-pi.
/// Centroids are in the sensor frame.
inline std::vector<Cluster> cluster_scan(
  const DownsampledScan & scan, double range_min, double range_max, double gap_m, int min_points)
{
  std::vector<Cluster> clusters;
  const int n = static_cast<int>(scan.ranges.size());
  if (n == 0) {
    return clusters;
  }

  auto point_of = [&](int k) {
      const double a = scan.angle_min + k * scan.angle_increment;
      const double r = scan.ranges[k];
      return Point2{r * std::cos(a), r * std::sin(a)};
    };
  auto close = [&](int k1, int k2) {
      const Point2 p1 = point_of(k1);
      const Point2 p2 = point_of(k2);
      const double rmax = std::max<double>(scan.ranges[k1], scan.ranges[k2]);
      const double thr = gap_m + 1.5 * scan.angle_increment * rmax;
      return std::hypot(p1.x - p2.x, p1.y - p2.y) < thr;
    };

  std::vector<std::vector<int>> groups;
  for (int k = 0; k < n; ++k) {
    if (!is_hit(scan.ranges[k], range_min, range_max)) {
      continue;
    }
    if (k > 0 && is_hit(scan.ranges[k - 1], range_min, range_max) && !groups.empty() &&
      groups.back().back() == k - 1 && close(k - 1, k))
    {
      groups.back().push_back(k);
    } else {
      groups.push_back({k});
    }
  }
  // wrap-around: join the last group to the first if the seam beams are neighbours
  if (groups.size() > 1 && groups.front().front() == 0 && groups.back().back() == n - 1 &&
    close(n - 1, 0))
  {
    groups.front().insert(groups.front().begin(), groups.back().begin(), groups.back().end());
    groups.pop_back();
  }

  for (const auto & g : groups) {
    if (static_cast<int>(g.size()) < min_points) {
      continue;
    }
    Cluster c;
    double sx = 0.0, sy = 0.0;
    for (int k : g) {
      const Point2 p = point_of(k);
      c.points.push_back(p);
      sx += p.x;
      sy += p.y;
    }
    c.centroid = Point2{sx / g.size(), sy / g.size()};
    clusters.push_back(std::move(c));
  }
  return clusters;
}

/// Known-pose occupancy grid (log-odds, Bresenham ray casting).
class OccupancyMapper
{
public:
  struct Params
  {
    double resolution{0.1};
    int width{140};
    int height{140};
    double origin_x{-7.0};
    double origin_y{-7.0};
    float l_occ{0.85f};
    float l_free{-0.40f};
    float l_min{-2.0f};
    float l_max{3.5f};
    float occupied_threshold{0.5f};
  };

  explicit OccupancyMapper(const Params & p)
  : p_(p), logodds_(static_cast<size_t>(p.width) * p.height, 0.0f),
    observed_(static_cast<size_t>(p.width) * p.height, 0)
  {
  }

  const Params & params() const {return p_;}

  void reset()
  {
    std::fill(logodds_.begin(), logodds_.end(), 0.0f);
    std::fill(observed_.begin(), observed_.end(), 0);
  }

  bool world_to_cell(double x, double y, int & cx, int & cy) const
  {
    cx = static_cast<int>(std::floor((x - p_.origin_x) / p_.resolution));
    cy = static_cast<int>(std::floor((y - p_.origin_y) / p_.resolution));
    return cx >= 0 && cy >= 0 && cx < p_.width && cy < p_.height;
  }

  /// Integrate one (downsampled) scan taken from sensor pose (sx, sy, syaw) in the map frame.
  void integrate(
    double sx, double sy, double syaw, const DownsampledScan & scan, double range_min,
    double range_max)
  {
    int x0, y0;
    if (!world_to_cell(sx, sy, x0, y0)) {
      return;
    }
    for (size_t k = 0; k < scan.ranges.size(); ++k) {
      const float r = scan.ranges[k];
      const bool hit = is_hit(r, range_min, range_max);
      const bool free_ray = std::isinf(r) && r > 0.0f;
      if (!hit && !free_ray) {
        continue;
      }
      const double a = syaw + scan.angle_min + static_cast<double>(k) * scan.angle_increment;
      const double len = hit ? static_cast<double>(r) : range_max;
      const double ex = sx + len * std::cos(a);
      const double ey = sy + len * std::sin(a);
      int x1 = static_cast<int>(std::floor((ex - p_.origin_x) / p_.resolution));
      int y1 = static_cast<int>(std::floor((ey - p_.origin_y) / p_.resolution));
      trace(x0, y0, x1, y1, hit);
    }
  }

  /// -1 unknown, 0 free, 100 occupied (row-major, y * width + x)
  std::vector<int8_t> occupancy() const
  {
    std::vector<int8_t> out(logodds_.size(), -1);
    for (size_t i = 0; i < logodds_.size(); ++i) {
      if (!observed_[i]) {
        continue;
      }
      out[i] = logodds_[i] > p_.occupied_threshold ? 100 : 0;
    }
    return out;
  }

private:
  void update(int cx, int cy, float delta)
  {
    if (cx < 0 || cy < 0 || cx >= p_.width || cy >= p_.height) {
      return;
    }
    const size_t i = static_cast<size_t>(cy) * p_.width + cx;
    observed_[i] = 1;
    logodds_[i] = std::min(p_.l_max, std::max(p_.l_min, logodds_[i] + delta));
  }

  // Bresenham from (x0,y0) to (x1,y1): cells before the end are free; the end cell is
  // occupied when the beam ended on a hit (and free when it was a max-range / no-hit beam).
  void trace(int x0, int y0, int x1, int y1, bool hit)
  {
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1;
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    while (true) {
      const bool last = (x == x1 && y == y1);
      if (x < 0 || y < 0 || x >= p_.width || y >= p_.height) {
        return;  // left the map before reaching the end: nothing more to update
      }
      if (last) {
        update(x, y, hit ? p_.l_occ : p_.l_free);
        return;
      }
      update(x, y, p_.l_free);
      const int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y += sy;
      }
    }
  }

  Params p_;
  std::vector<float> logodds_;
  std::vector<uint8_t> observed_;
};

}  // namespace perception
