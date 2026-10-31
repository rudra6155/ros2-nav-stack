// Ground-truth world geometry (the *.obstacles sidecar written by worlds/gen_worlds.py).
// Used for start/goal sampling, reachability checks and the min-obstacle-distance metric.
// It is evaluation-side ground truth: the navigation stack never sees it.
#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "navigation/astar.hpp"

namespace evaluation
{

struct Obstacle
{
  bool is_box{true};
  std::string name;
  double cx{0.0}, cy{0.0}, yaw{0.0}, sx{0.0}, sy{0.0}, r{0.0};

  /// Signed distance from (px, py) to the obstacle surface (negative inside).
  double sdf(double px, double py) const
  {
    if (!is_box) {
      return std::hypot(px - cx, py - cy) - r;
    }
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double dx = px - cx, dy = py - cy;
    const double lx = c * dx + s * dy, ly = -s * dx + c * dy;
    const double qx = std::fabs(lx) - sx / 2.0, qy = std::fabs(ly) - sy / 2.0;
    return std::hypot(std::max(qx, 0.0), std::max(qy, 0.0)) + std::min(std::max(qx, qy), 0.0);
  }
};

class WorldGeometry
{
public:
  static WorldGeometry load(const std::string & path)
  {
    std::ifstream in(path);
    if (!in) {
      throw std::runtime_error("cannot open world geometry file: " + path);
    }
    WorldGeometry w;
    std::string line;
    int lineno = 0;
    while (std::getline(in, line)) {
      ++lineno;
      if (line.empty() || line[0] == '#') {
        continue;
      }
      std::istringstream ss(line);
      std::string kind;
      ss >> kind;
      Obstacle o;
      if (kind == "arena") {
        ss >> w.arena_half_;
      } else if (kind == "box") {
        o.is_box = true;
        ss >> o.name >> o.cx >> o.cy >> o.yaw >> o.sx >> o.sy;
        w.obstacles_.push_back(o);
      } else if (kind == "cylinder") {
        o.is_box = false;
        ss >> o.name >> o.cx >> o.cy >> o.r;
        w.obstacles_.push_back(o);
      } else {
        throw std::runtime_error(
                path + ":" + std::to_string(lineno) + ": unknown record '" + kind + "'");
      }
      if (ss.fail()) {
        throw std::runtime_error(path + ":" + std::to_string(lineno) + ": malformed line");
      }
    }
    if (w.obstacles_.empty() || w.arena_half_ <= 0.0) {
      throw std::runtime_error("world geometry file has no arena/obstacles: " + path);
    }
    return w;
  }

  double arena_half() const {return arena_half_;}
  const std::vector<Obstacle> & obstacles() const {return obstacles_;}

  /// Distance from (x, y) to the nearest obstacle surface (walls included); negative inside one.
  double distance(double x, double y) const
  {
    double best = std::numeric_limits<double>::infinity();
    for (const auto & o : obstacles_) {
      best = std::min(best, o.sdf(x, y));
    }
    return best;
  }

  bool inside_arena(double x, double y) const
  {
    return std::fabs(x) < arena_half_ && std::fabs(y) < arena_half_;
  }

  /// Occupancy grid of the true world: a cell is occupied when its surface touches an obstacle.
  navigation::GridMap rasterize(double resolution, double origin_x, double origin_y, int width, int height) const
  {
    navigation::GridMap m;
    m.resolution = resolution;
    m.origin_x = origin_x;
    m.origin_y = origin_y;
    m.width = width;
    m.height = height;
    m.data.assign(static_cast<size_t>(width) * height, 0);
    for (int cy = 0; cy < height; ++cy) {
      for (int cx = 0; cx < width; ++cx) {
        const navigation::Point c = m.cell_center(cx, cy);
        if (distance(c.x, c.y) <= 0.5 * resolution) {
          m.data[m.index(cx, cy)] = 100;
        }
      }
    }
    return m;
  }

private:
  double arena_half_{0.0};
  std::vector<Obstacle> obstacles_;
};

}  // namespace evaluation
