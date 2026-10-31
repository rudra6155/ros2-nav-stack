#include "navigation/astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace navigation
{

bool GridMap::world_to_cell(double wx, double wy, int & cx, int & cy) const
{
  cx = static_cast<int>(std::floor((wx - origin_x) / resolution));
  cy = static_cast<int>(std::floor((wy - origin_y) / resolution));
  return in_bounds(cx, cy);
}

Point GridMap::cell_center(int cx, int cy) const
{
  return Point{origin_x + (cx + 0.5) * resolution, origin_y + (cy + 0.5) * resolution};
}

const char * to_string(PlanStatus s)
{
  switch (s) {
    case PlanStatus::OK: return "ok";
    case PlanStatus::INVALID_MAP: return "invalid or empty map";
    case PlanStatus::START_OUT_OF_MAP: return "start is outside the map";
    case PlanStatus::GOAL_OUT_OF_MAP: return "goal is outside the map";
    case PlanStatus::GOAL_IN_OBSTACLE: return "goal is inside an obstacle";
    case PlanStatus::NO_PATH: return "no path (goal unreachable)";
  }
  return "unknown";
}

double path_length(const std::vector<Point> & path)
{
  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return len;
}

AStarPlanner::AStarPlanner(const AStarParams & params)
: params_(params)
{
}

std::vector<uint8_t> AStarPlanner::occupied_mask(const GridMap & map) const
{
  std::vector<uint8_t> occ(map.data.size(), 0);
  for (size_t i = 0; i < map.data.size(); ++i) {
    const int v = map.data[i];
    if (v < 0) {
      occ[i] = params_.unknown_is_free ? 0 : 1;
    } else {
      occ[i] = v >= params_.occupied_threshold ? 1 : 0;
    }
  }
  return occ;
}

namespace
{
// Offsets (dx, dy) of all cells whose centre lies within `radius_cells` of the origin cell centre.
std::vector<std::pair<int, int>> disk_offsets(double radius_cells)
{
  std::vector<std::pair<int, int>> out;
  const int r = static_cast<int>(std::ceil(radius_cells));
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (std::hypot(dx, dy) <= radius_cells + 1e-9) {
        out.emplace_back(dx, dy);
      }
    }
  }
  return out;
}
}  // namespace

std::vector<uint8_t> AStarPlanner::inflate(
  const GridMap & map, const std::vector<uint8_t> & occupied) const
{
  std::vector<uint8_t> out(occupied.size(), 0);
  const auto offs = disk_offsets(params_.inflation_radius / map.resolution);
  for (int cy = 0; cy < map.height; ++cy) {
    for (int cx = 0; cx < map.width; ++cx) {
      if (!occupied[map.index(cx, cy)]) {
        continue;
      }
      for (const auto & o : offs) {
        const int nx = cx + o.first, ny = cy + o.second;
        if (map.in_bounds(nx, ny)) {
          out[map.index(nx, ny)] = 1;
        }
      }
    }
  }
  return out;
}

namespace
{
struct QueueEntry
{
  double f;
  double h;
  int idx;
};
struct QueueCompare
{
  // min-heap on f, ties broken towards the smaller heuristic (closer to the goal)
  bool operator()(const QueueEntry & a, const QueueEntry & b) const
  {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    return a.h > b.h;
  }
};

bool segment_free(const GridMap & map, const std::vector<uint8_t> & blocked, Point a, Point b)
{
  const double len = std::hypot(b.x - a.x, b.y - a.y);
  const int steps = std::max(1, static_cast<int>(std::ceil(len / (0.5 * map.resolution))));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    int cx, cy;
    if (!map.world_to_cell(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), cx, cy)) {
      return false;
    }
    if (blocked[map.index(cx, cy)]) {
      return false;
    }
  }
  return true;
}

std::vector<Point> resample(const std::vector<Point> & poly, double spacing)
{
  std::vector<Point> out;
  if (poly.empty()) {
    return out;
  }
  out.push_back(poly.front());
  for (size_t i = 1; i < poly.size(); ++i) {
    const double len = std::hypot(poly[i].x - poly[i - 1].x, poly[i].y - poly[i - 1].y);
    const int n = std::max(1, static_cast<int>(std::ceil(len / spacing)));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      out.push_back(Point{poly[i - 1].x + t * (poly[i].x - poly[i - 1].x),
          poly[i - 1].y + t * (poly[i].y - poly[i - 1].y)});
    }
  }
  return out;
}
}  // namespace

PlanResult AStarPlanner::plan(const GridMap & map, Point start, Point goal) const
{
  PlanResult res;
  if (!map.valid()) {
    res.status = PlanStatus::INVALID_MAP;
    return res;
  }
  int sx, sy, gx, gy;
  if (!map.world_to_cell(start.x, start.y, sx, sy)) {
    res.status = PlanStatus::START_OUT_OF_MAP;
    return res;
  }
  if (!map.world_to_cell(goal.x, goal.y, gx, gy)) {
    res.status = PlanStatus::GOAL_OUT_OF_MAP;
    return res;
  }

  const std::vector<uint8_t> occupied = occupied_mask(map);
  if (occupied[map.index(gx, gy)]) {
    res.status = PlanStatus::GOAL_IN_OBSTACLE;
    return res;
  }
  std::vector<uint8_t> blocked = inflate(map, occupied);

  // Relaxation around start and goal (see header).
  const double infl_cells = params_.inflation_radius / map.resolution;
  const double body_cells = params_.robot_radius / map.resolution;
  for (const auto & o : disk_offsets(infl_cells)) {
    for (int k = 0; k < 2; ++k) {
      const int nx = (k == 0 ? sx : gx) + o.first;
      const int ny = (k == 0 ? sy : gy) + o.second;
      if (map.in_bounds(nx, ny) && !occupied[map.index(nx, ny)]) {
        blocked[map.index(nx, ny)] = 0;
      }
    }
  }
  for (const auto & o : disk_offsets(body_cells)) {  // the robot's own footprint is never an obstacle
    const int nx = sx + o.first, ny = sy + o.second;
    if (map.in_bounds(nx, ny)) {
      blocked[map.index(nx, ny)] = 0;
    }
  }
  blocked[map.index(sx, sy)] = 0;

  const int W = map.width, H = map.height;
  const double D = map.resolution;
  const double D2 = map.resolution * std::sqrt(2.0);
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> g(static_cast<size_t>(W) * H, INF);
  std::vector<int> parent(static_cast<size_t>(W) * H, -1);
  std::vector<uint8_t> closed(static_cast<size_t>(W) * H, 0);

  auto heuristic = [&](int x, int y) {
      const double dx = std::abs(x - gx), dy = std::abs(y - gy);
      return D * (dx + dy) + (D2 - 2.0 * D) * std::min(dx, dy);  // octile distance
    };

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open;
  const int start_idx = map.index(sx, sy);
  const int goal_idx = map.index(gx, gy);
  g[start_idx] = 0.0;
  open.push({heuristic(sx, sy), heuristic(sx, sy), start_idx});

  static const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;
  while (!open.empty()) {
    const QueueEntry cur = open.top();
    open.pop();
    if (closed[cur.idx]) {
      continue;
    }
    closed[cur.idx] = 1;
    ++res.expansions;
    if (cur.idx == goal_idx) {
      found = true;
      break;
    }
    const int cx = cur.idx % W, cy = cur.idx / W;
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + DX[k], ny = cy + DY[k];
      if (!map.in_bounds(nx, ny)) {
        continue;
      }
      const int nidx = map.index(nx, ny);
      if (closed[nidx] || blocked[nidx]) {
        continue;
      }
      const bool diagonal = (DX[k] != 0 && DY[k] != 0);
      if (diagonal && (blocked[map.index(cx + DX[k], cy)] || blocked[map.index(cx, cy + DY[k])])) {
        continue;  // no corner cutting
      }
      const double ng = g[cur.idx] + (diagonal ? D2 : D);
      if (ng < g[nidx]) {
        g[nidx] = ng;
        parent[nidx] = cur.idx;
        const double h = heuristic(nx, ny);
        open.push({ng + h, h, nidx});
      }
    }
  }

  if (!found) {
    res.status = PlanStatus::NO_PATH;
    return res;
  }

  std::vector<Point> cells;
  for (int i = goal_idx; i != -1; i = parent[i]) {
    cells.push_back(map.cell_center(i % W, i / W));
  }
  std::reverse(cells.begin(), cells.end());

  std::vector<Point> poly;
  if (params_.smooth && cells.size() > 2) {
    size_t i = 0;
    poly.push_back(cells.front());
    while (i + 1 < cells.size()) {
      size_t j = cells.size() - 1;
      while (j > i + 1 && !segment_free(map, blocked, cells[i], cells[j])) {
        --j;
      }
      poly.push_back(cells[j]);
      i = j;
    }
  } else {
    poly = cells;
  }
  // exact endpoints instead of cell centres
  poly.front() = start;
  poly.back() = goal;

  res.path = resample(poly, params_.output_spacing);
  res.length = path_length(res.path);
  res.status = PlanStatus::OK;
  return res;
}

}  // namespace navigation
