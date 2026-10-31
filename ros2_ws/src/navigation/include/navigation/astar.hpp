// A* grid planner written for this project (nav2 is not used anywhere).
//
//  * 8-connected, octile-distance heuristic (admissible and consistent), no corner cutting
//  * runs on the *inflated* grid: every cell within `inflation_radius` of an occupied cell is blocked
//  * unknown cells (-1) are free or blocked according to `unknown_is_free`
//  * start/goal relaxation: the robot can always leave the inflated band it is standing in, and a
//    goal that is free but inside the inflation band is still reachable; a goal on an occupied
//    cell is rejected (empty path, status GOAL_IN_OBSTACLE)
//  * never returns a straight line when no path exists: the result is an empty path + a status
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace navigation
{

struct Point
{
  double x{0.0};
  double y{0.0};
};

struct GridMap
{
  int width{0};
  int height{0};
  double resolution{0.1};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<int8_t> data;  // row-major (y * width + x); -1 unknown, 0..100 occupancy

  bool valid() const
  {
    return width > 0 && height > 0 && resolution > 0.0 &&
           data.size() == static_cast<size_t>(width) * static_cast<size_t>(height);
  }
  int index(int cx, int cy) const {return cy * width + cx;}
  bool in_bounds(int cx, int cy) const {return cx >= 0 && cy >= 0 && cx < width && cy < height;}
  bool world_to_cell(double wx, double wy, int & cx, int & cy) const;
  Point cell_center(int cx, int cy) const;
};

struct AStarParams
{
  double inflation_radius{0.30};   // m, hard inflation around occupied cells
  double robot_radius{0.16};       // m, cells this close to the start are forced free
  int occupied_threshold{65};      // occupancy value >= threshold counts as occupied
  bool unknown_is_free{true};
  bool smooth{true};               // line-of-sight shortcutting of the grid path
  double output_spacing{0.10};     // m, spacing of the poses in the returned path
};

enum class PlanStatus
{
  OK,
  INVALID_MAP,
  START_OUT_OF_MAP,
  GOAL_OUT_OF_MAP,
  GOAL_IN_OBSTACLE,
  NO_PATH,
};

const char * to_string(PlanStatus s);

struct PlanResult
{
  PlanStatus status{PlanStatus::NO_PATH};
  std::vector<Point> path;  // empty unless status == OK
  double length{0.0};
  int expansions{0};
};

double path_length(const std::vector<Point> & path);

class AStarPlanner
{
public:
  explicit AStarPlanner(const AStarParams & params = AStarParams());

  const AStarParams & params() const {return params_;}

  PlanResult plan(const GridMap & map, Point start, Point goal) const;

  /// 1 where the cell itself is occupied (or unknown, if unknown_is_free is false).
  std::vector<uint8_t> occupied_mask(const GridMap & map) const;
  /// 1 where the cell is within inflation_radius of an occupied cell (occupied cells included).
  std::vector<uint8_t> inflate(const GridMap & map, const std::vector<uint8_t> & occupied) const;

private:
  AStarParams params_;
};

}  // namespace navigation
