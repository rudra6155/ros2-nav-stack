// A* unit tests: known maps with known geometric optima.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "navigation/astar.hpp"

using navigation::AStarParams;
using navigation::AStarPlanner;
using navigation::GridMap;
using navigation::PlanStatus;
using navigation::Point;

namespace
{
GridMap make_map(double size_m = 10.0, double res = 0.1, int8_t fill = 0)
{
  GridMap m;
  m.resolution = res;
  m.width = m.height = static_cast<int>(std::lround(size_m / res));
  m.origin_x = m.origin_y = 0.0;
  m.data.assign(static_cast<size_t>(m.width) * m.height, fill);
  return m;
}

// Fill the axis-aligned rectangle [x0,x1] x [y0,y1] (metres) with occupancy 100.
void fill_rect(GridMap & m, double x0, double y0, double x1, double y1)
{
  for (int cy = 0; cy < m.height; ++cy) {
    for (int cx = 0; cx < m.width; ++cx) {
      const Point c = m.cell_center(cx, cy);
      if (c.x >= x0 && c.x <= x1 && c.y >= y0 && c.y <= y1) {
        m.data[m.index(cx, cy)] = 100;
      }
    }
  }
}

double min_distance_to_occupied(const GridMap & m, Point p)
{
  double best = std::numeric_limits<double>::infinity();
  for (int cy = 0; cy < m.height; ++cy) {
    for (int cx = 0; cx < m.width; ++cx) {
      if (m.data[m.index(cx, cy)] >= 65) {
        const Point c = m.cell_center(cx, cy);
        best = std::min(best, std::hypot(c.x - p.x, c.y - p.y));
      }
    }
  }
  return best;
}
}  // namespace

TEST(AStar, EmptyMapPathIsGeometricOptimum)
{
  const GridMap map = make_map();
  AStarPlanner planner;
  const auto res = planner.plan(map, {1.0, 1.0}, {9.0, 9.0});
  ASSERT_EQ(res.status, PlanStatus::OK);
  const double optimum = 8.0 * std::sqrt(2.0);
  EXPECT_NEAR(res.length, optimum, 0.05 * optimum);
  EXPECT_NEAR(res.path.front().x, 1.0, 1e-9);
  EXPECT_NEAR(res.path.back().y, 9.0, 1e-9);
}

TEST(AStar, EmptyMapUnsmoothedGridPathIsStillOptimal)
{
  const GridMap map = make_map();
  AStarParams p;
  p.smooth = false;
  AStarPlanner planner(p);
  const auto res = planner.plan(map, {1.0, 1.0}, {9.0, 9.0});
  ASSERT_EQ(res.status, PlanStatus::OK);
  const double optimum = 8.0 * std::sqrt(2.0);
  EXPECT_NEAR(res.length, optimum, 0.05 * optimum);
}

TEST(AStar, ObliqueGoalOptimumIsOctile)
{
  // (1,1) -> (9,4): 3 m of diagonal + 5 m straight is the optimal 8-connected grid path.
  const GridMap map = make_map();
  AStarParams p;
  p.smooth = false;
  AStarPlanner planner(p);
  const auto res = planner.plan(map, {1.05, 1.05}, {9.05, 4.05});
  ASSERT_EQ(res.status, PlanStatus::OK);
  EXPECT_NEAR(res.length, 3.0 * std::sqrt(2.0) + 5.0, 0.01);
}

TEST(AStar, WallWithGapRoutesThroughGap)
{
  GridMap map = make_map();
  fill_rect(map, 4.9, 0.0, 5.1, 6.95);   // wall x = 5, from y = 0 ...
  fill_rect(map, 4.9, 8.05, 5.1, 10.0);  // ... to y = 10 with a gap at y in (7.0, 8.0)
  AStarPlanner planner;
  const auto res = planner.plan(map, {1.0, 1.0}, {9.0, 1.0});
  ASSERT_EQ(res.status, PlanStatus::OK);

  // every pose where the path crosses the wall column must be inside the gap
  int crossings = 0;
  for (const auto & p : res.path) {
    if (std::fabs(p.x - 5.0) < 0.1) {
      ++crossings;
      EXPECT_GT(p.y, 7.0);
      EXPECT_LT(p.y, 8.0);
    }
  }
  EXPECT_GT(crossings, 0);
  // nothing in the path may touch an occupied cell
  for (const auto & p : res.path) {
    EXPECT_GT(min_distance_to_occupied(map, p), 0.15) << "path point (" << p.x << "," << p.y << ")";
  }
  // detour: longer than the straight line, but close to the shortest route through the gap
  EXPECT_GT(res.length, 8.0);
  EXPECT_LT(res.length, 16.0);
}

TEST(AStar, ClosedWallMeansEmptyPathAndNoCrash)
{
  GridMap map = make_map();
  fill_rect(map, 4.9, 0.0, 5.1, 10.0);  // full wall, no gap
  AStarPlanner planner;
  const auto res = planner.plan(map, {1.0, 5.0}, {9.0, 5.0});
  EXPECT_EQ(res.status, PlanStatus::NO_PATH);
  EXPECT_TRUE(res.path.empty());  // in particular: NOT a straight line
}

TEST(AStar, GoalInsideObstacleMeansEmptyPath)
{
  GridMap map = make_map();
  fill_rect(map, 6.0, 6.0, 7.0, 7.0);
  AStarPlanner planner;
  const auto res = planner.plan(map, {1.0, 1.0}, {6.5, 6.5});
  EXPECT_EQ(res.status, PlanStatus::GOAL_IN_OBSTACLE);
  EXPECT_TRUE(res.path.empty());
}

TEST(AStar, OutOfMapAndInvalidMapAreReportedNotCrashed)
{
  AStarPlanner planner;
  const GridMap map = make_map();
  EXPECT_EQ(planner.plan(map, {-5.0, 1.0}, {9.0, 9.0}).status, PlanStatus::START_OUT_OF_MAP);
  EXPECT_EQ(planner.plan(map, {1.0, 1.0}, {50.0, 9.0}).status, PlanStatus::GOAL_OUT_OF_MAP);
  EXPECT_EQ(planner.plan(GridMap(), {1.0, 1.0}, {2.0, 2.0}).status, PlanStatus::INVALID_MAP);
}

TEST(AStar, PathKeepsInflationRadiusFromObstacles)
{
  GridMap map = make_map();
  fill_rect(map, 4.0, 3.0, 6.0, 7.0);  // a block between start and goal
  AStarParams p;
  p.inflation_radius = 0.30;
  AStarPlanner planner(p);
  const auto res = planner.plan(map, {1.0, 5.0}, {9.0, 5.0});
  ASSERT_EQ(res.status, PlanStatus::OK);
  for (const auto & q : res.path) {
    // 0.5 * resolution of slack for the cell-centre discretisation
    EXPECT_GE(min_distance_to_occupied(map, q), p.inflation_radius - 0.05);
  }
}

TEST(AStar, RobotInsideInflationBandCanStillLeave)
{
  GridMap map = make_map();
  fill_rect(map, 4.9, 0.0, 5.1, 10.0);
  AStarPlanner planner;
  // standing 0.2 m from the wall, i.e. inside the 0.30 m inflation band; goal on the same side
  const auto res = planner.plan(map, {4.7, 5.0}, {1.0, 5.0});
  ASSERT_EQ(res.status, PlanStatus::OK);
  EXPECT_NEAR(res.length, 3.7, 0.2);
}

TEST(AStar, UnknownCellsFollowTheUnknownIsFreeSwitch)
{
  const GridMap unknown = make_map(10.0, 0.1, -1);
  AStarParams p;
  p.unknown_is_free = true;
  EXPECT_EQ(AStarPlanner(p).plan(unknown, {1.0, 1.0}, {9.0, 9.0}).status, PlanStatus::OK);
  p.unknown_is_free = false;
  const auto res = AStarPlanner(p).plan(unknown, {1.0, 1.0}, {9.0, 9.0});
  EXPECT_NE(res.status, PlanStatus::OK);
  EXPECT_TRUE(res.path.empty());
}

TEST(AStar, OutputSpacingIsRespected)
{
  const GridMap map = make_map();
  AStarParams p;
  p.output_spacing = 0.1;
  const auto res = AStarPlanner(p).plan(map, {1.0, 1.0}, {9.0, 9.0});
  ASSERT_EQ(res.status, PlanStatus::OK);
  for (size_t i = 1; i < res.path.size(); ++i) {
    EXPECT_LE(std::hypot(res.path[i].x - res.path[i - 1].x, res.path[i].y - res.path[i - 1].y),
      0.1 + 1e-9);
  }
}
