// Episode reproducibility + feasibility tests (no ROS graph needed).
#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>

#include "evaluation/episode_sampler.hpp"
#include "evaluation/world_geometry.hpp"
#include "navigation/astar.hpp"

#ifndef WORLDS_DIR
#error "WORLDS_DIR must be defined by CMake"
#endif

namespace
{
evaluation::WorldGeometry load(const std::string & name)
{
  return evaluation::WorldGeometry::load(std::string(WORLDS_DIR) + "/" + name + ".obstacles");
}
}  // namespace

TEST(EpisodeSampler, SameSeedSamePoseBitForBit)
{
  for (const std::string world : {"simple", "cluttered"}) {
    const auto geo = load(world);
    const evaluation::EpisodeSampler sampler(geo);
    // "rerunning episode 7 with the same seed spawns the same pose" (seed = 1000 + episode)
    const auto a = sampler.sample(1007);
    const auto b = sampler.sample(1007);
    EXPECT_EQ(a.start_x, b.start_x) << world;
    EXPECT_EQ(a.start_y, b.start_y) << world;
    EXPECT_EQ(a.start_yaw, b.start_yaw) << world;
    EXPECT_EQ(a.goal_x, b.goal_x) << world;
    EXPECT_EQ(a.goal_y, b.goal_y) << world;
    // a *fresh* sampler (new process, new objects) agrees too
    const evaluation::EpisodeSampler sampler2(geo);
    const auto c = sampler2.sample(1007);
    EXPECT_EQ(a.start_x, c.start_x) << world;
    EXPECT_EQ(a.goal_y, c.goal_y) << world;
  }
}

TEST(EpisodeSampler, GoldenPoseForSeed1007IsStable)
{
  // Pins SplitMix64 + the rejection order: a change to either shows up here, because recorded
  // seeds in results/episodes.csv would no longer reproduce their poses.
  const auto geo = load("simple");
  const auto p = evaluation::EpisodeSampler(geo).sample(1007);
  EXPECT_NEAR(p.start_x, GOLDEN_SIMPLE_1007_START_X, 1e-9);
  EXPECT_NEAR(p.start_y, GOLDEN_SIMPLE_1007_START_Y, 1e-9);
  EXPECT_NEAR(p.start_yaw, GOLDEN_SIMPLE_1007_START_YAW, 1e-9);
  EXPECT_NEAR(p.goal_x, GOLDEN_SIMPLE_1007_GOAL_X, 1e-9);
  EXPECT_NEAR(p.goal_y, GOLDEN_SIMPLE_1007_GOAL_Y, 1e-9);
}

TEST(EpisodeSampler, DifferentSeedsGiveDifferentPoses)
{
  const auto geo = load("cluttered");
  const evaluation::EpisodeSampler sampler(geo);
  std::set<std::string> seen;
  for (uint64_t s = 1001; s <= 1020; ++s) {
    const auto p = sampler.sample(s);
    seen.insert(std::to_string(p.start_x) + "," + std::to_string(p.goal_x));
  }
  EXPECT_EQ(seen.size(), 20u);
}

TEST(EpisodeSampler, PosesAreFeasible)
{
  for (const std::string world : {"simple", "cluttered"}) {
    const auto geo = load(world);
    const evaluation::SamplerConfig cfg;
    const evaluation::EpisodeSampler sampler(geo, cfg);
    const auto truth = geo.rasterize(cfg.grid_resolution, cfg.grid_origin, cfg.grid_origin, cfg.grid_cells, cfg.grid_cells);
    const navigation::AStarPlanner planner(cfg.astar);
    for (uint64_t s = 1001; s <= 1020; ++s) {
      const auto p = sampler.sample(s);
      EXPECT_GE(geo.distance(p.start_x, p.start_y), cfg.min_clearance) << world << " seed " << s;
      EXPECT_GE(geo.distance(p.goal_x, p.goal_y), cfg.min_clearance) << world << " seed " << s;
      EXPECT_GE(std::hypot(p.goal_x - p.start_x, p.goal_y - p.start_y), cfg.min_start_goal_distance);
      EXPECT_EQ(planner.plan(truth, {p.start_x, p.start_y}, {p.goal_x, p.goal_y}).status,
        navigation::PlanStatus::OK) << world << " seed " << s;
      EXPECT_GE(p.start_yaw, -M_PI);
      EXPECT_LE(p.start_yaw, M_PI);
    }
  }
}

TEST(WorldGeometry, DistanceAndRasterization)
{
  const auto geo = load("simple");
  EXPECT_GT(geo.obstacles().size(), 4u);  // 4 walls + obstacles
  // the point just inside the north wall's inner face (y = 6.0) is ~0 m from the surface
  EXPECT_NEAR(geo.distance(0.0, 5.99), 0.01, 1e-6);
  EXPECT_GT(geo.distance(0.0, 0.0), 0.9);  // spawn area is kept clear by the generator
  const auto grid = geo.rasterize(0.1, -7.0, -7.0, 140, 140);
  int occ = 0;
  for (auto v : grid.data) {occ += (v == 100);}
  EXPECT_GT(occ, 200);  // walls + obstacles
}

TEST(WorldGeometry, MalformedFileThrows)
{
  EXPECT_THROW(evaluation::WorldGeometry::load("/nonexistent/file.obstacles"), std::runtime_error);
}
