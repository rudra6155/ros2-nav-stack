// Seeded, reproducible sampling of episode start/goal poses.
//
// The seed alone determines the result: the generator is SplitMix64 (a fixed, published
// algorithm - not std::uniform_real_distribution, whose output is implementation-defined), and
// the order in which random numbers are drawn is fixed. So `sample(seed)` returns bit-identical
// poses on any machine, and re-running episode k with its recorded seed spawns the same pose.
//
// Accepted episodes are feasible by construction: start and goal keep `min_clearance` from every
// obstacle surface, are at least `min_start_goal_distance` apart and are connected on the
// ground-truth grid (checked with the same A* and inflation the planner uses).
#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "evaluation/world_geometry.hpp"
#include "navigation/astar.hpp"

namespace evaluation
{

struct SamplerConfig
{
  double min_clearance{0.5};            // m from any obstacle surface (walls included)
  double min_start_goal_distance{5.0};  // m, straight line
  int max_attempts{20000};
  double grid_resolution{0.1};
  double grid_origin{-7.0};
  int grid_cells{140};
  navigation::AStarParams astar{};      // inflation used for the reachability check
};

struct EpisodePoses
{
  double start_x{0}, start_y{0}, start_yaw{0};
  double goal_x{0}, goal_y{0};
  int attempts{0};
};

class SplitMix64
{
public:
  explicit SplitMix64(uint64_t seed)
  : state_(seed) {}
  uint64_t next()
  {
    uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }
  /// uniform in [0, 1) with 53 random bits
  double uniform() {return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0);}
  double uniform(double lo, double hi) {return lo + (hi - lo) * uniform();}

private:
  uint64_t state_;
};

class EpisodeSampler
{
public:
  EpisodeSampler(const WorldGeometry & world, const SamplerConfig & cfg = SamplerConfig())
  : world_(world), cfg_(cfg), planner_(cfg.astar),
    truth_(world.rasterize(cfg.grid_resolution, cfg.grid_origin, cfg.grid_origin,
        cfg.grid_cells, cfg.grid_cells))
  {
  }

  EpisodePoses sample(uint64_t seed) const
  {
    SplitMix64 rng(seed);
    const double lim = world_.arena_half();
    auto free_point = [&](double & x, double & y) {
        x = rng.uniform(-lim, lim);
        y = rng.uniform(-lim, lim);
        return world_.inside_arena(x, y) && world_.distance(x, y) >= cfg_.min_clearance;
      };
    for (int attempt = 1; attempt <= cfg_.max_attempts; ++attempt) {
      EpisodePoses p;
      p.attempts = attempt;
      if (!free_point(p.start_x, p.start_y)) {continue;}
      if (!free_point(p.goal_x, p.goal_y)) {continue;}
      if (std::hypot(p.goal_x - p.start_x, p.goal_y - p.start_y) < cfg_.min_start_goal_distance) {
        continue;
      }
      const auto plan = planner_.plan(truth_, {p.start_x, p.start_y}, {p.goal_x, p.goal_y});
      if (plan.status != navigation::PlanStatus::OK) {continue;}
      p.start_yaw = rng.uniform(-M_PI, M_PI);
      return p;
    }
    throw std::runtime_error("episode sampler: no feasible start/goal found for seed " + std::to_string(seed));
  }

  const SamplerConfig & config() const {return cfg_;}

private:
  const WorldGeometry & world_;
  SamplerConfig cfg_;
  navigation::AStarPlanner planner_;
  navigation::GridMap truth_;
};

}  // namespace evaluation
