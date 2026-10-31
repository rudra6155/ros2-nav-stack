// Pure pursuit unit tests: ideal-kinematics closed loop (the node-level test with a lagged
// plant lives in control/test).
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "navigation/pure_pursuit.hpp"

using navigation::Point;
using navigation::Pose2D;
using navigation::PurePursuit;
using navigation::PurePursuitParams;

namespace
{
std::vector<Point> straight_path(double x0, double x1, double y = 0.0, double step = 0.1)
{
  std::vector<Point> p;
  for (double x = x0; x <= x1 + 1e-9; x += step) {
    p.push_back({x, y});
  }
  return p;
}

// Unicycle integration with the commanded velocities applied instantly.
void step(Pose2D & pose, double v, double w, double dt)
{
  pose.x += v * std::cos(pose.yaw) * dt;
  pose.y += v * std::sin(pose.yaw) * dt;
  pose.yaw += w * dt;
}
}  // namespace

TEST(PurePursuit, OffAxisStartConvergesBelow10cmWithin5s)
{
  PurePursuit pp;
  const auto path = straight_path(0.0, 12.0);
  pp.set_path(path);
  Pose2D pose{0.0, 0.5, 0.0};
  const double dt = 0.02;
  double err_at_5s = 1e9;
  for (double t = 0.0; t < 5.0; t += dt) {
    const auto cmd = pp.compute(pose);
    step(pose, cmd.linear, cmd.angular, dt);
    err_at_5s = PurePursuit::cross_track_error(path, pose.x, pose.y);
  }
  EXPECT_LT(err_at_5s, 0.1) << "cross-track error after 5 s: " << err_at_5s;
}

TEST(PurePursuit, VelocityLimitsAreNeverExceeded)
{
  PurePursuitParams prm;
  prm.max_linear_velocity = 0.4;
  prm.max_angular_velocity = 0.9;
  PurePursuit pp(prm);
  pp.set_path(straight_path(0.0, 8.0));
  for (double y = -2.0; y <= 2.0; y += 0.25) {
    for (double yaw = -3.14; yaw <= 3.14; yaw += 0.4) {
      pp.set_path(straight_path(0.0, 8.0));
      const auto cmd = pp.compute(Pose2D{1.0, y, yaw});
      EXPECT_LE(std::fabs(cmd.linear), prm.max_linear_velocity + 1e-9);
      EXPECT_LE(std::fabs(cmd.angular), prm.max_angular_velocity + 1e-9);
      EXPECT_GE(cmd.linear, 0.0);  // never reverses
    }
  }
}

TEST(PurePursuit, ReachesTheGoalAndStops)
{
  PurePursuit pp;
  pp.set_path(straight_path(0.0, 5.0));
  Pose2D pose{0.0, 0.0, 0.0};
  bool reached = false;
  for (int i = 0; i < 2000 && !reached; ++i) {
    const auto cmd = pp.compute(pose);
    reached = cmd.goal_reached;
    step(pose, cmd.linear, cmd.angular, 0.02);
  }
  ASSERT_TRUE(reached);
  const auto cmd = pp.compute(pose);
  EXPECT_TRUE(cmd.goal_reached);
  EXPECT_DOUBLE_EQ(cmd.linear, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular, 0.0);
  EXPECT_LT(std::hypot(pose.x - 5.0, pose.y), 0.15);
}

TEST(PurePursuit, TurnsOnTheSpotWhenFacingAwayFromThePath)
{
  PurePursuit pp;
  pp.set_path(straight_path(0.0, 8.0));
  const auto cmd = pp.compute(Pose2D{1.0, 0.0, M_PI});  // heading back along the path
  EXPECT_DOUBLE_EQ(cmd.linear, 0.0);
  EXPECT_GT(std::fabs(cmd.angular), 0.0);
}

TEST(PurePursuit, EmptyPathGivesZeroCommand)
{
  PurePursuit pp;
  const auto cmd = pp.compute(Pose2D{1.0, 2.0, 0.3});
  EXPECT_DOUBLE_EQ(cmd.linear, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular, 0.0);
  EXPECT_FALSE(pp.has_path());
}

TEST(PurePursuit, CrossTrackErrorMatchesGeometry)
{
  const auto path = straight_path(0.0, 10.0);
  EXPECT_NEAR(PurePursuit::cross_track_error(path, 5.0, 0.5), 0.5, 1e-9);
  EXPECT_NEAR(PurePursuit::cross_track_error(path, 5.0, -0.25), 0.25, 1e-9);
  EXPECT_NEAR(PurePursuit::cross_track_error(path, 12.0, 0.0), 2.0, 1e-9);  // beyond the end
}

TEST(PurePursuit, LargerLookaheadSteersMoreGently)
{
  const auto path = straight_path(0.0, 12.0);
  auto steer = [&](double lookahead) {
      PurePursuitParams prm;
      prm.lookahead_distance = lookahead;
      PurePursuit pp(prm);
      pp.set_path(path);
      return std::fabs(pp.compute(Pose2D{1.0, 0.4, 0.0}).angular);
    };
  EXPECT_GT(steer(0.4), steer(0.8));
  EXPECT_GT(steer(0.8), steer(1.6));
}

TEST(PurePursuit, TracksACircularArcWithinTolerance)
{
  std::vector<Point> arc;
  const double R = 2.0;
  for (double a = 0.0; a <= M_PI; a += 0.02) {
    arc.push_back({R * std::sin(a), R * (1.0 - std::cos(a))});
  }
  PurePursuit pp;
  pp.set_path(arc);
  Pose2D pose{0.0, 0.0, 0.0};
  double worst = 0.0;
  for (int i = 0; i < 4000; ++i) {
    const auto cmd = pp.compute(pose);
    if (cmd.goal_reached) {break;}
    step(pose, cmd.linear, cmd.angular, 0.02);
    worst = std::max(worst, PurePursuit::cross_track_error(arc, pose.x, pose.y));
  }
  // pure pursuit cuts corners on curves; the error must stay bounded
  EXPECT_LT(worst, 0.2);
}
