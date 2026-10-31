// Perception unit tests (no ROS graph): synthetic LaserScans with known geometry.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "perception/scan_processing.hpp"

using perception::Cluster;
using perception::DownsampledScan;
using perception::OccupancyMapper;

namespace
{
constexpr double kRangeMin = 0.12;
constexpr double kRangeMax = 6.0;
const float kInf = std::numeric_limits<float>::infinity();

// Same layout as the simulated lidar: 360 beams, 1 degree apart, first beam at -180 deg.
struct RawScan
{
  double angle_min{-M_PI};
  double angle_increment{M_PI / 180.0};
  std::vector<float> ranges = std::vector<float>(360, kInf);
};

// Infinite straight wall: the set of points {p : p . n = d}, n a unit normal at angle `normal_angle`,
// restricted to |beam angle - normal_angle| <= half_span.
RawScan wall_scan(double normal_angle, double d, double half_span)
{
  RawScan s;
  for (size_t i = 0; i < s.ranges.size(); ++i) {
    const double a = s.angle_min + i * s.angle_increment;
    const double rel = perception::wrap_to_pi(a - normal_angle);
    if (std::fabs(rel) <= half_span) {
      s.ranges[i] = static_cast<float>(d / std::cos(rel));
    }
  }
  return s;
}

DownsampledScan ds(const RawScan & s, double res_deg = 2.0)
{
  return perception::downsample_scan(
    s.ranges, s.angle_min, s.angle_increment, res_deg * M_PI / 180.0, kRangeMin, kRangeMax);
}
}  // namespace

TEST(Downsample, FixedAngularResolution)
{
  const auto d = ds(wall_scan(0.0, 1.0, 0.5));
  EXPECT_EQ(d.ranges.size(), 180u);  // 360 beams -> 2 degree bins
  EXPECT_NEAR(d.angle_increment, 2.0 * M_PI / 180.0, 1e-12);
  EXPECT_NEAR(d.angle_min, -M_PI, 1e-12);
}

TEST(Downsample, KeepsClosestHitInBin)
{
  RawScan s;
  s.ranges[180] = 3.0f;  // beam at angle 0
  s.ranges[181] = 2.0f;  // beam at +1 deg -> falls into the same 2-degree bin as the previous one or its neighbour
  s.ranges[179] = 4.0f;
  const auto d = ds(s);
  float min_r = kInf;
  for (float r : d.ranges) {
    if (std::isfinite(r)) {min_r = std::min(min_r, r);}
  }
  EXPECT_FLOAT_EQ(min_r, 2.0f);  // the closest return survives pooling
}

TEST(ObstacleDetector, WallAtOneMeterCentroidWithin10cm)
{
  // Wall 1.0 m straight ahead (x = 1.0), +-30 degrees wide.
  const auto d = ds(wall_scan(0.0, 1.0, 30.0 * M_PI / 180.0));
  const auto clusters = perception::cluster_scan(d, kRangeMin, kRangeMax, 0.15, 3);
  ASSERT_EQ(clusters.size(), 1u);
  const auto & c = clusters[0].centroid;
  EXPECT_NEAR(c.x, 1.0, 0.1);
  EXPECT_NEAR(c.y, 0.0, 0.1);
  const double err = std::hypot(c.x - 1.0, c.y - 0.0);
  EXPECT_LT(err, 0.1) << "centroid error " << err << " m";
}

TEST(ObstacleDetector, WallToTheLeftCentroid)
{
  // Wall 1.0 m to the left (y = 1.0).
  const auto d = ds(wall_scan(M_PI / 2.0, 1.0, 30.0 * M_PI / 180.0));
  const auto clusters = perception::cluster_scan(d, kRangeMin, kRangeMax, 0.15, 3);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_NEAR(clusters[0].centroid.x, 0.0, 0.1);
  EXPECT_NEAR(clusters[0].centroid.y, 1.0, 0.1);
}

TEST(ObstacleDetector, WallAcrossScanSeamIsOneCluster)
{
  // Wall behind the robot: its beams straddle the +-180 degree seam of the scan array.
  const auto d = ds(wall_scan(M_PI, 1.0, 30.0 * M_PI / 180.0));
  const auto clusters = perception::cluster_scan(d, kRangeMin, kRangeMax, 0.15, 3);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_NEAR(clusters[0].centroid.x, -1.0, 0.1);
  EXPECT_NEAR(clusters[0].centroid.y, 0.0, 0.1);
}

TEST(ObstacleDetector, TwoSeparateObstaclesAreTwoClusters)
{
  RawScan a = wall_scan(0.0, 1.0, 15.0 * M_PI / 180.0);
  const RawScan b = wall_scan(M_PI / 2.0, 2.0, 10.0 * M_PI / 180.0);
  for (size_t i = 0; i < a.ranges.size(); ++i) {
    if (std::isfinite(b.ranges[i])) {a.ranges[i] = b.ranges[i];}
  }
  const auto clusters =
    perception::cluster_scan(ds(a), kRangeMin, kRangeMax, 0.15, 3);
  ASSERT_EQ(clusters.size(), 2u);
}

TEST(ObstacleDetector, NoReturnsMeansNoObstacles)
{
  const RawScan s;  // all inf
  EXPECT_TRUE(perception::cluster_scan(ds(s), kRangeMin, kRangeMax, 0.15, 3).empty());
}

TEST(OccupancyMapper, WallBecomesOccupiedWithFreeSpaceInFront)
{
  OccupancyMapper::Params p;  // 14 x 14 m, 0.1 m cells, origin (-7,-7)
  OccupancyMapper m(p);
  const auto d = ds(wall_scan(0.0, 1.0, 30.0 * M_PI / 180.0));
  for (int i = 0; i < 3; ++i) {
    m.integrate(0.0, 0.0, 0.0, d, kRangeMin, kRangeMax);
  }
  const auto g = m.occupancy();
  auto at = [&](double x, double y) {
      int cx, cy;
      EXPECT_TRUE(m.world_to_cell(x, y, cx, cy));
      return g[static_cast<size_t>(cy) * p.width + cx];
    };
  EXPECT_EQ(at(1.02, 0.0), 100);   // the wall
  EXPECT_EQ(at(0.5, 0.0), 0);      // free space between robot and wall
  EXPECT_EQ(at(1.5, 0.0), -1);     // unobserved behind the wall
  EXPECT_EQ(at(-3.0, 3.0), -1);    // never seen
}

TEST(OccupancyMapper, SensorYawRotatesTheMap)
{
  OccupancyMapper::Params p;
  OccupancyMapper m(p);
  const auto d = ds(wall_scan(0.0, 1.0, 30.0 * M_PI / 180.0));
  m.integrate(0.0, 0.0, M_PI / 2.0, d, kRangeMin, kRangeMax);  // robot facing +y: the wall is at y = 1
  const auto g = m.occupancy();
  int cx, cy;
  ASSERT_TRUE(m.world_to_cell(0.0, 1.02, cx, cy));
  EXPECT_EQ(g[static_cast<size_t>(cy) * p.width + cx], 100);
}

TEST(OccupancyMapper, ResetForgetsEverything)
{
  OccupancyMapper::Params p;
  OccupancyMapper m(p);
  m.integrate(0.0, 0.0, 0.0, ds(wall_scan(0.0, 1.0, 0.5)), kRangeMin, kRangeMax);
  m.reset();
  for (int8_t v : m.occupancy()) {
    ASSERT_EQ(v, -1);
  }
}
