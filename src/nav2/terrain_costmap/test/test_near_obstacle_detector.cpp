#include "terrain_costmap/near_obstacle_detector.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{

using terrain_costmap::NearObstacleDetector;
using terrain_costmap::NearObstacleParameters;
using terrain_costmap::Point;
using terrain_costmap::TerrainAnalyzer;

std::vector<Point> sparseWall(double x = 0.65, double y = 0.05)
{
  // Two XY columns are insufficient for a 2D ground fit. Each contributes
  // actual lower returns and two upper returns, as after 0.1m input thinning.
  return {
    {x, y, 0.01, 100}, {x + 0.10, y, 0.02, 101},
    {x, y, 0.09, 102}, {x + 0.10, y, 0.10, 103},
    {x, y, 0.21, 104}, {x + 0.10, y, 0.22, 105},
    {x, y, 0.31, 106}, {x + 0.10, y, 0.32, 107}};
}

TEST(NearObstacleDetector, CollinearWallSurvivesAbsentGroundFit)
{
  const auto points = sparseWall();
  const auto terrain = TerrainAnalyzer({}).analyze(points, 0, 0);
  EXPECT_TRUE(terrain.support_surface.empty());
  EXPECT_TRUE(terrain.measured_obstacles.empty());

  const auto obstacles = NearObstacleDetector({}).detect(points);
  ASSERT_EQ(obstacles.size(), 4u);
  for (const auto & obstacle : obstacles) {
    EXPECT_GE(obstacle.point.z, 0.21);
    EXPECT_DOUBLE_EQ(obstacle.relative_height, obstacle.point.z - 0.02);
    EXPECT_FALSE(obstacle.synthetic);
  }
}

TEST(NearObstacleDetector, FlatGroundAndTenDegreeSlopesDoNotBecomeObstacles)
{
  for (double slope : {0.0, 0.17632698070846498}) {
    for (double direction : {0.0, 0.7, 1.57}) {
      std::vector<Point> points;
      for (int ix = 21; ix <= 55; ++ix) {
        for (int iy = -25; iy <= 25; ++iy) {
          const double x = ix * 0.02, y = iy * 0.02;
          points.push_back({x, y, -0.20 + slope *
            (x * std::cos(direction) + y * std::sin(direction)), 10});
        }
      }
      EXPECT_TRUE(NearObstacleDetector({}).detect(points).empty());
    }
  }
}

TEST(NearObstacleDetector, SingleNegativeOutlierCannotRaiseAFlatOrSlopingSurface)
{
  for (double slope : {0.0, 0.17632698070846498}) {
    std::vector<Point> points;
    for (int ix = 22; ix <= 55; ++ix) {
      for (int iy = -15; iy <= 15; ++iy) {
        const double x = ix * 0.02, y = iy * 0.02;
        points.push_back({x, y, slope * (x - 0.75), 20});
      }
    }
    points.push_back({0.75, 0.01, -0.30, 21});
    EXPECT_TRUE(NearObstacleDetector({}).detect(points).empty());
  }
}

TEST(NearObstacleDetector, RepeatedLowOutlierDoesNotCreateIndependentLowerLayer)
{
  std::vector<Point> points;
  for (int frame = 0; frame < 100; ++frame) {
    // All negative observations occupy one voxel even with small jitter.
    points.push_back({0.711 + frame * 0.0001, 0.011, -0.299, frame + 100});
    for (int ix = 0; ix < 4; ++ix) {
      for (int iy = 0; iy < 4; ++iy) {
        points.push_back({0.65 + ix * 0.05, -0.05 + iy * 0.05, 0.0, frame + 100});
      }
    }
  }
  EXPECT_TRUE(NearObstacleDetector({}).detect(points).empty());
}

TEST(NearObstacleDetector, DuplicatesCannotSupplyCandidateCluster)
{
  std::vector<Point> points{{0.65, 0.01, 0.01}, {0.75, 0.01, 0.02}};
  for (int i = 0; i < 200; ++i) {points.push_back({0.70, 0.01, 0.30, i});}
  EXPECT_TRUE(NearObstacleDetector({}).detect(points).empty());
}

TEST(NearObstacleDetector, SingleUpperOutlierAndSeparateUpperOutliersAreRejected)
{
  std::vector<Point> points;
  for (double y : {-0.50, 0.0, 0.50}) {
    points.push_back({0.65, y, 0.01});
    points.push_back({0.75, y, 0.02});
    points.push_back({0.70, y, 0.30});
    EXPECT_TRUE(NearObstacleDetector({}).detect(points).empty());
  }
}

TEST(NearObstacleDetector, BodyCornersAndFootprintBoundaryAreExcluded)
{
  std::vector<Point> points;
  for (double sign_x : {-1.0, 1.0}) {
    for (double sign_y : {-1.0, 1.0}) {
      for (double x : {0.33, 0.39, 0.40}) {
        for (double z : {0.01, 0.02, 0.21, 0.31}) {
          points.push_back({sign_x * x, sign_y * 0.25, z, 30});
        }
      }
    }
  }
  EXPECT_TRUE(NearObstacleDetector({}).detect(points).empty());
}

TEST(NearObstacleDetector, WorkingHeightRangeAppliesToLowerAndUpperEvidence)
{
  std::vector<Point> high, low;
  for (double x : {0.65, 0.70, 0.75}) {
    high.push_back({x, 0.01, 0.01});
    high.push_back({x, 0.01, 0.02});
    high.push_back({x, 0.01, 0.41});
    high.push_back({x, 0.01, 0.51});
    low.push_back({x, 0.01, -0.36});
    low.push_back({x, 0.01, -0.46});
    low.push_back({x, 0.01, 0.01});
  }
  EXPECT_TRUE(NearObstacleDetector({}).detect(high).empty());
  EXPECT_TRUE(NearObstacleDetector({}).detect(low).empty());
}

TEST(NearObstacleDetector, NearFieldRangeAndConfiguredBodyBoundsAreRespected)
{
  EXPECT_TRUE(NearObstacleDetector({}).detect(sparseWall(1.25)).empty());
  EXPECT_TRUE(NearObstacleDetector({}).detect(sparseWall(0.15)).empty());
  NearObstacleParameters parameters;
  parameters.self_maximum_x = 0.80;
  EXPECT_TRUE(NearObstacleDetector(parameters).detect(sparseWall()).empty());
  parameters = {};
  parameters.maximum_relative_z = 0.20;
  EXPECT_TRUE(NearObstacleDetector(parameters).detect(sparseWall()).empty());
}

TEST(NearObstacleDetector, RetainsActualCoordinatesAndNewestOriginalTimestamp)
{
  auto points = sparseWall();
  const Point newer{points[4].x + 0.001, points[4].y + 0.001, points[4].z + 0.001, 1000};
  points.push_back(newer);
  const auto obstacles = NearObstacleDetector({}).detect(points);
  ASSERT_EQ(obstacles.size(), 4u);
  bool retained_newer = false;
  for (const auto & obstacle : obstacles) {
    const auto original = std::find_if(points.begin(), points.end(), [&](const Point & p) {
        return p.x == obstacle.point.x && p.y == obstacle.point.y && p.z == obstacle.point.z &&
               p.observation_stamp_ns == obstacle.point.observation_stamp_ns;
      });
    EXPECT_NE(original, points.end());
    retained_newer |= obstacle.point.observation_stamp_ns == newer.observation_stamp_ns;
    EXPECT_FALSE(obstacle.synthetic);
  }
  EXPECT_TRUE(retained_newer);
}

TEST(NearObstacleDetector, DisabledDetectorAndNonFiniteReturnsProduceNoObstacles)
{
  NearObstacleParameters parameters;
  parameters.enabled = false;
  EXPECT_TRUE(NearObstacleDetector(parameters).detect(sparseWall()).empty());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(NearObstacleDetector({}).detect({{nan, 0, 0}, {0.7, nan, 0}, {0.7, 0, nan}}).empty());
}

TEST(NearObstacleDetector, InvalidGeometryParametersAreRejected)
{
  NearObstacleParameters parameters;
  parameters.voxel_size = 0.0;
  EXPECT_THROW(NearObstacleDetector{parameters}, std::invalid_argument);
  parameters = {};
  parameters.minimum_cluster_points = 2;
  EXPECT_THROW(NearObstacleDetector{parameters}, std::invalid_argument);
  parameters = {};
  parameters.maximum_range = std::numeric_limits<double>::infinity();
  EXPECT_THROW(NearObstacleDetector{parameters}, std::invalid_argument);
}

}  // namespace
