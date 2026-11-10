#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "astar_3d_planner/astar_3d.hpp"

namespace astar_3d_planner
{
namespace
{
OccupyMap::Config mapConfig()
{
  OccupyMap::Config config;
  config.inflate_radius = 0;
  config.support_radius = .15;
  config.support_merge_height = .15;
  return config;
}

Astar3D::Config plannerConfig()
{
  Astar3D::Config config;
  config.max_slope_deg = 10;
  config.max_step_height = .08;
  config.surface_connection_radius = .25;
  config.ground_check_depth = .6;
  config.robot_height = .75;
  config.max_search_nodes = 10000;
  return config;
}

std::vector<OccupyMap::Point> floatGrid(double phase_x, double phase_y, bool gap = false)
{
  std::vector<OccupyMap::Point> result;
  for (int x = -20; x <= 20; ++x) {
    if (gap && x >= -2 && x <= 2) {continue;}
    for (int y = -2; y <= 2; ++y) {
      result.push_back({static_cast<float>(x * .2 + phase_x),
        static_cast<float>(y * .2 + phase_y), static_cast<float>(-.5)});
    }
  }
  return result;
}
}  // namespace

TEST(SupportUpdates, FloatGridKeepsConnectedInBothDirectionsAcrossWorldGridPhases)
{
  for (const double phase_x : {0.0, .02557, .05, -.02557}) {
    for (const double phase_y : {0.0, 2.8, -2.8, .02557}) {
      SCOPED_TRACE(::testing::Message() << phase_x << ", " << phase_y);
      OccupyMap map(mapConfig());
      map.updateSupport(floatGrid(phase_x, phase_y));
      const auto lock = map.lockForReading();
      Astar3D astar(map, plannerConfig());
      VoxelKey start, goal;
      ASSERT_TRUE(astar.projectToSurface(-3.8 + phase_x, phase_y, -.5, .01, .1, start));
      ASSERT_TRUE(astar.projectToSurface(3.8 + phase_x, phase_y, -.5, .01, .1, goal));
      EXPECT_TRUE(astar.segmentTerrainTraversable(start, goal));
      EXPECT_TRUE(astar.segmentTerrainTraversable(goal, start));
      std::vector<VoxelKey> path;
      EXPECT_TRUE(astar.search(start, goal, path));
      EXPECT_TRUE(astar.search(goal, start, path));
    }
  }
}

TEST(SupportUpdates, RealMissingBandRemainsDisconnectedAfterBoundaryNormalization)
{
  for (const double phase_x : {0.0, .02557, -.02557}) {
    OccupyMap map(mapConfig());
    map.updateSupport(floatGrid(phase_x, 0, true));
    const auto lock = map.lockForReading();
    Astar3D astar(map, plannerConfig());
    VoxelKey start, goal;
    ASSERT_TRUE(astar.projectToSurface(-3.8 + phase_x, 0, -.5, .01, .1, start));
    ASSERT_TRUE(astar.projectToSurface(3.8 + phase_x, 0, -.5, .01, .1, goal));
    EXPECT_FALSE(astar.segmentTerrainTraversable(start, goal));
    std::vector<VoxelKey> path;
    EXPECT_FALSE(astar.search(start, goal, path));
    EXPECT_FALSE(astar.search(goal, start, path));
  }
}

TEST(SupportUpdates, BoundaryCorrectionIsSupportOnlyAndDoesNotRoundArbitraryCoordinates)
{
  OccupyMap map(mapConfig());
  const double x = static_cast<float>(2.8);
  const double y = static_cast<float>(-2.8);
  map.updateSupport({{x, y, 0}, {2.799, -2.801, 0}});
  map.updateObstacles({{x, y, .5}}, 0, 0);
  const auto lock = map.lockForReading();
  double height;
  EXPECT_TRUE(map.supportHeightUnlocked({28, -28, 0}, height));
  EXPECT_TRUE(map.supportHeightUnlocked({27, -29, 0}, height));
  VoxelKey support;
  ASSERT_TRUE(map.findSupportByHeightUnlocked(x, y, 0, .01, support));
  EXPECT_EQ(support.x, 28);
  EXPECT_EQ(support.y, -28);
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(x, y, .5)));
  EXPECT_FALSE(map.occupiedUnlocked({28, -28, 5}));
}

TEST(SupportUpdates, ContinuousMergeThresholdMergesNearbyHeightsAcrossTwoZBins)
{
  OccupyMap map(mapConfig());
  map.updateSupport({{.01, .01, .099}});
  map.updateSupport({{.01, .01, .201}});
  const auto lock = map.lockForReading();
  EXPECT_EQ(map.rawVoxelCountUnlocked(), 1u);
  VoxelKey layer;
  ASSERT_TRUE(map.findSupportByHeightUnlocked(.01, .01, .201, .001, layer));
  EXPECT_EQ(layer.z, 2);
  double height;
  EXPECT_FALSE(map.supportHeightUnlocked({0, 0, 0}, height));
  ASSERT_TRUE(map.supportHeightUnlocked(layer, height));
  EXPECT_DOUBLE_EQ(height, .201);
}

TEST(SupportUpdates, ContinuousMergeThresholdKeepsFarHeightsInAdjacentZBinsSeparate)
{
  for (const double shift : {0.0, -1.0, 1.0}) {
    OccupyMap map(mapConfig());
    map.updateSupport({{.01, .01, .001 + shift}});
    map.updateSupport({{.01, .01, .199 + shift}});
    const auto lock = map.lockForReading();
    EXPECT_EQ(map.rawVoxelCountUnlocked(), 2u);
    VoxelKey low, high;
    EXPECT_TRUE(map.findSupportByHeightUnlocked(.01, .01, .001 + shift, .001, low));
    EXPECT_TRUE(map.findSupportByHeightUnlocked(.01, .01, .199 + shift, .001, high));
    EXPECT_NE(low.z, high.z);
  }
}

TEST(SupportUpdates, ClosestObservedLayerWinsRatherThanClosestOldZIndex)
{
  OccupyMap map(mapConfig());
  map.updateSupport({{.01, .01, .099}, {.01, .01, .251}});
  map.updateSupport({{.01, .01, .18}});
  const auto lock = map.lockForReading();
  EXPECT_EQ(map.rawVoxelCountUnlocked(), 2u);
  VoxelKey layer;
  double height;
  ASSERT_TRUE(map.findSupportByHeightUnlocked(.01, .01, .099, .001, layer));
  ASSERT_TRUE(map.supportHeightUnlocked(layer, height));
  EXPECT_DOUBLE_EQ(height, .099);
  ASSERT_TRUE(map.findSupportByHeightUnlocked(.01, .01, .18, .001, layer));
  EXPECT_EQ(layer.z, 1);
}

TEST(SupportUpdates, RepeatedCorrectionsRekeyWithoutLosingProjectionOrOtherLayers)
{
  OccupyMap map(mapConfig());
  map.updateSupport({{.01, .01, .01}, {.01, .01, 3.01}, {1.01, .01, -.49}});
  map.updateObstacles({{1.01, .01, .51}}, 0, 0);
  for (int i = 1; i <= 30; ++i) {
    const double height = .01 + .04 * i;
    map.updateSupport({{.01, .01, height}});
    const auto lock = map.lockForReading();
    VoxelKey layer;
    ASSERT_TRUE(map.findSupportByHeightUnlocked(.01, .01, height, .001, layer));
    EXPECT_EQ(layer.z, map.posToKey(.01, .01, height).z);
    Astar3D astar(map, plannerConfig());
    VoxelKey projection;
    EXPECT_TRUE(astar.projectToSurface(.01, .01, height, .01, .01, projection));
    EXPECT_EQ(map.rawVoxelCountUnlocked(), 4u);
  }
  const auto lock = map.lockForReading();
  VoxelKey layer;
  EXPECT_TRUE(map.findSupportByHeightUnlocked(.01, .01, 3.01, .001, layer));
  EXPECT_TRUE(map.findSupportByHeightUnlocked(1.01, .01, -.49, .001, layer));
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(1.01, .01, .51)));
}

TEST(SupportUpdates, FloatBoundaryOverlapContributesToCommonHeightDatum)
{
  auto config = mapConfig();
  config.compensate_height_drift = true;
  config.height_drift_min_overlap = 30;
  config.height_drift_min_xy_span = .6;
  OccupyMap map(config);
  auto points = floatGrid(0, 2.8);
  map.updateSupport(points);
  for (auto & point : points) {point[2] += .01;}
  map.updateSupport(points);
  const auto lock = map.lockForReading();
  EXPECT_NEAR(map.heightDatumOffsetUnlocked(), .01, 1e-9);
  EXPECT_EQ(map.supportSnapshotUnlocked().size(), points.size());
}

TEST(SupportUpdates, RealStepAndDifferentStoreysRemainDistinctDuringCorrections)
{
  OccupyMap map(mapConfig());
  auto points = floatGrid(0, 0);
  for (auto & point : points) {if (point[0] > 0) {point[2] += .16;}}
  map.updateSupport(points);
  const auto lock = map.lockForReading();
  Astar3D astar(map, plannerConfig());
  VoxelKey start, goal;
  ASSERT_TRUE(astar.projectToSurface(-3.8, 0, -.5, .01, .1, start));
  ASSERT_TRUE(astar.projectToSurface(3.8, 0, -.34, .01, .1, goal));
  EXPECT_FALSE(astar.segmentTerrainTraversable(start, goal));
  std::vector<VoxelKey> path;
  EXPECT_FALSE(astar.search(start, goal, path));
}
}  // namespace astar_3d_planner
