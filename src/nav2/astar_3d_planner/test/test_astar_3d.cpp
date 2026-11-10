#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "astar_3d_planner/astar_3d.hpp"
#include "astar_3d_planner/astar_3d_planner.hpp"
#include "astar_3d_planner/occupy_map.hpp"

namespace astar_3d_planner
{

namespace
{

OccupyMap::Config testMapConfig()
{
  OccupyMap::Config config;
  config.resolution = 0.1;
  config.inflate_radius = 0.3;
  config.cost_inflate_cells = 0;
  config.support_radius = 0.11;
  config.obstacle_update_radius = 5.0;
  config.obstacle_clear_after_updates = 3;
  return config;
}

Astar3D::Config testPlannerConfig()
{
  Astar3D::Config config;
  config.max_slope_deg = 50.0;
  config.ground_check_depth = 0.4;
  config.robot_height = 0.4;
  config.planning_height_offset = 0.2;
  config.ground_gap_tolerance = 0.01;
  config.start_projection_xy_radius = 0.2;
  config.goal_projection_xy_radius = 0.2;
  config.projection_z_radius = 0.5;
  config.max_search_nodes = 10000;
  return config;
}

OccupyMap::Config terrainPipelineMapConfig()
{
  auto config = testMapConfig();
  config.inflate_radius = 0.48;
  config.cost_inflate_cells = 2;
  config.support_radius = 0.15;
  return config;
}

Astar3D::Config terrainPipelinePlannerConfig()
{
  auto config = testPlannerConfig();
  config.max_slope_deg = 10.0;
  config.max_step_height = 0.08;
  config.surface_connection_radius = 0.25;
  config.ground_check_depth = 0.6;
  config.robot_height = 0.75;
  config.ground_gap_tolerance = 0.12;
  return config;
}

std::vector<OccupyMap::Point> terrainPipelineGrid(double slope)
{
  std::vector<OccupyMap::Point> points;
  for (int ix = 0; ix <= 20; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      // Match the real terrain node's 20 cm support grid and base-origin
      // offset, while keeping samples away from ambiguous voxel boundaries.
      const double x = 0.02557 + ix * 0.2;
      points.push_back({x, 0.01 + iy * 0.2, slope * x});
    }
  }
  return points;
}

}  // namespace

TEST(Astar3D, TraversesRampAndRetainsHeight)
{
  OccupyMap map(testMapConfig());
  std::vector<OccupyMap::Point> terrain;
  for (int x = 0; x <= 12; ++x) {
    const int z = x / 2;
    for (int y = -1; y <= 1; ++y) {
      terrain.push_back(
        OccupyMap::Point{0.1 * x + 0.01, 0.1 * y + 0.01, 0.1 * z + 0.01});
    }
  }
  map.updateSupport(terrain);

  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  VoxelKey start;
  VoxelKey goal;
  ASSERT_TRUE(astar.projectToSurface(0.01, 0.01, 0.01, 0.2, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(1.21, 0.01, 0.61, 0.2, 0.5, goal));

  std::vector<VoxelKey> path;
  ASSERT_TRUE(astar.search(start, goal, path));
  ASSERT_FALSE(path.empty());
  EXPECT_GT(path.back().z, path.front().z);
}

TEST(Astar3D, SixDegreeRampUsesObservedTerrainGridSpacingInBothDirections)
{
  OccupyMap map(terrainPipelineMapConfig());
  const double slope = std::tan(6.0 * 3.14159265358979323846 / 180.0);
  map.updateSupport(terrainPipelineGrid(slope));
  const auto lock = map.lockForReading();
  Astar3D astar(map, terrainPipelinePlannerConfig());
  VoxelKey start, goal;
  ASSERT_TRUE(astar.projectToSurface(0.02557, 0.01, slope * 0.02557, 0.01, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(4.02557, 0.01, slope * 4.02557, 0.01, 0.5, goal));

  // Each edge crosses a 10 cm column without its own support observation.
  // Reusing the adjacent patch must not turn a 6 degree ramp into 12 degrees.
  EXPECT_TRUE(astar.segmentTraversable(start, goal));
  EXPECT_TRUE(astar.segmentTraversable(goal, start));
  EXPECT_TRUE(astar.segmentTerrainTraversable(start, goal));
  std::vector<VoxelKey> path;
  ASSERT_TRUE(astar.search(start, goal, path));
  ASSERT_TRUE(astar.search(goal, start, path));

  std::vector<OccupyMap::Point> continuous;
  for (const auto & key : path) {
    VoxelKey ground;
    double height;
    ASSERT_TRUE(map.findSurfaceGroundUnlocked(
        key, astar.preferredGroundGapCells(), astar.groundGapToleranceCells(), ground));
    ASSERT_TRUE(map.supportHeightUnlocked(ground, height));
    const auto position = map.keyToPos(key);
    continuous.push_back({position[0], position[1], height});
  }
  auto dense = astar.densifyPath(continuous, 0.05);
  ASSERT_TRUE(astar.refreshAndValidateTerrainPath(goal, dense));
  EXPECT_GT(dense.front()[2] - dense.back()[2], 0.4);
}

TEST(Astar3D, TerrainGridSpacingStillRejectsSlopeJustAboveTenDegrees)
{
  OccupyMap map(terrainPipelineMapConfig());
  // 3.6 cm rise per observed 20 cm patch is just above the 10 degree limit.
  constexpr double slope = 0.036 / 0.2;
  map.updateSupport(terrainPipelineGrid(slope));
  const auto lock = map.lockForReading();
  Astar3D astar(map, terrainPipelinePlannerConfig());
  VoxelKey start, goal;
  ASSERT_TRUE(astar.projectToSurface(0.02557, 0.01, slope * 0.02557, 0.01, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(4.02557, 0.01, slope * 4.02557, 0.01, 0.5, goal));
  EXPECT_FALSE(astar.segmentTraversable(start, goal));
  EXPECT_FALSE(astar.segmentTerrainTraversable(start, goal));
  std::vector<VoxelKey> path;
  EXPECT_FALSE(astar.search(start, goal, path));
  EXPECT_FALSE(astar.search(goal, start, path));
}

TEST(Astar3D, TerrainGridSpacingStillRejectsActualStep)
{
  OccupyMap map(terrainPipelineMapConfig());
  auto points = terrainPipelineGrid(0.0);
  for (auto & point : points) {
    if (point[0] > 2.0) {point[2] = 0.16;}
  }
  map.updateSupport(points);
  const auto lock = map.lockForReading();
  Astar3D astar(map, terrainPipelinePlannerConfig());
  VoxelKey start, goal;
  ASSERT_TRUE(astar.projectToSurface(0.02557, 0.01, 0.0, 0.01, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(4.02557, 0.01, 0.16, 0.01, 0.5, goal));
  EXPECT_FALSE(astar.segmentTraversable(start, goal));
  EXPECT_FALSE(astar.segmentTerrainTraversable(start, goal));
  std::vector<VoxelKey> path;
  EXPECT_FALSE(astar.search(start, goal, path));
}

TEST(Astar3D, TerrainGridSpacingStillRejectsUnobservedGap)
{
  OccupyMap map(terrainPipelineMapConfig());
  std::vector<OccupyMap::Point> points;
  for (const auto & point : terrainPipelineGrid(0.0)) {
    if (point[0] < 1.5 || point[0] > 2.7) {points.push_back(point);}
  }
  map.updateSupport(points);
  const auto lock = map.lockForReading();
  Astar3D astar(map, terrainPipelinePlannerConfig());
  VoxelKey start, goal;
  ASSERT_TRUE(astar.projectToSurface(0.02557, 0.01, 0.0, 0.01, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(4.02557, 0.01, 0.0, 0.01, 0.5, goal));
  EXPECT_FALSE(astar.segmentTraversable(start, goal));
  EXPECT_FALSE(astar.segmentTerrainTraversable(start, goal));
  std::vector<VoxelKey> path;
  EXPECT_FALSE(astar.search(start, goal, path));
}

TEST(Astar3D, OfflineTwentyCentimeterDiagonalNeedsSpacingAwareConnection)
{
  auto mc = terrainPipelineMapConfig(); mc.inflate_radius = 0.0;
  OccupyMap map(mc);
  map.updateSupport({{0.01,0.01,0}, {0.21,0.21,0}, {0.41,0.41,0}});
  const auto lock = map.lockForReading();
  auto ac = terrainPipelinePlannerConfig(); ac.surface_connection_radius = 0.25;
  Astar3D original(map, ac);
  VoxelKey start, goal;
  ASSERT_TRUE(original.projectToSurface(0.01,0.01,0,0.01,0.5,start));
  ASSERT_TRUE(original.projectToSurface(0.41,0.41,0,0.01,0.5,goal));
  std::vector<VoxelKey> path;
  EXPECT_FALSE(original.search(start,goal,path));
  ac.surface_connection_radius = 0.30;
  EXPECT_TRUE(Astar3D(map,ac).search(start,goal,path));
}

TEST(Astar3D, OfflineDiagonalConnectionStillRejectsMissingGroundStepAndObstacle)
{
  for (int mode=0; mode<3; ++mode) {
    auto mc = terrainPipelineMapConfig(); mc.inflate_radius = 0.0;
    OccupyMap map(mc);
    std::vector<OccupyMap::Point> points{{0.01,0.01,0}, {0.41,0.41,0}};
    if (mode!=0) points.push_back({0.21,0.21,mode==1 ? 0.16 : 0.0});
    map.updateSupport(points);
    if (mode==2) map.setStaticObstacles({{0.21,0.21,0.25}});
    auto ac = terrainPipelinePlannerConfig(); ac.surface_connection_radius = 0.30;
    Astar3D astar(map,ac); const auto lock=map.lockForReading();
    VoxelKey start, goal;
    ASSERT_TRUE(astar.projectToSurface(0.01,0.01,0,0.01,0.5,start));
    ASSERT_TRUE(astar.projectToSurface(0.41,0.41,0,0.01,0.5,goal));
    std::vector<VoxelKey> path;
    EXPECT_FALSE(astar.search(start,goal,path)) << "mode=" << mode;
  }
}

TEST(Astar3D, TerrainGridSegmentStillChecksObstacleInIntermediateColumn)
{
  auto map_config = terrainPipelineMapConfig();
  map_config.inflate_radius = 0.0;  // Isolate the collision sample between patches.
  OccupyMap map(map_config);
  const double slope = std::tan(6.0 * 3.14159265358979323846 / 180.0);
  map.updateSupport(terrainPipelineGrid(slope));
  map.updateObstacles({{0.12557, 0.01, 0.3}}, 0.0, 0.0);
  const auto lock = map.lockForReading();
  Astar3D astar(map, terrainPipelinePlannerConfig());
  VoxelKey start, goal;
  ASSERT_TRUE(astar.projectToSurface(0.02557, 0.01, slope * 0.02557, 0.01, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(0.22557, 0.01, slope * 0.22557, 0.01, 0.5, goal));
  EXPECT_FALSE(astar.segmentTraversable(start, goal));
  EXPECT_TRUE(astar.segmentTerrainTraversable(start, goal));
}

TEST(Astar3D, RejectsUnsupportedGap)
{
  OccupyMap map(testMapConfig());
  std::vector<OccupyMap::Point> terrain;
  for (int x = 0; x <= 3; ++x) {
    terrain.push_back(OccupyMap::Point{0.1 * x + 0.01, 0.01, 0.01});
  }
  for (int x = 12; x <= 15; ++x) {
    terrain.push_back(OccupyMap::Point{0.1 * x + 0.01, 0.01, 0.01});
  }
  map.updateSupport(terrain);

  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  VoxelKey start;
  VoxelKey goal;
  ASSERT_TRUE(astar.projectToSurface(0.01, 0.01, 0.01, 0.2, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(1.41, 0.01, 0.01, 0.2, 0.5, goal));

  std::vector<VoxelKey> path;
  EXPECT_FALSE(astar.search(start, goal, path));
}

TEST(Astar3D, MultiGoalSearchRejectsDisconnectedElevatedSurface)
{
  OccupyMap map(testMapConfig());
  std::vector<OccupyMap::Point> support;
  for (int x = 0; x <= 20; ++x) {
    for (int y = -2; y <= 2; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 0.01});
    }
  }
  // A locally valid horizontal plane at the goal XY, representing a table
  // top.  It has no supported ramp or other connection to the floor.
  for (int x = 16; x <= 20; ++x) {
    for (int y = -2; y <= 2; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 1.01});
    }
  }
  map.updateSupport(support);

  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  VoxelKey start;
  VoxelKey floor_goal;
  VoxelKey table_goal;
  ASSERT_TRUE(astar.projectToSurface(0.01, 0.01, 0.01, 0.2, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(1.91, 0.01, 0.01, 0.2, 0.5, floor_goal));
  ASSERT_TRUE(astar.projectToSurface(1.91, 0.01, 1.01, 0.2, 0.5, table_goal));
  ASSERT_NE(floor_goal.z, table_goal.z);

  std::vector<VoxelKey> path;
  VoxelKey reached_goal;
  ASSERT_TRUE(astar.searchToAny(
      start, std::vector<VoxelKey>{table_goal, floor_goal}, path, reached_goal));
  EXPECT_EQ(reached_goal, floor_goal);
  EXPECT_EQ(path.back(), floor_goal);
}

TEST(Astar3D, SurfaceProjectionUsesMetricRadiusAtOffCentreQuery)
{
  auto config = testMapConfig();
  config.inflate_radius = 0.0;
  OccupyMap map(config);
  // Logged goal (.18,-1.51) and observed support (.20,-1.40). Their
  // cell-index offset is (1,2), but the support cell centre is only .175 m
  // from the query. An index-space radius of two cells used to omit it.
  map.updateSupport({{.20, -1.40, -.34}});
  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  OccupyMap::SurfaceDiagnostics diagnostic;
  const auto candidates = astar.surfaceCandidates(.18, -1.51, -.34, .20, .5, &diagnostic);
  ASSERT_EQ(candidates.size(), 1u);
  EXPECT_EQ(diagnostic.accepted, 1u);
  const auto p = map.keyToPos(candidates.front());
  EXPECT_LT(std::hypot(p[0] - .18, p[1] + 1.51), .20);
}

TEST(Astar3D, SurfaceProjectionDoesNotRoundMetricRadiusOutward)
{
  auto config = testMapConfig();
  config.inflate_radius = 0.0;
  OccupyMap map(config);
  // The candidate is two indices away, yet its centre (.25,.05) is
  // .24 m from this query and must not pass a .20 m radius.
  map.updateSupport({{.20, .0, 0.0}});
  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  EXPECT_TRUE(astar.surfaceCandidates(.01, .05, 0.0, .20, .5).empty());
}

TEST(Astar3D, MetricProjectionStillRejectsObstaclesAndWrongHeight)
{
  auto config = testMapConfig();
  config.inflate_radius = 0.0;
  OccupyMap map(config);
  map.updateSupport({{.20, -1.40, -.34}});
  Astar3D astar(map, testPlannerConfig());
  {const auto lock = map.lockForReading();
    EXPECT_TRUE(astar.surfaceCandidates(.18, -1.51, .7, .20, .2).empty());}
  map.setStaticObstacles({{.25, -1.35, -.15}});
  const auto lock = map.lockForReading();
  EXPECT_TRUE(astar.surfaceCandidates(.18, -1.51, -.34, .20, .5).empty());
}

TEST(Astar3D, SurfaceProjectionPrefersExpectedHeightOverCloserTable)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.05;
  OccupyMap map(map_config);
  std::vector<OccupyMap::Point> support;

  // The table is horizontally close to the query but 0.6m above the expected
  // floor. The visible floor starts farther away because of the LiDAR blind
  // zone.
  for (int ix = 0; ix <= 4; ++ix) {
    for (int iy = -2; iy <= 2; ++iy) {
      support.push_back({0.10 + ix * 0.1, iy * 0.1, 0.10});
      support.push_back({0.80 + ix * 0.1, iy * 0.1, -0.50});
    }
  }
  map.updateSupport(support);

  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  const auto candidates = astar.surfaceCandidates(
    0.0, 0.0, -0.50, 1.50, 1.0);
  ASSERT_FALSE(candidates.empty());
  const auto candidate_position = map.keyToPos(candidates.front());
  EXPECT_NEAR(candidate_position[2], -0.25, 0.06);
  EXPECT_GT(candidate_position[0], 0.70);
}

TEST(Astar3D, StartProjectionBridgesBlindZoneTowardGoal)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.05;
  OccupyMap map(map_config);

  // Simulate the unobserved disc underneath a LiDAR. Side samples have an
  // exact expected height while the floor toward the goal is 10cm noisier.
  // A vertical-first nearest projection used to alternate between both sides.
  map.updateSupport(
    {
      OccupyMap::Point{0.01, 1.01, -0.49},
      OccupyMap::Point{0.01, -1.01, -0.49},
      OccupyMap::Point{1.01, 0.01, -0.39},
      OccupyMap::Point{-1.01, 0.01, -0.49},
      // A closer table top must still be rejected as another terrain layer.
      OccupyMap::Point{0.31, 0.01, 0.11}
    });

  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.start_projection_height_tolerance = 0.2;
  config.start_projection_direction_weight = 2.0;
  Astar3D astar(map, config);
  VoxelKey projected;
  ASSERT_TRUE(astar.projectStartToSurface(
      0.0, 0.0, -0.49, 1.5, 1.0, 3.0, 0.0, projected));
  const auto position = map.keyToPos(projected);
  EXPECT_GT(position[0], 0.9);
  EXPECT_NEAR(position[1], 0.05, 0.06);
  EXPECT_LT(position[2], 0.0);
}

TEST(Astar3D, StartProjectionRejectsKnownObstacleInBlindBridge)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.05;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport(
    {
      OccupyMap::Point{1.01, 0.01, -0.49},
      OccupyMap::Point{0.01, 1.01, -0.49}
    });
  map.updateObstacles(
    {OccupyMap::Point{0.51, 0.01, -0.29}}, 0.0, 0.0);

  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.start_projection_height_tolerance = 0.2;
  config.start_projection_direction_weight = 2.0;
  Astar3D astar(map, config);
  VoxelKey projected;
  ASSERT_TRUE(astar.projectStartToSurface(
      0.0, 0.0, -0.49, 1.5, 1.0, 3.0, 0.0, projected));
  const auto position = map.keyToPos(projected);
  EXPECT_LT(position[0], 0.2);
  EXPECT_GT(position[1], 0.9);
}

TEST(Astar3D, ObstacleIsNeverGroundSupport)
{
  OccupyMap map(testMapConfig());
  std::vector<OccupyMap::Point> support;
  for (int x = 0; x <= 20; ++x) {
    for (int y = -10; y <= 10; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 0.01});
    }
  }
  std::vector<OccupyMap::Point> obstacle;
  for (int y = -3; y <= 3; ++y) {
    obstacle.push_back({1.01, 0.1 * y + 0.01, 0.21});
  }
  map.updateSupport(support);
  map.updateObstacles(obstacle, 1.0, 0.0);

  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  VoxelKey start;
  VoxelKey goal;
  ASSERT_TRUE(astar.projectToSurface(0.01, 0.01, 0.01, 0.2, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(2.01, 0.01, 0.01, 0.2, 0.5, goal));
  EXPECT_FALSE(astar.segmentTraversable(start, goal));

  std::vector<VoxelKey> path;
  ASSERT_TRUE(astar.search(start, goal, path));
  ASSERT_GT(path.size(), 2u);
  const auto simplified = astar.simplifyPath(path);
  ASSERT_GT(simplified.size(), 2u);
  for (std::size_t i = 1; i < simplified.size(); ++i) {
    EXPECT_TRUE(astar.segmentTraversable(simplified[i - 1], simplified[i]));
  }
}

TEST(Astar3D, ReportsWhyGoalIsNotTraversable)
{
  auto map_config = testMapConfig();
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  std::vector<OccupyMap::Point> support;
  for (int x = 0; x <= 5; ++x) {
    support.push_back({0.1 * x + 0.01, 0.01, 0.01});
  }
  map.updateSupport(support);

  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  const VoxelKey unsupported = map.posToKey(1.01, 0.01, 0.21);
  EXPECT_EQ(
    map.traversabilityUnlocked(
      unsupported, astar.supportDepthCells(), astar.robotHeightCells(),
      astar.preferredGroundGapCells(), astar.groundGapToleranceCells()),
    OccupyMap::Traversability::kNoGroundSupport);
}

TEST(Astar3D, MergesNoisyHeightsWithinOneSurfaceLayer)
{
  auto map_config = testMapConfig();
  map_config.support_merge_height = 0.15;
  OccupyMap map(map_config);
  map.updateSupport({OccupyMap::Point{0.01, 0.01, 0.01}});
  map.updateSupport({OccupyMap::Point{0.01, 0.01, 0.11}});

  {
    const auto lock = map.lockForReading();
    EXPECT_EQ(map.rawVoxelCountUnlocked(), 1u);
  }

  // A genuinely separate surface in the same XY column must remain a second
  // layer rather than being flattened into the first one.
  map.updateSupport({OccupyMap::Point{0.01, 0.01, 0.31}});
  {
    const auto lock = map.lockForReading();
    EXPECT_EQ(map.rawVoxelCountUnlocked(), 2u);
  }
}

TEST(Astar3D, MillimetreHeightChangeAcrossVoxelBoundaryIsTraversableAtTenDegrees)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport({{0.01, 0.01, -0.001}, {0.11, 0.01, 0.001}});
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.max_slope_deg = 10.0;
  config.max_step_height = 0.08;
  Astar3D astar(map, config);
  const VoxelKey start{0, 0, 1};
  const VoxelKey goal{1, 0, 2};
  ASSERT_NE(start.z, goal.z);
  EXPECT_TRUE(astar.segmentTraversable(start, goal));
  std::vector<VoxelKey> path;
  EXPECT_TRUE(astar.search(start, goal, path));
}

TEST(Astar3D, RejectsActualSteepSlopeWithinOneHeightVoxel)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport({{0.01, 0.01, 0.011}, {0.11, 0.01, 0.049}});
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.max_slope_deg = 10.0;
  config.max_step_height = 0.08;
  Astar3D astar(map, config);
  const VoxelKey start{0, 0, 2};
  const VoxelKey goal{1, 0, 2};
  EXPECT_FALSE(astar.segmentTraversable(start, goal));
  std::vector<VoxelKey> path;
  EXPECT_FALSE(astar.search(start, goal, path));
}

TEST(Astar3D, ShortcutChecksIntermediateSlopeInsteadOfEndpointAverage)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport({{0.01, 0.01, 0.011}, {0.11, 0.01, 0.049}, {0.21, 0.01, 0.011}});
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.max_slope_deg = 10.0;
  config.max_step_height = 0.08;
  Astar3D astar(map, config);
  const VoxelKey start{0, 0, 2};
  const VoxelKey goal{2, 0, 2};
  EXPECT_FALSE(astar.segmentTraversable(start, goal));
  std::vector<VoxelKey> path;
  EXPECT_FALSE(astar.search(start, goal, path));
}

TEST(Astar3D, ActualStepLimitHasNoHalfVoxelAllowance)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport({{0.01, 0.01, 0.011}, {0.11, 0.01, 0.092}});
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.max_slope_deg = 80.0;  // Isolate the 8 cm step check from the slope check.
  config.max_step_height = 0.08;
  Astar3D astar(map, config);
  const VoxelKey start{0, 0, 2};
  const VoxelKey goal{1, 0, 2};
  EXPECT_FALSE(astar.segmentTraversable(start, goal));
  std::vector<VoxelKey> path;
  EXPECT_FALSE(astar.search(start, goal, path));
}

TEST(Astar3D, ActualHeightFixDoesNotBridgeMissingSupportOrIgnoreObstacles)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport({{0.01, 0.01, -0.001}, {0.21, 0.01, 0.001}});
  auto config = testPlannerConfig();
  config.max_slope_deg = 10.0;
  config.max_step_height = 0.08;
  const VoxelKey start{0, 0, 1};
  const VoxelKey goal{2, 0, 2};
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    EXPECT_FALSE(astar.segmentTraversable(start, goal));
    std::vector<VoxelKey> path;
    EXPECT_FALSE(astar.search(start, goal, path));
  }
  map.updateSupport({{0.11, 0.01, 0.0}});
  map.updateObstacles({{0.11, 0.01, 0.21}}, 0.0, 0.0);
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    EXPECT_FALSE(astar.segmentTraversable(start, goal));
    std::vector<VoxelKey> path;
    EXPECT_FALSE(astar.search(start, goal, path));
  }
}

TEST(Astar3D, ContinuousSupportHeightsUpdateAndResetWithTheirLayers)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  OccupyMap map(map_config);
  const VoxelKey lower{0, 0, 0};
  const VoxelKey upper{0, 0, 3};
  map.updateSupport({{0.01, 0.01, 0.099}, {0.01, 0.01, 0.31}});
  map.updateSupport({{0.01, 0.01, 0.101}, {0.01, 0.01, 0.32}});
  {
    const auto lock = map.lockForReading();
    double height;
    EXPECT_EQ(map.rawVoxelCountUnlocked(), 2u);
    VoxelKey current_lower;
    ASSERT_TRUE(map.findSupportByHeightUnlocked(.01, .01, .101, .01, current_lower));
    EXPECT_EQ(current_lower.z, 1);
    EXPECT_FALSE(map.supportHeightUnlocked(lower, height));
    ASSERT_TRUE(map.supportHeightUnlocked(current_lower, height));
    EXPECT_NEAR(height, 0.101, 1e-9);
    ASSERT_TRUE(map.supportHeightUnlocked(upper, height));
    EXPECT_NEAR(height, 0.32, 1e-9);
  }
  map.reset();
  {
    const auto lock = map.lockForReading();
    double height;
    EXPECT_FALSE(map.supportHeightUnlocked(lower, height));
    EXPECT_FALSE(map.supportHeightUnlocked(upper, height));
    Astar3D astar(map, testPlannerConfig());
    std::vector<VoxelKey> path;
    EXPECT_FALSE(astar.search(VoxelKey{0, 0, 2}, VoxelKey{0, 0, 2}, path));
    EXPECT_FALSE(astar.segmentTraversable(VoxelKey{0, 0, 2}, VoxelKey{0, 0, 2}));
  }
  map.updateSupport({{0.01, 0.01, 0.011}, {0.11, 0.01, 0.049}});
  auto config = testPlannerConfig();
  config.max_slope_deg = 10.0;
  config.max_step_height = 0.08;
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    std::vector<VoxelKey> path;
    EXPECT_FALSE(astar.search(VoxelKey{0, 0, 2}, VoxelKey{1, 0, 2}, path));
  }
  map.updateSupport({{0.11, 0.01, 0.012}});
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    std::vector<VoxelKey> path;
    EXPECT_TRUE(astar.search(VoxelKey{0, 0, 2}, VoxelKey{1, 0, 2}, path));
    EXPECT_TRUE(astar.segmentTraversable(VoxelKey{0, 0, 2}, VoxelKey{1, 0, 2}));
  }
}

TEST(Astar3D, UpdatedMergedHeightExtendsTheCollisionClearanceWindow)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport({{0.01, 0.01, 0.01}, {0.11, 0.01, 0.11}});
  map.updateSupport({{0.01, 0.01, 0.11}});
  map.updateObstacles({{0.01, 0.01, 0.51}}, 0.0, 0.0);
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.max_slope_deg = 10.0;
  config.max_step_height = 0.08;
  Astar3D astar(map, config);
  const VoxelKey updated_node{0, 0, 3};
  const VoxelKey neighbor{1, 0, 3};
  EXPECT_FALSE(map.traversableOnSurfaceUnlocked(
      updated_node, VoxelKey{0, 0, 1}, astar.robotHeightCells(),
      astar.preferredGroundGapCells(), astar.groundGapToleranceCells()));
  EXPECT_FALSE(astar.segmentTraversable(updated_node, neighbor));
  // Terrain-only validation permits obstacle grace but still sees equal real
  // floor heights after the corrected support was re-keyed.
  EXPECT_TRUE(astar.segmentTerrainTraversable(updated_node, neighbor));
  std::vector<OccupyMap::Point> cached{{0.05, 0.05, 0.11}, {0.15, 0.05, 0.11}};
  EXPECT_TRUE(astar.refreshAndValidateTerrainPath(updated_node, cached));
  EXPECT_TRUE(astar.refreshAndValidateTerrainPath(updated_node, cached));
  config.ground_gap_tolerance = 0.0;
  Astar3D zero_tolerance_astar(map, config);
  EXPECT_TRUE(zero_tolerance_astar.refreshAndValidateTerrainPath(updated_node, cached));
  EXPECT_TRUE(zero_tolerance_astar.refreshAndValidateTerrainPath(updated_node, cached));
}

TEST(Astar3D, RefreshedMergedLayerCollisionUsesItsCanonicalKeyAtZeroGapTolerance)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport({{0.01, 0.01, 0.01}});
  map.updateSupport({{0.01, 0.01, 0.11}});
  auto config = testPlannerConfig();
  config.max_step_height = 0.08;
  config.ground_gap_tolerance = 0.0;
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    EXPECT_EQ(astar.pathPointTraversability({0.05, 0.05, 0.11}),
      OccupyMap::Traversability::kTraversable);
  }
  map.updateObstacles({{0.01, 0.01, 0.51}}, 0.0, 0.0);
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    // The corrected layer now follows its actual Z bin, so both direct map
    // queries and refreshed-path queries must detect the same collision.
    EXPECT_EQ(map.traversabilityUnlocked(
        VoxelKey{0, 0, 3}, astar.supportDepthCells(), astar.robotHeightCells(),
        astar.preferredGroundGapCells(), astar.groundGapToleranceCells()),
      OccupyMap::Traversability::kOccupied);
    EXPECT_EQ(astar.pathPointTraversability({0.05, 0.05, 0.11}),
      OccupyMap::Traversability::kOccupied);
  }
}

TEST(Astar3D, RefreshedRouteCollisionRetainsNeighborSupportFallback)
{
  auto map_config = testMapConfig();
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport({{0.01, 0.01, 0.01}});
  map.updateSupport({{0.01, 0.01, 0.11}});
  auto config = testPlannerConfig();
  config.max_step_height = 0.08;
  config.ground_gap_tolerance = 0.0;
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    EXPECT_EQ(astar.pathPointTraversability({0.15, 0.05, 0.11}),
      OccupyMap::Traversability::kTraversable);
    EXPECT_EQ(astar.pathPointTraversability({1.05, 0.05, 0.11}),
      OccupyMap::Traversability::kNoGroundSupport);
  }
  map.updateObstacles({{0.11, 0.01, 0.51}}, 0.0, 0.0);
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    EXPECT_EQ(astar.pathPointTraversability({0.15, 0.05, 0.11}),
      OccupyMap::Traversability::kOccupied);
    EXPECT_EQ(astar.pathPointTraversability({0.15, 0.05, 0.23}),
      OccupyMap::Traversability::kGroundGapMismatch);
  }
}

TEST(Astar3D, CachedTerrainValidationRefreshesHeightAndRejectsNewSteepSlope)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport({{0.01, 0.01, 0.011}, {0.11, 0.01, 0.012}, {0.21, 0.01, 0.013}});
  auto config = testPlannerConfig();
  config.max_slope_deg = 10.0;
  config.max_step_height = 0.08;
  const VoxelKey projected_start{0, 0, 2};
  const std::vector<OccupyMap::Point> cached{
    {0.05, 0.05, 0.05}, {0.15, 0.05, 0.05}, {0.25, 0.05, 0.05}};
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    auto refreshed = cached;
    ASSERT_TRUE(astar.refreshAndValidateTerrainPath(projected_start, refreshed));
    EXPECT_NEAR(refreshed[0][2], 0.011, 1e-9);
    EXPECT_NEAR(refreshed[1][2], 0.012, 1e-9);
    EXPECT_NEAR(refreshed[2][2], 0.013, 1e-9);
  }
  // A dynamic obstacle does not relax the independent terrain validation.
  map.updateObstacles({{0.11, 0.01, 0.21}}, 0.0, 0.0);
  map.updateSupport({{0.11, 0.01, 0.049}});
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    auto refreshed = cached;
    EXPECT_FALSE(astar.refreshAndValidateTerrainPath(projected_start, refreshed));
    EXPECT_FALSE(astar.segmentTerrainTraversable(projected_start, VoxelKey{2, 0, 2}));
  }
  map.reset();
  map.updateSupport({{0.01, 0.01, 0.011}, {0.21, 0.01, 0.013}});
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    auto refreshed = cached;
    EXPECT_FALSE(astar.refreshAndValidateTerrainPath(projected_start, refreshed));
  }
}

TEST(Astar3D, OfflineSelectedStartConnectorRetainsSparseRecordedMapSupport)
{
  // Exact values from maps/lab_v1_navigation/support.csv. These are sparse
  // observed cells, not a fabricated flat floor or interpolated support map.
  struct Case
  {
    OccupyMap::Point start;
    VoxelKey selected;
    std::vector<OccupyMap::Point> support;
  };
  const std::vector<Case> cases{
    {{3.7990988277, -1.7080307375, -0.366370273082}, {40, -16, -2},
      {{4.0, -1.6, -0.366370273082}, {4.0, -1.8, -0.36402806378},
        {3.6, -1.6, -0.363371941179}, {3.8, -1.4, -0.373127536868}}},
    {{2.58, -0.28, -0.38998065578}, {28, -4, -2},
      {{2.4, -0.2, -0.374650414265}, {2.4, -0.4, -0.376487311895},
        {2.8, -0.4, -0.38998065578}}},
    {{4.30, -0.40, -0.386441255623}, {46, -2, -2},
      {{4.2, -0.2, -0.385202960715}, {4.4, -0.6, -0.377020244128},
        {4.6, -0.2, -0.386441255623}, {4.6, -0.4, -0.380212244704}}}};
  for (const auto & item : cases) {
    auto mc = terrainPipelineMapConfig(); mc.inflate_radius = 0.0;
    OccupyMap map(mc); map.updateSupport(item.support);
    const auto lock = map.lockForReading();
    auto config = terrainPipelinePlannerConfig();
    config.start_projection_xy_radius = 2.0;
    const auto selected = map.keyToPos(item.selected);
    const std::vector<OccupyMap::Point> route{
      item.start, {selected[0], selected[1], item.start[2]}};
    Astar3D online(map, config);
    auto dense = online.densifyPath(route, 0.05);
    Astar3D::TerrainValidation failure;
    EXPECT_FALSE(online.refreshAndValidateTerrainPath(item.selected, dense, &failure));
    EXPECT_STREQ(failure.reason, "missing_ground_support");
    config.retain_start_projection_bridge = true;
    Astar3D offline(map, config);
    dense = offline.densifyPath(route, 0.05);
    EXPECT_TRUE(offline.refreshAndValidateTerrainPath(item.selected, dense, &failure)) <<
      failure.reason << " at " << failure.position[0] << ',' << failure.position[1];
  }
}

TEST(Astar3D, OfflineSelectedStartConnectorDoesNotRelaxLaterGapsOrOtherRoutes)
{
  auto mc = terrainPipelineMapConfig(); mc.inflate_radius = 0.0;
  OccupyMap map(mc);
  // A neighbouring patch initially gives radius support, followed by a blind
  // connector to the explicitly selected observed support at x=0.65.
  map.updateSupport({{0.01, -0.19, 0.0}, {0.61, 0.01, 0.0}, {1.61, 0.01, 0.0}});
  const auto lock = map.lockForReading();
  auto config = terrainPipelinePlannerConfig();
  config.start_projection_xy_radius = 2.0;
  config.retain_start_projection_bridge = true;
  Astar3D astar(map, config);
  const VoxelKey selected{6, 0, 2};
  Astar3D::TerrainValidation failure;
  auto connector = astar.densifyPath({{0.05, 0.05, 0}, {0.65, 0.05, 0}}, 0.05);
  ASSERT_TRUE(astar.refreshAndValidateTerrainPath(selected, connector, &failure));
  auto later_gap = astar.densifyPath(
    {{0.05, 0.05, 0}, {0.65, 0.05, 0}, {1.65, 0.05, 0}}, 0.05);
  EXPECT_FALSE(astar.refreshAndValidateTerrainPath(selected, later_gap, &failure));
  EXPECT_STREQ(failure.reason, "missing_ground_support");
  EXPECT_GT(failure.position[0], 0.65);
  auto absent_projection = astar.densifyPath({{0.05, 0.05, 0}, {1.65, 0.15, 0}}, 0.05);
  EXPECT_FALSE(astar.refreshAndValidateTerrainPath(selected, absent_projection));
  auto bent_connector = astar.densifyPath(
    {{0.05, 0.05, 0}, {0.35, 0.15, 0}, {0.65, 0.05, 0}}, 0.05);
  EXPECT_FALSE(astar.refreshAndValidateTerrainPath(selected, bent_connector));
  // The same policy cannot exempt an A* edge, direct shortcut or cached route
  // which never visits this concrete selected start endpoint.
  EXPECT_FALSE(astar.segmentTerrainTraversable(VoxelKey{0, -2, 2}, selected));
}

TEST(Astar3D, OfflineSelectedStartConnectorRetainsCollisionSlopeAndStepChecks)
{
  for (int mode = 0; mode < 5; ++mode) {
    auto mc = terrainPipelineMapConfig(); mc.inflate_radius = 0.0;
    OccupyMap map(mc);
    std::vector<OccupyMap::Point> support{{0.01, -0.19, 0.0}, {0.61, 0.01, 0.0}};
    if (mode == 1) {support.push_back({0.31, -0.09, 0.16});}
    if (mode == 2) {support.push_back({0.31, -0.09, 0.07});}
    if (mode == 3) {support.push_back({0.01, 0.01, 0.0});}
    if (mode == 4) {support[1][2] = 0.16;}
    map.updateSupport(support);
    if (mode == 0) {map.setStaticObstacles({{0.31, 0.01, 0.25}});}
    const auto lock = map.lockForReading();
    auto config = terrainPipelinePlannerConfig();
    config.start_projection_xy_radius = 2.0;
    config.retain_start_projection_bridge = true;
    Astar3D astar(map, config);
    auto route = astar.densifyPath(
      {{0.05, 0.05, 0.0}, {0.65, 0.05, mode == 4 ? 0.16 : 0.0}}, 0.05);
    Astar3D::TerrainValidation failure;
    EXPECT_FALSE(astar.refreshAndValidateTerrainPath(
        VoxelKey{6, 0, mode == 4 ? 3 : 2}, route, &failure)) << "mode=" << mode;
    if (mode == 0) {EXPECT_STREQ(failure.reason, "occupied_body_column");}
    if (mode == 1) {EXPECT_STREQ(failure.reason, "step_limit");}
    if (mode == 2) {EXPECT_STREQ(failure.reason, "slope_limit");}
    if (mode == 3) {EXPECT_STREQ(failure.reason, "missing_ground_support");}
  }
}

TEST(Astar3D, CachedLoopCannotSkipSteepFirstBranchBySelectingLaterStartCrossing)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport({{0.01, 0.01, 0.011}, {0.11, 0.01, 0.049}, {0.21, 0.01, 0.011}});
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.max_slope_deg = 10.0;
  config.max_step_height = 0.08;
  Astar3D astar(map, config);
  // The first start-cell crossing is slightly off its centre. A later exact
  // crossing must not hide the steep first branch from validation.
  std::vector<OccupyMap::Point> loop{
    {-0.05, 0.05, 0.011}, {0.05, 0.09, 0.011}, {0.15, 0.09, 0.011},
    {0.25, 0.09, 0.011}, {0.05, 0.05, 0.011}};
  EXPECT_FALSE(astar.refreshAndValidateTerrainPath(VoxelKey{0, 0, 2}, loop));
}

TEST(Astar3D, SupportedRouteDoesNotHaveToVisitNewOffRouteProjection)
{
  OccupyMap map(testMapConfig());
  std::vector<OccupyMap::Point> support;
  for (int x = 0; x <= 20; ++x) {
    for (int y = -2; y <= 2; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 0.01});
    }
  }
  map.updateSupport(support);
  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  auto route = astar.densifyPath({{0.05, 0.05, 0.01}, {2.05, 0.05, 0.01}}, 0.05);
  // A newly preferred projection lies in the next row. The actual route is
  // continuously supported, and need not pass through this new voxel centre.
  EXPECT_TRUE(astar.refreshAndValidateTerrainPath(VoxelKey{1, 1, 2}, route));
}

TEST(Astar3D, VoxelRouteAndDenseOutputAgreeAtDiagonalGridBoundaries)
{
  for (int offset : {-3, -1, 1, 3}) {
    auto map_config = testMapConfig();
    map_config.support_radius = 0.0;
    OccupyMap map(map_config);
    std::vector<OccupyMap::Point> support;
    for (int x = 0; x <= 10; ++x) {
      for (int y = -4; y <= 14; ++y) {
        support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01,
            y == x + offset ? 0.01 : 0.05});
      }
    }
    map.updateSupport(support);
    const auto lock = map.lockForReading();
    auto config = testPlannerConfig();
    config.max_slope_deg = 10.0;
    Astar3D astar(map, config);
    const VoxelKey start{0, offset, 2}, goal{10, 10 + offset, 2};
    std::vector<VoxelKey> voxels;
    ASSERT_TRUE(astar.search(start, goal, voxels));
    std::vector<OccupyMap::Point> route;
    for (const auto & key : astar.simplifyPath(voxels)) {
      const auto point = map.keyToPos(key);
      route.push_back({point[0], point[1], 0.01});
    }
    auto dense = astar.densifyPath(route, 0.05);
    EXPECT_TRUE(astar.refreshAndValidateTerrainPath(start, dense)) << "offset=" << offset;
  }
}

TEST(Astar3D, ExactEndpointSimplificationRemovesRecordedStartAndGoalKinks)
{
  OccupyMap map(terrainPipelineMapConfig());
  std::vector<OccupyMap::Point> support;
  for (int x = -5; x <= 30; ++x) {
    for (int y = -15; y <= 10; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, -0.48});
    }
  }
  map.updateSupport(support);
  const auto lock = map.lockForReading();
  Astar3D astar(map, terrainPipelinePlannerConfig());
  // XY comes from three captured Oct-13 /plan publications. The synthetic
  // floor is flat: these tests verify the endpoint-assembly geometry only.
  const std::vector<std::vector<OccupyMap::Point>> routes{
    {{2.00352264, -0.93322216, -0.48}, {1.95, -0.95, -0.48},
      {-0.16216993, -0.64944512, -0.48}},
    {{1.02206975, -0.83001969, -0.48}, {0.95, -0.85, -0.48},
      {-0.16216993, -0.64944512, -0.48}},
    {{0.51285535, -0.74430657, -0.48}, {1.25, -0.45, -0.48},
      {1.24102867, -0.46859542, -0.48}}};
  for (const auto & route : routes) {
    const auto simplified = astar.simplifyGroundPath(route, 0.2);
    ASSERT_EQ(simplified.size(), 2u);
    EXPECT_EQ(simplified.front(), route.front());
    EXPECT_EQ(simplified.back(), route.back());
  }
}

TEST(Astar3D, StartProjectionRetainsObservedRobotColumnOnFlatFloor)
{
  OccupyMap map(terrainPipelineMapConfig());
  std::vector<OccupyMap::Point> support;
  for (int x = -5; x <= 25; ++x) {
    for (int y = -15; y <= 5; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, -0.48});
    }
  }
  map.updateSupport(support);
  const auto lock = map.lockForReading();
  auto config = terrainPipelinePlannerConfig();
  config.start_projection_xy_radius = 2.0;
  Astar3D astar(map, config);
  VoxelKey start;
  ASSERT_TRUE(astar.projectStartToSurface(
      1.02206975, -0.83001969, -0.48, 2.0, 0.5,
      -0.16216993, -0.64944512, start));
  EXPECT_EQ(start.x, 10);
  EXPECT_EQ(start.y, -9);
}

TEST(Astar3D, EndpointShortcutKeepsKnownObstacleDetour)
{
  OccupyMap map(testMapConfig());
  std::vector<OccupyMap::Point> support;
  for (int x = 0; x <= 20; ++x) {
    for (int y = -10; y <= 10; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 0.01});
    }
  }
  map.updateSupport(support);
  map.updateObstacles({{1.01, 0.01, 0.31}}, 0.0, 0.0);
  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  const std::vector<OccupyMap::Point> route{
    {0.05, 0.05, 0.01}, {0.55, 0.85, 0.01},
    {1.55, 0.85, 0.01}, {2.05, 0.05, 0.01}};
  const auto simplified = astar.simplifyGroundPath(route, 0.2);
  EXPECT_GT(simplified.size(), 2u);
  for (const auto & sample : astar.densifyPath(simplified, 0.025)) {
    EXPECT_EQ(astar.pathPointTraversability(sample), OccupyMap::Traversability::kTraversable);
  }
}

TEST(Astar3D, EndpointShortcutKeepsUnsupportedGapDetour)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  OccupyMap map(map_config);
  std::vector<OccupyMap::Point> support;
  for (int x = 0; x <= 20; ++x) {
    for (int y = 0; y <= 10; ++y) {
      if (x <= 2 || x >= 18 || y >= 8) {
        support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 0.01});
      }
    }
  }
  map.updateSupport(support);
  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  const std::vector<OccupyMap::Point> route{
    {0.05, 0.05, 0.01}, {0.05, 0.85, 0.01},
    {2.05, 0.85, 0.01}, {2.05, 0.05, 0.01}};
  const auto simplified = astar.simplifyGroundPath(route, 0.2);
  EXPECT_GT(simplified.size(), 2u);
  auto dense = astar.densifyPath(simplified, 0.05);
  EXPECT_TRUE(astar.refreshAndValidateTerrainPath(VoxelKey{0, 0, 2}, dense));
}

TEST(Astar3D, RouteBlindAllowanceStopsAtFirstObservedSupport)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport({{0.01, 0.01, 0.01}, {1.01, 0.01, 0.01}});
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.start_projection_xy_radius = 2.0;
  Astar3D astar(map, config);
  std::vector<OccupyMap::Point> route{{0.05, 0.05, 0.01}, {1.05, 0.05, 0.01}};
  const auto original = route;
  Astar3D::TerrainValidation failure;
  EXPECT_FALSE(astar.refreshAndValidateTerrainPath(VoxelKey{10, 0, 2}, route, &failure));
  EXPECT_STREQ(failure.reason, "missing_ground_support");
  EXPECT_EQ(route, original);
  // A large projection radius never allows shortcuts through a gap after
  // support has already been observed beneath the actual start.
  const auto simplified = astar.simplifyGroundPath(
    {route.front(), {0.25, 0.05, 0.01}, route.back()}, 2.0);
  EXPECT_EQ(simplified.size(), 3u);
}

TEST(Astar3D, StraightShortcutAcrossInitialBlindZoneIsBoundedAndCollisionChecked)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  std::vector<OccupyMap::Point> support;
  for (int x = 10; x <= 20; ++x) {
    for (int y = -3; y <= 3; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 0.01});
    }
  }
  map.updateSupport(support);
  auto config = testPlannerConfig();
  config.start_projection_xy_radius = 1.5;
  const std::vector<OccupyMap::Point> route{
    {0.05, 0.05, 0.01}, {1.05, 0.25, 0.01}, {2.05, 0.05, 0.01}};
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    EXPECT_EQ(astar.simplifyGroundPath(route, 1.1).size(), 2u);
    EXPECT_EQ(astar.simplifyGroundPath(route, 0.2).size(), 3u);
    auto dense = astar.densifyPath({route.front(), route.back()}, 0.05);
    EXPECT_TRUE(astar.refreshAndValidateTerrainPath(VoxelKey{10, 2, 2}, dense));
  }
  map.updateObstacles({{0.51, 0.01, 0.31}}, 0.0, 0.0);
  {
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    EXPECT_EQ(astar.simplifyGroundPath(route, 1.1).size(), 3u);
    auto dense = astar.densifyPath({route.front(), route.back()}, 0.05);
    Astar3D::TerrainValidation failure;
    EXPECT_FALSE(astar.refreshAndValidateTerrainPath(VoxelKey{10, 2, 2}, dense, &failure));
    EXPECT_STREQ(failure.reason, "occupied_body_column");
  }
}

TEST(Astar3D, DirectGroundCandidateDoesNotInheritSideProjectionBlindBudget)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  std::vector<OccupyMap::Point> support{{0.81, 0.21, 0.01}};
  for (int x = 16; x <= 30; ++x) {
    support.push_back({x * 0.1 + 0.01, 0.01, 0.01});
  }
  map.updateSupport(support);
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.start_projection_xy_radius = 2.0;
  config.max_step_height = 0.08;
  Astar3D astar(map, config);
  VoxelKey projected;
  ASSERT_TRUE(astar.projectStartToSurface(
      0.05, 0.05, 0.01, 2.0, 0.5, 3.05, 0.05, projected));
  EXPECT_EQ(projected.x, 8);
  EXPECT_EQ(projected.y, 2);
  const auto position = map.keyToPos(projected);
  const double old_budget = std::hypot(position[0] - 0.05, position[1] - 0.05) + 0.2;
  const std::vector<OccupyMap::Point> route{
    {0.05, 0.05, 0.01}, {0.85, 0.25, 0.01},
    {1.65, 0.05, 0.01}, {3.05, 0.05, 0.01}};
  EXPECT_GT(astar.simplifyGroundPath(route, old_budget).size(), 2u);
  std::vector<OccupyMap::Point> direct;
  ASSERT_TRUE(astar.tryDirectGroundPath(route.front(), route.back(), 2.0, direct));
  ASSERT_EQ(direct.size(), 2u);
  EXPECT_EQ(direct.front(), route.front());
  EXPECT_EQ(direct.back(), route.back());
  auto dense = astar.densifyPath(direct, 0.05);
  // Rechecking and cache reuse must not reinstate the unrelated side budget.
  EXPECT_TRUE(astar.refreshAndValidateTerrainPath(projected, dense));
}

TEST(Astar3D, DirectGroundCandidateCannotCrossSecondUnsupportedGap)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  map.updateSupport({{0.01, 0.01, 0.01}, {1.01, 0.01, 0.01}});
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.start_projection_xy_radius = 2.0;
  Astar3D astar(map, config);
  const std::vector<OccupyMap::Point> original{{9.0, 9.0, 9.0}};
  auto output = original;
  Astar3D::TerrainValidation failure;
  EXPECT_FALSE(astar.tryDirectGroundPath(
      {0.05, 0.05, 0.01}, {1.05, 0.05, 0.01}, 2.0, output, &failure));
  EXPECT_STREQ(failure.reason, "missing_ground_support");
  EXPECT_EQ(output, original);
}

TEST(Astar3D, DirectGroundCandidateChecksObstacleInsideInitialBlindZone)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  std::vector<OccupyMap::Point> support;
  for (int x = 16; x <= 30; ++x) {
    support.push_back({x * 0.1 + 0.01, 0.01, 0.01});
  }
  map.updateSupport(support);
  map.updateObstacles({{0.51, 0.01, 0.31}}, 0.0, 0.0);
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.start_projection_xy_radius = 2.0;
  Astar3D astar(map, config);
  std::vector<OccupyMap::Point> output;
  Astar3D::TerrainValidation failure;
  EXPECT_FALSE(astar.tryDirectGroundPath(
      {0.05, 0.05, 0.01}, {3.05, 0.05, 0.01}, 2.0, output, &failure));
  EXPECT_STREQ(failure.reason, "occupied_body_column");
}

TEST(Astar3D, DirectGroundCandidateCannotExpandConfiguredBlindRadiusOrUseUnknownGoal)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  std::vector<OccupyMap::Point> support;
  for (int x = 21; x <= 30; ++x) {
    support.push_back({x * 0.1 + 0.01, 0.01, 0.01});
  }
  map.updateSupport(support);
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.start_projection_xy_radius = 2.0;
  Astar3D astar(map, config);
  std::vector<OccupyMap::Point> output;
  Astar3D::TerrainValidation failure;
  EXPECT_FALSE(astar.tryDirectGroundPath(
      {0.05, 0.05, 0.01}, {3.05, 0.05, 0.01}, 5.0, output, &failure));
  EXPECT_STREQ(failure.reason, "missing_ground_support");
  EXPECT_FALSE(astar.tryDirectGroundPath(
      {0.05, 0.05, 0.01}, {1.05, 0.05, 0.01}, 2.0, output, &failure));
  EXPECT_STREQ(failure.reason, "missing_goal_ground_support");
}

TEST(Astar3D, DirectGroundCandidateCannotUseBlindPrefixToReachTableTop)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  std::vector<OccupyMap::Point> support;
  for (int x = 8; x <= 15; ++x) {
    support.push_back({x * 0.1 + 0.01, 0.01, 0.51});
  }
  map.updateSupport(support);
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.start_projection_xy_radius = 2.0;
  config.max_step_height = 0.08;
  Astar3D astar(map, config);
  std::vector<OccupyMap::Point> output;
  Astar3D::TerrainValidation failure;
  EXPECT_FALSE(astar.tryDirectGroundPath(
      {0.05, 0.05, 0.01}, {1.55, 0.05, 0.51}, 2.0, output, &failure));
  EXPECT_STREQ(failure.reason, "blind_ground_height_mismatch");
}

TEST(Astar3D, DirectGroundCandidateKeepsObservedSlopeAndStepLimits)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  auto config = testPlannerConfig();
  config.start_projection_xy_radius = 2.0;
  config.max_slope_deg = 10.0;
  config.max_step_height = 0.08;
  for (double rise : {0.04, 0.16}) {
    map.reset();
    map.updateSupport({{0.01, 0.01, 0.011}, {0.11, 0.01, 0.011 + rise}});
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    std::vector<OccupyMap::Point> output;
    Astar3D::TerrainValidation failure;
    EXPECT_FALSE(astar.tryDirectGroundPath(
        {0.05, 0.05, 0.011}, {0.15, 0.05, 0.011 + rise}, 2.0, output, &failure));
    EXPECT_STREQ(failure.reason, rise < 0.08 ? "slope_limit" : "step_limit");
  }
}

TEST(Astar3D, ConservativeDirectGroundLayerGateRetainsObservedRampFallback)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  constexpr double slope = 0.105;  // About six degrees, below the ten-degree limit.
  std::vector<OccupyMap::Point> support;
  for (int x = 10; x <= 20; ++x) {
    const double world_x = x * 0.1 + 0.01;
    support.push_back({world_x, 0.01, slope * world_x});
  }
  map.updateSupport(support);
  const auto lock = map.lockForReading();
  auto config = testPlannerConfig();
  config.start_projection_xy_radius = 2.0;
  config.max_slope_deg = 10.0;
  config.max_step_height = 0.08;
  Astar3D astar(map, config);
  std::vector<OccupyMap::Point> direct;
  Astar3D::TerrainValidation failure;
  EXPECT_FALSE(astar.tryDirectGroundPath(
      {0.05, 0.05, 0.0}, {2.05, 0.05, slope * 2.01}, 2.0, direct, &failure));
  EXPECT_STREQ(failure.reason, "blind_ground_height_mismatch");
  // The additional first-layer gate is specific to speculative direct lines.
  // It must not reject an A* route with a valid observed ramp component.
  std::vector<OccupyMap::Point> fallback{
    {0.05, 0.05, 0.0}, {1.05, 0.05, slope * 1.01},
    {2.05, 0.05, slope * 2.01}};
  EXPECT_TRUE(astar.refreshAndValidateTerrainPath(VoxelKey{10, 0, 3}, fallback));
}

TEST(Astar3D, TerrainFailureReportsActualSlopeAndStepSeparately)
{
  auto map_config = testMapConfig();
  map_config.support_radius = 0.0;
  OccupyMap map(map_config);
  auto config = testPlannerConfig();
  config.max_slope_deg = 10.0;
  config.max_step_height = 0.08;
  for (double rise : {0.04, 0.16}) {
    map.reset();
    map.updateSupport({{0.01, 0.01, 0.011}, {0.11, 0.01, 0.011 + rise}});
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    std::vector<OccupyMap::Point> route{
      {0.05, 0.05, 0.011}, {0.15, 0.05, 0.011 + rise}};
    Astar3D::TerrainValidation failure;
    EXPECT_FALSE(astar.refreshAndValidateTerrainPath(VoxelKey{0, 0, 2}, route, &failure));
    EXPECT_STREQ(failure.reason, rise < 0.08 ? "slope_limit" : "step_limit");
    EXPECT_EQ(failure.index, 1u);
    ASSERT_TRUE(failure.has_support_edge);
    EXPECT_EQ(failure.previous_support.x, 0);
    EXPECT_EQ(failure.current_support.x, 1);
    EXPECT_NEAR(failure.previous_height, 0.011, 1e-12);
    EXPECT_NEAR(failure.current_height, 0.011 + rise, 1e-12);
    EXPECT_NEAR(failure.horizontal_distance, 0.1, 1e-12);
  }
}

TEST(Astar3D, OpenSpacePathSimplifiesToStraightSegment)
{
  OccupyMap map(testMapConfig());
  std::vector<OccupyMap::Point> support;
  for (int x = 0; x <= 20; ++x) {
    for (int y = -2; y <= 2; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 0.01});
    }
  }
  map.updateSupport(support);

  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  VoxelKey start;
  VoxelKey goal;
  ASSERT_TRUE(astar.projectToSurface(0.01, 0.01, 0.01, 0.2, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(2.01, 0.01, 0.01, 0.2, 0.5, goal));
  std::vector<VoxelKey> path;
  ASSERT_TRUE(astar.search(start, goal, path));
  const auto simplified = astar.simplifyPath(path);
  ASSERT_EQ(simplified.size(), 2u);
  EXPECT_TRUE(astar.segmentTraversable(simplified.front(), simplified.back()));
}

TEST(Astar3D, RepeatedSurfaceSearchIsDeterministic)
{
  OccupyMap map(testMapConfig());
  std::vector<OccupyMap::Point> support;
  for (int x = 0; x <= 20; ++x) {
    for (int y = -10; y <= 10; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 0.01});
    }
  }
  map.updateSupport(support);

  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  VoxelKey start;
  VoxelKey goal;
  ASSERT_TRUE(astar.projectToSurface(0.01, 0.01, 0.01, 0.2, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(2.01, 0.71, 0.01, 0.2, 0.5, goal));

  std::vector<VoxelKey> reference;
  ASSERT_TRUE(astar.search(start, goal, reference));
  ASSERT_FALSE(reference.empty());
  for (int attempt = 0; attempt < 20; ++attempt) {
    std::vector<VoxelKey> repeated;
    ASSERT_TRUE(astar.search(start, goal, repeated));
    EXPECT_EQ(repeated, reference);
  }
}

TEST(Astar3D, RejectsDisconnectedVerticalSurfaceLayer)
{
  OccupyMap map(testMapConfig());
  std::vector<OccupyMap::Point> support;
  for (int x = 0; x <= 8; ++x) {
    for (int y = -2; y <= 2; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 0.01});
    }
  }
  for (int x = 9; x <= 18; ++x) {
    for (int y = -2; y <= 2; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 0.51});
    }
  }
  map.updateSupport(support);

  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  VoxelKey start;
  VoxelKey goal;
  ASSERT_TRUE(astar.projectToSurface(0.01, 0.01, 0.01, 0.2, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(1.71, 0.01, 0.51, 0.2, 0.5, goal));
  std::vector<VoxelKey> path;
  EXPECT_FALSE(astar.search(start, goal, path));
}

TEST(Astar3D, ProjectsUnsupportedGoalToNearbyGround)
{
  OccupyMap map(testMapConfig());
  std::vector<OccupyMap::Point> support;
  for (int x = 0; x <= 20; ++x) {
    for (int y = -5; y <= 5; ++y) {
      if (x >= 15 && x <= 19 && y >= -2 && y <= 2) {
        continue;
      }
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 0.01});
    }
  }
  map.updateSupport(support);

  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  VoxelKey start;
  VoxelKey projected_goal;
  ASSERT_TRUE(astar.projectToSurface(0.01, 0.01, 0.01, 0.2, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(
      1.71, 0.01, 0.01, 0.5, 0.5, projected_goal));
  const VoxelKey exact_goal = map.posToKey(1.71, 0.01, 0.21);
  EXPECT_EQ(
    map.traversabilityUnlocked(
      exact_goal, astar.supportDepthCells(), astar.robotHeightCells(),
      astar.preferredGroundGapCells(), astar.groundGapToleranceCells()),
    OccupyMap::Traversability::kNoGroundSupport);
  EXPECT_FALSE(projected_goal == exact_goal);
  EXPECT_LE(
    std::hypot(
      map.keyToPos(projected_goal)[0] - 1.71,
      map.keyToPos(projected_goal)[1] - 0.01),
    0.51);

  std::vector<VoxelKey> path;
  EXPECT_TRUE(astar.search(start, projected_goal, path));
}

TEST(Astar3D, ProjectsOccupiedGoalToNearbyCollisionFreeGround)
{
  OccupyMap map(testMapConfig());
  std::vector<OccupyMap::Point> support;
  for (int x = 0; x <= 20; ++x) {
    for (int y = -6; y <= 6; ++y) {
      support.push_back({0.1 * x + 0.01, 0.1 * y + 0.01, 0.01});
    }
  }
  map.updateSupport(support);
  map.updateObstacles({OccupyMap::Point{1.01, 0.01, 0.21}}, 1.0, 0.0);

  const auto lock = map.lockForReading();
  Astar3D astar(map, testPlannerConfig());
  VoxelKey start;
  VoxelKey projected_goal;
  ASSERT_TRUE(astar.projectToSurface(0.01, 0.01, 0.01, 0.2, 0.5, start));
  ASSERT_TRUE(astar.projectToSurface(
      1.01, 0.01, 0.01, 0.5, 0.5, projected_goal));
  const VoxelKey exact_goal = map.posToKey(1.01, 0.01, 0.21);
  EXPECT_EQ(
    map.traversabilityUnlocked(
      exact_goal, astar.supportDepthCells(), astar.robotHeightCells(),
      astar.preferredGroundGapCells(), astar.groundGapToleranceCells()),
    OccupyMap::Traversability::kOccupied);
  EXPECT_FALSE(projected_goal == exact_goal);
  EXPECT_TRUE(map.traversableUnlocked(
      projected_goal, astar.supportDepthCells(), astar.robotHeightCells(),
      astar.preferredGroundGapCells(), astar.groundGapToleranceCells()));

  std::vector<VoxelKey> path;
  EXPECT_TRUE(astar.search(start, projected_goal, path));
}

TEST(Astar3D, DensifiesSimplifiedPathForController)
{
  OccupyMap map(testMapConfig());
  Astar3D astar(map, testPlannerConfig());
  const std::vector<OccupyMap::Point> sparse{
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.2},
    {1.0, 0.5, 0.3}};
  const auto dense = astar.densifyPath(sparse, 0.05);

  ASSERT_EQ(dense.front(), sparse.front());
  ASSERT_EQ(dense.back(), sparse.back());
  ASSERT_GT(dense.size(), 25u);
  for (std::size_t i = 1; i < dense.size(); ++i) {
    const double spacing = std::hypot(
      dense[i][0] - dense[i - 1][0],
      dense[i][1] - dense[i - 1][1]);
    EXPECT_LE(spacing, 0.050001);
  }
}

TEST(Astar3DPlanner, StableCachedSuffixIgnoresTurnInPlaceJitter)
{
  nav_msgs::msg::Path cached_path;
  for (int i = 0; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.05 * i;
    pose.pose.position.y = 0.0;
    cached_path.poses.push_back(pose);
  }

  double progress = 0.0;
  nav_msgs::msg::Path first;
  ASSERT_TRUE(selectStablePathSuffix(
      cached_path, 0.02, 0.01, 0.75, 0.3, progress, first));
  ASSERT_FALSE(first.poses.empty());
  EXPECT_DOUBLE_EQ(first.poses.front().pose.position.x, 0.02);
  EXPECT_DOUBLE_EQ(first.poses.front().pose.position.y, 0.01);
  EXPECT_NEAR(progress, 0.02, 1e-9);

  nav_msgs::msg::Path jittered;
  ASSERT_TRUE(selectStablePathSuffix(
      cached_path, -0.01, -0.02, 0.75, 0.3, progress, jittered));
  ASSERT_GE(jittered.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(jittered.poses.front().pose.position.x, -0.01);
  EXPECT_DOUBLE_EQ(jittered.poses.front().pose.position.y, -0.02);
  EXPECT_NEAR(jittered.poses[1].pose.position.x, 0.02, 1e-9);
  EXPECT_NEAR(progress, 0.02, 1e-9);
}

TEST(Astar3DPlanner, StableCachedSuffixOnlyPrunesMonotonically)
{
  nav_msgs::msg::Path cached_path;
  for (int i = 0; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.05 * i;
    cached_path.poses.push_back(pose);
  }

  double progress = 0.0;
  nav_msgs::msg::Path advanced;
  ASSERT_TRUE(selectStablePathSuffix(
      cached_path, 1.0, 0.0, 0.75, 0.3, progress, advanced));
  ASSERT_FALSE(advanced.poses.empty());
  EXPECT_NEAR(advanced.poses.front().pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(progress, 1.0, 1e-9);
  const auto advanced_size = advanced.poses.size();

  nav_msgs::msg::Path stale_measurement;
  ASSERT_TRUE(selectStablePathSuffix(
      cached_path, 0.8, 0.0, 0.75, 0.3, progress, stale_measurement));
  ASSERT_GE(stale_measurement.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(stale_measurement.poses.front().pose.position.x, 0.8);
  EXPECT_GE(stale_measurement.poses[1].pose.position.x, 1.0);
  EXPECT_NEAR(progress, 1.0, 1e-9);
  EXPECT_LE(stale_measurement.poses.size(), advanced_size + 1u);
}

TEST(Astar3DPlanner, ZeroBacktrackStartsAtNearestForwardProgress)
{
  nav_msgs::msg::Path cached_path;
  for (int i = 0; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.05 * i;
    cached_path.poses.push_back(pose);
  }

  double progress = 0.0;
  nav_msgs::msg::Path output;
  ASSERT_TRUE(selectStablePathSuffix(
      cached_path, 1.0, 0.0, 0.75, 0.0, progress, output));
  ASSERT_FALSE(output.poses.empty());
  EXPECT_NEAR(output.poses.front().pose.position.x, 1.0, 1e-9);
}

TEST(Astar3DPlanner, EndpointProgressOutsideGoalProducesTwoPoseReturnPath)
{
  nav_msgs::msg::Path cached_path;
  for (int i = 0; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.05 * i;
    pose.pose.orientation.w = 1.0;
    cached_path.poses.push_back(pose);
  }

  // Reproduce a previously advanced monotonic cache index after the robot
  // stopped 30 cm short of (or drifted away from) the endpoint.
  double progress = 2.0;
  nav_msgs::msg::Path output;
  ASSERT_TRUE(selectStablePathSuffix(
      cached_path, 1.70, 0.0, 0.75, 0.0, progress, output));
  ASSERT_EQ(output.poses.size(), 2u);
  EXPECT_NEAR(output.poses.front().pose.position.x, 1.70, 1e-9);
  EXPECT_NEAR(output.poses.back().pose.position.x, 2.0, 1e-9);
}

TEST(Astar3DPlanner, OvershotEndpointProducesTwoPoseReturnPath)
{
  nav_msgs::msg::Path cached_path;
  for (int i = 0; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.05 * i;
    pose.pose.orientation.w = 1.0;
    cached_path.poses.push_back(pose);
  }

  double progress = 2.0;
  nav_msgs::msg::Path output;
  ASSERT_TRUE(selectStablePathSuffix(
      cached_path, 2.30, 0.0, 0.75, 0.0, progress, output));
  ASSERT_EQ(output.poses.size(), 2u);
  EXPECT_NEAR(output.poses.front().pose.position.x, 2.30, 1e-9);
  EXPECT_NEAR(output.poses.back().pose.position.x, 2.0, 1e-9);

  const auto & orientation = output.poses.front().pose.orientation;
  const double yaw = std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (
      orientation.y * orientation.y + orientation.z * orientation.z));
  EXPECT_NEAR(std::abs(yaw), M_PI, 1e-9);
}

TEST(Astar3DPlanner, ExactEndpointMayRemainSinglePoseForGoalChecker)
{
  nav_msgs::msg::Path cached_path;
  geometry_msgs::msg::PoseStamped endpoint;
  endpoint.pose.position.x = 2.0;
  endpoint.pose.position.y = -0.5;
  endpoint.pose.orientation.w = 1.0;
  cached_path.poses.push_back(endpoint);

  double progress = 0.0;
  nav_msgs::msg::Path output;
  ASSERT_TRUE(selectStablePathSuffix(
      cached_path, 2.0, -0.5, 0.25, 0.0, progress, output));
  ASSERT_EQ(output.poses.size(), 1u);
}

TEST(Astar3DPlanner, OffPathRobotIsPoseZeroAndReconnectsAtProjection)
{
  nav_msgs::msg::Path cached_path;
  for (int i = 0; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 0.05 * i;
    pose.pose.position.y = 0.0;
    cached_path.poses.push_back(pose);
  }

  double progress = 0.0;
  nav_msgs::msg::Path output;
  ASSERT_TRUE(selectStablePathSuffix(
      cached_path, 1.02, 0.10, 0.25, 0.0, progress, output));
  ASSERT_GE(output.poses.size(), 3u);
  EXPECT_DOUBLE_EQ(output.poses[0].pose.position.x, 1.02);
  EXPECT_DOUBLE_EQ(output.poses[0].pose.position.y, 0.10);
  EXPECT_NEAR(output.poses[1].pose.position.x, 1.02, 1e-9);
  EXPECT_NEAR(output.poses[1].pose.position.y, 0.0, 1e-9);
  EXPECT_GT(output.poses[2].pose.position.x, output.poses[1].pose.position.x);
  EXPECT_NEAR(progress, 1.02, 1e-9);
}

TEST(Astar3DPlanner, CrossingPathProjectionNeverReturnsToOldBranch)
{
  nav_msgs::msg::Path cached_path;
  for (const auto & point : std::vector<std::array<double, 2>>{
      {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0},
      {0.0, 1.0}, {0.0, 0.0}, {-1.0, 0.0}})
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    cached_path.poses.push_back(pose);
  }

  double progress = 0.0;
  nav_msgs::msg::Path initial;
  ASSERT_TRUE(selectStablePathSuffix(
      cached_path, 0.0, 0.0, 0.25, 0.0, progress, initial));
  EXPECT_NEAR(progress, 0.0, 1e-9);

  nav_msgs::msg::Path advanced;
  ASSERT_TRUE(selectStablePathSuffix(
      cached_path, 1.0, 1.0, 0.25, 0.0, progress, advanced));
  EXPECT_NEAR(progress, 2.0, 1e-9);

  nav_msgs::msg::Path at_crossing;
  ASSERT_TRUE(selectStablePathSuffix(
      cached_path, 0.0, 0.0, 0.25, 0.0, progress, at_crossing));
  EXPECT_NEAR(progress, 4.0, 1e-9);
  ASSERT_GE(at_crossing.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(at_crossing.poses.back().pose.position.x, -1.0);
}

TEST(Astar3D, ClearsStaleObstacleInsideObservedRegion)
{
  auto map_config = testMapConfig();
  map_config.inflate_radius = 0.0;
  OccupyMap map(map_config);
  const OccupyMap::Point obstacle{1.01, 0.01, 0.21};
  map.updateObstacles({obstacle}, 0.0, 0.0);

  {
    const auto lock = map.lockForReading();
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(1.01, 0.01, 0.21)));
  }

  map.updateObstacles({}, 0.0, 0.0);
  map.updateObstacles({}, 0.0, 0.0);
  map.updateObstacles({}, 0.0, 0.0);
  {
    const auto lock = map.lockForReading();
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(1.01, 0.01, 0.21)));
  }
}

TEST(Astar3D, KeepsStaleObstacleOutsideObservedRegion)
{
  auto map_config = testMapConfig();
  map_config.inflate_radius = 0.0;
  map_config.obstacle_update_radius = 1.0;
  map_config.obstacle_clear_after_updates = 1;
  OccupyMap map(map_config);
  const OccupyMap::Point obstacle{3.01, 0.01, 0.21};
  map.updateObstacles({obstacle}, 0.0, 0.0);
  map.updateObstacles({}, 0.0, 0.0);

  const auto lock = map.lockForReading();
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(3.01, 0.01, 0.21)));
}

TEST(Astar3D, ReobservedObstacleDoesNotExpire)
{
  auto map_config = testMapConfig();
  map_config.inflate_radius = 0.0;
  map_config.obstacle_clear_after_updates = 2;
  OccupyMap map(map_config);
  const OccupyMap::Point obstacle{1.01, 0.01, 0.21};
  map.updateObstacles({obstacle}, 0.0, 0.0);
  map.updateObstacles({}, 0.0, 0.0);
  map.updateObstacles({obstacle}, 0.0, 0.0);
  map.updateObstacles({}, 0.0, 0.0);

  const auto lock = map.lockForReading();
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(1.01, 0.01, 0.21)));
}

}  // namespace astar_3d_planner
