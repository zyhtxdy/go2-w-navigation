#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "astar_3d_planner/astar_3d.hpp"
#include "astar_3d_planner/swept_footprint.hpp"

namespace astar_3d_planner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
using Point = OccupyMap::Point;

OccupyMap::Config corridorMapConfig()
{
  OccupyMap::Config config;
  config.resolution = .1;
  config.inflate_radius = .24;
  config.cost_inflate_cells = 2;
  config.support_radius = .15;
  return config;
}

OccupyMap::Config rectangularMapConfig()
{
  auto config = corridorMapConfig();
  config.footprint_inscribed_radius = .26;
  return config;
}

Astar3D::Config corridorSearchConfig()
{
  Astar3D::Config config;
  config.max_step_height = .08;
  config.max_slope_deg = 10;
  config.robot_height = .75;
  config.surface_connection_radius = .3;
  config.max_search_nodes = 100000;
  return config;
}

SweptFootprint::Config paddedRectangle()
{
  SweptFootprint::Config config;
  config.min_x = -.41;
  config.max_x = .41;
  config.min_y = -.26;
  config.max_y = .26;
  config.robot_height = .75;
  return config;
}

std::vector<Point> corridorFloor()
{
  std::vector<Point> floor;
  for (int x = -10; x <= 30; ++x) {
    for (int y = -5; y <= 5; ++y) {
      floor.push_back({x * .1 + .01, y * .1 + .01, .01});
    }
  }
  return floor;
}

std::vector<Point> corridorWalls(double lower, double upper)
{
  std::vector<Point> walls;
  for (int x = -10; x <= 30; ++x) {
    for (int z = 1; z <= 7; ++z) {
      walls.push_back({x * .1 + .01, lower, z * .1 + .01});
      walls.push_back({x * .1 + .01, upper, z * .1 + .01});
    }
  }
  return walls;
}

std::vector<Point> groundPath(const OccupyMap & map, const Astar3D & astar,
  const std::vector<VoxelKey> & keys)
{
  std::vector<Point> points;
  for (const auto & key : keys) {
    VoxelKey ground;
    double height = 0;
    EXPECT_TRUE(map.findSurfaceGroundUnlocked(key, astar.preferredGroundGapCells(),
      astar.groundGapToleranceCells(), ground));
    EXPECT_TRUE(map.supportHeightUnlocked(ground, height));
    auto point = map.keyToPos(key);
    point[2] = height;
    points.push_back(point);
  }
  return points;
}

Astar3D::MotionConstraints rectangularMotion(
  const OccupyMap & map, const Astar3D & astar, const SweptFootprint & checker,
  double start_yaw = 0, double goal_yaw = 0)
{
  Astar3D::MotionConstraints motion;
  motion.start_yaw = start_yaw;
  motion.maximum_seconds = 2;
  motion.transition = [&checker](const Point & from, const Point & to,
      double incoming, double outgoing) {
      return checker.validatePath({from, to}, incoming, outgoing);
    };
  motion.terminal = [&map, &astar, &checker, goal_yaw](const VoxelKey & key, double yaw) {
      return checker.validatePath(groundPath(map, astar, {key}), yaw, goal_yaw);
    };
  return motion;
}
}  // namespace

TEST(RectangularPrefilter, ContinuousNarrowCorridorExposesInflatedColumnFalseRejection)
{
  OccupyMap map(corridorMapConfig());
  map.updateSupport(corridorFloor());
  // Complete source voxel faces are y=-.3 and y=+.3: .6 m of free space.
  map.setStaticObstacles(corridorWalls(-.31, .31));
  Astar3D inflated(map, corridorSearchConfig());
  SweptFootprint checker(map, paddedRectangle());
  const auto lock = map.lockForReading();
  const std::vector<Point> straight{{.01, .01, .01}, {2.01, .01, .01}};
  ASSERT_EQ(map.inflationCells(), 3);
  const auto body_cell = map.posToKey(.01, .01, .11);
  EXPECT_TRUE(map.occupiedUnlocked(body_cell));
  EXPECT_FALSE(map.rawObstacleOccupiedUnlocked(body_cell));
  EXPECT_TRUE(inflated.surfaceCandidates(.01, .01, .01, 0, .1).empty());
  Astar3D::TerrainValidation terrain;
  std::vector<Point> output;
  EXPECT_FALSE(inflated.tryDirectGroundPath(straight.front(), straight.back(), 0,
    output, &terrain));
  EXPECT_STREQ(terrain.reason, "occupied_body_column");
  EXPECT_TRUE(checker.poseCollisionFree(straight.front(), 0));
  EXPECT_TRUE(checker.poseCollisionFree(straight.back(), 0));
  SweptFootprint::Failure sweep;
  EXPECT_TRUE(checker.validatePath(straight, 0, 0, &sweep));
  EXPECT_GT(sweep.samples, 2u);
  // The continuous pose is deliberately off the voxel centre. A route through
  // y=.05 really collides with the upper wall and must not become a false pass.
  EXPECT_FALSE(checker.poseCollisionFree({.05, .05, .01}, 0));
}

TEST(RectangularPrefilter, NecessaryCollisionPrefilterRetainsExactContinuousCorridor)
{
  OccupyMap map(rectangularMapConfig());
  map.updateSupport(corridorFloor());
  map.setStaticObstacles(corridorWalls(-.31, .31));
  Astar3D astar(map, corridorSearchConfig());
  SweptFootprint checker(map, paddedRectangle());
  const auto lock = map.lockForReading();
  const std::vector<Point> straight{{.01, .01, .01}, {2.01, .01, .01}};
  ASSERT_EQ(map.inflationCells(), 3);
  const auto body_cell = map.posToKey(.01, .01, .11);
  EXPECT_FALSE(map.occupiedUnlocked(body_cell));
  EXPECT_FALSE(map.rawObstacleOccupiedUnlocked(body_cell));
  // The original .24 m radius and its ceil-to-three-cells range still
  // contribute clearance cost after ceasing to be a hard rejection here.
  EXPECT_GT(map.obstacleCostUnlocked(body_cell, astar.robotHeightCells() - 1), 0);

  VoxelKey start, goal;
  ASSERT_TRUE(astar.projectToSurface(.01, .01, .01, 0, .1, start));
  ASSERT_TRUE(astar.projectToSurface(2.01, .01, .01, 0, .1, goal));
  EXPECT_EQ(astar.pathPointTraversability(straight.front()),
    OccupyMap::Traversability::kTraversable);
  EXPECT_EQ(astar.pathPointTraversability(straight.back()),
    OccupyMap::Traversability::kTraversable);
  Astar3D::TerrainValidation terrain;
  std::vector<Point> output;
  ASSERT_TRUE(astar.tryDirectGroundPath(straight.front(), straight.back(), 0,
    output, &terrain)) << terrain.reason;
  EXPECT_EQ(output.front(), straight.front());
  EXPECT_EQ(output.back(), straight.back());
  EXPECT_TRUE(checker.poseCollisionFree(output.front(), 0));
  EXPECT_TRUE(checker.poseCollisionFree(output.back(), 0));
  EXPECT_TRUE(checker.validatePath(output, 0, 0));

  auto dense = astar.densifyPath(straight, .06);
  ASSERT_GT(dense.size(), 2u);
  ASSERT_TRUE(astar.refreshAndValidateTerrainPath(start, dense, &terrain, true)) <<
    terrain.reason;
  const auto simplified = astar.simplifyGroundPath(dense, 0);
  ASSERT_EQ(simplified.size(), 2u);
  EXPECT_EQ(simplified.front(), straight.front());
  EXPECT_EQ(simplified.back(), straight.back());
  EXPECT_TRUE(checker.validatePath(simplified, 0, 0));
}

TEST(RectangularPrefilter, AcceptedCandidateStillRequiresCompleteConnectorGeometry)
{
  OccupyMap map(rectangularMapConfig());
  map.updateSupport(corridorFloor());
  map.setStaticObstacles(corridorWalls(-.31, .31));
  Astar3D astar(map, corridorSearchConfig());
  SweptFootprint checker(map, paddedRectangle());
  const auto lock = map.lockForReading();
  const Point exact_start{.01, .01, .01};
  VoxelKey candidate;
  ASSERT_TRUE(astar.projectToSurface(.01, .01, .01, 0, .1, candidate));
  EXPECT_FALSE(checker.poseCollisionFree(groundPath(map, astar, {candidate}).front(), 0));
  Astar3D::ProjectionDiagnostics diagnostic;
  EXPECT_FALSE(astar.projectStartToSurface(.01, .01, .01, 0, .1, 2.01, .01,
    candidate, &diagnostic, [&](const VoxelKey & key) {
      const auto projected = groundPath(map, astar, {key}).front();
      return checker.validatePath({exact_start, projected}, 0, 0);
    }));
  EXPECT_GT(diagnostic.footprint_rejected, 0u);
}

TEST(RectangularPrefilter, NecessaryCollisionPrefilterStillRejectsRawBodyObstacles)
{
  OccupyMap map(rectangularMapConfig());
  map.updateSupport(corridorFloor());
  map.setStaticObstacles({{1.01, .01, .31}});
  Astar3D astar(map, corridorSearchConfig());
  SweptFootprint checker(map, paddedRectangle());
  const auto lock = map.lockForReading();
  const Point blocked{1.01, .01, .01};
  const auto body_cell = map.posToKey(1.01, .01, .31);
  EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(body_cell));
  EXPECT_TRUE(map.occupiedUnlocked(body_cell));
  EXPECT_TRUE(astar.surfaceCandidates(blocked[0], blocked[1], blocked[2], 0, .1).empty());
  EXPECT_EQ(astar.pathPointTraversability(blocked), OccupyMap::Traversability::kOccupied);
  std::vector<Point> output;
  Astar3D::TerrainValidation failure;
  EXPECT_FALSE(astar.tryDirectGroundPath({.01, .01, .01}, {2.01, .01, .01}, 0,
    output, &failure));
  EXPECT_STREQ(failure.reason, "occupied_body_column");
  EXPECT_TRUE(failure.has_collision);
  EXPECT_TRUE(map.occupiedUnlocked(failure.collision));
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(failure.collision, evidence));
  EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(evidence.key));
  EXPECT_FALSE(checker.validatePath({{.01, .01, .01}, {2.01, .01, .01}}, 0, 0));
}

TEST(RectangularPrefilter, NecessaryCollisionPrefilterPreservesGroundGapRejection)
{
  OccupyMap map(rectangularMapConfig());
  auto floor = corridorFloor();
  floor.erase(std::remove_if(floor.begin(), floor.end(), [](const Point & point) {
      return point[0] > .8 && point[0] < 1.3;
    }), floor.end());
  map.updateSupport(floor);
  Astar3D astar(map, corridorSearchConfig());
  const auto lock = map.lockForReading();
  std::vector<Point> output;
  Astar3D::TerrainValidation failure;
  EXPECT_FALSE(astar.tryDirectGroundPath({.01, .01, .01}, {2.01, .01, .01}, 0,
    output, &failure));
  EXPECT_STREQ(failure.reason, "missing_ground_support");
}

TEST(RectangularPrefilter, NecessaryCollisionPrefilterPreservesStepRejection)
{
  OccupyMap map(rectangularMapConfig());
  auto floor = corridorFloor();
  for (auto & point : floor) {
    if (point[0] > 1) {point[2] = .21;}
  }
  map.updateSupport(floor);
  Astar3D astar(map, corridorSearchConfig());
  const auto lock = map.lockForReading();
  std::vector<Point> output;
  Astar3D::TerrainValidation failure;
  EXPECT_FALSE(astar.tryDirectGroundPath({.01, .01, .01}, {2.01, .01, .21}, 0,
    output, &failure));
  EXPECT_STREQ(failure.reason, "step_limit");
}

TEST(RectangularPrefilter, ZeroInscribedRadiusSupportsFootprintsAwayFromBodyOrigin)
{
  auto config = rectangularMapConfig();
  config.footprint_inscribed_radius = 0;
  OccupyMap map(config);
  auto body = paddedRectangle();
  body.min_x = .2;
  body.max_x = 1.02;
  SweptFootprint checker(map, body);
  const Point origin{.01, .01, .01};
  map.setStaticObstacles({{.01, .01, .31}});
  {
    const auto lock = map.lockForReading();
    const auto source = map.posToKey(.01, .01, .31);
    EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(source));
    EXPECT_FALSE(map.occupiedUnlocked(source));
    const auto cost = map.obstacleCostUnlocked(source, 0);
    EXPECT_TRUE(std::isfinite(cost));
    EXPECT_GT(cost, 0);
    EXPECT_TRUE(checker.poseCollisionFree(origin, 0));
  }
  // Disabling the necessary centre mask does not disable body geometry.
  map.setStaticObstacles({{.31, .01, .31}});
  const auto lock = map.lockForReading();
  VoxelKey collision;
  EXPECT_FALSE(checker.poseCollisionFree(origin, 0, &collision));
  EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(collision));
}

TEST(RectangularPrefilter, HardMaskIsNecessaryForSubcellPosesAtNegativeCoordinates)
{
  OccupyMap map(rectangularMapConfig());
  const VoxelKey source{-7, -6, 3};
  map.setStaticObstacles({map.keyToPos(source)});
  SweptFootprint checker(map, paddedRectangle());
  const auto lock = map.lockForReading();
  std::size_t checked = 0;
  for (int dx = -3; dx <= 3; ++dx) {
    for (int dy = -3; dy <= 3; ++dy) {
      const VoxelKey query{source.x + dx, source.y + dy, source.z};
      if (!map.occupiedUnlocked(query)) {continue;}
      const auto center = map.keyToPos(query);
      for (const auto & subcell : std::vector<Point>{{0, 0, 0}, {-.049, -.049, 0},
          {-.049, .049, 0}, {.049, -.049, 0}, {.049, .049, 0}})
      {
        const Point origin{center[0] + subcell[0], center[1] + subcell[1], .01};
        for (const double yaw : {0.0, kPi / 4, kPi / 2}) {
          EXPECT_FALSE(checker.poseCollisionFree(origin, yaw));
          ++checked;
        }
      }
    }
  }
  EXPECT_GT(checked, 0u);
  // A three-cell offset formerly rejected by ceil(.24/.1) contains a real
  // clear subcell pose even when both world coordinates are negative.
  const VoxelKey released{source.x, source.y - 3, source.z};
  EXPECT_FALSE(map.occupiedUnlocked(released));
  auto origin = map.keyToPos(released);
  origin[1] -= .049;
  origin[2] = .01;
  EXPECT_TRUE(checker.poseCollisionFree(origin, 0));
}

TEST(RectangularPrefilter, RefinedStaticExtentDoesNotLoseAValidOffAxisPoseToWholeVoxelMask)
{
  auto config = rectangularMapConfig();
  config.refine_static_obstacle_extents = true;
  config.obstacle_extent_prefilter_margin = .02;
  OccupyMap map(config);
  map.setStaticObstacles({{.299, .199, .225}}, 0, {OccupyMap::kMeasured});
  auto body = paddedRectangle();
  body.refine_static_obstacle_extents = true;
  SweptFootprint checker(map, body);
  const auto read = map.lockForReading();
  // The old centre-distance mask rejects this entire query cell although
  // the refined rectangle has 2.74 mm of genuine separating clearance.
  const Point origin{.05, .05, .01};
  ASSERT_TRUE(checker.poseCollisionFree(origin, 2.1100337945190666));
  EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(.05, .05, .225)));
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(.299, .199, .225)));
}

TEST(RectangularPrefilter, RefinedMaskRemainsNecessaryAcrossNegativeSubcellsAndHeadings)
{
  std::size_t rejections = 0, released = 0;
  for (const bool dynamic : {false, true}) {
    auto config = rectangularMapConfig();
    config.refine_static_obstacle_extents = true;
    config.refine_measured_obstacle_extents = true;
    config.obstacle_extent_prefilter_margin = .02;
    OccupyMap map(config);
    const Point obstacle{-.601, -.401, .225};
    if (dynamic) {
      map.updateObstacles({obstacle}, 0, 0, 1, {OccupyMap::kMeasured});
    } else {map.setStaticObstacles({obstacle}, 0, {OccupyMap::kMeasured});}
    auto body = paddedRectangle(); body.refine_static_obstacle_extents = true;
    body.near_start_measured_refinement_radius = .8;
    body.near_start_center = std::array<double, 2>{obstacle[0], obstacle[1]};
    SweptFootprint checker(map, body);
    const auto read = map.lockForReading();
    const auto source = map.posToKey(obstacle[0], obstacle[1], obstacle[2]);
    for (int dx = -3; dx <= 3; ++dx) {
      for (int dy = -3; dy <= 3; ++dy) {
        const VoxelKey query{source.x + dx, source.y + dy, source.z};
        if (!map.occupiedUnlocked(query)) {++released; continue;}
        ++rejections;
        const auto center = map.keyToPos(query);
        for (const double sx : {-.0499, 0.0, .0499}) {
          for (const double sy : {-.0499, 0.0, .0499}) {
            for (int heading = 0; heading < 16; ++heading) {
              EXPECT_FALSE(checker.poseCollisionFree(
                {center[0] + sx, center[1] + sy, .01}, heading * kPi / 8));
            }
          }
        }
      }
    }
  }
  EXPECT_GT(rejections, 0u);
  EXPECT_GT(released, 0u);
}

TEST(RectangularPrefilter, RefinementCannotReleaseUnknownSyntheticOrMixedStaticVoxels)
{
  auto config = rectangularMapConfig();
  config.refine_static_obstacle_extents = true;
  config.refine_measured_obstacle_extents = true;
  config.obstacle_extent_prefilter_margin = .02;
  OccupyMap map(config);
  const Point obstacle{.299, .199, .225};
  for (const auto & sources : std::vector<std::vector<std::uint8_t>>{
      {}, {OccupyMap::kSynthetic}, {OccupyMap::kUnknown}})
  {
    map.setStaticObstacles({obstacle}, 0, sources);
    const auto read = map.lockForReading();
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(.05, .05, .225)));
  }
  map.setStaticObstacles({obstacle, obstacle}, 0,
    {OccupyMap::kMeasured, OccupyMap::kSynthetic});
  map.updateObstacles({obstacle}, 0, 0, 1, {OccupyMap::kMeasured});
  const auto read = map.lockForReading();
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(.05, .05, .225)));
}

TEST(RectangularPrefilter, InvalidRefinementMarginCannotChangeNecessaryMask)
{
  auto config = rectangularMapConfig();
  config.obstacle_extent_prefilter_margin = -.01;
  EXPECT_THROW(OccupyMap map(config), std::invalid_argument);
  config.obstacle_extent_prefilter_margin = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(OccupyMap map(config), std::invalid_argument);
}

TEST(RectangularPrefilter, DynamicRemovalRecomputesReleasedHaloClearanceCosts)
{
  auto config = rectangularMapConfig();
  config.obstacle_clear_after_updates = 1;
  OccupyMap map(config);
  const Point near{.01, .31, .31}, far{.01, -.41, .31};
  const auto query = map.posToKey(.01, .01, .31);
  ASSERT_TRUE(map.updateObstacles({near, far}, 0, 0, 1,
    {OccupyMap::kMeasured, OccupyMap::kMeasured}));
  {
    const auto lock = map.lockForReading();
    EXPECT_FALSE(map.occupiedUnlocked(query));
    EXPECT_NEAR(map.obstacleCostUnlocked(query, 0), 1.0 / 3.0, 1e-6);
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(near[0], near[1], near[2])));
  }
  ASSERT_TRUE(map.updateObstacles({far}, 0, 0, 2, {OccupyMap::kMeasured}));
  {
    const auto lock = map.lockForReading();
    EXPECT_FALSE(map.rawObstacleOccupiedUnlocked(map.posToKey(near[0], near[1], near[2])));
    EXPECT_NEAR(map.obstacleCostUnlocked(query, 0), 1.0 / 5.0, 1e-6);
  }
  ASSERT_TRUE(map.updateObstacles({}, 0, 0, 3));
  const auto lock = map.lockForReading();
  EXPECT_FALSE(map.rawObstacleOccupiedUnlocked(map.posToKey(far[0], far[1], far[2])));
  EXPECT_FALSE(map.occupiedUnlocked(query));
  EXPECT_DOUBLE_EQ(map.obstacleCostUnlocked(query, 0), 0);
}

TEST(RectangularPrefilter, LowObstacleClearanceCostUsesObservedFloorInsteadOfPlanningHeight)
{
  OccupyMap map(rectangularMapConfig());
  std::vector<Point> support;
  for(int x=0;x<=10;++x) {support.push_back({x*.1+.01,.01,.01});}
  map.updateSupport(support);
  // Outside the full body's side envelope, but inside the soft-cost halo.
  // The return is below the .2 m nominal search-node height.
  map.setStaticObstacles({{.51,.41,.11}});
  const auto lock=map.lockForReading();
  auto config=corridorSearchConfig();config.obstacle_cost_weight=0;
  Astar3D geometric(map,config);VoxelKey start,goal,reached;
  ASSERT_TRUE(geometric.projectToSurface(.01,.01,.01,.01,.1,start));
  ASSERT_TRUE(geometric.projectToSurface(1.01,.01,.01,.01,.1,goal));
  std::vector<VoxelKey> path;double geometric_cost=0,weighted_cost=0;
  ASSERT_TRUE(geometric.search(start,goal,path,&geometric_cost));
  config.obstacle_cost_weight=2;
  Astar3D weighted(map,config);
  ASSERT_TRUE(weighted.search(start,goal,path,&weighted_cost));
  EXPECT_GT(weighted_cost,geometric_cost);
  SweptFootprint checker(map,paddedRectangle());
  Astar3D::MotionConstraints motion;
  motion.maximum_seconds=2;
  motion.transition=[&](const auto & from,const auto & to,double in,double out) {
      return checker.validatePath({from,to},in,out);
    };
  motion.terminal=[&](const auto & key,double yaw) {
      return checker.validatePath(groundPath(map,weighted,{key}),yaw,0);
    };
  double informed_cost=0;
  ASSERT_TRUE(weighted.searchToAny(start,{goal},path,reached,&informed_cost,nullptr,&motion));
  EXPECT_NEAR(informed_cost,weighted_cost,1e-9);
}

TEST(RectangularPrefilter, FullRectangleRejectsCorridorNarrowerThanBody)
{
  OccupyMap map(rectangularMapConfig());
  map.updateSupport(corridorFloor());
  // Voxel faces y=-.2 and y=+.3 leave .5 m, less than the .52 m body.
  map.setStaticObstacles(corridorWalls(-.21, .31));
  SweptFootprint checker(map, paddedRectangle());
  const auto lock = map.lockForReading();
  EXPECT_FALSE(map.rawObstacleOccupiedUnlocked(map.posToKey(.01, .05, .31)));
  VoxelKey collision;
  EXPECT_FALSE(checker.poseCollisionFree({.01, .05, .01}, 0, &collision));
  EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(collision));
  EXPECT_FALSE(checker.validatePath({{.01, .05, .01}, {2.01, .05, .01}}, 0, 0));
}

TEST(RectangularPrefilter, FullRectangleRejectsTurningSweepBetweenClearEndpointPoses)
{
  OccupyMap map(rectangularMapConfig());
  // Voxel faces y=-.3 and y=+.4 leave .7 m, enough for straight travel.
  map.setStaticObstacles(corridorWalls(-.31, .41));
  SweptFootprint checker(map, paddedRectangle());
  const auto lock = map.lockForReading();
  const Point position{1.05, .05, .01};
  EXPECT_TRUE(checker.poseCollisionFree(position, 0));
  EXPECT_TRUE(checker.poseCollisionFree(position, kPi));
  SweptFootprint::Failure failure;
  EXPECT_FALSE(checker.validatePath({position}, 0, kPi, &failure));
  EXPECT_STREQ(failure.reason, "footprint_collision");
  EXPECT_TRUE(failure.has_collision);
  EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(failure.collision));
  EXPECT_GT(failure.samples, 1u);
}

TEST(RectangularPrefilter, FullRectangleRejectsTranslationBetweenClearEndpointPoses)
{
  OccupyMap map(rectangularMapConfig());
  map.updateSupport(corridorFloor());
  auto walls = corridorWalls(-.31, .41);
  // This obstacle misses the centre column, but intersects the rectangle's
  // lateral body during translation. Endpoint-only checks cannot detect it.
  walls.push_back({1.01, .21, .31});
  map.setStaticObstacles(walls);
  SweptFootprint checker(map, paddedRectangle());
  const auto lock = map.lockForReading();
  const std::vector<Point> straight{{.01, .05, .01}, {2.01, .05, .01}};
  EXPECT_TRUE(checker.poseCollisionFree(straight.front(), 0));
  EXPECT_TRUE(checker.poseCollisionFree(straight.back(), 0));
  EXPECT_FALSE(map.rawObstacleOccupiedUnlocked(map.posToKey(1.01, .05, .31)));
  SweptFootprint::Failure failure;
  EXPECT_FALSE(checker.validatePath(straight, 0, 0, &failure));
  EXPECT_TRUE(failure.has_collision);
  EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(failure.collision));
  EXPECT_GT(failure.samples, 2u);
}

TEST(RectangularPrefilter, SearchStillRejectsTerminalTurnWithClearGoalPose)
{
  OccupyMap map(rectangularMapConfig());
  map.updateSupport(corridorFloor());
  map.setStaticObstacles(corridorWalls(-.31, .41));
  Astar3D astar(map, corridorSearchConfig());
  SweptFootprint checker(map, paddedRectangle());
  const auto lock = map.lockForReading();
  VoxelKey start, goal, reached;
  ASSERT_TRUE(astar.projectToSurface(.05, .05, .01, 0, .1, start));
  ASSERT_TRUE(astar.projectToSurface(2.05, .05, .01, 0, .1, goal));
  EXPECT_TRUE(checker.poseCollisionFree({.05, .05, .01}, 0));
  EXPECT_TRUE(checker.poseCollisionFree({2.05, .05, .01}, kPi));
  auto motion = rectangularMotion(map, astar, checker, 0, kPi);
  std::vector<VoxelKey> route;
  Astar3D::SearchDiagnostics diagnostic;
  EXPECT_FALSE(astar.searchToAny(start, {goal}, route, reached, nullptr,
    &diagnostic, &motion));
  EXPECT_TRUE(route.empty());
  EXPECT_GT(diagnostic.terminal_rejected, 0u);
  EXPECT_GT(diagnostic.motion_rejected, 0u);
}

TEST(RectangularPrefilter, GridAlignedCorridorRetainsRealRectangularSearchValidation)
{
  OccupyMap map(rectangularMapConfig());
  map.updateSupport(corridorFloor());
  map.setStaticObstacles(corridorWalls(-.31, .41));
  Astar3D astar(map, corridorSearchConfig());
  SweptFootprint checker(map, paddedRectangle());
  const auto lock = map.lockForReading();
  VoxelKey start, goal, reached;
  ASSERT_TRUE(astar.projectToSurface(.05, .05, .01, 0, .1, start));
  ASSERT_TRUE(astar.projectToSurface(2.05, .05, .01, 0, .1, goal));
  auto motion = rectangularMotion(map, astar, checker);
  std::vector<VoxelKey> route;
  Astar3D::SearchDiagnostics diagnostic;
  ASSERT_TRUE(astar.searchToAny(start, {goal}, route, reached, nullptr,
    &diagnostic, &motion));
  ASSERT_GE(route.size(), 2u);
  EXPECT_TRUE(reached == goal);
  auto points = groundPath(map, astar, route);
  EXPECT_TRUE(astar.refreshAndValidateTerrainPath(start, points, nullptr, true));
  EXPECT_TRUE(checker.validatePath(points, 0, 0));
  for (const auto & point : points) {
    EXPECT_NEAR(point[1], .05, 1e-9);
    EXPECT_TRUE(checker.poseCollisionFree(point, 0));
  }
}
}  // namespace astar_3d_planner
