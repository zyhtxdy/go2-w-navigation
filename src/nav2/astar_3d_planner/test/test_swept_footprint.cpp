#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "astar_3d_planner/swept_footprint.hpp"

namespace astar_3d_planner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
void setMeasuredStatic(OccupyMap & map, const std::vector<OccupyMap::Point> & points)
{
  map.setStaticObstacles(points, 0,
    std::vector<std::uint8_t>(points.size(), OccupyMap::kMeasured));
}
OccupyMap::Config mapConfig(double resolution = .05)
{
  OccupyMap::Config config;
  config.resolution = resolution;
  config.inflate_radius = .3;
  return config;
}
}

TEST(SweptFootprint, StraightNarrowCorridorPassesWithoutDoubleInflation)
{
  OccupyMap map(mapConfig());
  std::vector<OccupyMap::Point> walls;
  for (int i = -20; i <= 60; ++i) {
    walls.push_back({i * .05 + .025, .375, .225});
    walls.push_back({i * .05 + .025, -.375, .225});
  }
  setMeasuredStatic(map, walls);
  SweptFootprint checker(map, SweptFootprint::Config{});
  const auto lock = map.lockForReading();
  // Inflated wall voxels intersect the rectangle, but source walls do not.
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(1, .2, .225)));
  EXPECT_TRUE(checker.validatePath({{0, 0, 0}, {2, 0, 0}}, 0, 0));
  EXPECT_FALSE(checker.poseCollisionFree({1, 0, 0}, kPi / 4));
}

TEST(SweptFootprint, FrontAndRearCornersAreCheckedOutsideCentreCircle)
{
  OccupyMap map(mapConfig());
  SweptFootprint checker(map, SweptFootprint::Config{});
  for (double x : {-.375, .375}) {
    setMeasuredStatic(map, {{x, .225, .225}});
    const auto lock = map.lockForReading();
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(0, 0, .225)));
    VoxelKey collision;
    EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0, &collision));
    EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(collision));
  }
}

TEST(SweptFootprint, IntersectsVoxelVolumeEvenWhenVoxelCentreIsOutsideBody)
{
  OccupyMap map(mapConfig(.1));
  setMeasuredStatic(map, {{.25, .05, .25}});
  SweptFootprint::Config config;
  config.min_x = config.min_y = -.21;
  config.max_x = config.max_y = .21;
  SweptFootprint checker(map, config);
  const auto lock = map.lockForReading();
  EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));
}

TEST(SweptFootprint, DetectsCollisionBetweenClearEndpointPoses)
{
  OccupyMap map(mapConfig());
  setMeasuredStatic(map, {{1.025, .025, .225}});
  SweptFootprint checker(map, SweptFootprint::Config{});
  const auto lock = map.lockForReading();
  EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, 0));
  EXPECT_TRUE(checker.poseCollisionFree({2, 0, 0}, 0));
  SweptFootprint::Failure failure;
  EXPECT_FALSE(checker.validatePath({{0, 0, 0}, {2, 0, 0}}, 0, 0, &failure));
  EXPECT_STREQ(failure.reason, "footprint_collision");
  EXPECT_TRUE(failure.has_collision);
  EXPECT_EQ(failure.index, 1u);
}

TEST(SweptFootprint, StartAndGoalRotationSweepCornersBetweenClearOrientations)
{
  OccupyMap map(mapConfig(.02));
  setMeasuredStatic(map, {{.31, .31, .21}});
  SweptFootprint::Config config;
  config.min_x = -.5; config.max_x = .5;
  config.min_y = -.1; config.max_y = .1;
  SweptFootprint checker(map, config);
  const auto lock = map.lockForReading();
  EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, 0));
  EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, kPi / 2));
  EXPECT_FALSE(checker.validatePath({{0, 0, 0}}, 0, kPi / 2));
  EXPECT_FALSE(checker.validatePath({{0, 0, 0}, {0, 2, 0}}, 0, kPi / 2));
}

TEST(SweptFootprint, InteriorTurnCannotBypassSweepCheck)
{
  OccupyMap map(mapConfig(.02));
  setMeasuredStatic(map, {{2.31, .31, .21}});
  SweptFootprint::Config config;
  config.min_x = -.5; config.max_x = .5;
  config.min_y = -.1; config.max_y = .1;
  SweptFootprint checker(map, config);
  const auto lock = map.lockForReading();
  EXPECT_TRUE(checker.poseCollisionFree({2, 0, 0}, 0));
  EXPECT_TRUE(checker.poseCollisionFree({2, 0, 0}, kPi / 2));
  SweptFootprint::Failure failure;
  EXPECT_FALSE(checker.validatePath({{0, 0, 0}, {2, 0, 0}, {2, 2, 0}},
      0, kPi / 2, &failure));
  EXPECT_EQ(failure.index, 1u);
}

TEST(SweptFootprint, AsymmetricBodyRotatesAboutBaseOriginAndChecksBothHalfTurns)
{
  OccupyMap map(mapConfig(.02));
  setMeasuredStatic(map, {{.01, -.41, .21}});
  SweptFootprint::Config config;
  config.min_x = -.1; config.max_x = .6;
  config.min_y = -.1; config.max_y = .1;
  SweptFootprint checker(map, config);
  const auto lock = map.lockForReading();
  EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, 0));
  EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, kPi / 2));
  EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, kPi));
  // The positive arc is clear, the negative arc hits the forward overhang.
  EXPECT_FALSE(checker.validatePath({{0, 0, 0}}, 0, kPi));
  EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, -kPi / 2));
}

TEST(SweptFootprint, LowObstaclesBlockWhileSupportAndOverheadCellsDoNot)
{
  OccupyMap map(mapConfig());
  map.updateSupport({{0, 0, 0}});
  setMeasuredStatic(map, {{.025, .025, 1.025}});
  SweptFootprint checker(map, SweptFootprint::Config{});
  {const auto lock = map.lockForReading();
    EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, 0));}
  map.updateObstacles({{.025, .025, .075}}, 0, 0, 1);
  {const auto lock = map.lockForReading();
    EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));}
}

TEST(SweptFootprint, RecordedOverheadVoxelDoesNotExtendConfiguredHeight)
{
  OccupyMap map(mapConfig(.1));
  setMeasuredStatic(map, {{5.090843043513203, -.5719806119698493, .485483703285637}});
  SweptFootprint::Config body;
  body.min_x = -.41; body.max_x = .41; body.min_y = -.26; body.max_y = .26;
  SweptFootprint checker(map, body);
  const OccupyMap::Point end{4.5, -.6, -.377020239830017};
  auto begin = end; const double yaw = -.514;
  begin[0] -= .05*std::cos(yaw); begin[1] -= .05*std::sin(yaw);
  const auto lock = map.lockForReading();
  ASSERT_GT(.4, end[2] + body.robot_height);
  EXPECT_TRUE(checker.validatePath({begin, end}, yaw, yaw));
}

TEST(SweptFootprint, TopVoxelRangeUsesWorldGroundHeightIncludingNegativeCoordinates)
{
  for (double floor : {-.377, -.01, .0, .055, 2.055}) {
    for (double height : {.75, .8}) {
      OccupyMap map(mapConfig(.1));
      SweptFootprint::Config body; body.robot_height = height;
      SweptFootprint checker(map, body);
      const int top = map.posToKey(0, 0, floor + height).z;
      setMeasuredStatic(map, {{.01, .01, (top + 1.5)*.1}});
      {const auto lock = map.lockForReading();
        EXPECT_TRUE(checker.poseCollisionFree({0, 0, floor}, 0)) << floor << ',' << height;}
      setMeasuredStatic(map, {{.01, .01, (top + .5)*.1}});
      {const auto lock = map.lockForReading();
        EXPECT_FALSE(checker.poseCollisionFree({0, 0, floor}, 0)) << floor << ',' << height;}
    }
  }
}

TEST(SweptFootprint, VerticalSweepEnvelopeKeepsBetweenSampleCeilingCollision)
{
  OccupyMap map(mapConfig(.1));
  setMeasuredStatic(map, {{.15, .05, .45}});
  SweptFootprint::Config body;
  body.min_x = body.min_y = -.001; body.max_x = body.max_y = .001;
  SweptFootprint checker(map, body);
  const OccupyMap::Point begin{.18, .05, -.35041}, end{.22, .05, -.34961};
  const auto lock = map.lockForReading();
  ASSERT_TRUE(checker.poseCollisionFree(begin, 0));
  ASSERT_TRUE(checker.poseCollisionFree(end, 0));
  EXPECT_FALSE(checker.validatePath({begin, end}, 0, 0));
  EXPECT_FALSE(checker.poseCollisionFree({.2006, .05, -.349998}, 0));
}

TEST(SweptFootprint, DynamicClearingPreservesStaticBodyCollision)
{
  OccupyMap map(mapConfig());
  SweptFootprint checker(map, SweptFootprint::Config{});
  map.updateObstacles({{.375, .225, .225}}, 0, 0, 1);
  {const auto lock = map.lockForReading();
    EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));}
  map.clearDynamicObstacles();
  {const auto lock = map.lockForReading();
    EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, 0));}
  setMeasuredStatic(map, {{.375, .225, .225}});
  map.updateObstacles({{.375, .225, .225}}, 0, 0, 2);
  map.clearDynamicObstacles();
  const auto lock = map.lockForReading();
  EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));
}

TEST(SweptFootprint, InvalidInputsAndExcessiveWorkAreRejectedExplicitly)
{
  OccupyMap map(mapConfig());
  SweptFootprint::Config config;
  config.max_samples = 10;
  SweptFootprint checker(map, config);
  const auto lock = map.lockForReading();
  SweptFootprint::Failure failure;
  EXPECT_FALSE(checker.validatePath({{0, 0, 0}, {10, 0, 0}}, 0, 0, &failure));
  EXPECT_STREQ(failure.reason, "footprint_sampling_budget_exceeded");
  EXPECT_FALSE(failure.has_collision);
  EXPECT_FALSE(checker.validatePath({{0, 0, 0}},
      std::numeric_limits<double>::quiet_NaN(), 0, &failure));
  EXPECT_STREQ(failure.reason, "footprint_invalid_pose");
  EXPECT_FALSE(checker.validatePath({{1e20, 0, 0}}, 0, 0, &failure));
  EXPECT_STREQ(failure.reason, "footprint_invalid_pose");
  EXPECT_FALSE(failure.has_collision);
  config.min_x = 1e12; config.max_x = 1e12 + 1;
  EXPECT_THROW(SweptFootprint invalid(map, config), std::invalid_argument);
}
namespace
{
SweptFootprint::Config refinedBody()
{
  SweptFootprint::Config c;
  c.min_x = c.min_y = -.21; c.max_x = c.max_y = .21;
  c.near_start_static_refinement_radius = .6;
  c.near_start_center = std::array<double, 2>{0, 0};
  return c;
}
}

TEST(SweptFootprint, NearStartRefinesOnlyEmptyPartOfSavedVoxelWithMargin)
{
  OccupyMap map(mapConfig(.1));
  setMeasuredStatic(map, {{.25, .05, .25}});
  SweptFootprint refined(map, refinedBody());
  auto strict_config = refinedBody(); strict_config.near_start_static_refinement_radius = 0;
  SweptFootprint strict(map, strict_config);
  const auto read = map.lockForReading();
  EXPECT_FALSE(strict.poseCollisionFree({0, 0, 0}, 0));
  EXPECT_TRUE(refined.poseCollisionFree({0, 0, 0}, 0));
  EXPECT_FALSE(refined.poseCollisionFree({.025, 0, 0}, 0));  // retained 2cm margin
  EXPECT_FALSE(refined.poseCollisionFree({.05, 0, 0}, 0));   // saved point inside body
  EXPECT_FALSE(refined.poseCollisionFree({0, 0, 0}, kPi/4)); // actual corner collision
}

TEST(SweptFootprint, NearStartBoundsIncludeEveryPointRegardlessOfRepresentativeOrder)
{
  OccupyMap map(mapConfig(.1));
  for (const auto & points : std::vector<std::vector<OccupyMap::Point>>{
    {{.205, .05, .25}, {.29, .09, .25}},
    {{.29, .09, .25}, {.205, .05, .25}}})
  {
    setMeasuredStatic(map, points);
    SweptFootprint refined(map, refinedBody());
    const auto read = map.lockForReading();
    EXPECT_FALSE(refined.poseCollisionFree({0, 0, 0}, 0));
  }
}

TEST(SweptFootprint, LiveUnknownAndSyntheticEvidenceRetainCompleteVoxel)
{
  OccupyMap map(mapConfig(.1));
  const std::vector<OccupyMap::Point> points{{.25, .05, .25}};
  setMeasuredStatic(map, points);
  SweptFootprint refined(map, refinedBody());
  std::int64_t stamp = 1;
  for (auto source : {OccupyMap::kMeasured, OccupyMap::kUnknown, OccupyMap::kSynthetic}) {
    map.updateObstacles(points, 0, 0, stamp++, {source}, true);
    {const auto read = map.lockForReading();
      EXPECT_FALSE(refined.poseCollisionFree({0, 0, 0}, 0));}
    map.clearDynamicObstacles();
    {const auto read = map.lockForReading();
      EXPECT_TRUE(refined.poseCollisionFree({0, 0, 0}, 0));}
  }
}

TEST(SweptFootprint, RefinementDoesNotFollowSearchEdgesAwayFromOriginalStart)
{
  OccupyMap map(mapConfig(.1));
  setMeasuredStatic(map, {{2.25, .05, .25}});
  SweptFootprint refined(map, refinedBody());
  const auto read = map.lockForReading();
  EXPECT_FALSE(refined.poseCollisionFree({2, 0, 0}, 0));
  EXPECT_FALSE(refined.validatePath({{2, 0, 0}, {2, -.1, 0}}, -kPi/2, -kPi/2));
}

TEST(SweptFootprint, RefinementStillChecksWholeStartTurnAndInteriorOfMotion)
{
  OccupyMap map(mapConfig(.1));
  auto config = refinedBody(); config.min_y = -.05; config.max_y = .05;
  setMeasuredStatic(map, {{.14, .14, .25}});
  SweptFootprint turn(map, config);
  {const auto read = map.lockForReading();
    EXPECT_TRUE(turn.poseCollisionFree({0, 0, 0}, 0));
    EXPECT_TRUE(turn.poseCollisionFree({0, 0, 0}, kPi/2));
    EXPECT_FALSE(turn.validatePath({{0, 0, 0}}, 0, kPi/2));}
  setMeasuredStatic(map, {{.45, .05, .25}});
  SweptFootprint translate(map, refinedBody());
  const auto read = map.lockForReading();
  EXPECT_TRUE(translate.poseCollisionFree({0, 0, 0}, 0));
  EXPECT_TRUE(translate.poseCollisionFree({.8, 0, 0}, 0));
  EXPECT_FALSE(translate.validatePath({{0, 0, 0}, {.8, 0, 0}}, 0, 0));
}

TEST(SweptFootprint, StaticBoundsSurviveSnapshotsAndReplaceAndResetCleanly)
{
  OccupyMap map(mapConfig(.1));
  map.updateSupport({{0, 0, 0}}, 1);
  setMeasuredStatic(map, {{.25, .05, .25}});
  auto snapshot = map.planningSnapshotAfterExpiry(10, 5);
  setMeasuredStatic(map, {{.205, .05, .25}});
  SweptFootprint original(map, refinedBody()), saved(*snapshot, refinedBody());
  {const auto read = map.lockForReading();
    EXPECT_FALSE(original.poseCollisionFree({0, 0, 0}, 0));}
  {const auto read = snapshot->lockForReading();
    EXPECT_TRUE(saved.poseCollisionFree({0, 0, 0}, 0));}
  map.reset();
  const auto read = map.lockForReading();
  OccupyMap::ObstacleBounds bounds;
  EXPECT_FALSE(map.staticObstacleBoundsUnlocked(map.posToKey(.25, .05, .25), bounds));
}

TEST(SweptFootprint, InvalidRefinementCannotSilentlyDisableCollisionChecks)
{
  OccupyMap map(mapConfig());
  auto config = refinedBody(); config.near_start_static_obstacle_margin = -.01;
  EXPECT_THROW(SweptFootprint(map, config), std::invalid_argument);
  config = refinedBody(); config.near_start_static_refinement_radius = 2;
  EXPECT_THROW(SweptFootprint(map, config), std::invalid_argument);
  config = refinedBody(); (*config.near_start_center)[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(SweptFootprint(map, config), std::invalid_argument);
}

namespace
{
SweptFootprint::Config globalExtentBody()
{
  auto config = refinedBody();
  config.refine_static_obstacle_extents = true;
  config.near_start_measured_refinement_radius = .8;
  return config;
}
}

TEST(SweptFootprint, GlobalStaticExtentsRetainActualFarObstacleAndCornerChecks)
{
  OccupyMap map(mapConfig(.1));
  setMeasuredStatic(map, {{2.25, .05, .25}});
  SweptFootprint checker(map, globalExtentBody());
  const auto read = map.lockForReading();
  EXPECT_TRUE(checker.poseCollisionFree({2, 0, 0}, 0));
  EXPECT_FALSE(checker.poseCollisionFree({2.05, 0, 0}, 0));
  EXPECT_FALSE(checker.poseCollisionFree({2, 0, 0}, kPi/4));
}

TEST(SweptFootprint, MeasuredRefinementIsBoundedToActualStartAndKeepsMargin)
{
  OccupyMap map(mapConfig(.1));
  map.updateObstacles({{.25, .05, .25}, {2.25, .05, .25}}, 0, 0, 1,
    {OccupyMap::kMeasured, OccupyMap::kMeasured});
  SweptFootprint checker(map, globalExtentBody());
  const auto read = map.lockForReading();
  EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, 0));
  EXPECT_FALSE(checker.poseCollisionFree({.015, 0, 0}, 0));
  EXPECT_FALSE(checker.poseCollisionFree({2, 0, 0}, 0));
  EXPECT_FALSE(checker.validatePath({{2, 0, 0}, {2, -.2, 0}}, -kPi/2, -kPi/2));
}

TEST(SweptFootprint, MeasuredBoundsRetainAllPointsAndAllStillActiveUpdates)
{
  OccupyMap map(mapConfig(.1));
  SweptFootprint checker(map, globalExtentBody());
  map.updateObstacles({{.205, .05, .25}}, 0, 0, 1, {OccupyMap::kMeasured});
  map.updateObstacles({{.29, .09, .25}}, 0, 0, 2, {OccupyMap::kMeasured});
  {const auto read = map.lockForReading();
    EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));}
  map.updateObstacles({{.29, .09, .25}}, 0, 0, 3, {OccupyMap::kMeasured}, true);
  {const auto read = map.lockForReading();
    EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, 0));}
  map.updateObstacles({{.205, .05, .25}, {.29, .09, .25}}, 0, 0, 4,
    {OccupyMap::kMeasured, OccupyMap::kMeasured}, true);
  const auto read = map.lockForReading();
  EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));
}

TEST(SweptFootprint, MeasuredBoundsCannotOverwriteFixedPointInSameVoxel)
{
  OccupyMap map(mapConfig(.1));
  setMeasuredStatic(map, {{.205, .05, .25}});
  map.updateObstacles({{.29, .09, .25}}, 0, 0, 1, {OccupyMap::kMeasured});
  SweptFootprint checker(map, globalExtentBody());
  {const auto read = map.lockForReading();
    EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));}
  map.clearDynamicObstacles();
  const auto read = map.lockForReading();
  EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));
}

TEST(SweptFootprint, GlobalRefinementKeepsUnknownSyntheticAndMixedVoxelsConservative)
{
  OccupyMap map(mapConfig(.1));
  setMeasuredStatic(map, {{.29, .05, .25}});
  SweptFootprint checker(map, globalExtentBody());
  std::int64_t stamp = 1;
  for (auto source : {OccupyMap::kUnknown, OccupyMap::kSynthetic}) {
    map.updateObstacles({{.29, .05, .25}}, 0, 0, stamp++, {source}, true);
    map.updateObstacles({{.29, .05, .25}}, 0, 0, stamp++, {OccupyMap::kMeasured});
    const auto read = map.lockForReading();
    EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));
  }
}

TEST(SweptFootprint, MeasuredBoundsExpireAndSnapshotsKeepTheirOriginalGeneration)
{
  OccupyMap map(mapConfig(.1));
  map.updateSupport({{0, 0, 0}}, 1);
  setMeasuredStatic(map, {{.29, .05, .25}});
  map.updateObstacles({{.25, .05, .25}}, 0, 0, 10, {OccupyMap::kMeasured}, true,
    std::nullopt, {10});
  auto snapshot = map.planningSnapshotAfterExpiry(11, 5);
  map.updateObstacles({{.205, .05, .25}}, 0, 0, 12, {OccupyMap::kMeasured}, true,
    std::nullopt, {12});
  SweptFootprint current(map, globalExtentBody()), saved(*snapshot, globalExtentBody());
  {const auto read = snapshot->lockForReading();
    EXPECT_TRUE(saved.poseCollisionFree({0, 0, 0}, 0));}
  {const auto read = map.lockForReading();
    EXPECT_FALSE(current.poseCollisionFree({0, 0, 0}, 0));}
  {const auto read = map.lockForReadingAfterExpiry(20, 5);
    EXPECT_TRUE(current.poseCollisionFree({0, 0, 0}, 0));
    OccupyMap::ObstacleBounds bounds;
    EXPECT_FALSE(map.measuredObstacleBoundsUnlocked(map.posToKey(.25, .05, .25), bounds));}
}

TEST(SweptFootprint, RefinedMeasuredGeometryStillChecksIntermediateTurnsAndMotion)
{
  OccupyMap map(mapConfig(.1));
  auto config = globalExtentBody(); config.min_y = -.05; config.max_y = .05;
  map.updateObstacles({{.14, .14, .25}}, 0, 0, 1, {OccupyMap::kMeasured});
  SweptFootprint turn(map, config);
  {const auto read = map.lockForReading();
    EXPECT_TRUE(turn.poseCollisionFree({0, 0, 0}, 0));
    EXPECT_TRUE(turn.poseCollisionFree({0, 0, 0}, kPi/2));
    EXPECT_FALSE(turn.validatePath({{0, 0, 0}}, 0, kPi/2));}
  map.updateObstacles({{.45, .05, .25}}, 0, 0, 2, {OccupyMap::kMeasured}, true);
  SweptFootprint translate(map, globalExtentBody());
  const auto read = map.lockForReading();
  EXPECT_FALSE(translate.validatePath({{0, 0, 0}, {1, 0, 0}}, 0, 0));
}

TEST(SweptFootprint, InvalidGlobalAndMeasuredRefinementParametersFailClosed)
{
  OccupyMap map(mapConfig());
  auto config = globalExtentBody(); config.static_obstacle_extent_margin = 0;
  EXPECT_THROW(SweptFootprint(map, config), std::invalid_argument);
  config = globalExtentBody(); config.near_start_measured_refinement_radius = 1.1;
  EXPECT_THROW(SweptFootprint(map, config), std::invalid_argument);
  config = globalExtentBody(); config.near_start_measured_obstacle_margin = .01;
  EXPECT_THROW(SweptFootprint(map, config), std::invalid_argument);
}

TEST(SweptFootprint, GuidanceDelegatesOnlyTheUnplannedRotationArc)
{
  OccupyMap map(mapConfig(.02));
  setMeasuredStatic(map, {{.31, .31, .21}});
  SweptFootprint::Config config;
  config.min_x=-.5; config.max_x=.5; config.min_y=-.1; config.max_y=.1;
  SweptFootprint strict(map,config);
  config.check_in_place_turn_sweeps=false;
  SweptFootprint guidance(map,config);
  const auto read=map.lockForReading();
  ASSERT_TRUE(strict.poseCollisionFree({0,0,0},0));
  ASSERT_TRUE(strict.poseCollisionFree({0,0,0},kPi/2));
  EXPECT_FALSE(strict.validatePath({{0,0,0}},0,kPi/2));
  EXPECT_TRUE(guidance.validatePath({{0,0,0}},0,kPi/2));
  // This is a geometric guide, explicitly not proof of a valid turning motion.
  EXPECT_TRUE(guidance.validatePath({{0,0,0},{0,1,0}},0,kPi/2));
}

TEST(SweptFootprint, GuidanceStillRejectsActualStartGoalAndSegmentBodyCollisions)
{
  OccupyMap map(mapConfig(.05));
  auto config=globalExtentBody(); config.check_in_place_turn_sweeps=false;
  SweptFootprint guidance(map,config);
  setMeasuredStatic(map, {{.025,.025,.225}});
  {const auto read=map.lockForReading();
    EXPECT_FALSE(guidance.validatePath({{0,0,0},{2,0,0}},0,0));}
  setMeasuredStatic(map, {{2.025,.025,.225}});
  {const auto read=map.lockForReading();
    EXPECT_FALSE(guidance.validatePath({{0,0,0},{2,0,0}},0,0));}
  setMeasuredStatic(map, {{1.025,.025,.225}});
  const auto read=map.lockForReading();
  EXPECT_TRUE(guidance.poseCollisionFree({0,0,0},0));
  EXPECT_TRUE(guidance.poseCollisionFree({2,0,0},0));
  EXPECT_FALSE(guidance.validatePath({{0,0,0},{2,0,0}},0,0));
}

TEST(SweptFootprint, GuidanceKeepsBodyWidthHeightAndActualTerminalOrientation)
{
  OccupyMap map(mapConfig(.05));
  auto config=globalExtentBody(); config.check_in_place_turn_sweeps=false;
  config.min_x=-.4;config.max_x=.4;config.min_y=-.25;config.max_y=.25;
  SweptFootprint guidance(map,config);
  setMeasuredStatic(map, {{1.975,.325,.225}});
  {const auto read=map.lockForReading();
    EXPECT_TRUE(guidance.validatePath({{0,0,0},{2,0,0}},0,0));
    EXPECT_FALSE(guidance.validatePath({{0,0,0},{2,0,0}},0,kPi/2));}
  setMeasuredStatic(map, {{1.025,.225,.225}});
  const auto read=map.lockForReading();
  EXPECT_FALSE(guidance.validatePath({{0,0,0},{2,0,0}},0,0));
}

TEST(SweptFootprint, StaticRefinementRequiresPureMeasuredProvenance)
{
  OccupyMap map(mapConfig(.1));
  SweptFootprint checker(map, globalExtentBody());
  const OccupyMap::Point point{.29, .05, .25};
  for (const auto & sources : std::vector<std::vector<std::uint8_t>>{
      {}, {OccupyMap::kUnknown}, {OccupyMap::kSynthetic}})
  {
    map.setStaticObstacles({point}, 0, sources);
    const auto read = map.lockForReading();
    EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));
  }
  for (const auto source : {OccupyMap::kUnknown, OccupyMap::kSynthetic}) {
    map.setStaticObstacles({point, point}, 0, {OccupyMap::kMeasured, source});
    const auto read = map.lockForReading();
    EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));
  }
  setMeasuredStatic(map, {point});
  const auto read = map.lockForReading();
  EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, 0));
}

TEST(SweptFootprint, LiveMeasuredBoundsCannotRefineFixedUnknownSyntheticOrMixedVoxel)
{
  OccupyMap map(mapConfig(.1));
  SweptFootprint checker(map, globalExtentBody());
  const OccupyMap::Point point{.29, .05, .25};
  std::int64_t stamp = 1;
  for (const auto source : {OccupyMap::kUnknown, OccupyMap::kSynthetic}) {
    map.setStaticObstacles({point}, 0, {source});
    map.updateObstacles({point}, 0, 0, stamp++, {OccupyMap::kMeasured}, true);
    const auto read = map.lockForReading();
    OccupyMap::ObstacleBounds bounds;
    EXPECT_FALSE(map.measuredObstacleBoundsUnlocked(map.posToKey(.29, .05, .25), bounds));
    EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));
  }
  map.setStaticObstacles({point, point}, 0, {OccupyMap::kMeasured, OccupyMap::kUnknown});
  map.updateObstacles({point}, 0, 0, stamp++, {OccupyMap::kMeasured}, true);
  const auto read = map.lockForReading();
  EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));
}

TEST(SweptFootprint, RepeatedMeasuredCellExpiresOldExtremaOnlyAfterFullUpdateWindow)
{
  auto config = mapConfig(.1); config.obstacle_clear_after_updates = 3;
  OccupyMap map(config);
  SweptFootprint checker(map, globalExtentBody());
  map.updateObstacles({{.205, .05, .25}}, 0, 0, 1, {OccupyMap::kMeasured});
  for (std::int64_t stamp = 2; stamp <= 3; ++stamp) {
    map.updateObstacles({{.29, .09, .25}}, 0, 0, stamp, {OccupyMap::kMeasured});
    const auto read = map.lockForReading();
    EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));
  }
  map.updateObstacles({{.29, .09, .25}}, 0, 0, 4, {OccupyMap::kMeasured});
  const auto read = map.lockForReading();
  EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, 0));
  OccupyMap::ObstacleBounds bounds;
  ASSERT_TRUE(map.measuredObstacleBoundsUnlocked(map.posToKey(.29, .09, .25), bounds));
  EXPECT_DOUBLE_EQ(bounds.minimum[0], .29);
}

TEST(SweptFootprint, RepeatedMeasuredCellExpiresOldUnknownAndSyntheticAfterFullWindow)
{
  auto config = mapConfig(.1); config.obstacle_clear_after_updates = 3;
  OccupyMap map(config);
  SweptFootprint checker(map, globalExtentBody());
  std::int64_t stamp = 1;
  for (const auto source : {OccupyMap::kUnknown, OccupyMap::kSynthetic}) {
    map.updateObstacles({{.29, .09, .25}}, 0, 0, stamp++, {source}, true);
    for (int i = 0; i < 2; ++i) {
      map.updateObstacles({{.29, .09, .25}}, 0, 0, stamp++, {OccupyMap::kMeasured});
      const auto read = map.lockForReading();
      EXPECT_FALSE(checker.poseCollisionFree({0, 0, 0}, 0));
    }
    map.updateObstacles({{.29, .09, .25}}, 0, 0, stamp++, {OccupyMap::kMeasured});
    const auto read = map.lockForReading();
    EXPECT_TRUE(checker.poseCollisionFree({0, 0, 0}, 0));
  }
}

TEST(SweptFootprint, EmptyUpdatesAgeBoundsWithoutClearingUnobservedDistantOccupancy)
{
  auto config = mapConfig(.1);
  config.obstacle_clear_after_updates = 2;
  config.obstacle_update_radius = .1;
  OccupyMap map(config);
  auto body = globalExtentBody(); body.near_start_center = std::array<double, 2>{5, 0};
  SweptFootprint checker(map, body);
  map.updateObstacles({{5.29, .05, .25}}, 0, 0, 1, {OccupyMap::kMeasured});
  {const auto read = map.lockForReading();
    EXPECT_TRUE(checker.poseCollisionFree({5, 0, 0}, 0));}
  map.updateObstacles({}, 0, 0, 2);
  map.updateObstacles({}, 0, 0, 3);
  const auto read = map.lockForReading();
  EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(map.posToKey(5.29, .05, .25)));
  EXPECT_FALSE(checker.poseCollisionFree({5, 0, 0}, 0));
}
}  // namespace astar_3d_planner
