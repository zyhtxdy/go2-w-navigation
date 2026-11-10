#include <gtest/gtest.h>
#include <stdexcept>

#include "astar_3d_planner/astar_3d.hpp"

namespace astar_3d_planner
{
TEST(StartDiagnostics, OfflineConnectorFilterTriesAnotherObservedStartWithoutChangingDefault)
{
  OccupyMap::Config map_config;
  map_config.inflate_radius = 0;
  OccupyMap map(map_config);
  map.updateSupport({{.45, .05, .01}, {-.45, .05, .01}});
  Astar3D astar(map, Astar3D::Config{});
  const auto lock = map.lockForReading();
  VoxelKey normal, filtered;
  ASSERT_TRUE(astar.projectStartToSurface(.05, .05, .01, 1, 1, 2, .05, normal));
  EXPECT_GT(map.keyToPos(normal)[0], 0);
  Astar3D::ProjectionDiagnostics diagnostic;
  ASSERT_TRUE(astar.projectStartToSurface(.05, .05, .01, 1, 1, 2, .05, filtered,
    &diagnostic, [](const VoxelKey & key) {return key.x < 0;}));
  EXPECT_LT(map.keyToPos(filtered)[0], 0);
  EXPECT_EQ(diagnostic.footprint_rejected, 1u);
  EXPECT_FALSE(astar.projectStartToSurface(.05, .05, .01, 1, 1, 2, .05, filtered,
    &diagnostic, [](const VoxelKey &) {return false;}));
  EXPECT_STREQ(diagnostic.reason, "start_footprint_bridges_blocked");
  EXPECT_EQ(diagnostic.footprint_rejected, 2u);
}

TEST(StartDiagnostics, DistinguishesAbsentSupportFromWrongHeight)
{
  OccupyMap map(OccupyMap::Config{});
  Astar3D astar(map, Astar3D::Config{});
  VoxelKey output;
  Astar3D::ProjectionDiagnostics diagnostics;
  {
    const auto lock = map.lockForReading();
    EXPECT_FALSE(astar.projectStartToSurface(0, 0, 0, .3, .1, 1, 0, output, &diagnostics));
    EXPECT_STREQ(diagnostics.reason, "no_support_xy");
    EXPECT_FALSE(diagnostics.has_collision);
  }
  map.updateSupport({{.01, .01, 1.01}}, 100);
  const auto lock = map.lockForReading();
  EXPECT_FALSE(astar.projectStartToSurface(0, 0, 0, .3, .1, 1, 0, output, &diagnostics));
  EXPECT_STREQ(diagnostics.reason, "support_outside_height_window");
  EXPECT_EQ(diagnostics.surface.layers, 1u);
  EXPECT_EQ(diagnostics.surface.outside_height, 1u);
  ASSERT_EQ(diagnostics.surface.candidates.size(), 1u);
  EXPECT_STREQ(diagnostics.surface.candidates.front().reason, "outside_height_window");
}

TEST(StartDiagnostics, ReportsRealInflationSourceEvenWhenItsCellHasNoSupport)
{
  auto config = OccupyMap::Config{}; config.inflate_radius = .48;
  OccupyMap map(config);
  map.updateSupport({{.01, .01, -.39}}, 100);
  map.updateObstacles({{.41, .21, .21}}, 0, 0, 200, {OccupyMap::kMeasured});
  auto planner = Astar3D::Config{}; planner.robot_height = .75;
  Astar3D astar(map, planner);
  const auto lock = map.lockForReading();
  VoxelKey output;
  Astar3D::ProjectionDiagnostics diagnostics;
  EXPECT_FALSE(astar.projectStartToSurface(.01, .01, -.39, .1, .1, 1, 0, output, &diagnostics));
  EXPECT_STREQ(diagnostics.reason, "surface_clearance_blocked");
  EXPECT_TRUE(diagnostics.has_collision);
  EXPECT_EQ(diagnostics.surface.occupied, 1u);
  ASSERT_EQ(diagnostics.surface.candidates.size(), 1u);
  EXPECT_STREQ(diagnostics.surface.candidates.front().reason, "body_clearance_blocked");
  EXPECT_TRUE(diagnostics.surface.candidates.front().has_collision);
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(diagnostics.collision, evidence));
  EXPECT_EQ(evidence.stamp_ns, 200);
  EXPECT_EQ(evidence.sources, OccupyMap::kMeasured);
  EXPECT_DOUBLE_EQ(evidence.point[0], .41);
  EXPECT_EQ(map.inflationCells(), 5);
  bool contains_collision = false;
  for (const auto & point : map.inflationSnapshotUnlocked(evidence.key)) {
    contains_collision |= map.posToKey(point[0], point[1], point[2]) == diagnostics.collision;
  }
  EXPECT_TRUE(contains_collision);
}

TEST(StartDiagnostics, DistinguishesBlockedBlindBridgeFromBlockedSurface)
{
  auto config = OccupyMap::Config{}; config.inflate_radius = 0;
  OccupyMap map(config);
  map.updateSupport({{1.01, .01, -.49}}, 100);
  map.updateObstacles({{.51, .01, -.29}}, 0, 0, 100, {OccupyMap::kSynthetic});
  Astar3D astar(map, Astar3D::Config{});
  const auto lock = map.lockForReading();
  VoxelKey output;
  Astar3D::ProjectionDiagnostics diagnostics;
  EXPECT_FALSE(astar.projectStartToSurface(.01, .01, -.49, 1.5, .5, 2, 0, output, &diagnostics));
  EXPECT_STREQ(diagnostics.reason, "start_bridge_blocked");
  EXPECT_EQ(diagnostics.surface.accepted, 1u);
  EXPECT_EQ(diagnostics.surface.occupied, 0u);
  EXPECT_EQ(diagnostics.bridge_rejected, 1u);
  EXPECT_TRUE(diagnostics.has_collision);
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(diagnostics.collision, evidence));
  EXPECT_EQ(evidence.sources, OccupyMap::kSynthetic);
}

TEST(StartDiagnostics, SuccessfulProjectionIsUnchangedAndResetsPreviousFailure)
{
  OccupyMap map(OccupyMap::Config{});
  map.updateSupport({{.01, .01, -.49}, {1.01, .01, -.49}});
  Astar3D astar(map, Astar3D::Config{});
  const auto lock = map.lockForReading();
  VoxelKey without, with;
  Astar3D::ProjectionDiagnostics diagnostics;
  diagnostics.has_collision = true; diagnostics.reason = "old failure";
  ASSERT_TRUE(astar.projectStartToSurface(.01, .01, -.49, 1.5, .5, 2, 0, without));
  ASSERT_TRUE(astar.projectStartToSurface(.01, .01, -.49, 1.5, .5, 2, 0, with, &diagnostics));
  EXPECT_TRUE(with == without);
  EXPECT_STREQ(diagnostics.reason, "valid");
  EXPECT_FALSE(diagnostics.has_collision);
}

TEST(StartDiagnostics, LegacyCloudIsUnknownAndExpiredSourcesCannotBeReported)
{
  auto config = OccupyMap::Config{}; config.obstacle_clear_after_updates = 1;
  OccupyMap map(config);
  map.updateObstacles({{.01, .01, .21}}, 0, 0, 100);
  const auto cell = map.posToKey(.01, .01, .21);
  {
    const auto lock = map.lockForReading();
    OccupyMap::ObstacleEvidence evidence;
    ASSERT_TRUE(map.obstacleEvidenceUnlocked(cell, evidence));
    EXPECT_EQ(evidence.sources, OccupyMap::kUnknown);
    EXPECT_EQ(evidence.stamp_ns, 100);
    EXPECT_EQ(evidence.observed_stamp_ns, 0);
    EXPECT_FALSE(map.obstacleEvidenceUnlocked({cell.x, cell.y, cell.z + 1}, evidence));
  }
  map.updateObstacles({}, 0, 0, 200);
  const auto lock = map.lockForReading();
  OccupyMap::ObstacleEvidence evidence;
  EXPECT_FALSE(map.obstacleEvidenceUnlocked(cell, evidence));
  EXPECT_TRUE(map.inflationSnapshotUnlocked(cell).empty());
}

TEST(StartDiagnostics, DuplicateSourcesAndLateFramesAreReportedWithoutChangingOccupancy)
{
  OccupyMap map(OccupyMap::Config{});
  map.updateObstacles({{.01, .01, .21}, {.02, .02, .22}}, 0, 0, 200,
    {OccupyMap::kMeasured, OccupyMap::kSynthetic});
  map.updateObstacles({{.01, .01, .21}}, 0, 0, 100, {OccupyMap::kUnknown});
  EXPECT_THROW(map.updateObstacles({{.01, .01, .21}}, 0, 0, 300, {1, 2}), std::invalid_argument);
  const auto lock = map.lockForReading();
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(map.posToKey(.01, .01, .21), evidence));
  EXPECT_EQ(evidence.sources, OccupyMap::kMeasured | OccupyMap::kSynthetic);
  EXPECT_EQ(evidence.stamp_ns, 200);
  EXPECT_DOUBLE_EQ(evidence.point[0], .01);
  EXPECT_EQ(map.obstacleStampUnlocked(), 200);
}

TEST(StartDiagnostics, ObservationTimestampSizeMismatchDoesNotMutateOccupancy)
{
  auto config = OccupyMap::Config{}; config.inflate_radius = 0;
  OccupyMap map(config);
  map.updateObstacles({{.01, .01, .21}}, 0, 0, 100,
    {OccupyMap::kMeasured}, false, std::nullopt, {80});
  EXPECT_THROW(map.updateObstacles({{1.01, .01, .21}}, 0, 0, 200,
    {OccupyMap::kMeasured}, true, std::nullopt, {80, 90}), std::invalid_argument);
  const auto lock = map.lockForReading();
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(map.posToKey(.01, .01, .21), evidence));
  EXPECT_EQ(map.obstacleStampUnlocked(), 100);
  EXPECT_EQ(evidence.observed_stamp_ns, 80);
  EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(1.01, .01, .21)));
}

TEST(StartDiagnostics, VoxelRepresentativeKeepsNewestRealPointAndItsObservationTogether)
{
  auto config = OccupyMap::Config{}; config.inflate_radius = 0;
  OccupyMap map(config);
  // Lexically smallest point is old; provenance must follow the newer point.
  map.updateObstacles({{.01, .01, .21}, {.04, .03, .24}, {.02, .02, .22}},
    0, 0, 200, {OccupyMap::kMeasured, OccupyMap::kMeasured, OccupyMap::kSynthetic},
    false, std::nullopt, {80, 90, 0});
  const auto lock = map.lockForReading();
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(map.posToKey(.01, .01, .21), evidence));
  EXPECT_EQ(evidence.stamp_ns, 200);
  EXPECT_EQ(evidence.observed_stamp_ns, 90);
  EXPECT_DOUBLE_EQ(evidence.point[0], .04);
  EXPECT_DOUBLE_EQ(evidence.point[1], .03);
  EXPECT_DOUBLE_EQ(evidence.observed_z, .24);
  EXPECT_EQ(evidence.sources, OccupyMap::kMeasured | OccupyMap::kSynthetic);
}

TEST(StartDiagnostics, EqualObservationTimesKeepDeterministicPointAndUnknownIsNotCloudTime)
{
  auto config = OccupyMap::Config{}; config.inflate_radius = 0;
  OccupyMap map(config);
  map.updateObstacles({{.04, .03, .24}, {.01, .01, .21}},
    0, 0, 200, {}, false, std::nullopt, {90, 90});
  {
    const auto lock = map.lockForReading();
    OccupyMap::ObstacleEvidence evidence;
    ASSERT_TRUE(map.obstacleEvidenceUnlocked(map.posToKey(.01, .01, .21), evidence));
    EXPECT_EQ(evidence.observed_stamp_ns, 90);
    EXPECT_DOUBLE_EQ(evidence.point[0], .01);
    EXPECT_DOUBLE_EQ(evidence.observed_z, .21);
  }
  map.updateObstacles({{.04, .03, .24}}, 0, 0, 300);
  const auto lock = map.lockForReading();
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(map.posToKey(.01, .01, .21), evidence));
  EXPECT_EQ(evidence.stamp_ns, 300);
  EXPECT_EQ(evidence.observed_stamp_ns, 0);
}

TEST(StartDiagnostics, StaticEvidenceAndRestorationHaveNoInventedObservationTimestamp)
{
  auto config = OccupyMap::Config{}; config.inflate_radius = 0;
  config.obstacle_clear_after_updates = 1;
  OccupyMap map(config);
  map.setStaticObstacles({{.01, .01, .21}}, 10);
  map.updateObstacles({{.02, .02, .22}}, 0, 0, 100,
    {OccupyMap::kMeasured}, false, std::nullopt, {80});
  map.updateObstacles({}, 0, 0, 200);
  const auto lock = map.lockForReading();
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(map.posToKey(.01, .01, .21), evidence));
  EXPECT_EQ(evidence.sources, OccupyMap::kStaticMap);
  EXPECT_EQ(evidence.stamp_ns, 10);
  EXPECT_EQ(evidence.observed_stamp_ns, 0);
  EXPECT_DOUBLE_EQ(evidence.observed_z, .21);
}

TEST(StartDiagnostics, NewDiagnosticRepresentativeCannotChangeSyntheticNearFieldClearance)
{
  auto config = OccupyMap::Config{}; config.inflate_radius = 0;
  config.synthetic_obstacle_clearance_radius = .95;
  OccupyMap map(config);
  map.updateObstacles({{.91, .01, .21}, {.99, .02, .22}}, -2, 0, 100,
    {OccupyMap::kSynthetic, OccupyMap::kSynthetic}, false, std::nullopt, {80, 90});
  {
    const auto lock = map.lockForReading();
    OccupyMap::ObstacleEvidence evidence;
    ASSERT_TRUE(map.obstacleEvidenceUnlocked(map.posToKey(.91, .01, .21), evidence));
    EXPECT_DOUBLE_EQ(evidence.point[0], .99);
    EXPECT_EQ(evidence.observed_stamp_ns, 90);
  }
  // Original policy selects .91 for clearance and clears this synthetic cell.
  map.updateObstacles({}, 0, 0, 200);
  const auto lock = map.lockForReading();
  EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(.91, .01, .21)));
}

TEST(StartDiagnostics, OverlappingInflationKeepsOnlyActiveContributingSources)
{
  auto config = OccupyMap::Config{}; config.obstacle_clear_after_updates = 1;
  OccupyMap map(config);
  map.updateObstacles({{.01, .01, .21}, {.21, .01, .21}}, 0, 0, 100,
    {OccupyMap::kMeasured, OccupyMap::kSynthetic});
  map.updateObstacles({{.21, .01, .21}}, 0, 0, 200, {OccupyMap::kSynthetic});
  const auto lock = map.lockForReading();
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(map.posToKey(.01, .01, .21), evidence));
  EXPECT_DOUBLE_EQ(evidence.point[0], .21);
  EXPECT_EQ(evidence.sources, OccupyMap::kSynthetic);
  EXPECT_EQ(evidence.stamp_ns, 200);
}

TEST(StartDiagnostics, EvidenceFollowsHeightDatumWhilePreservingObservedHeight)
{
  auto config = OccupyMap::Config{}; config.compensate_height_drift = true;
  OccupyMap map(config);
  std::vector<OccupyMap::Point> floor;
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 10; ++y) {floor.push_back({.01 + .2 * x, .01 + .2 * y, .01});}
  }
  map.updateSupport(floor, 100);
  map.updateObstacles({{.01, .01, .51}}, 0, 0, 100, {OccupyMap::kMeasured});
  const auto key = map.posToKey(.01, .01, .51);
  for (auto & point : floor) {point[2] += .01;}
  map.updateSupport(floor, 200);
  const auto lock = map.lockForReading();
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(key, evidence));
  EXPECT_NEAR(evidence.point[2], .52, 1e-9);
  EXPECT_DOUBLE_EQ(evidence.observed_z, .51);
  EXPECT_EQ(evidence.stamp_ns, 100);
}
TEST(StartDiagnostics, NearFieldClearsOnlyPureSyntheticHistoryImmediately)
{
  auto config = OccupyMap::Config{};
  config.inflate_radius = 0; config.synthetic_obstacle_clearance_radius = .65;
  OccupyMap map(config);
  const std::vector<OccupyMap::Point> points{
    {.91, .01, .21}, {.91, .11, .21}, {.91, .21, .21}, {.91, .31, .21}, {.92, .32, .22}};
  map.updateObstacles(points, 0, 0, 100,
    {OccupyMap::kSynthetic, OccupyMap::kMeasured, OccupyMap::kUnknown,
      OccupyMap::kMeasured, OccupyMap::kSynthetic});
  map.updateObstacles({}, .51, 0, 200);
  const auto lock = map.lockForReading();
  EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(.91, .01, .21)));
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(.91, .11, .21)));
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(.91, .21, .21)));
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(.91, .31, .21)));
}

TEST(StartDiagnostics, SyntheticObservationCannotDowngradeMeasuredHistory)
{
  auto config = OccupyMap::Config{};
  config.synthetic_obstacle_clearance_radius = .65;
  OccupyMap map(config);
  map.updateObstacles({{.91, .01, .21}}, 0, 0, 100, {OccupyMap::kMeasured});
  map.updateObstacles({{.91, .01, .21}}, 0, 0, 200, {OccupyMap::kSynthetic});
  map.updateObstacles({}, .51, 0, 300);
  const auto lock = map.lockForReading();
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(map.posToKey(.91, .01, .21), evidence));
  EXPECT_EQ(evidence.sources, OccupyMap::kMeasured | OccupyMap::kSynthetic);
}

TEST(StartDiagnostics, IncomingNearSyntheticIsIgnoredButMeasuredAndLegacyRemain)
{
  auto config = OccupyMap::Config{};
  config.synthetic_obstacle_clearance_radius = .65; config.inflate_radius = 0;
  OccupyMap map(config);
  map.updateObstacles({{.11, .01, .21}, {.21, .01, .21}, {.31, .01, .21}}, 0, 0, 100,
    {OccupyMap::kSynthetic, OccupyMap::kMeasured, OccupyMap::kUnknown});
  const auto lock = map.lockForReading();
  EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(.11, .01, .21)));
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(.21, .01, .21)));
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(.31, .01, .21)));
}
TEST(StartDiagnostics, ObservedDiagonalGridConnectionNeedsRadiusAtLeastItsSpacing)
{
  auto map_config = OccupyMap::Config{}; map_config.inflate_radius = 0;
  OccupyMap map(map_config);
  map.updateSupport({{.01, .01, .01}, {.21, .21, .01}});
  auto config = Astar3D::Config{}; config.max_slope_deg = 10; config.max_step_height = .08;
  const auto lock = map.lockForReading();
  VoxelKey start, goal, reached;
  Astar3D astar(map, config);
  ASSERT_TRUE(astar.projectToSurface(.01, .01, .01, .01, .1, start));
  ASSERT_TRUE(astar.projectToSurface(.21, .21, .01, .01, .1, goal));
  std::vector<VoxelKey> path;
  Astar3D::SearchDiagnostics diagnostics;
  EXPECT_FALSE(astar.searchToAny(start, {goal}, path, reached, nullptr, &diagnostics));
  EXPECT_STREQ(diagnostics.reason, "ground_disconnected");
  // Metric surface-candidate filtering now removes this diagonal before
  // expansion instead of generating it and rejecting it as an edge later.
  EXPECT_EQ(diagnostics.outside_connection_radius, 0u);
  const auto neighbors = astar.surfaceCandidates(.05, .05, .01,
    config.surface_connection_radius, .2);
  EXPECT_EQ(std::count(neighbors.begin(), neighbors.end(), goal), 0);
  config.surface_connection_radius = .30;
  ASSERT_TRUE(Astar3D(map, config).searchToAny(start, {goal}, path, reached, nullptr, &diagnostics));
  EXPECT_STREQ(diagnostics.reason, "valid");
}

TEST(StartDiagnostics, DiagonalConnectionStillRejectsActualStepAndIntermediateObstacle)
{
  for (bool step : {false, true}) {
    auto map_config = OccupyMap::Config{}; map_config.inflate_radius = 0;
    OccupyMap map(map_config);
    map.updateSupport({{.01, .01, .01}, {.21, .21, step ? .17 : .01}});
    if (!step) {map.updateObstacles({{.11, .11, .11}}, 0, 0);}
    auto config = Astar3D::Config{};
    config.surface_connection_radius = .30; config.max_slope_deg = 10; config.max_step_height = .08;
    const auto lock = map.lockForReading();
    Astar3D astar(map, config);
    VoxelKey start, goal, reached;
    ASSERT_TRUE(astar.projectToSurface(.01, .01, .01, .01, .1, start));
    ASSERT_TRUE(astar.projectToSurface(.21, .21, step ? .17 : .01, .01, .1, goal));
    std::vector<VoxelKey> path;
    Astar3D::SearchDiagnostics diagnostics;
    EXPECT_FALSE(astar.searchToAny(start, {goal}, path, reached, nullptr, &diagnostics));
    if (step) {EXPECT_GT(diagnostics.height_rejected, 0u);} else {EXPECT_GT(diagnostics.segment_rejected, 0u);}
  }
}

TEST(StartDiagnostics, SearchLimitIsDistinguishedFromDisconnectedGround)
{
  OccupyMap map(OccupyMap::Config{});
  map.updateSupport({{.01, .01, .01}, {.21, .01, .01}, {.41, .01, .01}});
  auto config = Astar3D::Config{}; config.max_search_nodes = 1;
  Astar3D astar(map, config);
  const auto lock = map.lockForReading();
  VoxelKey start, goal, reached;
  ASSERT_TRUE(astar.projectToSurface(.01, .01, .01, .01, .1, start));
  ASSERT_TRUE(astar.projectToSurface(.41, .01, .01, .01, .1, goal));
  std::vector<VoxelKey> path;
  Astar3D::SearchDiagnostics diagnostics;
  EXPECT_FALSE(astar.searchToAny(start, {goal}, path, reached, nullptr, &diagnostics));
  EXPECT_STREQ(diagnostics.reason, "search_node_limit");
  EXPECT_EQ(diagnostics.expanded, 1u);
}

TEST(StartDiagnostics, DirectCollisionEvidencePointsToActuallyTestedInflatedCell)
{
  auto config = OccupyMap::Config{}; config.inflate_radius = .2;
  OccupyMap map(config);
  std::vector<OccupyMap::Point> floor;
  for (int i = 0; i <= 5; ++i) {floor.push_back({.01 + .2 * i, .01, .01});}
  map.updateSupport(floor);
  map.updateObstacles({{.51, .11, .21}}, 0, 0, 100, {OccupyMap::kMeasured});
  Astar3D astar(map, Astar3D::Config{});
  const auto lock = map.lockForReading();
  std::vector<OccupyMap::Point> path;
  Astar3D::TerrainValidation failure;
  EXPECT_FALSE(astar.tryDirectGroundPath({.01, .01, .01}, {1.01, .01, .01}, 2, path, &failure));
  EXPECT_STREQ(failure.reason, "occupied_body_column");
  EXPECT_TRUE(failure.has_collision);
  EXPECT_TRUE(map.occupiedUnlocked(failure.collision));
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(failure.collision, evidence));
  EXPECT_DOUBLE_EQ(evidence.point[0], .51);
}
}  // namespace astar_3d_planner
