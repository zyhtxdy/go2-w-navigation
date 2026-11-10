#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <vector>
#include "astar_3d_planner/astar_3d.hpp"

namespace astar_3d_planner
{
namespace
{
OccupyMap::Config config()
{
  OccupyMap::Config result;
  result.compensate_height_drift = true;
  result.inflate_radius = 0.3;
  result.support_radius = 0.15;
  return result;
}
std::vector<OccupyMap::Point> floor(double offset = 0.0, double slope = 0.0)
{
  std::vector<OccupyMap::Point> result;
  for (int x = 0; x <= 10; ++x) {
    for (int y = -4; y <= 4; ++y) {
      result.push_back({x * .2 + .01, y * .2 + .01, .01 + offset + slope * (x * .2 + .01)});
    }
  }
  return result;
}
double height(OccupyMap & map, double x, double y, double near_z)
{
  const auto lock = map.lockForReading();
  VoxelKey ground;
  double value = 0.0;
  EXPECT_TRUE(map.findSupportByHeightUnlocked(x, y, near_z, .4, ground));
  EXPECT_TRUE(map.supportHeightUnlocked(ground, value));
  return value;
}
}  // namespace

TEST(HeightDatum, CommonSlowDriftMovesOldGeometryWithoutLosingObstacleInflation)
{
  OccupyMap map(config());
  map.updateSupport(floor());
  map.updateSupport({{-2.01, .01, .01}});
  map.updateObstacles({{.01, .01, .51}}, 0, 0);
  const auto original_key = map.posToKey(.01, .01, .51);
  for (int i = 1; i <= 20; ++i) {map.updateSupport(floor(i * .01));}
  EXPECT_NEAR(height(map, -2.01, .01, .21), .21, 1e-8);
  EXPECT_NEAR(height(map, 1.01, .01, .21), .21, 1e-8);
  const auto lock = map.lockForReading();
  EXPECT_NEAR(map.heightDatumOffsetUnlocked(), .2, 1e-8);
  EXPECT_TRUE(map.posToKey(.01, .01, .71) == original_key);
  EXPECT_TRUE(map.occupiedUnlocked(original_key));
  EXPECT_TRUE(map.occupiedUnlocked({original_key.x + 2, original_key.y, original_key.z}));
  EXPECT_EQ(map.supportSnapshotUnlocked().size(), 100u);
}

TEST(HeightDatum, CommonTranslationPreservesSixDegreeRampAndRejectsRealStep)
{
  const double slope = std::tan(6.0 * 3.141592653589793 / 180.0);
  for (bool step : {false, true}) {
    OccupyMap map(config());
    auto initial = floor(0, step ? 0 : slope);
    if (step) {for (auto & p : initial) {if (p[0] >= 1.0) {p[2] += .16;}}}
    map.updateSupport(initial);
    for (int i = 1; i <= 20; ++i) {
      auto translated = initial;
      for (auto & p : translated) {p[2] += .01 * i;}
      map.updateSupport(translated);
    }
    const auto lock = map.lockForReading();
    Astar3D::Config planner;
    planner.max_slope_deg = 10;
    planner.max_step_height = .08;
    planner.robot_height = .75;
    Astar3D astar(map, planner);
    VoxelKey start, goal;
    ASSERT_TRUE(astar.projectToSurface(.01, .01, initial.front()[2] + .2, .11, .4, start));
    const double end_height = .21 + (step ? .16 : 2.01 * slope);
    ASSERT_TRUE(astar.projectToSurface(2.01, .01, end_height, .11, .4, goal));
    EXPECT_EQ(astar.segmentTerrainTraversable(start, goal), !step);
    EXPECT_NEAR(map.heightDatumOffsetUnlocked(), .2, 1e-8);
  }
}

TEST(HeightDatum, DistinctStoreysRetainTheirSeparation)
{
  OccupyMap map(config());
  auto levels = floor();
  auto upper = floor(1.0);levels.insert(levels.end(), upper.begin(), upper.end());
  map.updateSupport(levels);
  for (int i = 1; i <= 12; ++i) {
    auto points = levels;for (auto & p : points) {p[2] += .01 * i;}
    map.updateSupport(points);
  }
  EXPECT_NEAR(height(map, .01, .01, .13), .13, 1e-8);
  EXPECT_NEAR(height(map, .01, .01, 1.13), 1.13, 1e-8);
}

TEST(HeightDatum, LocalChangeInsufficientOverlapAndLargeJumpDoNotMoveWholeMap)
{
  for (int scenario : {0, 1, 2}) {
    OccupyMap map(config());map.updateSupport(floor());map.updateSupport({{-2.01, .01, .01}});
    auto points = floor();
    if (scenario == 0) {for (int i = 0; i < 12; ++i) {points[i][2] += .01;}}
    if (scenario == 1) {points.resize(12);for (auto & p : points) {p[2] += .01;}}
    if (scenario == 2) {for (auto & p : points) {p[2] += .16;}}
    map.updateSupport(points);
    EXPECT_NEAR(height(map, -2.01, .01, .01), .01, 1e-9);
    const auto lock = map.lockForReading();EXPECT_DOUBLE_EQ(map.heightDatumOffsetUnlocked(), 0.0);
  }
}

TEST(HeightDatum, IndependentSmallNoiseDoesNotAccumulateUnboundedOffset)
{
  OccupyMap map(config());map.updateSupport(floor());map.updateSupport({{-2.01, .01, .01}});
  std::mt19937 random(20261013);std::uniform_real_distribution<double> noise(-.004, .004);
  for (int i = 0; i < 100; ++i) {
    auto points = floor();for (auto & p : points) {p[2] += noise(random);}
    map.updateSupport(points);
  }
  const auto lock = map.lockForReading();EXPECT_LT(std::abs(map.heightDatumOffsetUnlocked()), .01);
}

TEST(HeightDatum, OutOfOrderMessagesCannotMoveDatumBackAndResetClearsIt)
{
  OccupyMap map(config());map.updateSupport(floor(), 100);
  map.updateObstacles({{.01, .01, .51}}, 0, 0, 100);
  map.updateSupport(floor(.01), 200);
  map.updateSupport(floor(), 150);
  map.updateObstacles({}, 0, 0, 150);
  EXPECT_NEAR(height(map, .01, .01, .02), .02, 1e-8);
  map.reset();map.updateSupport(floor(), 100);
  const auto lock = map.lockForReading();EXPECT_DOUBLE_EQ(map.heightDatumOffsetUnlocked(), 0.0);
}

TEST(HeightDatum, CompensationRequiresTwoDimensionalIndependentOverlap)
{
  OccupyMap map(config());std::vector<OccupyMap::Point> line;
  for (int i = 0; i < 60; ++i) {line.push_back({i * .2 + .01, .01, .01});}
  map.updateSupport(line);
  for (auto & p : line) {p[2] += .01;}
  map.updateSupport(line);
  const auto lock = map.lockForReading();EXPECT_DOUBLE_EQ(map.heightDatumOffsetUnlocked(), 0.0);
}

TEST(HeightDatum, InvalidOrExcessiveCorrectionSettingsAreRejected)
{
  auto cfg = config();cfg.height_drift_max_update = .16;
  EXPECT_THROW(OccupyMap map(cfg), std::invalid_argument);
  cfg = config();cfg.height_drift_min_consensus = .1;
  EXPECT_THROW(OccupyMap map(cfg), std::invalid_argument);
}
}  // namespace astar_3d_planner
