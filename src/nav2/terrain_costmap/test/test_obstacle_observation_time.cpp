#include "terrain_costmap/terrain_hazard_stabilizer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace
{

using terrain_costmap::freshMeasuredObstacles;
using terrain_costmap::HazardStabilizerParameters;
using terrain_costmap::TerrainHazardStabilizer;
using terrain_costmap::TerrainPoint;

TerrainPoint measured(std::int64_t stamp, double x = 1.0)
{
  return {{x, 0.0, 0.5, stamp}, 0.4, false};
}

TerrainPoint synthetic(double x = 1.0)
{
  return {{x, 0.0, 0.5, 0}, 0.4, true};
}

TEST(ObstacleObservationTime, FreshWindowUsesOriginalStampAndExactNanosecondBoundary)
{
  constexpr std::int64_t now = 1791625420000000000LL;
  constexpr std::int64_t age = 600000000LL;
  const std::vector<TerrainPoint> observations{
    measured(now, 1.0),
    measured(now - age, 2.0),
    measured(now - age - 1, 3.0),
    measured(now + 1, 4.0),
    measured(0, 5.0),
    measured(-1, 6.0),
    synthetic(7.0)};

  const auto fresh = freshMeasuredObstacles(observations, now, age);
  ASSERT_EQ(fresh.size(), 2u);
  EXPECT_EQ(fresh[0].point.x, 1.0);
  EXPECT_EQ(fresh[0].point.observation_stamp_ns, now);
  EXPECT_EQ(fresh[1].point.x, 2.0);
  EXPECT_EQ(fresh[1].point.observation_stamp_ns, now - age);
  EXPECT_EQ(fresh[1].relative_height, 0.4);
  EXPECT_FALSE(fresh[1].synthetic);

  // Reclassifying the same history in a later frame must not renew its age.
  EXPECT_TRUE(freshMeasuredObstacles(observations, now + age + 2, age).empty());
}

TEST(ObstacleObservationTime, FreshWindowPreservesEveryFreshMeasuredReturn)
{
  const std::vector<TerrainPoint> observations{
    measured(100, 1.01), measured(110, 1.02), measured(100, 1.03)};
  const auto fresh = freshMeasuredObstacles(observations, 110, 20);
  ASSERT_EQ(fresh.size(), 3u);
  EXPECT_EQ(fresh[0].point.x, 1.01);
  EXPECT_EQ(fresh[1].point.x, 1.02);
  EXPECT_EQ(fresh[2].point.x, 1.03);
  // This filter is an age filter, not a new entry gate or a voxel subsampler.
  EXPECT_EQ(freshMeasuredObstacles({measured(110)}, 110, 20).size(), 1u);
}

TEST(ObstacleObservationTime, HistoricalCopiesCannotSupplyIndependentEntryObservations)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 2;
  parameters.exit_missed_observations = 3;
  TerrainHazardStabilizer stabilizer(parameters);

  for (std::int64_t frame = 100; frame <= 200; frame += 10) {
    EXPECT_TRUE(stabilizer.update({measured(100)}, {}, 0.0, 0.0, frame).empty());
  }
}

TEST(ObstacleObservationTime, SamePositionInTwoActualFramesSuppliesIndependentEvidence)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 2;
  TerrainHazardStabilizer stabilizer(parameters);

  EXPECT_TRUE(stabilizer.update({measured(100)}, {}, 0.0, 0.0, 100).empty());
  const auto active = stabilizer.update({measured(110)}, {}, 0.0, 0.0, 110);
  ASSERT_EQ(active.size(), 1u);
  EXPECT_EQ(active.front().point.observation_stamp_ns, 110);
}

TEST(ObstacleObservationTime, ManyReturnsInOneVoxelAndFrameCountOnlyOnce)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 2;
  TerrainHazardStabilizer stabilizer(parameters);
  const std::vector<TerrainPoint> repeated{
    measured(100, 1.00), measured(100, 1.01), measured(100, 1.02)};

  EXPECT_TRUE(stabilizer.update(repeated, {}, 0.0, 0.0, 100).empty());
  const auto active = stabilizer.update({measured(110)}, {}, 0.0, 0.0, 110);
  ASSERT_EQ(active.size(), 1u);
  EXPECT_EQ(active.front().point.observation_stamp_ns, 110);
}

TEST(ObstacleObservationTime, VoxelRepresentativeUsesNewestActualObservationNotVectorOrder)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 1;
  TerrainHazardStabilizer stabilizer(parameters);
  const std::vector<TerrainPoint> shuffled{
    measured(80, 1.01), measured(100, 1.02), measured(90, 1.03)};

  const auto active = stabilizer.update(shuffled, {}, 0.0, 0.0, 110);
  ASSERT_EQ(active.size(), 1u);
  EXPECT_EQ(active.front().point.observation_stamp_ns, 100);
  EXPECT_EQ(active.front().point.x, 1.02);

  const auto newer = stabilizer.update(
    {measured(110, 1.04), measured(105, 1.05)}, {}, 0.0, 0.0, 120);
  ASSERT_EQ(newer.size(), 1u);
  EXPECT_EQ(newer.front().point.observation_stamp_ns, 110);
  EXPECT_EQ(newer.front().point.x, 1.04);
}

TEST(ObstacleObservationTime, DuplicateOrOlderUpdateFrameIsFullyIdempotent)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 1;
  parameters.exit_missed_observations = 3;
  TerrainHazardStabilizer stabilizer(parameters);
  ASSERT_EQ(stabilizer.update({measured(100)}, {}, 0.0, 0.0, 100).size(), 1u);

  for (int repeat = 0; repeat < 10; ++repeat) {
    const auto duplicate = stabilizer.update({}, {}, 0.0, 0.0, 100);
    ASSERT_EQ(duplicate.size(), 1u);
    EXPECT_EQ(duplicate.front().point.observation_stamp_ns, 100);
  }
  const auto older = stabilizer.update({measured(99)}, {}, 0.0, 0.0, 99);
  ASSERT_EQ(older.size(), 1u);
  EXPECT_EQ(older.front().point.observation_stamp_ns, 100);
  EXPECT_EQ(stabilizer.update({}, {}, 0.0, 0.0, 110).size(), 1u);
  EXPECT_EQ(stabilizer.update({}, {}, 0.0, 0.0, 120).size(), 1u);
  EXPECT_TRUE(stabilizer.update({}, {}, 0.0, 0.0, 130).empty());
}

TEST(ObstacleObservationTime, HistoricalCopiesDoNotResetMissesOrRenewMeasuredStamp)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 1;
  parameters.exit_missed_observations = 2;
  TerrainHazardStabilizer stabilizer(parameters);
  ASSERT_EQ(stabilizer.update({measured(100)}, {}, 0.0, 0.0, 100).size(), 1u);

  const auto held = stabilizer.update({measured(100)}, {}, 0.0, 0.0, 110);
  ASSERT_EQ(held.size(), 1u);
  EXPECT_EQ(held.front().point.observation_stamp_ns, 100);
  EXPECT_TRUE(stabilizer.update({measured(100)}, {}, 0.0, 0.0, 120).empty());
}

TEST(ObstacleObservationTime, ExpiredMeasuredEvidenceCannotBeBornAgainFromHistoricalCopies)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 1;
  parameters.exit_missed_observations = 2;
  TerrainHazardStabilizer stabilizer(parameters);
  ASSERT_EQ(stabilizer.update({measured(100)}, {}, 0.0, 0.0, 100).size(), 1u);
  EXPECT_EQ(stabilizer.update({}, {}, 0.0, 0.0, 110).size(), 1u);
  ASSERT_TRUE(stabilizer.update({}, {}, 0.0, 0.0, 120).empty());

  for (std::int64_t frame = 130; frame <= 200; frame += 10) {
    EXPECT_TRUE(stabilizer.update({measured(100)}, {}, 0.0, 0.0, frame).empty());
  }
  const auto active = stabilizer.update({measured(210)}, {}, 0.0, 0.0, 210);
  ASSERT_EQ(active.size(), 1u);
  EXPECT_EQ(active.front().point.observation_stamp_ns, 210);
}

TEST(ObstacleObservationTime, FutureAndMissingMeasuredStampsCannotActivatePositiveTimeStream)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 1;
  TerrainHazardStabilizer stabilizer(parameters);

  EXPECT_TRUE(stabilizer.update(
    {measured(101), measured(0), measured(-1)}, {}, 0.0, 0.0, 100).empty());
  EXPECT_EQ(stabilizer.trackedVoxelCount(), 0u);
  EXPECT_EQ(stabilizer.update({measured(110)}, {}, 0.0, 0.0, 110).size(), 1u);
}

TEST(ObstacleObservationTime, SyntheticHistoryCannotPayMeasuredEntryThreshold)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 3;
  parameters.synthetic_enter_observations = 3;
  TerrainHazardStabilizer stabilizer(parameters);

  EXPECT_TRUE(stabilizer.update({}, {synthetic()}, 0.0, 0.0, 100).empty());
  EXPECT_TRUE(stabilizer.update({}, {synthetic()}, 0.0, 0.0, 110).empty());
  EXPECT_TRUE(stabilizer.update({measured(120)}, {}, 0.0, 0.0, 120).empty());
  EXPECT_TRUE(stabilizer.update({measured(130)}, {}, 0.0, 0.0, 130).empty());
  const auto active = stabilizer.update({measured(140)}, {}, 0.0, 0.0, 140);
  ASSERT_EQ(active.size(), 1u);
  EXPECT_FALSE(active.front().synthetic);
  EXPECT_EQ(active.front().point.observation_stamp_ns, 140);
}

TEST(ObstacleObservationTime, SyntheticObservationCannotRenewRetainedMeasuredEvidence)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 1;
  parameters.synthetic_enter_observations = 1;
  parameters.exit_missed_observations = 2;
  TerrainHazardStabilizer stabilizer(parameters);
  ASSERT_EQ(stabilizer.update({measured(100)}, {}, 0.0, 0.0, 100).size(), 1u);

  const auto held = stabilizer.update({}, {synthetic()}, 0.0, 0.0, 110);
  ASSERT_EQ(held.size(), 1u);
  EXPECT_FALSE(held.front().synthetic);
  EXPECT_EQ(held.front().point.observation_stamp_ns, 100);
  const auto expired = stabilizer.update({}, {synthetic()}, 0.0, 0.0, 120);
  EXPECT_TRUE(std::none_of(expired.begin(), expired.end(), [](const TerrainPoint & point) {
    return !point.synthetic;
  }));
}

TEST(ObstacleObservationTime, ExpiredMeasuredWatermarkDoesNotHideNewSyntheticHazard)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 1;
  parameters.synthetic_enter_observations = 2;
  parameters.exit_missed_observations = 2;
  TerrainHazardStabilizer stabilizer(parameters);
  ASSERT_EQ(stabilizer.update({measured(100)}, {}, 0.0, 0.0, 100).size(), 1u);
  EXPECT_EQ(stabilizer.update({}, {}, 0.0, 0.0, 110).size(), 1u);
  ASSERT_TRUE(stabilizer.update({}, {}, 0.0, 0.0, 120).empty());

  // An old measured copy cannot mask the new synthetic observation, or pay
  // its entry threshold. The first synthetic frame starts its own count.
  EXPECT_TRUE(stabilizer.update(
    {measured(100)}, {synthetic()}, 0.0, 0.0, 130).empty());
  const auto active = stabilizer.update(
    {measured(100)}, {synthetic()}, 0.0, 0.0, 140);
  ASSERT_EQ(active.size(), 1u);
  EXPECT_TRUE(active.front().synthetic);

  // Transitioning the retained state to synthetic must not discard the
  // consumed-measurement watermark and allow that old return to upgrade it.
  const auto retained = stabilizer.update(
    {measured(100)}, {synthetic()}, 0.0, 0.0, 150);
  ASSERT_EQ(retained.size(), 1u);
  EXPECT_TRUE(retained.front().synthetic);
  const auto fresh = stabilizer.update({measured(160)}, {}, 0.0, 0.0, 160);
  ASSERT_EQ(fresh.size(), 1u);
  EXPECT_FALSE(fresh.front().synthetic);
  EXPECT_EQ(fresh.front().point.observation_stamp_ns, 160);
}

TEST(ObstacleObservationTime, ActiveSyntheticUpgradingToMeasuredNeverDropsCollisionOccupancy)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 3;
  parameters.synthetic_enter_observations = 1;
  TerrainHazardStabilizer stabilizer(parameters);
  const auto initial = stabilizer.update({}, {synthetic()}, 0.0, 0.0, 100);
  ASSERT_EQ(initial.size(), 1u);
  EXPECT_TRUE(initial.front().synthetic);

  for (std::int64_t frame = 110; frame <= 130; frame += 10) {
    const auto upgraded = stabilizer.update({measured(frame)}, {}, 0.0, 0.0, frame);
    ASSERT_EQ(upgraded.size(), 1u);
    EXPECT_FALSE(upgraded.front().synthetic);
    EXPECT_EQ(upgraded.front().point.observation_stamp_ns, frame);
  }
}

TEST(ObstacleObservationTime, SyntheticEntryStillRequiresDistinctUpdateFrames)
{
  HazardStabilizerParameters parameters;
  parameters.synthetic_enter_observations = 3;
  TerrainHazardStabilizer stabilizer(parameters);

  EXPECT_TRUE(stabilizer.update({}, {synthetic()}, 0.0, 0.0, 100).empty());
  for (int repeat = 0; repeat < 5; ++repeat) {
    EXPECT_TRUE(stabilizer.update({}, {synthetic()}, 0.0, 0.0, 100).empty());
  }
  EXPECT_TRUE(stabilizer.update({}, {synthetic()}, 0.0, 0.0, 110).empty());
  const auto active = stabilizer.update({}, {synthetic()}, 0.0, 0.0, 120);
  ASSERT_EQ(active.size(), 1u);
  EXPECT_TRUE(active.front().synthetic);
}

TEST(ObstacleObservationTime, ClearDropsOccupancyAndAllowsEarlierEpoch)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 2;
  TerrainHazardStabilizer stabilizer(parameters);
  EXPECT_TRUE(stabilizer.update({measured(100)}, {}, 0.0, 0.0, 100).empty());
  ASSERT_EQ(stabilizer.update({measured(110)}, {}, 0.0, 0.0, 110).size(), 1u);

  stabilizer.clear();
  EXPECT_EQ(stabilizer.trackedVoxelCount(), 0u);
  EXPECT_TRUE(stabilizer.update({measured(20)}, {}, 0.0, 0.0, 20).empty());
  const auto active = stabilizer.update({measured(30)}, {}, 0.0, 0.0, 30);
  ASSERT_EQ(active.size(), 1u);
  EXPECT_EQ(active.front().point.observation_stamp_ns, 30);
}

TEST(ObstacleObservationTime, SyntheticNearFieldPolicyNeverSuppressesFreshMeasuredObstacle)
{
  HazardStabilizerParameters parameters;
  parameters.synthetic_clearance_radius = 0.65;
  parameters.measured_enter_observations = 1;
  parameters.synthetic_enter_observations = 1;
  TerrainHazardStabilizer stabilizer(parameters);

  const auto active = stabilizer.update(
    {measured(100, 0.5)}, {synthetic(0.5)}, 0.0, 0.0, 100);
  ASSERT_EQ(active.size(), 1u);
  EXPECT_FALSE(active.front().synthetic);
  EXPECT_EQ(active.front().point.observation_stamp_ns, 100);
}

}  // namespace
