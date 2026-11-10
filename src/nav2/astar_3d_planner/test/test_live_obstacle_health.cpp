#include <gtest/gtest.h>
#include <atomic>
#include <thread>
#include "astar_3d_planner/live_obstacle_health.hpp"
#include "astar_3d_planner/occupy_map.hpp"

namespace astar_3d_planner
{
TEST(LiveObstacleHealth, ConcurrentConsumptionCannotLookNewerThanTheAtomicClockSnapshot)
{
  LiveObstacleHealth health;
  const auto steady_now = []() {return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();};
  constexpr std::int64_t ros_now = 1000000000000LL;
  health.consumed(ros_now - 20000, steady_now());
  std::atomic<bool> start{false};
  std::thread writer([&]() {
      while (!start.load()) {std::this_thread::yield();}
      for (int i = 19999; i >= 0; --i) {health.consumed(ros_now - i, steady_now());}
    });
  start.store(true);
  for (int i = 0; i < 20000; ++i) {
    const auto state = health.statusNow([]() {return ros_now;});
    EXPECT_TRUE(state.ready);
    EXPECT_GE(state.checked_steady_ns, state.processed_steady_ns);
  }
  writer.join();
}

TEST(LiveObstacleHealth, StartupFailsClosedAndNewEmptySnapshotCountsAsSuccessfulConsumption)
{
  LiveObstacleHealth health;
  EXPECT_FALSE(health.status(10000000000LL, 20000000000LL).ready);
  OccupyMap map(OccupyMap::Config{});
  ASSERT_TRUE(map.updateObstacles({}, 0, 0, 10000000000LL, {}, true));
  ASSERT_TRUE(health.consumed(10000000000LL, 20000000000LL));
  EXPECT_TRUE(health.status(10000000000LL, 20000000000LL).ready);
  EXPECT_EQ(health.status(10000000000LL, 20000000000LL).updates, 1u);
}

TEST(LiveObstacleHealth, BothSensorAgeAndSteadyConsumptionAgeAreRequired)
{
  LiveObstacleHealth health;
  health.consumed(10000000000LL, 20000000000LL);
  EXPECT_TRUE(health.status(11000000000LL, 21000000000LL).ready);
  EXPECT_FALSE(health.status(11000000001LL, 21000000000LL).ready);
  EXPECT_FALSE(health.status(10000000000LL, 21000000001LL).ready);  // Frozen ROS clock.
  EXPECT_FALSE(health.status(9999999999LL, 20000000000LL).ready);
  EXPECT_FALSE(health.status(10000000000LL, 19999999999LL).ready);
}

TEST(LiveObstacleHealth, DuplicateOrOldDataCannotRenewHealthOrOccupancy)
{
  LiveObstacleHealth health;
  OccupyMap map(OccupyMap::Config{});
  ASSERT_TRUE(map.updateObstacles({{1, 0, .5}}, 0, 0, 10000000000LL, {}, true));
  health.consumed(10000000000LL, 20000000000LL);
  EXPECT_FALSE(map.updateObstacles({}, 0, 0, 10000000000LL, {}, true));
  EXPECT_FALSE(map.updateObstacles({}, 0, 0, 9999999999LL, {}, true));
  EXPECT_FALSE(health.consumed(10000000000LL, 20900000000LL));
  EXPECT_FALSE(health.consumed(9999999999LL, 20900000000LL));
  EXPECT_FALSE(health.status(10000000000LL, 21000000001LL).ready);
  const auto read = map.lockForReading();
  EXPECT_GT(map.rawVoxelCountUnlocked(), 0u);  // Timeout never pretends the scene is empty.
}

TEST(LiveObstacleHealth, RejectedEpochCannotBecomeAConsumptionHeartbeat)
{
  LiveObstacleHealth health;
  OccupyMap map(OccupyMap::Config{});
  const auto epoch = map.dynamicObstacleEpoch();
  map.clearDynamicObstacles(true);
  EXPECT_FALSE(map.updateObstacles({}, 0, 0, 100, {}, true, epoch));
  EXPECT_FALSE(health.status(100, 200).ready);
  ASSERT_TRUE(map.updateObstacles({}, 0, 0, 100, {}, true, map.dynamicObstacleEpoch()));
  health.consumed(100, 200);
  EXPECT_TRUE(health.status(100, 200).ready);
}

TEST(LiveObstacleHealth, ResetRequiresNewConsumptionAndPermitsARewoundEpoch)
{
  LiveObstacleHealth health;
  health.consumed(100, 200);
  health.reset();
  EXPECT_FALSE(health.status(50, 300).ready);
  EXPECT_TRUE(health.consumed(50, 300));
  EXPECT_TRUE(health.status(50, 300).ready);
  EXPECT_FALSE(health.consumed(0, 400));
  EXPECT_FALSE(health.consumed(60, 0));
}
}  // namespace astar_3d_planner

namespace astar_3d_planner
{
TEST(LiveObstacleHealth, TwoSecondBudgetAcceptsDelayedButExpiringConsumption)
{
  LiveObstacleHealth health(2.0);
  health.consumed(10000000000LL, 20000000000LL);
  EXPECT_TRUE(health.status(11500000000LL, 21500000000LL).ready);
  EXPECT_TRUE(health.status(12000000000LL, 22000000000LL).ready);
  EXPECT_FALSE(health.status(12000000001LL, 22000000000LL).ready);
  EXPECT_FALSE(health.status(10000000000LL, 22000000001LL).ready);
  EXPECT_FALSE(health.consumed(10000000000LL, 22000000000LL));
  EXPECT_FALSE(health.status(10000000000LL, 22000000001LL).ready);
  health.setTimeout(1.0);
  EXPECT_FALSE(health.status(11500000000LL, 21500000000LL).ready);
}
TEST(LiveObstacleHealth, InvalidTimeoutDoesNotReplaceExistingBudget)
{
  LiveObstacleHealth health(2.0);
  health.consumed(10000000000LL, 20000000000LL);
  EXPECT_THROW(health.setTimeout(0.0), std::invalid_argument);
  EXPECT_THROW(health.setTimeout(-1.0), std::invalid_argument);
  EXPECT_THROW(health.setTimeout(INFINITY), std::invalid_argument);
  EXPECT_THROW(health.setTimeout(NAN), std::invalid_argument);
  EXPECT_TRUE(health.status(11500000000LL, 21500000000LL).ready);
}
}
