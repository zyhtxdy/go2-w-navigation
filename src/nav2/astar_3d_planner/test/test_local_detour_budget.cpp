#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "astar_3d_planner/local_detour_budget.hpp"

namespace astar_3d_planner
{
namespace
{
using State = LocalDetourBudget::State;
const std::array<double, 4> goal{5.0, 0.0, 0.0, 0.0};
const std::array<double, 3> origin{0.0, 0.0, 0.0};
}

TEST(LocalDetourBudget, ClearReferenceDoesNotSpendTheBudget)
{
  LocalDetourBudget budget;
  EXPECT_EQ(budget.update(goal, 0.0, origin, false).state, State::Clear);
  EXPECT_EQ(budget.update(goal, 100.0, origin, false).state, State::Clear);
  const auto result = budget.update(goal, 101.0, origin, true);
  EXPECT_TRUE(result.allowed());
  EXPECT_EQ(result.state, State::Allowed);
  EXPECT_STREQ(result.reason(), "local_detour_active");
}

TEST(LocalDetourBudget, StationaryRobotExhaustsProgressBudgetAndCannotRenewIt)
{
  LocalDetourBudget budget;
  EXPECT_TRUE(budget.update(goal, 10.0, origin, true).allowed());
  EXPECT_TRUE(budget.update(goal, 13.99, origin, true).allowed());
  const auto result = budget.update(goal, 14.0, origin, true);
  EXPECT_EQ(result.state, State::Stalled);
  EXPECT_STREQ(result.reason(), "local_detour_no_progress");
  EXPECT_FALSE(budget.update(goal, 14.1, {1.0, 0.0, 0.0}, true).allowed());
}

TEST(LocalDetourBudget, ContinuousMovementRenewsProgressButNotTotalDuration)
{
  LocalDetourBudget budget;
  for (int second = 0; second < 12; ++second) {
    EXPECT_TRUE(budget.update(goal, second, {0.1*second, 0.0, 0.0}, true).allowed());
  }
  const auto result = budget.update(goal, 12.0, {1.2, 0.0, 0.0}, true);
  EXPECT_EQ(result.state, State::Expired);
  EXPECT_STREQ(result.reason(), "local_detour_timeout");
  EXPECT_FALSE(budget.update(goal, 13.0, {1.3, 0.0, 0.0}, true).allowed());
}

TEST(LocalDetourBudget, SmallMovementsAccumulateBeforeCountingAsProgress)
{
  LocalDetourBudget budget;
  EXPECT_TRUE(budget.update(goal, 0.0, origin, true).allowed());
  EXPECT_TRUE(budget.update(goal, 1.0, {0.02, 0.0, 0.0}, true).allowed());
  EXPECT_TRUE(budget.update(goal, 2.0, {0.04, 0.0, 0.0}, true).allowed());
  EXPECT_TRUE(budget.update(goal, 3.0, {0.06, 0.0, 0.0}, true).allowed());
  EXPECT_TRUE(budget.update(goal, 6.9, {0.06, 0.0, 0.0}, true).allowed());
  EXPECT_EQ(budget.update(goal, 7.0, {0.06, 0.0, 0.0}, true).state, State::Stalled);
}

TEST(LocalDetourBudget, RotationCountsAsProgressAcrossTheAngleWrap)
{
  LocalDetourBudget budget;
  constexpr double pi = 3.14159265358979323846;
  EXPECT_TRUE(budget.update(goal, 0.0, {0.0, 0.0, pi-0.1}, true).allowed());
  EXPECT_TRUE(budget.update(goal, 3.0, {0.0, 0.0, -pi+0.15}, true).allowed());
  EXPECT_TRUE(budget.update(goal, 6.9, {0.0, 0.0, -pi+0.15}, true).allowed());
  EXPECT_EQ(budget.update(goal, 7.0, {0.0, 0.0, -pi+0.15}, true).state, State::Stalled);
}

TEST(LocalDetourBudget, SameGoalObservationsDoNotResetAnExpiredEpisode)
{
  LocalDetourBudget budget;
  for (int second = 0; second < 12; ++second) {
    // There is deliberately no path stamp/size argument to renew an episode.
    auto same_goal = goal;
    same_goal[0] += second % 2 ? 0.005 : -0.005;
    EXPECT_TRUE(budget.update(same_goal, second, {0.1*second, 0.0, 0.0}, true).allowed());
  }
  EXPECT_EQ(budget.update(goal, 12.0, {1.2, 0.0, 0.0}, true).state, State::Expired);
}

TEST(LocalDetourBudget, ValidClearAndDifferentGoalReleaseLatchedFailures)
{
  LocalDetourBudget budget;
  budget.update(goal, 0.0, origin, true);
  EXPECT_EQ(budget.update(goal, 4.0, origin, true).state, State::Stalled);
  EXPECT_EQ(budget.update(goal, 5.0, origin, false).state, State::Clear);
  EXPECT_TRUE(budget.update(goal, 6.0, origin, true).allowed());
  EXPECT_EQ(budget.update(goal, 10.0, origin, true).state, State::Stalled);
  auto next_goal = goal; next_goal[0] += 1.0;
  EXPECT_TRUE(budget.update(next_goal, 11.0, origin, true).allowed());
  budget.reset();
  EXPECT_TRUE(budget.update(goal, 0.0, origin, true).allowed());
}

TEST(LocalDetourBudget, InvalidInputsFailClosedUntilExplicitRecovery)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int mutation = 0; mutation < 4; ++mutation) {
    LocalDetourBudget budget;
    budget.update(goal, 10.0, origin, true);
    auto bad_goal = goal; auto bad_pose = origin; double stamp = 11.0;
    if (mutation == 0) {stamp = nan;}
    if (mutation == 1) {stamp = 9.0;}
    if (mutation == 2) {bad_goal[0] = nan;}
    if (mutation == 3) {bad_pose[2] = nan;}
    EXPECT_EQ(budget.update(bad_goal, stamp, bad_pose, false).state, State::Invalid);
    EXPECT_FALSE(budget.update(goal, 12.0, origin, true).allowed());
    EXPECT_EQ(budget.update(goal, 13.0, origin, false).state, State::Clear);
    EXPECT_TRUE(budget.update(goal, 14.0, origin, true).allowed());
  }
}

TEST(LocalDetourBudget, InvalidConfigurationIsRejected)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (double invalid : {0.0, -1.0, nan}) {
    for (int field = 0; field < 4; ++field) {
      LocalDetourBudget::Config config;
      if (field == 0) {config.max_duration_s = invalid;}
      if (field == 1) {config.no_progress_timeout_s = invalid;}
      if (field == 2) {config.progress_distance = invalid;}
      if (field == 3) {config.progress_angle = invalid;}
      EXPECT_THROW(LocalDetourBudget{config}, std::invalid_argument);
    }
  }
}
}  // namespace astar_3d_planner
