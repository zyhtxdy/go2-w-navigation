#include "go2_relocalization/runtime.hpp"
#include <gtest/gtest.h>
#include <limits>

using go2_relocalization::PoseHistory;
using go2_relocalization::ConfirmationGate;
TEST(ConfirmationTimeout, ComputeLatencyDoesNotConsumeIdleResultTimeout)
{
  EXPECT_FALSE(go2_relocalization::confirmationIdleExpired(8.1, 5, 7.4, 3));
  EXPECT_TRUE(go2_relocalization::confirmationIdleExpired(10.5, 5, 7.4, 3));
  EXPECT_TRUE(go2_relocalization::confirmationIdleExpired(4, 5, 7.4, 3));
}

TEST(GlobalVerificationSchedule, SlowGlobalJobLeavesTimeForFreshTracking)
{
  // Global scan measured at t=10 finishes at t=12.5. Its old measurement
  // timestamp must not force another expensive global search immediately.
  EXPECT_FALSE(go2_relocalization::globalCheckDue(true, 12.6, 12.5, 1));
  EXPECT_FALSE(go2_relocalization::globalCheckDue(true, 13.4, 12.5, 1));
  EXPECT_TRUE(go2_relocalization::globalCheckDue(true, 13.5, 12.5, 1));
}

TEST(GlobalVerificationSchedule, LostOrRewoundStateAlwaysChecksGlobally)
{
  EXPECT_TRUE(go2_relocalization::globalCheckDue(false, 12.6, 12.5, 1));
  EXPECT_TRUE(go2_relocalization::globalCheckDue(true, 12.4, 12.5, 1));
  EXPECT_TRUE(go2_relocalization::globalCheckDue(true, 12.6, -1, 1));
  EXPECT_TRUE(go2_relocalization::globalCheckDue(true,
      std::numeric_limits<double>::quiet_NaN(), 12.5, 1));
}

TEST(GlobalVerificationSchedule, SlowCycleStartsBeforeItsRemainingGlobalDeadline)
{
  // Last full verification was measured at t=10; a retrieval and fresh-scan
  // verification cycle completed at t=13, and its next cycle needs ~4 s.
  // Waiting until t=14 would leave no room inside the unchanged 8 s deadline.
  EXPECT_FALSE(go2_relocalization::globalCheckDueBeforeDeadline(
      true, 13.1, 13, 1, 10, 8, 4, .25));
  EXPECT_TRUE(go2_relocalization::globalCheckDueBeforeDeadline(
      true, 13.8, 13, 1, 10, 8, 4, .25));
  // Fast work keeps its normal quiet interval, avoiding needless CPU load.
  EXPECT_FALSE(go2_relocalization::globalCheckDueBeforeDeadline(
      true, 13.8, 13, 1, 10, 8, .5, .25));
  EXPECT_TRUE(go2_relocalization::globalCheckDueBeforeDeadline(
      true, 14, 13, 1, 10, 8, .5, .25));
}

TEST(GlobalVerificationSchedule, TightDeadlineNeverChangesMeasurementFreshness)
{
  EXPECT_TRUE(go2_relocalization::globalCheckDueBeforeDeadline(
      true, 17.9, 17.8, 1, 10, 8, .5, .25));
  EXPECT_TRUE(go2_relocalization::globalCheckDueBeforeDeadline(
      true, 13.1, 13, 1, -1, 8, 4, .25));
  EXPECT_TRUE(go2_relocalization::globalCheckDueBeforeDeadline(
      true, 13.1, 13, 1, 10, 8,
      std::numeric_limits<double>::quiet_NaN(), .25));
  // An expired proposal is still expired regardless of scheduling pressure.
  EXPECT_FALSE(go2_relocalization::globalProposalFresh(18.1, 10, 8));
}

TEST(GlobalProposalAge, SeedMayBeOlderThanPoseGateButStillNeedsFreshVerification)
{
  EXPECT_TRUE(go2_relocalization::globalProposalFresh(14, 10, 8));
  EXPECT_FALSE(go2_relocalization::globalProposalFresh(19, 10, 8));
  EXPECT_FALSE(go2_relocalization::globalProposalFresh(9, 10, 8));
  EXPECT_FALSE(go2_relocalization::globalProposalFresh(14, 0, 8));
  EXPECT_FALSE(go2_relocalization::globalProposalFresh(14,
      std::numeric_limits<double>::quiet_NaN(), 8));
}

TEST(GlobalResultAge, LoggedNxComputeLatencyUsesSeparateMaintenanceBudget)
{
  // Logged NX jobs finished at 3.13/3.53 s: valid maintenance evidence,
  // while the same scan is still too old to initialize or publish as current.
  EXPECT_TRUE(go2_relocalization::globalResultFresh(13.13,10,true,3,4,8));
  EXPECT_TRUE(go2_relocalization::globalResultFresh(13.53,10,true,3,4,8));
  EXPECT_FALSE(go2_relocalization::globalResultFresh(13.13,10,false,3,4,8));
  EXPECT_FALSE(go2_relocalization::globalResultFresh(14.01,10,true,3,4,8));
  // Dequeuing within budget does not waive the check after current tracking.
  EXPECT_TRUE(go2_relocalization::globalResultFresh(13.9,10,true,3,4,8));
  EXPECT_FALSE(go2_relocalization::globalResultFresh(14.1,10,true,3,4,8));
}

TEST(GlobalResultAge, CannotExtendGlobalDeadlineOrAcceptRewoundAndInvalidTime)
{
  EXPECT_TRUE(go2_relocalization::globalResultFresh(11.2,10,true,3,4,1.2));
  EXPECT_FALSE(go2_relocalization::globalResultFresh(11.21,10,true,3,4,1.2));
  EXPECT_FALSE(go2_relocalization::globalResultFresh(9.9,10,true,3,4,8));
  EXPECT_FALSE(go2_relocalization::globalResultFresh(14,0,true,3,4,8));
  EXPECT_FALSE(go2_relocalization::globalResultFresh(14,10,true,3,
    std::numeric_limits<double>::quiet_NaN(),8));
}
namespace
{
Eigen::Isometry3d pose(double x, double yaw = 0)
{
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation().x() = x;
  value.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return value;
}
}
TEST(PoseHistory, UsesExactTimestampEvenAfterLargeGap)
{
  PoseHistory history;
  ASSERT_TRUE(history.add(1, pose(0)));
  ASSERT_TRUE(history.add(2, pose(1)));
  ASSERT_TRUE(history.at(2, .05));
  EXPECT_DOUBLE_EQ(history.at(2, .05)->translation().x(), 1);
  EXPECT_FALSE(history.at(1.5, .05));
}
TEST(PoseHistory, InterpolatesPositionAndQuaternion)
{
  PoseHistory history;
  ASSERT_TRUE(history.add(1, pose(0, 0)));
  ASSERT_TRUE(history.add(1.08, pose(2, .4)));
  const auto middle = history.at(1.04, .05);
  ASSERT_TRUE(middle);
  EXPECT_NEAR(middle->translation().x(), 1, 1e-9);
  EXPECT_NEAR(Eigen::AngleAxisd(middle->linear()).angle(), .2, 1e-9);
  EXPECT_FALSE(history.at(1.3, .05));
  EXPECT_FALSE(history.at(.8, .05));
}
TEST(PoseHistory, RejectsNonMonotonicAndNonFiniteData)
{
  PoseHistory history;
  EXPECT_TRUE(history.add(1, pose(0)));
  EXPECT_FALSE(history.add(1, pose(2)));
  EXPECT_FALSE(history.add(.9, pose(2)));
  EXPECT_FALSE(history.add(2, pose(std::numeric_limits<double>::quiet_NaN())));
  history.clear();
  EXPECT_FALSE(history.latest());
  EXPECT_FALSE(history.at(1, .05));
}
TEST(Confirmation, RequiresDistinctSeparatedFrames)
{
  ConfirmationGate gate(3, .2, .3, .15);
  EXPECT_FALSE(gate.observe(1, pose(5)));
  EXPECT_FALSE(gate.observe(1, pose(5)));
  EXPECT_FALSE(gate.observe(1.1, pose(5)));
  EXPECT_EQ(gate.count(), 1);
  EXPECT_FALSE(gate.observe(1.3, pose(5.05)));
  EXPECT_TRUE(gate.observe(1.6, pose(5.03)));
}
TEST(Confirmation, RejectsPositionAndOrientationJumps)
{
  ConfirmationGate gate(3, .2, .3, .15);
  EXPECT_FALSE(gate.observe(1, pose(5)));
  EXPECT_FALSE(gate.observe(1.3, pose(5.5)));
  EXPECT_EQ(gate.count(), 1);
  EXPECT_FALSE(gate.observe(1.6, pose(5.5, .3)));
  EXPECT_EQ(gate.count(), 1);
}
TEST(Confirmation, ComparesAnchorRatherThanDriftingNeighbors)
{
  ConfirmationGate gate(3, .2, .3, .15);
  EXPECT_FALSE(gate.observe(1, pose(0)));
  EXPECT_FALSE(gate.observe(1.3, pose(.2)));
  EXPECT_FALSE(gate.observe(1.6, pose(.4)));
  EXPECT_EQ(gate.count(), 1);
}
TEST(Confirmation, MapOdomConsistencyAllowsBodyMotion)
{
  ConfirmationGate gate(3, .2, .3, .15);
  const auto map_odom = pose(10, 1.3);
  for (int i = 0; i < 3; ++i) {
    const auto odom_body = pose(i * .5, i * .1);
    const auto map_body = map_odom * odom_body;
    EXPECT_EQ(gate.observe(1 + i * .3, map_body * odom_body.inverse()), i == 2);
  }
}
TEST(Confirmation, InvalidOrLongGapClearsConfirmation)
{
  ConfirmationGate gate(3, .2, .3, .15);
  EXPECT_FALSE(gate.observe(1, pose(0)));
  EXPECT_FALSE(gate.observe(1.3, pose(0)));
  EXPECT_FALSE(gate.observe(20, pose(0)));
  EXPECT_EQ(gate.count(), 1);
  EXPECT_FALSE(gate.observe(21, pose(std::numeric_limits<double>::quiet_NaN())));
  EXPECT_EQ(gate.count(), 0);
}

TEST(GlobalResultAge, SixSecondMaintenanceStillRequiresFreshStartupAndOriginalTime)
{
  using go2_relocalization::globalResultFresh;
  EXPECT_TRUE(globalResultFresh(14.01,10,true,3,6,12));
  EXPECT_TRUE(globalResultFresh(15.1,10,true,3,6,12));
  EXPECT_TRUE(globalResultFresh(16.,10,true,3,6,12));
  EXPECT_FALSE(globalResultFresh(16.001,10,true,3,6,12));
  EXPECT_FALSE(globalResultFresh(14.01,10,false,3,6,12));
  EXPECT_FALSE(globalResultFresh(9.99,10,true,3,6,12));
  EXPECT_FALSE(globalResultFresh(16.,10,true,3,6,5));
}
