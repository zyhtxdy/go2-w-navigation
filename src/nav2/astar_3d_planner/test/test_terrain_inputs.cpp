#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "astar_3d_planner/astar_3d_planner.hpp"
#include "astar_3d_planner/cloud_observations.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace astar_3d_planner
{
namespace
{
TerrainInputBuffer::Cloud cloud(int stamp)
{
  auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();
  result->header.stamp.sec = stamp;
  return result;
}

sensor_msgs::msg::PointCloud2 observedCloud(bool timestamps = true)
{
  sensor_msgs::msg::PointCloud2 result;
  result.header.stamp.sec = 12;
  sensor_msgs::PointCloud2Modifier modifier(result);
  if (timestamps) {
    modifier.setPointCloud2Fields(7,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "obstacle_source", 1, sensor_msgs::msg::PointField::UINT8,
      "_padding", 3, sensor_msgs::msg::PointField::UINT8,
      "observation_sec", 1, sensor_msgs::msg::PointField::INT32,
      "observation_nanosec", 1, sensor_msgs::msg::PointField::UINT32);
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(5);
  sensor_msgs::PointCloud2Iterator<float> x(result, "x"), y(result, "y"), z(result, "z");
  for (int i = 0; i < 5; ++i, ++x, ++y, ++z) {*x = i; *y = .05f; *z = .21f;}
  if (timestamps) {
    sensor_msgs::PointCloud2Iterator<std::uint8_t> source(result, "obstacle_source");
    sensor_msgs::PointCloud2Iterator<std::int32_t> seconds(result, "observation_sec");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> nanoseconds(result, "observation_nanosec");
    for (int i = 0; i < 5; ++i, ++source, ++seconds, ++nanoseconds) {
      *source = i % 2 ? OccupyMap::kSynthetic : OccupyMap::kMeasured;
      *seconds = 10; *nanoseconds = 100 + i;
    }
  }
  return result;
}

sensor_msgs::msg::PointField & field(
  sensor_msgs::msg::PointCloud2 & message, const std::string & name)
{
  return *std::find_if(message.fields.begin(), message.fields.end(),
      [&name](const auto & item) {return item.name == name;});
}
}  // namespace

TEST(TerrainInputs, ObservationTimesFollowStrideFiniteFilteringAndTransform)
{
  auto message = observedCloud();
  sensor_msgs::PointCloud2Iterator<float> x(message, "x");
  x += 2; *x = std::numeric_limits<float>::quiet_NaN();
  auto transform = tf2::Transform::getIdentity();
  tf2::Quaternion rotation; rotation.setRPY(0, 0, std::acos(-1.0) / 2);
  transform.setRotation(rotation); transform.setOrigin(tf2::Vector3(1, 2, 3));
  std::vector<std::uint8_t> sources;
  std::vector<std::int64_t> observations;
  const auto points = transformCloudObservations(message, transform, 2, &sources, &observations);
  ASSERT_EQ(points.size(), 2u);
  ASSERT_EQ(sources.size(), points.size());
  EXPECT_EQ(observations, (std::vector<std::int64_t>{10000000100ll, 10000000104ll}));
  EXPECT_EQ(sources, (std::vector<std::uint8_t>{OccupyMap::kMeasured, OccupyMap::kMeasured}));
  EXPECT_NEAR(points[1][0], .95, 1e-6);
  EXPECT_NEAR(points[1][1], 6, 1e-6);
  EXPECT_NEAR(points[1][2], 3.21, 1e-6);
}

TEST(TerrainInputs, LegacyCloudObservationTimesAreUnknownDespiteNonzeroHeader)
{
  const auto message = observedCloud(false);
  std::vector<std::uint8_t> sources;
  std::vector<std::int64_t> observations{123};
  const auto points = transformCloudObservations(
    message, tf2::Transform::getIdentity(), 1, &sources, &observations);
  EXPECT_EQ(points.size(), 5u);
  EXPECT_EQ(sources, std::vector<std::uint8_t>(5, OccupyMap::kUnknown));
  EXPECT_EQ(observations, std::vector<std::int64_t>(5, 0));
}

TEST(TerrainInputs, IncompleteAndMalformedObservationFieldsCannotBeDecoded)
{
  for (int mutation = 0; mutation < 6; ++mutation) {
    auto message = observedCloud();
    if (mutation == 0) {field(message, "observation_sec").name = "missing_sec";}
    if (mutation == 1) {
      field(message, "observation_sec").datatype = sensor_msgs::msg::PointField::UINT32;
    }
    if (mutation == 2) {field(message, "observation_nanosec").count = 2;}
    if (mutation == 3) {field(message, "observation_nanosec").offset = message.point_step - 3;}
    if (mutation == 4) {field(message, "observation_sec").offset = message.point_step + 4;}
    if (mutation == 5) {message.fields.push_back(field(message, "observation_sec"));}
    std::vector<std::int64_t> observations;
    EXPECT_THROW(transformCloudObservations(
      message, tf2::Transform::getIdentity(), 1, nullptr, &observations), std::invalid_argument)
      << "mutation=" << mutation;
  }
}

TEST(TerrainInputs, InvalidObservationTimestampCannotHideInADecimatedPoint)
{
  for (bool invalid_seconds : {false, true}) {
    auto message = observedCloud();
    if (invalid_seconds) {
      sensor_msgs::PointCloud2Iterator<std::int32_t> seconds(message, "observation_sec");
      ++seconds; *seconds = -1;
    } else {
      sensor_msgs::PointCloud2Iterator<std::uint32_t> nanoseconds(message, "observation_nanosec");
      ++nanoseconds; *nanoseconds = 1000000000u;
    }
    std::vector<std::int64_t> observations;
    EXPECT_THROW(transformCloudObservations(
      message, tf2::Transform::getIdentity(), 2, nullptr, &observations), std::invalid_argument);
  }
}

TEST(TerrainInputs, PackedLayoutValidationRejectsTruncationAndInvalidCoordinates)
{
  for (int mutation = 0; mutation < 4; ++mutation) {
    auto message = observedCloud();
    if (mutation == 0) {message.data.pop_back();}
    if (mutation == 1) {message.row_step += 4;}
    if (mutation == 2) {field(message, "z").offset = message.point_step;}
    if (mutation == 3) {field(message, "x").datatype = sensor_msgs::msg::PointField::INT32;}
    EXPECT_THROW(transformCloudObservations(message, tf2::Transform::getIdentity(), 1),
      std::invalid_argument);
  }
}

TEST(TerrainInputs, EmptyCloudClearsOptionalProvenanceWithoutConstructingIterators)
{
  sensor_msgs::msg::PointCloud2 message;
  std::vector<std::uint8_t> sources{OccupyMap::kMeasured};
  std::vector<std::int64_t> observations{123};
  EXPECT_TRUE(transformCloudObservations(
    message, tf2::Transform::getIdentity(), 1, &sources, &observations).empty());
  EXPECT_TRUE(sources.empty());
  EXPECT_TRUE(observations.empty());
}

TEST(TerrainInputs, BigEndianCloudCannotBeReadAsNativeNumericFields)
{
  auto message = observedCloud();
  message.is_bigendian = true;
  std::vector<std::int64_t> observations;
  EXPECT_THROW(transformCloudObservations(
    message, tf2::Transform::getIdentity(), 1, nullptr, &observations), std::invalid_argument);
}

TEST(TerrainInputs, TypedFourByteFieldsMustStayAlignedForEveryPoint)
{
  for (int mutation = 0; mutation < 3; ++mutation) {
    auto message = observedCloud();
    if (mutation == 0) {field(message, "x").offset = 1;}
    if (mutation == 1) {field(message, "observation_sec").offset = 13;}
    if (mutation == 2) {
      --message.point_step;
      message.row_step = message.width * message.point_step;
      message.data.resize(message.row_step * message.height);
    }
    std::vector<std::int64_t> observations;
    EXPECT_THROW(transformCloudObservations(
      message, tf2::Transform::getIdentity(), 1, nullptr, &observations), std::invalid_argument)
      << "mutation=" << mutation;
  }
}

TEST(TerrainInputs, AlternatingCallbacksOneFrameApartStillMatch)
{
  TerrainInputBuffer buffer;
  TerrainInputBuffer::Cloud support, obstacles;
  buffer.pushSupport(cloud(1));
  for (int stamp = 1; stamp <= 20; ++stamp) {
    buffer.pushSupport(cloud(stamp + 1));
    buffer.pushObstacles(cloud(stamp));
    ASSERT_TRUE(buffer.popPair(support, obstacles));
    EXPECT_EQ(support->header.stamp.sec, stamp);
    EXPECT_EQ(obstacles->header.stamp.sec, stamp);
  }
}

TEST(TerrainInputs, BatchedCallbacksRetainPairsInTimestampOrder)
{
  TerrainInputBuffer buffer;
  TerrainInputBuffer::Cloud support, obstacles;
  for (int stamp : {3, 1, 2}) {buffer.pushSupport(cloud(stamp));}
  for (int stamp : {2, 3, 1}) {buffer.pushObstacles(cloud(stamp));}
  for (int stamp = 1; stamp <= 3; ++stamp) {
    ASSERT_TRUE(buffer.hasPair());
    ASSERT_TRUE(buffer.popPair(support, obstacles));
    EXPECT_EQ(support->header.stamp.sec, stamp);
    EXPECT_EQ(obstacles->header.stamp.sec, stamp);
  }
  EXPECT_FALSE(buffer.hasPair());
}

TEST(TerrainInputs, MissingStreamIsBoundedAndCannotMixDifferentFrames)
{
  TerrainInputBuffer buffer;
  TerrainInputBuffer::Cloud support, obstacles;
  for (int stamp = 1; stamp <= 100; ++stamp) {buffer.pushSupport(cloud(stamp));}
  EXPECT_EQ(buffer.supportSize(), 10u);
  buffer.pushObstacles(cloud(90));
  EXPECT_FALSE(buffer.popPair(support, obstacles));
  buffer.pushObstacles(cloud(95));
  ASSERT_TRUE(buffer.popPair(support, obstacles));
  EXPECT_EQ(support->header.stamp.sec, 95);
  EXPECT_EQ(obstacles->header.stamp.sec, 95);
  EXPECT_EQ(buffer.supportSize(), 5u);
  EXPECT_EQ(buffer.obstacleSize(), 0u);
}

TEST(TerrainInputs, DuplicateOldFramesCannotRollBackAndCleanupAllowsRestart)
{
  TerrainInputBuffer buffer;
  TerrainInputBuffer::Cloud support, obstacles;
  buffer.pushSupport(cloud(2)); buffer.pushObstacles(cloud(2));
  ASSERT_TRUE(buffer.popPair(support, obstacles));
  for (int stamp : {1, 2}) {
    buffer.pushSupport(cloud(stamp)); buffer.pushObstacles(cloud(stamp));
  }
  EXPECT_FALSE(buffer.hasPair());
  buffer.clear();
  buffer.pushSupport(cloud(1)); buffer.pushObstacles(cloud(1));
  EXPECT_TRUE(buffer.popPair(support, obstacles));
}

TEST(TerrainInputs, OfflineLiveObstacleStampRejectsInvalidStaleAndFutureData)
{
  auto message = cloud(10);
  EXPECT_TRUE(liveObstacleStampFresh(message->header.stamp, 10000000000ll, 1.0));
  EXPECT_TRUE(liveObstacleStampFresh(message->header.stamp, 11000000000ll, 1.0));
  EXPECT_FALSE(liveObstacleStampFresh(message->header.stamp, 11000000001ll, 1.0));
  EXPECT_FALSE(liveObstacleStampFresh(message->header.stamp, 9999999999ll, 1.0));
  message->header.stamp.sec = 0;
  EXPECT_FALSE(liveObstacleStampFresh(message->header.stamp, 1, 1.0));
  message->header.stamp.sec = -1;
  EXPECT_FALSE(liveObstacleStampFresh(message->header.stamp, 10000000000ll, 1.0));
  message->header.stamp.sec = 10;
  message->header.stamp.nanosec = 1000000000u;
  EXPECT_FALSE(liveObstacleStampFresh(message->header.stamp, 11000000000ll, 1.0));
}

TEST(TerrainInputs, PreviousReplayQueueCannotPoisonRecoveredObstacleBaseline)
{
  OccupyMap::Config config;
  config.inflate_radius = 0;
  OccupyMap map(config);
  map.setStaticObstacles({{0, 0, .5}});
  map.updateObstacles({{1, 0, .5}}, 0, 0, 25000000000ll, {}, true);
  map.clearDynamicObstacles(true);
  const auto epoch = map.dynamicObstacleEpoch();
  const auto current = cloud(10);
  const auto delayed = cloud(25);
  const auto now = 10000000000ll;
  // This delayed DDS delivery is taken after reset, so its epoch alone is current.
  if (liveObstacleStampFresh(delayed->header.stamp, now, 1.0)) {
    map.updateObstacles({{2, 0, .5}}, 0, 0, 25000000000ll, {}, true, epoch);
  }
  ASSERT_TRUE(liveObstacleStampFresh(current->header.stamp, now, 1.0));
  map.updateObstacles({{3, 0, .5}}, 0, 0, now, {}, true, epoch);
  {auto read = map.lockForReading();
    EXPECT_EQ(map.obstacleStampUnlocked(), now);
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(0, 0, .5)));
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(1, 0, .5)));
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(2, 0, .5)));
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(3, 0, .5)));}
}
}  // namespace astar_3d_planner

namespace astar_3d_planner
{
TEST(LocalizationHeartbeat, CallbackDelayInsideSharedOneSecondBudgetRemainsReady)
{
  constexpr std::int64_t received = 10000000000LL;
  for (const auto age : std::array<std::int64_t, 3>{0, 630000000, kLocalizationHeartbeatTimeoutNs - 1}) {
    const auto status = localizationHeartbeatStatus(true, received, received + age);
    EXPECT_TRUE(status.ready);
    EXPECT_STREQ(status.reason, "ready");
    EXPECT_NEAR(status.age_s, age * 1e-9, 1e-12);
  }
}

TEST(LocalizationHeartbeat, SharedBudgetExpiresAtOneSecondWithoutChangingSourceTime)
{
  constexpr std::int64_t received = 10000000000LL;
  EXPECT_EQ(kLocalizationHeartbeatTimeoutNs, 1000000000LL);
  for (const auto age : {kLocalizationHeartbeatTimeoutNs, kLocalizationHeartbeatTimeoutNs + 1}) {
    const auto status = localizationHeartbeatStatus(true, received, received + age);
    EXPECT_FALSE(status.ready);
    EXPECT_STREQ(status.reason, "localization_heartbeat_stale");
    EXPECT_GE(status.age_s, 1.0);
  }
}

TEST(LocalizationHeartbeat, MissingReceptionDoesNotPretendToBeExplicitFalse)
{
  for (const bool available : {false, true}) {
    const auto status = localizationHeartbeatStatus(available, 0, 10000000000LL);
    EXPECT_FALSE(status.ready);
    EXPECT_STREQ(status.reason, "localization_heartbeat_missing");
    EXPECT_EQ(status.age_s, -1.0);
  }
}

TEST(LocalizationHeartbeat, ExplicitFalseHasPriorityOverHeartbeatAge)
{
  constexpr std::int64_t received = 10000000000LL;
  for (const auto age : std::array<std::int64_t, 3>{0, 630000000, kLocalizationHeartbeatTimeoutNs + 1}) {
    const auto status = localizationHeartbeatStatus(false, received, received + age);
    EXPECT_FALSE(status.ready);
    EXPECT_STREQ(status.reason, "localization_unavailable");
    EXPECT_NEAR(status.age_s, age * 1e-9, 1e-12);
  }
  // A bad clock cannot hide an explicit revocation either.
  const auto status = localizationHeartbeatStatus(false, received, received - 1);
  EXPECT_FALSE(status.ready);
  EXPECT_STREQ(status.reason, "localization_unavailable");
  EXPECT_EQ(status.age_s, -1.0);
}

TEST(LocalizationHeartbeat, InvalidSteadyClockCannotAuthorizeAvailability)
{
  for (const auto received : {-1LL, 10000000001LL}) {
    const auto status = localizationHeartbeatStatus(true, received, 10000000000LL);
    EXPECT_FALSE(status.ready);
    EXPECT_STREQ(status.reason, "localization_heartbeat_clock_invalid");
    EXPECT_EQ(status.age_s, -1.0);
  }
}

TEST(LiveObstacleInput, ConfiguredTwoSecondBudgetMatchesConsumption)
{
  builtin_interfaces::msg::Time stamp; stamp.sec = 10;
  EXPECT_FALSE(liveObstacleStampFresh(stamp, 11500000000LL, 1.0));
  EXPECT_TRUE(liveObstacleStampFresh(stamp, 11500000000LL, 2.0));
  EXPECT_TRUE(liveObstacleStampFresh(stamp, 12000000000LL, 2.0));
  EXPECT_FALSE(liveObstacleStampFresh(stamp, 12000000001LL, 2.0));
  EXPECT_FALSE(liveObstacleStampFresh(stamp, 9999999999LL, 2.0));
  EXPECT_FALSE(liveObstacleStampFresh(stamp, 10000000000LL, 0.0));
}
}
