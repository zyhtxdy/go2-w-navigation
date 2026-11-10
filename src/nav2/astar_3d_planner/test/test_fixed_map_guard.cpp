#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <limits>
#include <thread>
#include <vector>

#include "astar_3d_planner/fixed_map_guard.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace astar_3d_planner
{
namespace
{
using Layer = FixedMapGuard::Layer;
using Decision = FixedMapGuard::Decision;
const std::string kMapId(64, 'a');
const std::vector<std::string> kServer{"/offline_map_server"};

sensor_msgs::msg::PointCloud2 cloud(const std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 result;
  result.header.frame_id = "map";
  result.header.stamp.sec = 1;
  result.height = 1;
  result.width = points.size();
  result.point_step = 12;
  result.row_step = points.size() * 12;
  result.is_dense = true;
  for (size_t index = 0; index < 3; ++index) {
    sensor_msgs::msg::PointField field;
    field.name = index == 0 ? "x" : (index == 1 ? "y" : "z");
    field.offset = index * 4;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    result.fields.push_back(field);
  }
  for (const auto & point : points) {
    for (float value : point) {
      uint32_t bits;
      std::memcpy(&bits, &value, sizeof(bits));
      for (unsigned int shift = 0; shift < 32; shift += 8) {
        result.data.push_back(static_cast<uint8_t>(bits >> shift));
      }
    }
  }
  return result;
}

sensor_msgs::msg::PointCloud2 sourcedCloud(uint8_t source)
{
  auto result = cloud({{3.f, 4.f, .5f}});
  result.point_step = result.row_step = 16;
  sensor_msgs::msg::PointField field;
  field.name = "obstacle_source"; field.offset = 12;
  field.datatype = sensor_msgs::msg::PointField::UINT8; field.count = 1;
  result.fields.push_back(field);
  result.data.insert(result.data.end(), {source, 0, 0, 0});
  return result;
}

class FixedMapGuardTest : public ::testing::Test
{
protected:
  const sensor_msgs::msg::PointCloud2 support_ = cloud({{1.f, 2.f, -.3f}, {2.f, 2.f, -.3f}});
  const sensor_msgs::msg::PointCloud2 obstacles_ = cloud({{3.f, 4.f, .5f}});

  FixedMapGuard guard() const
  {
    return FixedMapGuard("map", kMapId, payloadSha256(support_.data),
      payloadSha256(obstacles_.data));
  }

  void load(FixedMapGuard & value) const
  {
    EXPECT_EQ(value.validateCloud(Layer::Support, support_), Decision::Accept);
    EXPECT_EQ(value.validateCloud(Layer::Obstacles, obstacles_), Decision::Accept);
    EXPECT_TRUE(value.observeIdentity(kMapId));
  }
};

TEST(FixedMapSha256, MatchesStandardVectors)
{
  EXPECT_EQ(payloadSha256({}),
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(payloadSha256({'a', 'b', 'c'}),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST_F(FixedMapGuardTest, CanonicalSourcePayloadIsAcceptedAndSourceTamperingLatchesFault)
{
  auto sourced = sourcedCloud(1);
  FixedMapGuard value("map", kMapId, payloadSha256(support_.data), payloadSha256(sourced.data));
  EXPECT_EQ(value.validateCloud(Layer::Support, support_), Decision::Accept);
  EXPECT_EQ(value.validateCloud(Layer::Obstacles, sourced), Decision::Accept);
  EXPECT_TRUE(value.observePublishers(kServer, kServer, kServer));
  EXPECT_TRUE(value.observeIdentity(kMapId)); EXPECT_TRUE(value.ready());
  sourced.data[12] = 2;  // Same XYZ, altered collision semantics.
  EXPECT_EQ(value.validateCloud(Layer::Obstacles, sourced), Decision::Reject);
  EXPECT_EQ(value.reason(), "obstacles_payload_sha256_mismatch");
}

TEST_F(FixedMapGuardTest, SourceLayoutCannotBeUsedAsSupportAndMalformedSourcesAreRejected)
{
  for (int variant = 0; variant < 4; ++variant) {
    auto sourced = sourcedCloud(1);
    if (variant == 0) {sourced.data[12] = 3;}
    if (variant == 1) {sourced.data[13] = 1;}
    if (variant == 2) {sourced.fields.back().datatype = sensor_msgs::msg::PointField::FLOAT32;}
    FixedMapGuard value("map", kMapId, payloadSha256(sourced.data), payloadSha256(sourced.data));
    EXPECT_EQ(value.validateCloud(variant == 3 ? Layer::Support : Layer::Obstacles, sourced),
      Decision::Reject) << variant;
    EXPECT_EQ(value.reason(), variant == 0 ? "obstacles_invalid_obstacle_source" :
      variant == 1 ? "obstacles_noncanonical_source_padding" :
      variant == 2 ? "obstacles_noncanonical_xyz_fields" : "support_noncanonical_xyz_layout");
  }
}

TEST_F(FixedMapGuardTest, RequiresAllThreeInputsAndOneCommonPublisher)
{
  auto value = guard();
  EXPECT_FALSE(value.ready());
  EXPECT_EQ(value.reason(), "waiting_for_publishers");
  EXPECT_TRUE(value.observePublishers(kServer, kServer, kServer));
  EXPECT_EQ(value.reason(), "waiting_for_map_id");
  EXPECT_TRUE(value.observeIdentity(kMapId));
  EXPECT_EQ(value.reason(), "waiting_for_support");
  EXPECT_EQ(value.validateCloud(Layer::Support, support_), Decision::Accept);
  EXPECT_EQ(value.reason(), "waiting_for_obstacles");
  EXPECT_FALSE(value.ready());
  EXPECT_EQ(value.validateCloud(Layer::Obstacles, obstacles_), Decision::Accept);
  EXPECT_TRUE(value.ready());
  EXPECT_EQ(value.reason(), "ready");
}

TEST_F(FixedMapGuardTest, IgnoresIdenticalRepeatsAndHeaderStampChanges)
{
  auto value = guard();
  load(value);
  EXPECT_TRUE(value.observePublishers(kServer, kServer, kServer));
  auto repeated = support_;
  repeated.header.stamp.sec = 999;
  repeated.header.stamp.nanosec = 123;
  EXPECT_EQ(value.validateCloud(Layer::Support, repeated), Decision::Duplicate);
  EXPECT_EQ(value.validateCloud(Layer::Support, support_), Decision::Duplicate);
  EXPECT_TRUE(value.ready());
  EXPECT_NE(value.report().find("\"accepted\":1,\"duplicates\":2,\"rejected\":0"),
    std::string::npos);
}

TEST_F(FixedMapGuardTest, ChangedPayloadLatchesFaultEvenAfterCorrectMessagesReturn)
{
  auto value = guard();
  load(value);
  EXPECT_TRUE(value.observePublishers(kServer, kServer, kServer));
  auto changed = support_;
  changed.data.front() ^= 1;
  EXPECT_EQ(value.validateCloud(Layer::Support, changed), Decision::Reject);
  EXPECT_EQ(value.reason(), "support_payload_sha256_mismatch");
  EXPECT_TRUE(value.faulted());
  EXPECT_FALSE(value.ready());
  EXPECT_EQ(value.validateCloud(Layer::Support, support_), Decision::Reject);
  EXPECT_EQ(value.validateCloud(Layer::Obstacles, obstacles_), Decision::Reject);
  EXPECT_FALSE(value.observeIdentity(kMapId));
  EXPECT_FALSE(value.observePublishers(kServer, kServer, kServer));
  EXPECT_EQ(value.reason(), "support_payload_sha256_mismatch");
}

TEST_F(FixedMapGuardTest, WrongObstaclePayloadIsRejectedBeforeMapApplication)
{
  auto value = guard();
  auto changed = obstacles_;
  changed.data.front() ^= 1;
  EXPECT_EQ(value.validateCloud(Layer::Obstacles, changed), Decision::Reject);
  EXPECT_EQ(value.reason(), "obstacles_payload_sha256_mismatch");
}

TEST_F(FixedMapGuardTest, EmptyObstacleMapIsValidButEmptySupportIsNot)
{
  const auto empty = cloud({});
  FixedMapGuard value("map", kMapId, payloadSha256(support_.data), payloadSha256(empty.data));
  EXPECT_EQ(value.validateCloud(Layer::Support, support_), Decision::Accept);
  EXPECT_EQ(value.validateCloud(Layer::Obstacles, empty), Decision::Accept);
  EXPECT_TRUE(value.observeIdentity(kMapId));
  EXPECT_TRUE(value.observePublishers(kServer, kServer, kServer));
  EXPECT_TRUE(value.ready());
  FixedMapGuard invalid("map", kMapId, payloadSha256(empty.data), payloadSha256(empty.data));
  EXPECT_EQ(invalid.validateCloud(Layer::Support, empty), Decision::Reject);
  EXPECT_EQ(invalid.reason(), "support_empty_support");
}

TEST_F(FixedMapGuardTest, RejectsEveryNoncanonicalLayout)
{
  std::vector<sensor_msgs::msg::PointCloud2> invalid;
  auto changed = support_;
  changed.header.frame_id = "odom"; invalid.push_back(changed);
  changed = support_; changed.is_bigendian = true; invalid.push_back(changed);
  changed = support_; changed.height = 2; invalid.push_back(changed);
  changed = support_; changed.point_step = 16; invalid.push_back(changed);
  changed = support_; changed.row_step += 4; invalid.push_back(changed);
  changed = support_; changed.data.pop_back(); invalid.push_back(changed);
  changed = support_; changed.data.push_back(0); invalid.push_back(changed);
  changed = support_; changed.width = std::numeric_limits<uint32_t>::max();
  invalid.push_back(changed);
  changed = support_; changed.fields.pop_back(); invalid.push_back(changed);
  changed = support_; changed.fields[1].name = "x"; invalid.push_back(changed);
  changed = support_; changed.fields[2].offset = 7; invalid.push_back(changed);
  changed = support_; changed.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
  invalid.push_back(changed);
  changed = support_; changed.fields[1].count = 2; invalid.push_back(changed);
  for (size_t index = 0; index < invalid.size(); ++index) {
    SCOPED_TRACE(index);
    auto value = guard();
    EXPECT_EQ(value.validateCloud(Layer::Support, invalid[index]), Decision::Reject);
    EXPECT_TRUE(value.faulted());
  }
}

TEST_F(FixedMapGuardTest, AcceptsFieldsInDifferentOrderWhenOffsetsAreCanonical)
{
  auto changed = support_;
  std::swap(changed.fields[0], changed.fields[2]);
  auto value = guard();
  EXPECT_EQ(value.validateCloud(Layer::Support, changed), Decision::Accept);
}

TEST_F(FixedMapGuardTest, RejectsNanAndInfinityEvenWithMatchingHash)
{
  for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()})
  {
    auto support = cloud({{1.f, invalid, 0.f}});
    FixedMapGuard value("map", kMapId, payloadSha256(support.data), payloadSha256(obstacles_.data));
    EXPECT_EQ(value.validateCloud(Layer::Support, support), Decision::Reject);
    EXPECT_EQ(value.reason(), "support_nonfinite_xyz");
  }
}

TEST_F(FixedMapGuardTest, RejectsInvalidOrConflictingMapId)
{
  for (const auto & identity : {std::string{}, std::string(63, 'a'), std::string(64, 'g'),
      std::string(64, 'b')})
  {
    auto value = guard();
    EXPECT_FALSE(value.observeIdentity(identity));
    EXPECT_TRUE(value.faulted());
    EXPECT_FALSE(value.observeIdentity(kMapId));
  }
}

TEST_F(FixedMapGuardTest, DetectsInvalidExpectedIdentityAndNormalizesHexCase)
{
  const auto support_hash = payloadSha256(support_.data);
  const auto obstacle_hash = payloadSha256(obstacles_.data);
  FixedMapGuard invalid("map", "bad", support_hash, obstacle_hash);
  EXPECT_TRUE(invalid.faulted());
  EXPECT_EQ(invalid.reason(), "invalid_expected_map_identity");
  FixedMapGuard bad_hash("map", kMapId, std::string(64, 'g'), obstacle_hash);
  EXPECT_TRUE(bad_hash.faulted());
  FixedMapGuard empty_frame("", kMapId, support_hash, obstacle_hash);
  EXPECT_TRUE(empty_frame.faulted());
  FixedMapGuard normalized("map", std::string(64, 'A'), support_hash, obstacle_hash);
  EXPECT_TRUE(normalized.observeIdentity(kMapId));
  EXPECT_TRUE(normalized.observeIdentity(std::string(64, 'A')));
  EXPECT_FALSE(normalized.faulted());
}

TEST_F(FixedMapGuardTest, MissingPublishersWaitWithoutFaultAndCanRecover)
{
  auto value = guard();
  load(value);
  EXPECT_FALSE(value.observePublishers(kServer, {}, kServer));
  EXPECT_FALSE(value.faulted());
  EXPECT_FALSE(value.ready());
  EXPECT_TRUE(value.observePublishers(kServer, kServer, kServer));
  EXPECT_TRUE(value.ready());
  EXPECT_FALSE(value.observePublishers({}, {}, {}));
  EXPECT_FALSE(value.ready());
  EXPECT_FALSE(value.faulted());
  EXPECT_TRUE(value.observePublishers(kServer, kServer, kServer));
  EXPECT_TRUE(value.ready());
}

TEST_F(FixedMapGuardTest, DuplicatePublisherEndpointsLatchEvenWithSameNodeName)
{
  const std::vector<std::string> duplicate{"/offline_map_server", "/offline_map_server"};
  for (int topic = 0; topic < 3; ++topic) {
    auto value = guard();
    EXPECT_FALSE(value.observePublishers(
        topic == 0 ? duplicate : kServer,
        topic == 1 ? duplicate : kServer,
        topic == 2 ? duplicate : kServer));
    EXPECT_TRUE(value.faulted());
    EXPECT_EQ(value.reason(), "multiple_fixed_map_publishers");
    EXPECT_FALSE(value.observePublishers(kServer, kServer, kServer));
  }
}

TEST_F(FixedMapGuardTest, DifferentSourcesAreRejectedEvenBeforeAllTopicsAppear)
{
  auto value = guard();
  EXPECT_FALSE(value.observePublishers(kServer, {"/other_map_server"}, {}));
  EXPECT_TRUE(value.faulted());
  EXPECT_EQ(value.reason(), "fixed_map_publisher_source_conflict");
}

TEST_F(FixedMapGuardTest, UnknownPublisherNameWaitsWithoutClaimingHealthyGraph)
{
  auto value = guard();
  load(value);
  EXPECT_FALSE(value.observePublishers(kServer, kServer, {""}));
  EXPECT_FALSE(value.ready());
  EXPECT_FALSE(value.faulted());
  EXPECT_EQ(value.reason(), "waiting_for_publisher_identity");
  EXPECT_TRUE(value.observePublishers(kServer, kServer, kServer));
  EXPECT_TRUE(value.ready());
}

TEST_F(FixedMapGuardTest, ReportPreservesIdentityCountsAndEscapesStrings)
{
  auto value = guard();
  load(value);
  const std::vector<std::string> odd_name{"/source\"\n\\"};
  EXPECT_TRUE(value.observePublishers(odd_name, odd_name, odd_name));
  const auto report = value.report();
  EXPECT_NE(report.find("\"expected_map_id\":\"" + kMapId + "\""), std::string::npos);
  EXPECT_NE(report.find(payloadSha256(support_.data)), std::string::npos);
  EXPECT_NE(report.find("\"points\":2,"), std::string::npos);
  EXPECT_NE(report.find("\"publisher_counts\":{\"support\":1,\"obstacles\":1,\"identity\":1}"),
    std::string::npos);
  EXPECT_NE(report.find("/source\\\"\\n\\\\"), std::string::npos);
  EXPECT_EQ(report.find('\n'), std::string::npos);
}

TEST_F(FixedMapGuardTest, ConcurrentCallbacksKeepVerificationAndCountersConsistent)
{
  auto value = guard();
  load(value);
  EXPECT_TRUE(value.observePublishers(kServer, kServer, kServer));
  std::vector<std::thread> workers;
  for (int worker = 0; worker < 4; ++worker) {
    workers.emplace_back([&]() {
        for (int index = 0; index < 100; ++index) {
          EXPECT_EQ(value.validateCloud(Layer::Support, support_), Decision::Duplicate);
          EXPECT_TRUE(value.ready());
          EXPECT_FALSE(value.report().empty());
        }
      });
  }
  for (auto & worker : workers) {worker.join();}
  EXPECT_FALSE(value.faulted());
  EXPECT_NE(value.report().find("\"accepted\":1,\"duplicates\":400,\"rejected\":0"),
    std::string::npos);
}

}  // namespace
}  // namespace astar_3d_planner
