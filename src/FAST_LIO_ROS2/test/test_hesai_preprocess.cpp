#include "preprocess.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
#include <limits>
#include <vector>

namespace
{
constexpr int32_t epoch = 1700000000;

// Match the official ROS2 driver: packed 26-byte points, with the double
// timestamp at the unaligned offset 18. This is NOT PCL's padded Point layout.
sensor_msgs::msg::PointCloud2::UniquePtr cloud(const std::vector<double> &offsets)
{
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp.sec = epoch;
  msg->width = offsets.size();
  msg->height = 1;
  msg->point_step = 26;
  msg->row_step = msg->point_step * msg->width;
  const auto field = [&](const char *name, uint32_t offset, uint8_t datatype) {
    sensor_msgs::msg::PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = datatype;
    f.count = 1;
    msg->fields.push_back(f);
  };
  using F = sensor_msgs::msg::PointField;
  field("x", 0, F::FLOAT32);
  field("y", 4, F::FLOAT32);
  field("z", 8, F::FLOAT32);
  field("intensity", 12, F::FLOAT32);
  field("ring", 16, F::UINT16);
  field("timestamp", 18, F::FLOAT64);
  msg->data.resize(msg->row_step);
  for (size_t i = 0; i < offsets.size(); ++i)
  {
    const float xyz_intensity[4] = {2.0f, 0.0f, 0.0f, static_cast<float>(i + 1)};
    const uint16_t ring = i % 16;
    const double stamp = static_cast<double>(epoch) + offsets[i];
    auto *data = msg->data.data() + i * msg->point_step;
    std::memcpy(data, xyz_intensity, 16);
    std::memcpy(data + 16, &ring, 2);
    std::memcpy(data + 18, &stamp, 8);
  }
  return msg;
}

class HesaiPreprocess : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pre.set(false, HESAI_XT16, 0.5, 1);
    pre.N_SCANS = 16;
    pre.SCAN_RATE = 10;
    pre.time_unit = SEC;
  }

  PointCloudXYZI::Ptr run(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
  {
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    pre.process(msg, output);
    return output;
  }

  Preprocess pre;
};

TEST_F(HesaiPreprocess, PreservesEpochPrecisionAndOrdersOffsets)
{
  const auto result = run(cloud({0.09, 0.0, 0.05, 0.01}));
  ASSERT_EQ(result->size(), 4u);
  EXPECT_NEAR(result->points[0].curvature, 0.0, 0.001);
  EXPECT_NEAR(result->points[1].curvature, 10.0, 0.001);
  EXPECT_NEAR(result->points[2].curvature, 50.0, 0.001);
  EXPECT_NEAR(result->points[3].curvature, 90.0, 0.001);
  EXPECT_FLOAT_EQ(result->points[0].intensity, 2.0f);
}

TEST_F(HesaiPreprocess, DownsamplingKeepsScanEnd)
{
  pre.point_filter_num = 2;
  const auto result = run(cloud({0.09, 0.0, 0.05, 0.01}));
  ASSERT_EQ(result->size(), 3u);
  EXPECT_NEAR(result->points.back().curvature, 90.0, 0.001);
}

TEST_F(HesaiPreprocess, RejectsMissingOrIncorrectTimeField)
{
  auto msg = cloud({0.0, 0.05});
  msg->fields.back().name = "time";
  EXPECT_TRUE(run(msg)->empty());
  msg->fields.back().name = "timestamp";
  msg->fields.back().datatype = sensor_msgs::msg::PointField::FLOAT32;
  EXPECT_TRUE(run(msg)->empty());
}

TEST_F(HesaiPreprocess, RejectsMismatchedHeaderClockAndWrongUnits)
{
  auto msg = cloud({0.0, 0.05});
  msg->header.stamp.sec = epoch + 1;
  EXPECT_TRUE(run(msg)->empty());
  msg->header.stamp.sec = epoch;
  pre.time_unit = NS;
  EXPECT_TRUE(run(msg)->empty());
}

TEST_F(HesaiPreprocess, FiltersInvalidRingsNonFinitePointsAndBlindZone)
{
  auto msg = cloud({0.0, 0.01, 0.02, 0.03});
  const uint16_t invalid_ring = 16;
  std::memcpy(msg->data.data() + 16, &invalid_ring, 2);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(msg->data.data() + 26, &nan, 4);
  const float near = 0.1f;
  std::memcpy(msg->data.data() + 52, &near, 4);
  const auto result = run(msg);
  ASSERT_EQ(result->size(), 1u);
  EXPECT_NEAR(result->points[0].curvature, 30.0, 0.001);
}

TEST_F(HesaiPreprocess, HandlesEmptyAndTruncatedCloudWithoutStaleOutput)
{
  ASSERT_FALSE(run(cloud({0.0, 0.05}))->empty());
  EXPECT_TRUE(run(cloud({}))->empty());
  auto msg = cloud({0.0, 0.05});
  msg->data.pop_back();
  EXPECT_TRUE(run(msg)->empty());
  EXPECT_FALSE(pre.given_offset_time);
}

TEST_F(HesaiPreprocess, RejectsRelativeTimestampsAndOversizedFrameDuration)
{
  EXPECT_TRUE(run(cloud({-1700000000.0, -1699999999.95}))->empty());
  EXPECT_TRUE(run(cloud({0.0, 0.2}))->empty());
}

TEST_F(HesaiPreprocess, RequiresDirectPointModeAndDoesNotDivideByZero)
{
  pre.feature_enabled = true;
  EXPECT_TRUE(run(cloud({0.0, 0.05}))->empty());
  pre.feature_enabled = false;
  pre.point_filter_num = 0;
  EXPECT_EQ(run(cloud({0.0, 0.05}))->size(), 2u);
}
}  // namespace
