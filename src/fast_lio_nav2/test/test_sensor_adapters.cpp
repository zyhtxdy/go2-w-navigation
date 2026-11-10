// Exercise the real callbacks without starting mapping, navigation or a robot
// interface. Pre-include dependencies so private exposure affects only these
// two adapter classes, not ROS/standard-library implementation types.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "fast_lio_nav2/gyro_bias_calibration.hpp"

#define FAST_LIO_NAV2_NO_MAIN
#define private public
#include "../src/imu_bias_corrector_node.cpp"
#include "../src/lio_base_odom_node.cpp"
#undef private
#undef FAST_LIO_NAV2_NO_MAIN

namespace
{
class SensorAdapters : public ::testing::Test
{
public:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

void installCalibration(LioBaseOdomNode & node)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "body";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(node.tf_buffer_.setTransform(transform, "unit_test", true));
}

nav_msgs::msg::Odometry::SharedPtr odometry(
  const std::string & world, const std::string & child, int stamp)
{
  auto message = std::make_shared<nav_msgs::msg::Odometry>();
  message->header.frame_id = world;
  message->child_frame_id = child;
  message->header.stamp.sec = stamp;
  message->pose.pose.orientation.w = 1.0;
  message->pose.pose.position.x = 1.0;
  return message;
}

TEST_F(SensorAdapters, WrongWorldFrameCannotSeedOrAdvanceLioFusion)
{
  LioBaseOdomNode node;
  installCalibration(node);
  node.odomCallback(odometry("map", "body", 1));
  EXPECT_FALSE(node.have_last_pose_);
  node.odomCallback(odometry("odom", "body", 2));
  ASSERT_TRUE(node.have_last_pose_);
  node.odomCallback(odometry("map", "body", 3));
  EXPECT_EQ(node.last_stamp_.seconds(), 2.0);
}

TEST_F(SensorAdapters, WrongBodyFrameCannotSeedLioFusion)
{
  LioBaseOdomNode node;
  installCalibration(node);
  node.odomCallback(odometry("odom", "base_link", 1));
  EXPECT_FALSE(node.have_last_pose_);
  node.odomCallback(odometry("odom", "body", 2));
  EXPECT_TRUE(node.have_last_pose_);
}

TEST_F(SensorAdapters, MovingImuIsPassedThroughAndOnlyStationaryBiasIsRemoved)
{
  using namespace std::chrono_literals;
  auto adapter = std::make_shared<ImuBiasCorrectorNode>();
  auto receiver = std::make_shared<rclcpp::Node>("sensor_adapter_test_receiver");
  std::vector<sensor_msgs::msg::Imu> received;
  auto subscription = receiver->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/corrected", rclcpp::SensorDataQoS(),
    [&received](const sensor_msgs::msg::Imu & message) {received.push_back(message);});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(receiver);
  const auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
  while (adapter->imu_pub_->get_subscription_count() == 0 &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(adapter->imu_pub_->get_subscription_count(), 0u);

  auto message = std::make_shared<sensor_msgs::msg::Imu>();
  message->header.frame_id = "go2_imu";
  message->header.stamp.sec = 10;
  message->orientation.w = 1.0;
  message->angular_velocity.z = 0.35;
  adapter->imuCallback(message);
  const auto receive_deadline = std::chrono::steady_clock::now() + 2s;
  while (received.empty() && std::chrono::steady_clock::now() < receive_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(received.size(), 1u);
  EXPECT_DOUBLE_EQ(received.back().angular_velocity.z, 0.35);
  EXPECT_EQ(received.back().header.frame_id, "go2_imu");
  EXPECT_FALSE(adapter->calibration_->calibrated());

  // The callback's original 400-sample unconditional average would have
  // accepted the startup turn; the new window accepts only the still segment.
  for (int i = 1; i <= 600; ++i) {
    message->header.stamp.sec = 10 + i / 500;
    message->header.stamp.nanosec = (i % 500) * 2000000u;
    adapter->imuCallback(message);
  }
  EXPECT_FALSE(adapter->calibration_->calibrated());
  message->angular_velocity.z = 0.003;
  for (int i = 601; i <= 1202; ++i) {
    message->header.stamp.sec = 10 + i / 500;
    message->header.stamp.nanosec = (i % 500) * 2000000u;
    adapter->imuCallback(message);
  }
  EXPECT_TRUE(adapter->calibration_->calibrated());
  EXPECT_NEAR(adapter->calibration_->bias()[2], 0.003, 1.0e-12);
  message->header.stamp.sec = 13;
  message->header.stamp.nanosec = 0;
  message->angular_velocity.z = 0.2;
  adapter->imuCallback(message);
  const auto corrected_deadline = std::chrono::steady_clock::now() + 2s;
  while ((received.empty() || received.back().header.stamp.sec != 13) &&
    std::chrono::steady_clock::now() < corrected_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(received.back().header.stamp.sec, 13);
  EXPECT_NEAR(received.back().angular_velocity.z, 0.197, 1.0e-12);
  (void)subscription;
}
}  // namespace
