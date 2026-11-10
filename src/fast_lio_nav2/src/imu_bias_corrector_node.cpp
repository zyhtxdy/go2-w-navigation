#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "fast_lio_nav2/gyro_bias_calibration.hpp"

class ImuBiasCorrectorNode : public rclcpp::Node
{
public:
  ImuBiasCorrectorNode()
  : Node("imu_bias_corrector")
  {
    const auto input_topic =
      this->declare_parameter<std::string>("input_topic", "/go2/imu");
    const auto output_topic =
      this->declare_parameter<std::string>("output_topic", "/imu/corrected");
    fast_lio_nav2::GyroBiasCalibrationConfig config;
    config.samples = static_cast<std::size_t>(std::max<std::int64_t>(
      this->declare_parameter<std::int64_t>("calibration_samples", 400), 2));
    config.minimum_duration =
      this->declare_parameter<double>("calibration_minimum_duration", 1.0);
    config.maximum_gyro_norm =
      this->declare_parameter<double>("calibration_maximum_gyro_norm", 0.05);
    config.maximum_axis_variance =
      this->declare_parameter<double>("calibration_maximum_axis_variance", 2.0e-4);
    config.maximum_orientation_change =
      this->declare_parameter<double>("calibration_maximum_orientation_change", 0.01);
    config.maximum_sample_gap =
      this->declare_parameter<double>("calibration_maximum_sample_gap", 0.1);
    calibration_ = std::make_unique<fast_lio_nav2::GyroBiasCalibration>(config);

    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(20);
    imu_pub_ =
      this->create_publisher<sensor_msgs::msg::Imu>(output_topic, qos);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      input_topic, qos,
      std::bind(&ImuBiasCorrectorNode::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Keep the robot still: need %zu consecutive samples over %.2f s from %s; "
      "gyro <= %.3f rad/s, axis variance <= %.6f, relative attitude <= %.3f rad. "
      "Publishing uncorrected gyro until accepted.",
      config.samples, config.minimum_duration, input_topic.c_str(),
      config.maximum_gyro_norm, config.maximum_axis_variance,
      config.maximum_orientation_change);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const std::array<double, 3> omega{
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z};

    if (!std::isfinite(omega[0]) ||
      !std::isfinite(omega[1]) ||
      !std::isfinite(omega[2]))
    {
      // A rejected sample interrupts the continuous stationary window.
      calibration_->add(omega, 0.0, false);
      return;
    }

    const bool was_calibrated = calibration_->calibrated();
    if (!was_calibrated) {
      const bool orientation_available =
        std::isfinite(msg->orientation_covariance[0]) &&
        msg->orientation_covariance[0] >= 0.0;
      const double stamp =
        static_cast<double>(msg->header.stamp.sec) +
        static_cast<double>(msg->header.stamp.nanosec) * 1.0e-9;
      calibration_->add(omega, stamp, orientation_available, {
        msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w});
      if (!orientation_available && !warned_missing_orientation_) {
        RCLCPP_WARN(
          this->get_logger(),
          "No usable IMU orientation for stationary detection; a slow constant turn "
          "cannot be distinguished from bias by gyro alone. Start while stationary.");
        warned_missing_orientation_ = true;
      }
    }

    auto output = *msg;
    if (!calibration_->calibrated()) {
      // Keep the EKF yaw-rate input alive during motion or an incomplete window.
      imu_pub_->publish(output);
      return;
    }
    const auto & bias = calibration_->bias();
    const auto & variance = calibration_->variance();
    if (!was_calibrated) {
      RCLCPP_INFO(
        this->get_logger(),
        "IMU gyro bias calibrated: [%.7f, %.7f, %.7f] rad/s",
        bias[0], bias[1], bias[2]);
    }

    output.angular_velocity.x -= bias[0];
    output.angular_velocity.y -= bias[1];
    output.angular_velocity.z -= bias[2];
    output.angular_velocity_covariance[0] = std::max(
      output.angular_velocity_covariance[0], variance[0]);
    output.angular_velocity_covariance[4] = std::max(
      output.angular_velocity_covariance[4], variance[1]);
    output.angular_velocity_covariance[8] = std::max(
      output.angular_velocity_covariance[8], variance[2]);
    imu_pub_->publish(output);
  }

  std::unique_ptr<fast_lio_nav2::GyroBiasCalibration> calibration_;
  bool warned_missing_orientation_{false};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

#ifndef FAST_LIO_NAV2_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuBiasCorrectorNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
