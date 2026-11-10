#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "fast_lio_nav2/gyro_bias_calibration.hpp"

namespace
{
using fast_lio_nav2::GyroBiasCalibration;
using fast_lio_nav2::GyroBiasCalibrationConfig;

GyroBiasCalibration::Quaternion yaw(double angle)
{
  return {0.0, 0.0, std::sin(angle / 2.0), std::cos(angle / 2.0)};
}

TEST(GyroBiasCalibration, MovingStartCannotBecomePermanentBias)
{
  GyroBiasCalibration calibration{GyroBiasCalibrationConfig{}};
  for (int i = 0; i < 700; ++i) {
    EXPECT_FALSE(calibration.add({0.0, 0.0, 0.35}, i * 0.002, false));
  }
  EXPECT_EQ(calibration.windowSamples(), 0u);
  const GyroBiasCalibration::Vector real_bias{0.004, -0.002, 0.003};
  for (int i = 700; i <= 1201; ++i) {
    calibration.add(real_bias, i * 0.002, true, yaw(0.98));
  }
  ASSERT_TRUE(calibration.calibrated());
  for (std::size_t i = 0; i < real_bias.size(); ++i) {
    EXPECT_NEAR(calibration.bias()[i], real_bias[i], 1.0e-12);
  }
}

TEST(GyroBiasCalibration, SampleCountAloneCannotSkipStationaryDuration)
{
  GyroBiasCalibration calibration{GyroBiasCalibrationConfig{}};
  for (int i = 0; i < 400; ++i) {
    EXPECT_FALSE(calibration.add({0.002, 0.0, 0.0}, i * 0.001, true, yaw(0.5)));
  }
  for (int i = 400; i <= 1001; ++i) {
    calibration.add({0.002, 0.0, 0.0}, i * 0.001, true, yaw(0.5));
  }
  EXPECT_TRUE(calibration.calibrated());
}

TEST(GyroBiasCalibration, ConstantSlowTurnWithZeroVarianceIsRejectedByAttitude)
{
  GyroBiasCalibration calibration{GyroBiasCalibrationConfig{}};
  for (int i = 0; i < 2000; ++i) {
    // Below the gyro magnitude gate and with exactly zero gyro variance.
    const double stamp = i * 0.002;
    EXPECT_FALSE(calibration.add({0.0, 0.0, 0.03}, stamp, true, yaw(0.03 * stamp)));
  }
  EXPECT_FALSE(calibration.calibrated());
}

TEST(GyroBiasCalibration, MissingOrientationKeepsDocumentedGyroOnlyFallback)
{
  GyroBiasCalibration calibration{GyroBiasCalibrationConfig{}};
  for (int i = 0; i <= 501; ++i) {
    calibration.add({0.0, 0.0, 0.003}, i * 0.002, false);
  }
  EXPECT_TRUE(calibration.calibrated());
  EXPECT_FALSE(calibration.windowHasOrientation());
}

TEST(GyroBiasCalibration, VibrationCannotPassEvenIfMagnitudeAndAttitudePass)
{
  GyroBiasCalibration calibration{GyroBiasCalibrationConfig{}};
  for (int i = 0; i < 2000; ++i) {
    EXPECT_FALSE(calibration.add(
      {0.0, 0.0, i % 2 == 0 ? 0.02 : -0.02}, i * 0.002, true, yaw(0.0)));
  }
  EXPECT_FALSE(calibration.calibrated());
}

TEST(GyroBiasCalibration, MotionInterruptsAndRequiresANewCompleteWindow)
{
  GyroBiasCalibration calibration{GyroBiasCalibrationConfig{}};
  for (int i = 0; i < 490; ++i) {
    EXPECT_FALSE(calibration.add({0.0, 0.0, 0.003}, i * 0.002, true, yaw(0.0)));
  }
  EXPECT_FALSE(calibration.add({0.0, 0.0, 0.35}, 0.98, true, yaw(0.0)));
  EXPECT_EQ(calibration.windowSamples(), 0u);
  for (int i = 491; i < 990; ++i) {
    EXPECT_FALSE(calibration.add({0.0, 0.0, 0.003}, i * 0.002, true, yaw(0.1)));
  }
  EXPECT_FALSE(calibration.calibrated());
  EXPECT_TRUE(calibration.add({0.0, 0.0, 0.003}, 1.984, true, yaw(0.1)));
}

TEST(GyroBiasCalibration, TimestampGapAndInvalidSampleInterruptTheWindow)
{
  GyroBiasCalibration calibration{GyroBiasCalibrationConfig{}};
  for (int i = 0; i < 490; ++i) {
    calibration.add({0.0, 0.0, 0.003}, i * 0.002, false);
  }
  EXPECT_FALSE(calibration.add({0.0, 0.0, 0.003}, 1.5, false));
  EXPECT_EQ(calibration.windowSamples(), 1u);
  EXPECT_FALSE(calibration.add(
    {0.0, std::numeric_limits<double>::quiet_NaN(), 0.003}, 1.502, false));
  EXPECT_EQ(calibration.windowSamples(), 0u);
  calibration.add({0.0, 0.0, 0.003}, 1.6, false);
  EXPECT_FALSE(calibration.add({0.0, 0.0, 0.003}, 1.59, false));
  EXPECT_EQ(calibration.windowSamples(), 1u);
}

TEST(GyroBiasCalibration, QuaternionSignAndScaleDoNotImplyMotion)
{
  GyroBiasCalibration calibration{GyroBiasCalibrationConfig{}};
  for (int i = 0; i <= 501; ++i) {
    auto q = yaw(2.0);
    for (double & component : q) {
      component *= i % 2 == 0 ? 2.0 : -0.5;
    }
    calibration.add({0.002, 0.0, 0.0}, i * 0.002, true, q);
  }
  EXPECT_TRUE(calibration.calibrated());
}

TEST(GyroBiasCalibration, InvalidThresholdsAreRejected)
{
  GyroBiasCalibrationConfig config;
  config.maximum_gyro_norm = 0.0;
  EXPECT_THROW(GyroBiasCalibration{config}, std::invalid_argument);
}
}  // namespace
