#ifndef FAST_LIO_NAV2__GYRO_BIAS_CALIBRATION_HPP_
#define FAST_LIO_NAV2__GYRO_BIAS_CALIBRATION_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace fast_lio_nav2
{

// A startup stationary window, not a motion/bias observer. A gyro alone cannot
// distinguish an arbitrarily slow constant turn from a constant sensor bias.
struct GyroBiasCalibrationConfig
{
  std::size_t samples{400};
  double minimum_duration{1.0};                 // seconds
  double maximum_gyro_norm{0.05};              // rad/s
  double maximum_axis_variance{2.0e-4};        // (rad/s)^2
  double maximum_orientation_change{0.01};     // rad from the window's first pose
  double maximum_sample_gap{0.1};              // seconds
};

class GyroBiasCalibration
{
public:
  using Vector = std::array<double, 3>;
  using Quaternion = std::array<double, 4>;  // xyzw

  explicit GyroBiasCalibration(const GyroBiasCalibrationConfig & config)
  : config_(config)
  {
    if (config.samples < 2 ||
      !positiveFinite(config.minimum_duration) ||
      !positiveFinite(config.maximum_gyro_norm) ||
      !positiveFinite(config.maximum_axis_variance) ||
      !positiveFinite(config.maximum_orientation_change) ||
      !positiveFinite(config.maximum_sample_gap))
    {
      throw std::invalid_argument("Invalid stationary gyro calibration thresholds");
    }
  }

  bool add(
    const Vector & omega, double stamp, bool orientation_available,
    Quaternion orientation = Quaternion{0.0, 0.0, 0.0, 1.0})
  {
    if (calibrated_) {
      return true;
    }
    if (!std::isfinite(stamp) || stamp < 0.0 ||
      !std::all_of(omega.begin(), omega.end(), [](double v) {return std::isfinite(v);}))
    {
      resetWindow();
      return false;
    }
    if (std::hypot(std::hypot(omega[0], omega[1]), omega[2]) > config_.maximum_gyro_norm) {
      resetWindow();
      return false;
    }

    orientation_available = orientation_available && normalize(orientation);
    if (count_ > 0 &&
      (stamp <= last_stamp_ || stamp - last_stamp_ > config_.maximum_sample_gap ||
      orientation_available != have_orientation_))
    {
      // Do not combine windows across clock jumps, missing samples or a change
      // in whether relative orientation can be checked. This sample may start
      // a new window, but can never complete the interrupted window.
      resetWindow();
    }
    if (count_ == 0) {
      first_stamp_ = stamp;
      first_orientation_ = orientation;
      have_orientation_ = orientation_available;
    } else if (have_orientation_) {
      double dot = 0.0;
      for (std::size_t i = 0; i < orientation.size(); ++i) {
        dot += first_orientation_[i] * orientation[i];
      }
      // q and -q describe the same orientation. Compare with the first pose,
      // not just the preceding pose: small constant turns must accumulate.
      const double angle = 2.0 * std::acos(std::min(1.0, std::abs(dot)));
      if (angle > config_.maximum_orientation_change) {
        resetWindow();
        return false;
      }
    }

    ++count_;
    last_stamp_ = stamp;
    for (std::size_t i = 0; i < omega.size(); ++i) {
      const double delta = omega[i] - mean_[i];
      mean_[i] += delta / static_cast<double>(count_);
      m2_[i] += delta * (omega[i] - mean_[i]);
    }
    if (count_ < config_.samples || stamp - first_stamp_ < config_.minimum_duration) {
      return false;
    }

    for (std::size_t i = 0; i < variance_.size(); ++i) {
      variance_[i] = std::max(m2_[i] / static_cast<double>(count_ - 1), 1.0e-8);
      if (variance_[i] > config_.maximum_axis_variance) {
        resetWindow();
        return false;
      }
    }
    bias_ = mean_;
    calibrated_ = true;
    return true;
  }

  bool calibrated() const {return calibrated_;}
  const Vector & bias() const {return bias_;}
  const Vector & variance() const {return variance_;}
  std::size_t windowSamples() const {return count_;}
  bool windowHasOrientation() const {return have_orientation_;}

private:
  static bool positiveFinite(double value) {return std::isfinite(value) && value > 0.0;}

  static bool normalize(Quaternion & q)
  {
    double scale = 0.0;
    for (const double value : q) {
      if (!std::isfinite(value)) {
        return false;
      }
      scale = std::max(scale, std::abs(value));
    }
    if (scale == 0.0) {
      return false;
    }
    double norm_squared = 0.0;
    for (double & value : q) {
      value /= scale;
      norm_squared += value * value;
    }
    const double norm = std::sqrt(norm_squared);
    for (double & value : q) {
      value /= norm;
    }
    return true;
  }

  void resetWindow()
  {
    count_ = 0;
    mean_.fill(0.0);
    m2_.fill(0.0);
    have_orientation_ = false;
  }

  GyroBiasCalibrationConfig config_;
  bool calibrated_{false};
  std::size_t count_{0};
  double first_stamp_{0.0};
  double last_stamp_{0.0};
  bool have_orientation_{false};
  Quaternion first_orientation_{0.0, 0.0, 0.0, 1.0};
  Vector mean_{0.0, 0.0, 0.0};
  Vector m2_{0.0, 0.0, 0.0};
  Vector bias_{0.0, 0.0, 0.0};
  Vector variance_{1.0e-6, 1.0e-6, 1.0e-6};
};

}  // namespace fast_lio_nav2

#endif  // FAST_LIO_NAV2__GYRO_BIAS_CALIBRATION_HPP_
