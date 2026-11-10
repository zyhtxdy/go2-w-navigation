#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>

namespace go2_relocalization
{
inline double rotationDistance(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}

// Confirmation idleness is measured since result completion. Sensor-time
// spacing and result freshness are separate gates, so compute time cannot
// silently extend the validity of a localization estimate.
inline bool confirmationIdleExpired(
  double now, double input_stamp, double completed_stamp, double timeout)
{
  return !std::isfinite(now) || !std::isfinite(input_stamp) ||
         !std::isfinite(completed_stamp) || now - completed_stamp > timeout ||
         now - input_stamp < -0.2;
}

// Periodic verification is scheduled after the previous global job completed.
// Otherwise a job taking longer than its interval immediately schedules another
// global job forever, starving lightweight tracking. Loss still searches now.
inline bool globalCheckDue(bool tracking, double now, double completed, double interval)
{
  return !tracking || !std::isfinite(now) || !std::isfinite(completed) ||
         !std::isfinite(interval) || interval <= 0 || completed < 0 || now < completed ||
         now - completed + 1e-6 >= interval;
}

// Waiting the full idle interval after a slow job can consume the remaining
// all-hypothesis verification budget before the next result arrives. This
// only starts work sooner; the actual scan-age and verification gates remain
// independent and must still reject late results.
inline bool globalCheckDueBeforeDeadline(
  bool tracking, double now, double completed, double interval,
  double verified_stamp, double maximum_age, double estimated_seconds, double reserve_seconds)
{
  if (globalCheckDue(tracking, now, completed, interval)) {return true;}
  if (!std::isfinite(verified_stamp) || verified_stamp <= 0 ||
    !std::isfinite(maximum_age) || maximum_age <= 0 ||
    !std::isfinite(estimated_seconds) || estimated_seconds < 0 ||
    !std::isfinite(reserve_seconds) || reserve_seconds < 0)
  {return true;}
  return now - verified_stamp + estimated_seconds + reserve_seconds >= maximum_age;
}

// Only bounds the age of a pose INITIAL GUESS. The fresh scan used to verify
// the proposal is independently subject to the unchanged max_input_age gate.
inline bool globalProposalFresh(double now, double stamp, double maximum_age)
{
  return std::isfinite(now) && std::isfinite(stamp) && std::isfinite(maximum_age) &&
         stamp > 0 && maximum_age > 0 && now >= stamp && now - stamp <= maximum_age;
}

// Startup poses retain the strict foreground scan budget. While tracking,
// completed all-hypothesis jobs have a separate compute/queue budget; their
// original stamp is still used for the global deadline, and a CURRENT scan
// must independently pass tracking before the result can be published.
inline bool globalResultFresh(double now,double stamp,bool tracking,
  double foreground_age,double background_age,double global_deadline)
{
  if(!std::isfinite(foreground_age) || foreground_age<=0 ||
    !std::isfinite(background_age) || background_age<=0 ||
    !std::isfinite(global_deadline) || global_deadline<=0) {return false;}
  const double limit=tracking ? std::min(background_age,global_deadline) : foreground_age;
  return globalProposalFresh(now,stamp,limit);
}

struct TimedPose
{
  double stamp;
  Eigen::Isometry3d pose;
};

// Lookup uses sensor timestamps, never callback arrival times.
class PoseHistory
{
public:
  bool add(double stamp, const Eigen::Isometry3d & pose)
  {
    if (!std::isfinite(stamp) || !pose.matrix().allFinite()) {return false;}
    if (!poses_.empty() && stamp <= poses_.back().stamp) {return false;}
    poses_.push_back({stamp, pose});
    while (poses_.size() > 4000 || poses_.front().stamp < stamp - 10.0) {
      poses_.pop_front();
    }
    return true;
  }
  void clear() {poses_.clear();}
  std::optional<TimedPose> latest() const
  {
    if (poses_.empty()) {return std::nullopt;}
    return poses_.back();
  }
  std::optional<Eigen::Isometry3d> at(double stamp, double max_age) const
  {
    if (!std::isfinite(stamp) || !std::isfinite(max_age) || max_age <= 0 || poses_.empty()) {
      return std::nullopt;
    }
    const auto upper = std::lower_bound(poses_.begin(), poses_.end(), stamp,
      [](const TimedPose & p, double t) {return p.stamp < t;});
    if (upper != poses_.end() && std::abs(upper->stamp - stamp) < 1e-9) {return upper->pose;}
    if (upper == poses_.begin()) {
      return std::abs(upper->stamp - stamp) <= max_age ?
        std::optional<Eigen::Isometry3d>(upper->pose) : std::nullopt;
    }
    if (upper == poses_.end()) {
      return std::abs(poses_.back().stamp - stamp) <= max_age ?
        std::optional<Eigen::Isometry3d>(poses_.back().pose) : std::nullopt;
    }
    const auto & lower = *std::prev(upper);
    if (upper->stamp - stamp > max_age || stamp - lower.stamp > max_age) {
      return std::nullopt;
    }
    const double alpha = (stamp - lower.stamp) / (upper->stamp - lower.stamp);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = (1.0 - alpha) * lower.pose.translation() + alpha * upper->pose.translation();
    pose.linear() = Eigen::Quaterniond(lower.pose.linear()).slerp(
      alpha, Eigen::Quaterniond(upper->pose.linear())).toRotationMatrix();
    return pose;
  }

private:
  std::deque<TimedPose> poses_;
};

class ConfirmationGate
{
public:
  ConfirmationGate(int frames, double interval, double translation, double rotation,
    double max_gap = 3.0)
  : frames_(frames), interval_(interval), translation_(translation), rotation_(rotation),
    max_gap_(max_gap) {}

  // Compare map->odom, allowing the robot to move between observations.
  bool observe(double stamp, const Eigen::Isometry3d & map_to_odom)
  {
    if (!std::isfinite(stamp) || !map_to_odom.matrix().allFinite()) {reset(); return false;}
    if (count_ > 0 && stamp <= last_stamp_) {return false;}
    if (count_ > 0 && stamp - last_stamp_ < interval_) {return false;}
    if (count_ == 0 || stamp - last_stamp_ > max_gap_ ||
      (map_to_odom.translation() - anchor_.translation()).norm() > translation_ ||
      rotationDistance(map_to_odom, anchor_) > rotation_)
    {
      anchor_ = map_to_odom;
      count_ = 1;
    } else {
      ++count_;
    }
    last_stamp_ = stamp;
    return count_ >= frames_;
  }
  void reset() {count_ = 0; last_stamp_ = -1.0;}
  int count() const {return count_;}

private:
  int frames_;
  double interval_, translation_, rotation_, max_gap_;
  int count_{0};
  double last_stamp_{-1.0};
  Eigen::Isometry3d anchor_{Eigen::Isometry3d::Identity()};
};
}  // namespace go2_relocalization
