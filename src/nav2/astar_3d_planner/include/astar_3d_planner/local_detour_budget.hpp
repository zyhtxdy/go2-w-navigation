#ifndef ASTAR_3D_PLANNER__LOCAL_DETOUR_BUDGET_HPP_
#define ASTAR_3D_PLANNER__LOCAL_DETOUR_BUDGET_HPP_

#include <array>
#include <cmath>
#include <stdexcept>

namespace astar_3d_planner
{
// Caller serializes access. Time is a steady-clock value, never a ROS stamp.
// No path identifier is accepted: replacing a path must not renew this budget.
class LocalDetourBudget
{
public:
  struct Config
  {
    double max_duration_s{12.0};
    double no_progress_timeout_s{4.0};
    double progress_distance{0.05};
    double progress_angle{0.2};
  };
  enum class State {Clear, Allowed, Stalled, Expired, Invalid};
  struct Decision
  {
    State state{State::Clear};
    bool allowed() const {return state == State::Clear || state == State::Allowed;}
    const char * reason() const
    {
      switch (state) {
        case State::Clear: return "valid";
        case State::Allowed: return "local_detour_active";
        case State::Stalled: return "local_detour_no_progress";
        case State::Expired: return "local_detour_timeout";
        default: return "local_detour_invalid_state";
      }
    }
  };

  LocalDetourBudget() = default;
  explicit LocalDetourBudget(const Config & config) {configure(config);}

  void configure(const Config & config)
  {
    if (!positive(config.max_duration_s) || !positive(config.no_progress_timeout_s) ||
      !positive(config.progress_distance) || !positive(config.progress_angle))
    {throw std::invalid_argument("Positive finite local detour limits required");}
    config_ = config;
    reset();
  }

  void reset()
  {
    have_goal_ = have_time_ = false;
    clearEpisode();
  }

  Decision decision() const {return {state_};}

  // goal = requested x/y/z/yaw; robot = current x/y/yaw.
  // A valid clear observation or a different goal releases a latched failure.
  Decision update(const std::array<double, 4> & goal, double now_s,
    const std::array<double, 3> & robot, bool needs_detour)
  {
    if (!std::isfinite(now_s) || now_s < 0.0 || !finite(goal) || !finite(robot) ||
      (have_time_ && now_s < last_now_))
    {state_ = State::Invalid; return decision();}
    last_now_ = now_s; have_time_ = true;
    if (!have_goal_ || std::hypot(goal[0]-goal_[0], goal[1]-goal_[1], goal[2]-goal_[2]) >
      0.01 || angleDistance(goal[3], goal_[3]) > 0.01)
    {goal_ = goal; have_goal_ = true; clearEpisode();}
    if (!needs_detour) {clearEpisode(); return decision();}
    if (state_ == State::Stalled || state_ == State::Expired || state_ == State::Invalid) {
      return decision();
    }
    if (state_ == State::Clear) {
      state_ = State::Allowed;
      began_ = last_progress_ = now_s; progress_pose_ = robot;
      return decision();
    }
    if (now_s-began_ >= config_.max_duration_s) {
      state_ = State::Expired; return decision();
    }
    // A late observation must not revive an already exhausted progress budget.
    if (now_s-last_progress_ >= config_.no_progress_timeout_s) {
      state_ = State::Stalled; return decision();
    }
    if (std::hypot(robot[0]-progress_pose_[0], robot[1]-progress_pose_[1]) >=
      config_.progress_distance || angleDistance(robot[2], progress_pose_[2]) >=
      config_.progress_angle)
    {last_progress_ = now_s; progress_pose_ = robot;}
    return decision();
  }

private:
  static bool positive(double value) {return std::isfinite(value) && value > 0.0;}
  template<std::size_t N>
  static bool finite(const std::array<double, N> & values)
  {
    for (const auto value : values) {if (!std::isfinite(value)) {return false;}}
    return true;
  }
  static double angleDistance(double a, double b)
  {
    constexpr double tau = 6.28318530717958647692;
    return std::abs(std::remainder(std::remainder(a, tau)-std::remainder(b, tau), tau));
  }
  void clearEpisode() {state_ = State::Clear; began_ = last_progress_ = 0.0;}

  Config config_{};
  State state_{State::Clear};
  bool have_goal_{false}, have_time_{false};
  std::array<double, 4> goal_{};
  std::array<double, 3> progress_pose_{};
  double last_now_{0.0}, began_{0.0}, last_progress_{0.0};
};
}  // namespace astar_3d_planner

#endif
