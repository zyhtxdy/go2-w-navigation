#ifndef ASTAR_3D_PLANNER__LIVE_OBSTACLE_HEALTH_HPP_
#define ASTAR_3D_PLANNER__LIVE_OBSTACLE_HEALTH_HPP_

#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <functional>
#include <mutex>

namespace astar_3d_planner
{
// Successful map consumption, not DDS reception. Uses both clocks so a paused
// replay clock or repeated old sensor stamps cannot keep navigation enabled.
class LiveObstacleHealth
{
public:
  explicit LiveObstacleHealth(double timeout = 1.0) {setTimeout(timeout);}

  void setTimeout(double timeout)
  {
    if (!std::isfinite(timeout) || timeout <= 0.0 || timeout > 3600.0) {
      throw std::invalid_argument("Invalid live obstacle freshness timeout");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    maximum_age_ns_ = static_cast<std::int64_t>(std::llround(timeout * 1e9));
  }

  struct Status
  {
    bool ready{false};
    const char * reason{"waiting_for_live_obstacle_update"};
    std::int64_t stamp_ns{0};
    std::int64_t processed_steady_ns{0};
    std::uint64_t updates{0};
    std::int64_t checked_steady_ns{0};
  };

  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = Status{};
  }

  bool consumed(std::int64_t stamp_ns, std::int64_t steady_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stamp_ns <= 0 || steady_ns <= 0 || stamp_ns <= status_.stamp_ns) {return false;}
    status_.stamp_ns = stamp_ns;
    status_.processed_steady_ns = steady_ns;
    ++status_.updates;
    return true;
  }

  Status status(std::int64_t ros_ns, std::int64_t steady_ns) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return evaluate(ros_ns, steady_ns);
  }

  // Sampling a clock before acquiring the mutex can put "now" before a
  // concurrent consumption. Production callers must sample both clocks while
  // holding the same lock used to commit successful processing.
  Status statusNow(const std::function<std::int64_t()> & ros_now) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    return evaluate(ros_now(), steady_ns);
  }

private:
  Status evaluate(std::int64_t ros_ns, std::int64_t steady_ns) const
  {
    auto result = status_;
    result.checked_steady_ns = steady_ns;
    if (result.stamp_ns <= 0) {return result;}
    if (ros_ns < result.stamp_ns || steady_ns < result.processed_steady_ns) {
      result.reason = "live_obstacle_clock_invalid";
    } else if (ros_ns - result.stamp_ns > maximum_age_ns_) {
      result.reason = "live_obstacle_source_stale";
    } else if (steady_ns - result.processed_steady_ns > maximum_age_ns_) {
      result.reason = "live_obstacle_processing_stale";
    } else {
      result.ready = true;
      result.reason = "ready";
    }
    return result;
  }

  mutable std::mutex mutex_;
  Status status_;
  std::int64_t maximum_age_ns_{1000000000LL};
};
}  // namespace astar_3d_planner

#endif
