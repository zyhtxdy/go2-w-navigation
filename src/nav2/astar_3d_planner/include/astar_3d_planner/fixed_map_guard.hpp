#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace astar_3d_planner
{

// SHA-256 of the actual packed message bytes, independent of its header stamp.
std::string payloadSha256(const std::vector<uint8_t> & data);

// Validate the fixed map before admitting it to an offline planner. A conflicting
// map or publisher latches a fault: recreating the guard is required to recover.
// Missing publishers are a waiting state, and may recover without restarting.
class FixedMapGuard
{
public:
  enum class Layer {Support, Obstacles};
  enum class Decision {Accept, Duplicate, Reject};

  FixedMapGuard(
    std::string frame, std::string expected_map_id,
    std::string expected_support_sha256, std::string expected_obstacles_sha256);

  Decision validateCloud(Layer layer, const sensor_msgs::msg::PointCloud2 & cloud);
  bool observeIdentity(const std::string & map_id);

  // One entry per discovered publisher endpoint. Entries contain the fully
  // qualified node name, so two endpoints from one node still count as two.
  bool observePublishers(
    const std::vector<std::string> & support,
    const std::vector<std::string> & obstacles,
    const std::vector<std::string> & identity);

  bool ready() const;
  bool faulted() const;
  std::string reason() const;
  // Compact JSON suitable for persistent diagnostics and std_msgs/String.
  std::string report() const;

private:
  struct CloudState
  {
    std::string expected_sha256;
    std::string received_sha256;
    uint32_t points{0};
    uint32_t received_points{0};
    uint64_t accepted{0};
    uint64_t duplicates{0};
    uint64_t rejected{0};
    bool verified{false};
  };

  bool readyUnlocked() const;
  std::string reasonUnlocked() const;
  void faultUnlocked(const std::string & reason);

  mutable std::mutex mutex_;
  std::string frame_;
  std::string expected_map_id_;
  std::string received_map_id_;
  CloudState support_;
  CloudState obstacles_;
  bool identity_verified_{false};
  bool graph_healthy_{false};
  bool faulted_{false};
  std::string fault_reason_;
  std::string graph_reason_{"waiting_for_publishers"};
  std::vector<std::string> support_sources_;
  std::vector<std::string> obstacle_sources_;
  std::vector<std::string> identity_sources_;
};

}  // namespace astar_3d_planner
