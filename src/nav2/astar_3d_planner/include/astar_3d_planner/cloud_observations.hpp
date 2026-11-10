#ifndef ASTAR_3D_PLANNER__CLOUD_OBSERVATIONS_HPP_
#define ASTAR_3D_PLANNER__CLOUD_OBSERVATIONS_HPP_

#include <cstdint>
#include <vector>

#include "astar_3d_planner/occupy_map.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"

namespace astar_3d_planner
{
// Decode packed terrain clouds without creating ROS nodes. Optional observation
// times stay paired with their original point through filtering and decimation.
// Legacy clouds lacking both observation fields explicitly yield unknown (0).
std::vector<OccupyMap::Point> transformCloudObservations(
  const sensor_msgs::msg::PointCloud2 & cloud, const tf2::Transform & transform,
  int point_stride, std::vector<std::uint8_t> * obstacle_sources = nullptr,
  std::vector<std::int64_t> * observed_stamps_ns = nullptr);
}  // namespace astar_3d_planner

#endif  // ASTAR_3D_PLANNER__CLOUD_OBSERVATIONS_HPP_
