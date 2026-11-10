#include "terrain_costmap/near_obstacle_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace terrain_costmap
{
namespace
{

double squaredXYDistance(const Point & a, const Point & b)
{
  const double dx = a.x - b.x, dy = a.y - b.y;
  return dx * dx + dy * dy;
}

}  // namespace

NearObstacleDetector::NearObstacleDetector(const NearObstacleParameters & parameters)
: parameters_(parameters)
{
  for (double value : {
      parameters.minimum_range, parameters.maximum_range,
      parameters.minimum_relative_z, parameters.maximum_relative_z,
      parameters.self_minimum_x, parameters.self_maximum_x,
      parameters.self_minimum_y, parameters.self_maximum_y,
      parameters.voxel_size, parameters.maximum_xy_distance,
      parameters.minimum_obstacle_height, parameters.maximum_ground_slope,
      parameters.lower_layer_tolerance, parameters.cluster_radius})
  {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("non-finite near obstacle detector parameter");
    }
  }
  if (parameters.minimum_range < 0.0 ||
    parameters.maximum_range <= parameters.minimum_range ||
    parameters.maximum_relative_z <= parameters.minimum_relative_z ||
    parameters.self_maximum_x <= parameters.self_minimum_x ||
    parameters.self_maximum_y <= parameters.self_minimum_y ||
    parameters.voxel_size <= 0.0 || parameters.maximum_xy_distance <= 0.0 ||
    parameters.minimum_obstacle_height <= 0.0 || parameters.maximum_ground_slope < 0.0 ||
    parameters.lower_layer_tolerance < 0.0 || parameters.minimum_cluster_points < 3 ||
    parameters.cluster_radius <= 0.0)
  {
    throw std::invalid_argument("invalid near obstacle detector parameters");
  }
}

std::vector<TerrainPoint> NearObstacleDetector::detect(
  const std::vector<Point> & base_points) const
{
  if (!parameters_.enabled) {return {};}

  using Voxel = std::tuple<std::int64_t, std::int64_t, std::int64_t>;
  std::map<Voxel, Point> occupied;
  for (const Point & point : base_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const double range = std::hypot(point.x, point.y);
    if (range < parameters_.minimum_range || range > parameters_.maximum_range ||
      point.z < parameters_.minimum_relative_z || point.z > parameters_.maximum_relative_z ||
      (point.x >= parameters_.self_minimum_x && point.x <= parameters_.self_maximum_x &&
      point.y >= parameters_.self_minimum_y && point.y <= parameters_.self_maximum_y))
    {
      continue;
    }
    const double vx = std::floor(point.x / parameters_.voxel_size);
    const double vy = std::floor(point.y / parameters_.voxel_size);
    const double vz = std::floor(point.z / parameters_.voxel_size);
    // Reject an unusably small voxel configuration without an overflowing cast.
    constexpr double index_limit = static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 2;
    if (!(std::abs(vx) < index_limit && std::abs(vy) < index_limit && std::abs(vz) < index_limit)) {
      continue;
    }
    const Voxel key{
      static_cast<std::int64_t>(vx), static_cast<std::int64_t>(vy), static_cast<std::int64_t>(vz)};
    const auto inserted = occupied.emplace(key, point);
    if (!inserted.second &&
      point.observation_stamp_ns > inserted.first->second.observation_stamp_ns)
    {
      inserted.first->second = point;
    }
  }

  std::vector<Point> points;
  points.reserve(occupied.size());
  for (const auto & entry : occupied) {points.push_back(entry.second);}
  const double maximum_xy_squared =
    parameters_.maximum_xy_distance * parameters_.maximum_xy_distance;
  std::vector<TerrainPoint> candidates;
  std::vector<std::size_t> nearby;
  for (const Point & point : points) {
    nearby.clear();
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (squaredXYDistance(point, points[i]) <= maximum_xy_squared) {nearby.push_back(i);}
    }
    std::sort(nearby.begin(), nearby.end(), [&](std::size_t a, std::size_t b) {
        return points[a].z < points[b].z || (points[a].z == points[b].z && a < b);
      });
    for (std::size_t i = 1; i < nearby.size(); ++i) {
      const Point & low = points[nearby[i - 1]];
      const Point & high = points[nearby[i]];
      if (high.z - low.z > parameters_.lower_layer_tolerance) {continue;}
      // The higher of two independently occupied voxels bounds the low layer.
      // A lone negative outlier cannot lower this reference, and a bounded
      // ground slope cannot pass the height gate even across the full XY span.
      const double xy_span = std::sqrt(std::max(
          squaredXYDistance(point, low), squaredXYDistance(point, high)));
      const double relative_height = point.z - high.z;
      if (relative_height > parameters_.minimum_obstacle_height +
        parameters_.maximum_ground_slope * xy_span)
      {
        candidates.push_back({point, relative_height, false});
      }
      // Later pairs have a higher reference. Do not hunt for a more favorable
      // layer after the first independently observed, coherent lower layer.
      break;
    }
  }

  // Require a spatially connected cluster of independent candidate voxels.
  // Three unrelated high outliers cannot validate one another by point count.
  const double cluster_squared = parameters_.cluster_radius * parameters_.cluster_radius;
  std::vector<unsigned char> visited(candidates.size(), 0);
  std::vector<std::size_t> cluster;
  std::vector<TerrainPoint> obstacles;
  for (std::size_t seed = 0; seed < candidates.size(); ++seed) {
    if (visited[seed]) {continue;}
    cluster.clear();
    cluster.push_back(seed);
    visited[seed] = 1;
    for (std::size_t cursor = 0; cursor < cluster.size(); ++cursor) {
      const Point & point = candidates[cluster[cursor]].point;
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        if (visited[i]) {continue;}
        const Point & other = candidates[i].point;
        const double dz = point.z - other.z;
        if (squaredXYDistance(point, other) + dz * dz <= cluster_squared) {
          visited[i] = 1;
          cluster.push_back(i);
        }
      }
    }
    if (cluster.size() >= static_cast<std::size_t>(parameters_.minimum_cluster_points)) {
      for (std::size_t i : cluster) {obstacles.push_back(candidates[i]);}
    }
  }
  return obstacles;
}

}  // namespace terrain_costmap
