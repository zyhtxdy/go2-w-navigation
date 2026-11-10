#include "astar_3d_planner/cloud_observations.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace astar_3d_planner
{
namespace
{
const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  const sensor_msgs::msg::PointField * result = nullptr;
  for (const auto & field : cloud.fields) {
    if (field.name != name) {continue;}
    if (result) {throw std::invalid_argument("Duplicate point field: " + name);}
    result = &field;
  }
  return result;
}

void requireField(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const sensor_msgs::msg::PointField * field, const std::string & name,
  std::uint8_t datatype, std::uint32_t bytes)
{
  if (!field || field->datatype != datatype || field->count != 1 ||
    field->offset > cloud.point_step || bytes > cloud.point_step - field->offset ||
    (bytes == 4 && (field->offset % 4 != 0 || cloud.point_step % 4 != 0)))
  {
    throw std::invalid_argument("Invalid point field: " + name);
  }
}
}  // namespace

std::vector<OccupyMap::Point> transformCloudObservations(
  const sensor_msgs::msg::PointCloud2 & cloud, const tf2::Transform & transform,
  int point_stride, std::vector<std::uint8_t> * obstacle_sources,
  std::vector<std::int64_t> * observed_stamps_ns)
{
  if (obstacle_sources) {obstacle_sources->clear();}
  if (observed_stamps_ns) {observed_stamps_ns->clear();}
  if (point_stride < 1) {throw std::invalid_argument("Point stride must be positive");}
  // Humble's iterators use data.front()/back() during construction.
  if (cloud.width == 0 || cloud.height == 0) {
    if (!cloud.data.empty()) {throw std::invalid_argument("Empty point cloud has a payload");}
    return {};
  }
  // Humble's typed iterators do not byte-swap numeric fields. Our terrain
  // publishers use little-endian, aligned layouts on the laptop and NX.
  if (cloud.is_bigendian) {
    throw std::invalid_argument("Big-endian terrain clouds are unsupported");
  }
  const auto packed_row = static_cast<std::uint64_t>(cloud.width) * cloud.point_step;
  const auto expected_bytes = packed_row * cloud.height;
  if (cloud.point_step == 0 || cloud.row_step != packed_row ||
    cloud.data.size() != expected_bytes)
  {
    throw std::invalid_argument("Terrain cloud must contain complete packed point rows");
  }
  for (const auto * name : {"x", "y", "z"}) {
    requireField(cloud, findField(cloud, name), name, sensor_msgs::msg::PointField::FLOAT32, 4);
  }

  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint8_t>> source;
  if (obstacle_sources) {
    const auto * field = findField(cloud, "obstacle_source");
    if (field) {
      requireField(cloud, field, "obstacle_source", sensor_msgs::msg::PointField::UINT8, 1);
      source = std::make_unique<sensor_msgs::PointCloud2ConstIterator<std::uint8_t>>(
        cloud, "obstacle_source");
    }
  }
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::int32_t>> seconds;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint32_t>> nanoseconds;
  if (observed_stamps_ns) {
    const auto * sec_field = findField(cloud, "observation_sec");
    const auto * nsec_field = findField(cloud, "observation_nanosec");
    if (sec_field || nsec_field) {
      requireField(cloud, sec_field, "observation_sec", sensor_msgs::msg::PointField::INT32, 4);
      requireField(cloud, nsec_field, "observation_nanosec", sensor_msgs::msg::PointField::UINT32, 4);
      seconds = std::make_unique<sensor_msgs::PointCloud2ConstIterator<std::int32_t>>(
        cloud, "observation_sec");
      nanoseconds = std::make_unique<sensor_msgs::PointCloud2ConstIterator<std::uint32_t>>(
        cloud, "observation_nanosec");
    }
  }
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  std::vector<OccupyMap::Point> points;
  const auto count = static_cast<std::uint64_t>(cloud.width) * cloud.height;
  const auto capacity = count / static_cast<std::uint64_t>(point_stride) + 1;
  points.reserve(capacity);
  if (obstacle_sources) {obstacle_sources->reserve(capacity);}
  if (observed_stamps_ns) {observed_stamps_ns->reserve(capacity);}
  for (std::uint64_t index = 0; index < count; ++index, ++x, ++y, ++z) {
    const auto provenance = source ? **source : static_cast<std::uint8_t>(OccupyMap::kUnknown);
    std::int64_t observed_stamp = 0;
    if (seconds) {
      if (**seconds < 0 || **nanoseconds >= 1000000000u) {
        throw std::invalid_argument("Invalid point observation timestamp");
      }
      observed_stamp = static_cast<std::int64_t>(**seconds) * 1000000000ll + **nanoseconds;
      ++(*seconds); ++(*nanoseconds);
    }
    if (source) {++(*source);}
    if (index % point_stride != 0 ||
      !std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
    const tf2::Vector3 transformed = transform * tf2::Vector3(*x, *y, *z);
    points.push_back({transformed.x(), transformed.y(), transformed.z()});
    if (obstacle_sources) {obstacle_sources->push_back(provenance);}
    if (observed_stamps_ns) {observed_stamps_ns->push_back(observed_stamp);}
  }
  return points;
}
}  // namespace astar_3d_planner
