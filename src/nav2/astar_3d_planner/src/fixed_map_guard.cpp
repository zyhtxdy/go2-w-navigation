#include "astar_3d_planner/fixed_map_guard.hpp"

#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "sensor_msgs/msg/point_field.hpp"

namespace astar_3d_planner
{
namespace
{

bool normalizeHash(std::string & value)
{
  if (value.size() != 64) {return false;}
  for (char & character : value) {
    if (character >= 'A' && character <= 'F') {character += 'a' - 'A';}
    if (!((character >= '0' && character <= '9') ||
      (character >= 'a' && character <= 'f')))
    {
      return false;
    }
  }
  return true;
}

std::string quoted(const std::string & value)
{
  std::ostringstream stream;
  stream << '"';
  for (unsigned char character : value) {
    switch (character) {
      case '"': stream << "\\\""; break;
      case '\\': stream << "\\\\"; break;
      case '\n': stream << "\\n"; break;
      case '\r': stream << "\\r"; break;
      case '\t': stream << "\\t"; break;
      default:
        if (character < 0x20) {
          stream << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<unsigned int>(character) << std::dec;
        } else {
          stream << character;
        }
    }
  }
  stream << '"';
  return stream.str();
}

std::string sourcesJson(const std::vector<std::string> & sources)
{
  std::ostringstream stream;
  stream << '[';
  for (size_t index = 0; index < sources.size(); ++index) {
    if (index) {stream << ',';}
    stream << quoted(sources[index]);
  }
  stream << ']';
  return stream.str();
}

std::string formatError(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & frame, bool allow_sources)
{
  if (cloud.header.frame_id != frame) {return "frame_mismatch";}
  const bool sources = allow_sources && cloud.point_step == 16;
  if (cloud.is_bigendian || cloud.height != 1 ||
    (!sources && cloud.point_step != 12))
  {
    return "noncanonical_xyz_layout";
  }
  const uint64_t bytes = static_cast<uint64_t>(cloud.width) * cloud.point_step;
  if (bytes > std::numeric_limits<uint32_t>::max() ||
    cloud.row_step != bytes || cloud.data.size() != bytes)
  {
    return "invalid_xyz_data_size";
  }
  if (cloud.fields.size() != (sources ? 4u : 3u)) {return "noncanonical_xyz_fields";}
  std::array<bool, 4> found{{false, false, false, false}};
  for (const auto & field : cloud.fields) {
    size_t index = 4;
    if (field.name == "x") {index = 0;}
    if (field.name == "y") {index = 1;}
    if (field.name == "z") {index = 2;}
    if (sources && field.name == "obstacle_source") {index = 3;}
    if (index == 4 || found[index] || field.offset != index * 4 ||
      field.datatype != (index == 3 ? sensor_msgs::msg::PointField::UINT8 :
      sensor_msgs::msg::PointField::FLOAT32) || field.count != 1)
    {
      return "noncanonical_xyz_fields";
    }
    found[index] = true;
  }
  // Decode canonical little endian XYZ; source bytes are semantic data, not floats.
  for (size_t point = 0; point < cloud.data.size(); point += cloud.point_step) {
    for (size_t axis = 0; axis < 3; ++axis) {
      const size_t offset = point + axis * 4;
      const uint32_t bits = static_cast<uint32_t>(cloud.data[offset]) |
        (static_cast<uint32_t>(cloud.data[offset + 1]) << 8) |
        (static_cast<uint32_t>(cloud.data[offset + 2]) << 16) |
        (static_cast<uint32_t>(cloud.data[offset + 3]) << 24);
      float value;
      std::memcpy(&value, &bits, sizeof(value));
      if (!std::isfinite(value)) {return "nonfinite_xyz";}
    }
    if (sources) {
      const auto tag = cloud.data[point + 12];
      if (tag != 1 && tag != 2 && tag != 4) {return "invalid_obstacle_source";}
      if (cloud.data[point + 13] || cloud.data[point + 14] || cloud.data[point + 15]) {
        return "noncanonical_source_padding";
      }
    }
  }
  return {};
}

}  // namespace

std::string payloadSha256(const std::vector<uint8_t> & data)
{
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0;
  if (EVP_Digest(data.data(), data.size(), digest.data(), &length, EVP_sha256(), nullptr) != 1 ||
    length != 32)
  {
    throw std::runtime_error("Could not compute fixed map SHA-256");
  }
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < length; ++index) {
    stream << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return stream.str();
}

FixedMapGuard::FixedMapGuard(
  std::string frame, std::string expected_map_id,
  std::string expected_support_sha256, std::string expected_obstacles_sha256)
: frame_(std::move(frame)), expected_map_id_(std::move(expected_map_id))
{
  support_.expected_sha256 = std::move(expected_support_sha256);
  obstacles_.expected_sha256 = std::move(expected_obstacles_sha256);
  if (frame_.empty() || !normalizeHash(expected_map_id_) ||
    !normalizeHash(support_.expected_sha256) || !normalizeHash(obstacles_.expected_sha256))
  {
    faultUnlocked("invalid_expected_map_identity");
  }
}

FixedMapGuard::Decision FixedMapGuard::validateCloud(
  Layer layer, const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::lock_guard<std::mutex> lock(mutex_);
  CloudState & state = layer == Layer::Support ? support_ : obstacles_;
  const std::string name = layer == Layer::Support ? "support" : "obstacles";
  state.received_points = cloud.width;
  if (faulted_) {
    ++state.rejected;
    return Decision::Reject;
  }
  std::string error = formatError(cloud, frame_, layer == Layer::Obstacles);
  if (error.empty() && layer == Layer::Support && cloud.width == 0) {
    error = "empty_support";
  }
  if (!error.empty()) {
    ++state.rejected;
    faultUnlocked(name + "_" + error);
    return Decision::Reject;
  }
  try {
    state.received_sha256 = payloadSha256(cloud.data);
  } catch (const std::exception &) {
    ++state.rejected;
    faultUnlocked(name + "_digest_error");
    return Decision::Reject;
  }
  if (state.received_sha256 != state.expected_sha256) {
    ++state.rejected;
    faultUnlocked(name + "_payload_sha256_mismatch");
    return Decision::Reject;
  }
  if (state.verified) {
    ++state.duplicates;
    return Decision::Duplicate;
  }
  state.verified = true;
  state.points = cloud.width;
  ++state.accepted;
  return Decision::Accept;
}

bool FixedMapGuard::observeIdentity(const std::string & map_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  received_map_id_ = map_id;
  if (faulted_) {return false;}
  std::string normalized = map_id;
  if (!normalizeHash(normalized)) {
    faultUnlocked("invalid_map_id");
    return false;
  }
  if (normalized != expected_map_id_) {
    faultUnlocked("map_id_mismatch");
    return false;
  }
  identity_verified_ = true;
  return true;
}

bool FixedMapGuard::observePublishers(
  const std::vector<std::string> & support,
  const std::vector<std::string> & obstacles,
  const std::vector<std::string> & identity)
{
  std::lock_guard<std::mutex> lock(mutex_);
  support_sources_ = support;
  obstacle_sources_ = obstacles;
  identity_sources_ = identity;
  graph_healthy_ = false;
  if (faulted_) {return false;}
  if (support.size() > 1 || obstacles.size() > 1 || identity.size() > 1) {
    faultUnlocked("multiple_fixed_map_publishers");
    return false;
  }
  std::set<std::string> names;
  for (const auto * sources : {&support, &obstacles, &identity}) {
    for (const auto & source : *sources) {
      if (!source.empty()) {names.insert(source);}
    }
  }
  if (names.size() > 1) {
    faultUnlocked("fixed_map_publisher_source_conflict");
    return false;
  }
  if (support.empty() || obstacles.empty() || identity.empty()) {
    graph_reason_ = "waiting_for_publishers";
    return false;
  }
  if (support.front().empty() || obstacles.front().empty() || identity.front().empty()) {
    graph_reason_ = "waiting_for_publisher_identity";
    return false;
  }
  graph_healthy_ = true;
  graph_reason_.clear();
  return true;
}

void FixedMapGuard::faultUnlocked(const std::string & reason)
{
  if (!faulted_) {
    faulted_ = true;
    fault_reason_ = reason;
  }
}

bool FixedMapGuard::readyUnlocked() const
{
  return !faulted_ && graph_healthy_ && identity_verified_ &&
         support_.verified && obstacles_.verified;
}

std::string FixedMapGuard::reasonUnlocked() const
{
  if (faulted_) {return fault_reason_;}
  if (!graph_healthy_) {return graph_reason_;}
  if (!identity_verified_) {return "waiting_for_map_id";}
  if (!support_.verified) {return "waiting_for_support";}
  if (!obstacles_.verified) {return "waiting_for_obstacles";}
  return "ready";
}

bool FixedMapGuard::ready() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return readyUnlocked();
}

bool FixedMapGuard::faulted() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return faulted_;
}

std::string FixedMapGuard::reason() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return reasonUnlocked();
}

std::string FixedMapGuard::report() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream stream;
  const auto cloud_json = [&stream](const CloudState & state) {
      stream << "{\"expected_sha256\":" << quoted(state.expected_sha256) <<
        ",\"received_sha256\":" << quoted(state.received_sha256) <<
        ",\"verified\":" << (state.verified ? "true" : "false") <<
        ",\"points\":" << state.points << ",\"received_points\":" << state.received_points <<
        ",\"accepted\":" << state.accepted << ",\"duplicates\":" << state.duplicates <<
        ",\"rejected\":" << state.rejected << '}';
    };
  stream << "{\"ready\":" << (readyUnlocked() ? "true" : "false") <<
    ",\"faulted\":" << (faulted_ ? "true" : "false") <<
    ",\"reason\":" << quoted(reasonUnlocked()) <<
    ",\"frame_id\":" << quoted(frame_) <<
    ",\"expected_map_id\":" << quoted(expected_map_id_) <<
    ",\"received_map_id\":" << quoted(received_map_id_) <<
    ",\"identity_verified\":" << (identity_verified_ ? "true" : "false") <<
    ",\"graph_healthy\":" << (graph_healthy_ ? "true" : "false") << ",\"support\":";
  cloud_json(support_);
  stream << ",\"obstacles\":";
  cloud_json(obstacles_);
  stream << ",\"publisher_counts\":{\"support\":" << support_sources_.size() <<
    ",\"obstacles\":" << obstacle_sources_.size() <<
    ",\"identity\":" << identity_sources_.size() << "},\"publisher_sources\":{\"support\":" <<
    sourcesJson(support_sources_) << ",\"obstacles\":" << sourcesJson(obstacle_sources_) <<
    ",\"identity\":" << sourcesJson(identity_sources_) << "}}";
  return stream.str();
}

}  // namespace astar_3d_planner
