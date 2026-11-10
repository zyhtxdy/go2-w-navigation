#include "terrain_costmap/terrain_hazard_stabilizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace terrain_costmap
{

std::vector<TerrainPoint> freshMeasuredObstacles(
  const std::vector<TerrainPoint> & measured,
  std::int64_t frame_stamp_ns, std::int64_t maximum_age_ns)
{
  if (maximum_age_ns < 0) {throw std::invalid_argument("negative obstacle age window");}
  std::vector<TerrainPoint> fresh;
  fresh.reserve(measured.size());
  for (const auto & point : measured) {
    const auto stamp = point.point.observation_stamp_ns;
    if (!point.synthetic && stamp > 0 && stamp <= frame_stamp_ns &&
      frame_stamp_ns - stamp <= maximum_age_ns)
    {
      fresh.push_back(point);
    }
  }
  return fresh;
}

TerrainHazardStabilizer::TerrainHazardStabilizer(
  const HazardStabilizerParameters & parameters)
: parameters_(parameters)
{
  if (parameters_.voxel_size <= 0.0 ||
    parameters_.update_radius <= parameters_.voxel_size ||
    parameters_.measured_enter_observations < 1 ||
    parameters_.synthetic_enter_observations < 1 ||
    parameters_.exit_missed_observations < 1 ||
    !std::isfinite(parameters_.synthetic_clearance_radius) ||
    parameters_.synthetic_clearance_radius < 0.0)
  {
    throw std::invalid_argument("invalid terrain hazard stabilizer parameters");
  }
}

std::size_t TerrainHazardStabilizer::KeyHash::operator()(const Key & key) const
{
  const auto mix = [](std::size_t seed, std::uint32_t value) {
      seed ^= static_cast<std::size_t>(value) + 0x9e3779b9U +
        (seed << 6U) + (seed >> 2U);
      return seed;
    };
  std::size_t seed = 0;
  seed = mix(seed, static_cast<std::uint32_t>(key.x));
  seed = mix(seed, static_cast<std::uint32_t>(key.y));
  return mix(seed, static_cast<std::uint32_t>(key.z));
}

TerrainHazardStabilizer::Key TerrainHazardStabilizer::makeKey(
  const Point & point) const
{
  return {
    static_cast<int>(std::llround(point.x / parameters_.voxel_size)),
    static_cast<int>(std::llround(point.y / parameters_.voxel_size)),
    static_cast<int>(std::llround(point.z / parameters_.voxel_size))
  };
}

std::vector<TerrainPoint> TerrainHazardStabilizer::update(
  const std::vector<TerrainPoint> & measured,
  const std::vector<TerrainPoint> & synthetic,
  double center_x, double center_y, std::int64_t frame_stamp_ns)
{
  // A replayed/duplicated publication is not another hit or miss. The node
  // explicitly clears the tracker if the source clock starts a new epoch.
  const bool stamped = frame_stamp_ns > 0;
  const bool duplicate = stamped && frame_stamp_ns <= last_frame_stamp_ns_;
  std::unordered_map<Key, TerrainPoint, KeyHash> observations;
  observations.reserve(measured.size() + synthetic.size());

  // Add synthetic markers first. A measured return in the same voxel always
  // wins because it is direct evidence and uses the shorter entry threshold.
  for (const TerrainPoint & point : synthetic) {
    if (parameters_.synthetic_clearance_radius > 0.0 &&
      std::hypot(point.point.x - center_x, point.point.y - center_y) <=
      parameters_.synthetic_clearance_radius) {continue;}
    observations[makeKey(point.point)] = point;
  }
  for (const TerrainPoint & point : measured) {
    if (stamped && (point.point.observation_stamp_ns <= 0 ||
      point.point.observation_stamp_ns > frame_stamp_ns)) {continue;}
    const auto key = makeKey(point.point);
    const auto retained = states_.find(key);
    if (stamped && retained != states_.end() &&
      point.point.observation_stamp_ns <= retained->second.last_observation_stamp_ns)
    {
      continue;
    }
    const auto previous = observations.find(key);
    if (previous == observations.end() || previous->second.synthetic ||
      point.point.observation_stamp_ns > previous->second.point.observation_stamp_ns)
    {
      observations[key] = point;
    }
  }

  if (!duplicate) {
    if (stamped) {last_frame_stamp_ns_ = frame_stamp_ns;}
    for (auto iterator = states_.begin(); iterator != states_.end();) {
      const double distance = std::hypot(
        iterator->second.point.point.x - center_x,
        iterator->second.point.point.y - center_y);
      if (distance > parameters_.update_radius ||
        (iterator->second.point.synthetic && parameters_.synthetic_clearance_radius > 0.0 &&
        distance <= parameters_.synthetic_clearance_radius))
      {
        iterator = states_.erase(iterator);
        continue;
      }

      const auto observation = observations.find(iterator->first);
      const bool fresh_observation = observation != observations.end() &&
        (!stamped ||
        (observation->second.synthetic ?
        (iterator->second.point.synthetic || (!iterator->second.active &&
        iterator->second.misses >= parameters_.exit_missed_observations)) :
        observation->second.point.observation_stamp_ns >
        iterator->second.last_observation_stamp_ns));
      if (!fresh_observation) {
        if (observation != observations.end()) {observations.erase(observation);}
        ++iterator->second.misses;
        iterator->second.hits = 0;
        if (iterator->second.misses >= parameters_.exit_missed_observations) {
          if (!stamped) {
            iterator = states_.erase(iterator);
            continue;
          }
          // Retain a bounded, inactive watermark inside the update radius.
          // Otherwise the same historical return could be born again as a
          // first observation after its active state has expired.
          iterator->second.active = false;
          iterator->second.misses = parameters_.exit_missed_observations;
        }
      } else {
        const bool was_active = iterator->second.active;
        if (stamped && iterator->second.point.synthetic != observation->second.synthetic) {
          // Synthetic fit confirmations cannot count as measured sensor frames.
          iterator->second.hits = 0;
        }
        // Active measured evidence cannot be renewed by a synthetic marker.
        // After expiry, synthetic hazards can enter using their own confirmations.
        if (stamped || iterator->second.point.synthetic || !observation->second.synthetic) {
          iterator->second.point = observation->second;
        }
        if (!observation->second.synthetic) {
          iterator->second.last_observation_stamp_ns =
            observation->second.point.observation_stamp_ns;
        }
        ++iterator->second.hits;
        iterator->second.misses = 0;
        const int entry_observations = iterator->second.point.synthetic ?
          parameters_.synthetic_enter_observations :
          parameters_.measured_enter_observations;
        iterator->second.active =
          was_active || iterator->second.hits >= entry_observations;
        observations.erase(observation);
      }
      ++iterator;
    }

    for (const auto & observation : observations) {
      State state;
      state.point = observation.second;
      state.last_observation_stamp_ns = observation.second.point.observation_stamp_ns;
      state.hits = 1;
      const int entry_observations = state.point.synthetic ?
        parameters_.synthetic_enter_observations :
        parameters_.measured_enter_observations;
      state.active = entry_observations == 1;
      states_.emplace(observation.first, state);
    }
  }

  std::vector<TerrainPoint> stable;
  stable.reserve(states_.size());
  for (const auto & entry : states_) {
    if (entry.second.active) {
      stable.push_back(entry.second.point);
    }
  }
  std::sort(
    stable.begin(), stable.end(),
    [](const TerrainPoint & left, const TerrainPoint & right) {
      if (left.point.x != right.point.x) {
        return left.point.x < right.point.x;
      }
      if (left.point.y != right.point.y) {
        return left.point.y < right.point.y;
      }
      return left.point.z < right.point.z;
    });
  return stable;
}

void TerrainHazardStabilizer::clear()
{
  states_.clear();
  last_frame_stamp_ns_ = 0;
}

std::size_t TerrainHazardStabilizer::trackedVoxelCount() const
{
  return states_.size();
}

}  // namespace terrain_costmap
