#ifndef TERRAIN_COSTMAP__TERRAIN_HAZARD_STABILIZER_HPP_
#define TERRAIN_COSTMAP__TERRAIN_HAZARD_STABILIZER_HPP_

#include "terrain_costmap/terrain_analyzer.hpp"

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace terrain_costmap
{

// Keep actual measured returns within an independent observation-time window.
// Ground fitting may still use a longer history. Unknown/future times are not
// silently promoted to fresh measurements.
std::vector<TerrainPoint> freshMeasuredObstacles(
  const std::vector<TerrainPoint> & measured,
  std::int64_t frame_stamp_ns, std::int64_t maximum_age_ns);

struct HazardStabilizerParameters
{
  double voxel_size{0.20};
  double update_radius{5.0};
  int measured_enter_observations{2};
  int synthetic_enter_observations{3};
  int exit_missed_observations{3};
  // Apply the analyzer's existing near-field synthetic-marker policy also
  // to retained history. Measured states are never cleared by this radius.
  double synthetic_clearance_radius{0.0};
};

// Applies temporal hysteresis only to the rolling global obstacle stream.
// The local costmap still receives immediate, unfiltered hazards.
class TerrainHazardStabilizer
{
public:
  explicit TerrainHazardStabilizer(
    const HazardStabilizerParameters & parameters);

  std::vector<TerrainPoint> update(
    const std::vector<TerrainPoint> & measured,
    const std::vector<TerrainPoint> & synthetic,
    double center_x, double center_y, std::int64_t frame_stamp_ns = 0);

  void clear();

  std::size_t trackedVoxelCount() const;

private:
  struct Key
  {
    int x;
    int y;
    int z;

    bool operator==(const Key & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct KeyHash
  {
    std::size_t operator()(const Key & key) const;
  };

  struct State
  {
    TerrainPoint point;
    int hits{0};
    int misses{0};
    bool active{false};
    std::int64_t last_observation_stamp_ns{0};
  };

  Key makeKey(const Point & point) const;

  HazardStabilizerParameters parameters_;
  std::unordered_map<Key, State, KeyHash> states_;
  std::int64_t last_frame_stamp_ns_{0};
};

}  // namespace terrain_costmap

#endif  // TERRAIN_COSTMAP__TERRAIN_HAZARD_STABILIZER_HPP_
