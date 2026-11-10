#ifndef ASTAR_3D_PLANNER__SWEPT_FOOTPRINT_HPP_
#define ASTAR_3D_PLANNER__SWEPT_FOOTPRINT_HPP_

#include <cstddef>
#include <optional>
#include <vector>
#include "astar_3d_planner/occupy_map.hpp"

namespace astar_3d_planner
{
// Planar rectangular bounding envelope of the configured, padded Nav2 footprint.
// Path Z is observed ground height, not base_link height. Caller holds map lock.
class SweptFootprint
{
public:
  struct Config
  {
    double min_x{-0.4}, max_x{0.4}, min_y{-0.25}, max_y{0.25};
    double robot_height{0.75};
    std::size_t max_samples{20000};
    double near_start_static_refinement_radius{0.0};
    double near_start_static_obstacle_margin{0.02};
    bool refine_static_obstacle_extents{false};
    double static_obstacle_extent_margin{0.02};
    double near_start_measured_refinement_radius{0.0};
    double near_start_measured_obstacle_margin{0.03};
    // Global guidance checks occupied poses and segment corridors. Requiring
    // a turn-in-place program at every polyline vertex is optional.
    bool check_in_place_turn_sweeps{true};
    // Fixed for the entire planning request, including all subpaths and turns.
    std::optional<std::array<double, 2>> near_start_center;
  };
  struct Failure
  {
    const char * reason{"valid"};
    std::size_t index{0};
    OccupyMap::Point position{};
    double yaw{0.0};
    bool has_collision{false};
    VoxelKey collision{};
    std::size_t samples{0};
  };

  SweptFootprint(const OccupyMap & map, const Config & config);
  bool poseCollisionFree(const OccupyMap::Point & ground, double yaw,
    VoxelKey * collision = nullptr, double sweep_padding = 0.0,
    double vertical_padding = 0.0) const;
  // Strict mode validates a turn/translate maneuver. Guidance mode validates
  // segment corridors plus start/goal poses, not the unplanned turning motion.
  // A motion-bound envelope covers the intervals between checked samples.
  bool validatePath(const std::vector<OccupyMap::Point> & path,
    double start_yaw, double goal_yaw, Failure * failure = nullptr) const;

private:
  const OccupyMap & map_;
  Config config_;
  double corner_radius_{0.0};
};
}  // namespace astar_3d_planner
#endif
