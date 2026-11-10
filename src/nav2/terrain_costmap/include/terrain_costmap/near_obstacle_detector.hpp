#ifndef TERRAIN_COSTMAP__NEAR_OBSTACLE_DETECTOR_HPP_
#define TERRAIN_COSTMAP__NEAR_OBSTACLE_DETECTOR_HPP_

#include "terrain_costmap/terrain_analyzer.hpp"

#include <vector>

namespace terrain_costmap
{

struct NearObstacleParameters
{
  bool enabled{true};
  double minimum_range{0.40};
  double maximum_range{1.20};
  double minimum_relative_z{-0.35};
  double maximum_relative_z{0.40};
  double self_minimum_x{-0.40};
  double self_maximum_x{0.40};
  double self_minimum_y{-0.25};
  double self_maximum_y{0.25};
  double voxel_size{0.05};
  double maximum_xy_distance{0.15};
  double minimum_obstacle_height{0.15};
  double maximum_ground_slope{0.17632698070846498};  // tan(10 degrees).
  double lower_layer_tolerance{0.05};
  int minimum_cluster_points{3};
  double cluster_radius{0.20};
};

// A local collision observation supplement for places without a ground fit.
// Input and output coordinates are in the CURRENT base frame. Callers must
// transform historical observations into that frame before calling detect().
// Prefer one original sensor frame: this helper does not check observation
// ages, and lower-layer evidence must be as fresh as the upper returns.
// This detector neither fits ground nor creates terrain/support observations.
class NearObstacleDetector
{
public:
  explicit NearObstacleDetector(const NearObstacleParameters & parameters);

  // Every output retains one actual input Point and its source timestamp.
  // relative_height is above a nearby, measured lower layer; that layer is
  // NOT confirmed ground. synthetic is always false. Duplicate 3D voxels
  // contribute once, retaining the observation with the newest source stamp.
  std::vector<TerrainPoint> detect(const std::vector<Point> & base_points) const;

private:
  NearObstacleParameters parameters_;
};

}  // namespace terrain_costmap

#endif  // TERRAIN_COSTMAP__NEAR_OBSTACLE_DETECTOR_HPP_
