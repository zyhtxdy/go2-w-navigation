#ifndef TERRAIN_COSTMAP__EVIDENCE_CLEARING_HPP_
#define TERRAIN_COSTMAP__EVIDENCE_CLEARING_HPP_

#include "terrain_costmap/terrain_analyzer.hpp"

#include <cstddef>
#include <vector>

namespace terrain_costmap
{

struct EvidenceClearingParameters
{
  double range_min{0.40};
  double range_max{8.0};
  double angle_min{-3.14159265358979323846};
  double angle_increment{0.00872664625997164788};
  std::size_t bin_count{720};
  double endpoint_margin{0.10};
};

// Points are in the scan frame. Only XY is used. Each bin starts at
// angle_min + index * angle_increment; equivalent angles wrap at one turn.
// A finite ray requires a current return and stops before the nearest current
// return, classified obstacle (including history), or range_max. Missing or
// invalid evidence yields NaN, never infinity. Invalid geometry yields all NaN.
std::vector<float> makeEvidenceClearingRanges(
  const std::vector<Point> & current_returns,
  const std::vector<Point> & classified_obstacles,
  const EvidenceClearingParameters & parameters);

}  // namespace terrain_costmap

#endif  // TERRAIN_COSTMAP__EVIDENCE_CLEARING_HPP_
