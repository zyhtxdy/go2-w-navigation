#ifndef ASTAR_3D_PLANNER__PATH_REFINER_HPP_
#define ASTAR_3D_PLANNER__PATH_REFINER_HPP_

#include <functional>
#include <vector>
#include "astar_3d_planner/occupy_map.hpp"

namespace astar_3d_planner
{
// Bounded any-angle shortcutting and corner bevels. Every accepted complete
// maneuver is rechecked by the caller against terrain and swept footprint.
class PathRefiner
{
public:
  using Points = std::vector<OccupyMap::Point>;
  using ClearanceCost = std::function<double(const OccupyMap::Point &)>;
  struct Config
  {
    std::size_t maximum_checks{64};
    double maximum_seconds{0.06};
    double corner_cut_distance{0.12};
    double turn_weight{0.15};
    double clearance_weight{2.0};
    double grid_resolution{0.1};
    std::size_t maximum_cost_samples{4096};
  };
  struct Diagnostics
  {
    std::size_t checks{0}, shortcuts{0}, bevels{0};
    bool budget_exhausted{false};
    bool clearance_unavailable{false};
    std::size_t clearance_rejected{0};
    double clearance_before{0.0}, clearance_after{0.0};
  };
  explicit PathRefiner(const Config & config);
  Points refine(const Points & input, double start_yaw, double goal_yaw,
    bool retain_first_connector, const std::function<bool(Points &)> & validate,
    Diagnostics * diagnostics = nullptr, const ClearanceCost & clearance = {}) const;
  // Exact integration of a voxel-constant soft cost along XY arc length.
  // This is a preference, not a replacement for collision/terrain checking.
  double score(const Points & points, double start_yaw, double goal_yaw,
    const ClearanceCost & clearance, double * integrated_cost = nullptr) const;
  static double objective(const Points & points, double start_yaw, double goal_yaw,
    double turn_weight);
private:
  Config config_;
};
}  // namespace astar_3d_planner
#endif
