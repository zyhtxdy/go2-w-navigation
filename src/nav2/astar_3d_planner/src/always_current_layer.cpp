#include "nav2_costmap_2d/layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace astar_3d_planner
{

// Nav2 Humble's PlannerServer unconditionally waits for its hosted 2D
// GlobalCostmap to become current before invoking any global planner.  The 3D
// A* plugin owns a separate point-cloud voxel map, so this layer deliberately
// contributes no 2D costs and only keeps that prerequisite current.
class AlwaysCurrentLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    enabled_ = true;
    current_ = true;
  }

  void reset() override
  {
    current_ = true;
  }

  bool isClearable() override
  {
    return false;
  }

  void updateBounds(
    double, double, double, double *, double *, double *, double *) override
  {
    current_ = true;
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D &, int, int, int, int) override
  {
    current_ = true;
  }
};

}  // namespace astar_3d_planner

PLUGINLIB_EXPORT_CLASS(
  astar_3d_planner::AlwaysCurrentLayer,
  nav2_costmap_2d::Layer)
