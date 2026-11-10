#ifndef ASTAR_3D_PLANNER__GOAL_CONNECTIONS_HPP_
#define ASTAR_3D_PLANNER__GOAL_CONNECTIONS_HPP_
#include "astar_3d_planner/astar_3d.hpp"
#include "astar_3d_planner/swept_footprint.hpp"

namespace astar_3d_planner
{
struct GoalPose
{
  VoxelKey key;
  OccupyMap::Point point;
};
struct GoalConnection
{
  VoxelKey anchor;
  // Includes the searched anchor centre and the exact supported terminal pose.
  std::vector<OccupyMap::Point> points;
};
struct GoalConnectionDiagnostics
{
  std::size_t poses_checked{0}, poses_clear{0}, connectors_checked{0};
  bool limit_reached{false};
};
// Called only as an offline fallback, with one locked/immutable map view.
// All terminal points lie in the original tolerance disk. Analytic final
// connections never receive a blind prefix or bypass terrain/body/turn checks.
std::vector<GoalConnection> subcellGoalConnections(
  const OccupyMap & map, const Astar3D & astar, const SweptFootprint & checker,
  double x, double y, double z, double yaw, double tolerance, double vertical_radius,
  double max_connection_length, GoalConnectionDiagnostics * diagnostic = nullptr,
  std::vector<GoalPose> * terminal_poses = nullptr);
}
#endif
