#include "astar_3d_planner/goal_connections.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <tuple>

namespace astar_3d_planner
{
std::vector<GoalConnection> subcellGoalConnections(
  const OccupyMap & map, const Astar3D & astar, const SweptFootprint & checker,
  double x, double y, double z, double yaw, double tolerance, double vertical_radius,
  double max_connection_length, GoalConnectionDiagnostics * diagnostic,
  std::vector<GoalPose> * terminal_poses)
{
  GoalConnectionDiagnostics stats;
  std::vector<GoalConnection> output;
  if (terminal_poses) {terminal_poses->clear();}
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(yaw) ||
    !std::isfinite(tolerance) || tolerance <= 0 || !std::isfinite(max_connection_length) ||
    max_connection_length <= 0 || !std::isfinite(vertical_radius) || vertical_radius < 0)
  {if (diagnostic) {*diagnostic = stats;} return output;}
  struct Pose {VoxelKey key; OccupyMap::Point p; double offset, dz; int clearance_rank;};
  std::vector<Pose> endpoints;
  const double resolution = map.resolution();
  constexpr int subdivisions = 8;
  const auto columns = astar.surfaceCandidates(x, y, z,
    tolerance + resolution * std::sqrt(.5), vertical_radius);
  for (const auto & key : columns) {
    VoxelKey ground; double height;
    if (!map.findSurfaceGroundUnlocked(key, astar.preferredGroundGapCells(),
        astar.groundGapToleranceCells(), ground) || !map.supportHeightUnlocked(ground, height)) {continue;}
    const auto centre = map.keyToPos(key);
    std::vector<Pose> local;
    for (int ix = 0; ix < subdivisions; ++ix) {
      for (int iy = 0; iy < subdivisions; ++iy) {
        const double px = centre[0] + resolution * ((ix + .5) / subdivisions - .5);
        const double py = centre[1] + resolution * ((iy + .5) / subdivisions - .5);
        const double offset = std::hypot(px - x, py - y);
        if (offset > tolerance + 1e-9) {continue;}
        ++stats.poses_checked;
        const OccupyMap::Point p{px, py, height};
        if (!checker.poseCollisionFree(p, yaw)) {continue;}
        ++stats.poses_clear;
        // Prefer room for the existing continuous-sweep envelope. This is a
        // ranking preference, not an extra hard clearance threshold.
        const int rank = checker.poseCollisionFree(p, yaw, nullptr, resolution * .25) ? 2 :
          checker.poseCollisionFree(p, yaw, nullptr, resolution * .125) ? 1 : 0;
        local.push_back({key, p, offset, std::abs(height-z), rank});
      }
    }
    std::stable_sort(local.begin(), local.end(), [](const Pose & a, const Pose & b) {
        return std::make_tuple(-a.clearance_rank, a.offset) <
               std::make_tuple(-b.clearance_rank, b.offset);
      });
    if (local.size() > 4) {local.resize(4);}
    endpoints.insert(endpoints.end(), local.begin(), local.end());
    if (stats.poses_checked >= 2048) {stats.limit_reached = true; break;}
  }
  std::stable_sort(endpoints.begin(), endpoints.end(), [](const Pose & a, const Pose & b) {
      return std::make_tuple(a.dz, -a.clearance_rank, a.offset) <
             std::make_tuple(b.dz, -b.clearance_rank, b.offset);
    });
  if (endpoints.size() > 32) {endpoints.resize(32); stats.limit_reached = true;}
  if (terminal_poses) {
    for (const auto & pose : endpoints) {terminal_poses->push_back({pose.key, pose.p});}
  }
  std::set<std::tuple<int, int, int, double, double, double>> seen;
  for (const auto & endpoint : endpoints) {
    // Connect from several approach directions, preferring an approach already
    // aligned with the final orientation. A longer checked last edge avoids
    // forcing the body through an unsafe coarse terminal-cell centre.
    for (double angle : {yaw, yaw + std::acos(-1.0), yaw + std::acos(-1.0)/2,
        yaw - std::acos(-1.0)/2})
    {
      for (int step = 1; step <= 5; ++step) {
        const double length = max_connection_length * step / 5;
        const auto anchors = astar.surfaceCandidates(
          endpoint.p[0] - length * std::cos(angle), endpoint.p[1] - length * std::sin(angle),
          endpoint.p[2], resolution * .8, resolution);
        for (const auto & anchor : anchors) {
          const auto signature = std::make_tuple(anchor.x, anchor.y, anchor.z,
            endpoint.p[0], endpoint.p[1], endpoint.p[2]);
          if (!seen.insert(signature).second) {continue;}
          if (stats.connectors_checked >= 512 || output.size() >= 64) {
            stats.limit_reached = true;
            if (diagnostic) {*diagnostic = stats;} return output;
          }
          ++stats.connectors_checked;
          VoxelKey ground; double height;
          if (!map.findSurfaceGroundUnlocked(anchor, astar.preferredGroundGapCells(),
              astar.groundGapToleranceCells(), ground) || !map.supportHeightUnlocked(ground, height)) {continue;}
          auto from = map.keyToPos(anchor); from[2] = height;
          const double distance = std::hypot(from[0]-endpoint.p[0], from[1]-endpoint.p[1]);
          if (distance > max_connection_length + 1e-9) {continue;}
          std::vector<OccupyMap::Point> connector;
          if (!astar.tryDirectGroundPath(from, endpoint.p, 0.0, connector)) {continue;}
          const double approach = std::atan2(endpoint.p[1]-from[1], endpoint.p[0]-from[0]);
          if (!checker.validatePath(connector, approach, yaw)) {continue;}
          output.push_back({anchor, std::move(connector)});
        }
      }
    }
  }
  if (diagnostic) {*diagnostic = stats;}
  return output;
}
}
