#ifndef ASTAR_3D_PLANNER__ASTAR_3D_HPP_
#define ASTAR_3D_PLANNER__ASTAR_3D_HPP_

#include <functional>

#include <cstddef>
#include <vector>

#include "astar_3d_planner/occupy_map.hpp"

namespace astar_3d_planner
{

class Astar3D
{
public:
  struct TerrainValidation
  {
    const char * reason{"valid"};
    std::size_t index{0};
    OccupyMap::Point position{};
    bool has_collision{false};
    VoxelKey collision{};
    bool has_support_edge{false};
    VoxelKey previous_support{};
    VoxelKey current_support{};
    double previous_height{0.0};
    double current_height{0.0};
    double horizontal_distance{0.0};
  };
  struct ProjectionDiagnostics
  {
    const char * reason{"valid"};
    const char * stage{"start_projection"};
    OccupyMap::SurfaceDiagnostics surface;
    std::size_t bridge_rejected{0};
    std::size_t footprint_rejected{0};
    bool has_collision{false};
    VoxelKey collision{};
    OccupyMap::Point sample{};
  };
  struct SearchDiagnostics
  {
    const char * reason{"ground_disconnected"};
    std::size_t expanded{0};
    std::size_t outside_connection_radius{0};
    std::size_t height_rejected{0};
    std::size_t segment_rejected{0};
    std::size_t motion_rejected{0};
    std::size_t terminal_rejected{0};
    std::size_t heuristic_expanded{0};
    // Populated only when an ordinary (no heading) search exhausts OpenList.
    // Same-snapshot, same-goal retries can skip these already proved dead ends.
    std::vector<VoxelKey> exhausted_component;
  };
  // Heading belongs to the search state whenever a rectangular maneuver is
  // constrained. Two arrivals at the same support with different headings
  // must not discard one another before the next turn is checked.
  struct MotionConstraints
  {
    double start_yaw{0.0};
    double turn_cost_weight{0.15};  // Equivalent route metres per radian.
    double maximum_seconds{0.35};
    bool use_exact_start{false};
    // False is valid only for heading-independent corridor/terminal checks
    // with zero turn cost. Ground layers remain distinct in either mode.
    bool include_heading_in_state{true};
    OccupyMap::Point exact_start{};
    std::function<bool(const OccupyMap::Point &, const OccupyMap::Point &,
      double, double)> transition;
    std::function<bool(const VoxelKey &, double)> terminal;
  };
  struct Config
  {
    double heuristic_weight{1.0};
    double obstacle_cost_weight{2.0};
    double slope_cost_weight{0.2};
    double ground_gap_cost_weight{0.1};
    double max_slope_deg{50.0};
    double max_step_height{0.18};
    double surface_connection_radius{0.25};
    double ground_check_depth{0.6};
    double robot_height{0.45};
    double planning_height_offset{0.2};
    double ground_gap_tolerance{0.12};
    double start_projection_xy_radius{1.5};
    double start_projection_height_tolerance{0.2};
    double start_projection_direction_weight{2.0};
    // Preserve only the already selected, bounded robot-to-support connector.
    // Offline sparse maps may contain incidental neighbouring support before
    // that connector reaches its actual observed endpoint. Disabled online.
    bool retain_start_projection_bridge{false};
    double goal_projection_xy_radius{0.5};
    double projection_z_radius{1.0};
    double unspecified_goal_projection_z_radius{10.0};
    std::size_t max_search_nodes{500000};
    // Internal fallback budget; zero preserves the ordinary search behaviour.
    double maximum_search_seconds{0.0};
  };

  Astar3D(const OccupyMap & map, const Config & config);

  bool projectToSurface(
    double x, double y, double desired_ground_z, double horizontal_radius,
    double vertical_radius, VoxelKey & candidate) const;
  bool projectStartToSurface(
    double x, double y, double desired_ground_z, double horizontal_radius,
    double vertical_radius, double target_x, double target_y,
    VoxelKey & candidate, ProjectionDiagnostics * diagnostics = nullptr,
    const std::function<bool(const VoxelKey &)> & validate_connector = {}) const;
  std::vector<VoxelKey> surfaceCandidates(
    double x, double y, double desired_ground_z, double horizontal_radius,
    double vertical_radius, OccupyMap::SurfaceDiagnostics * diagnostics = nullptr) const;
  // Collision-only check of a previously selected connector. Its complete
  // ground/slope/step validation remains mandatory before using a path.
  bool startBridgeCollisionFree(
    double x, double y, double desired_ground_z,
    const VoxelKey & candidate, VoxelKey * collision = nullptr,
    OccupyMap::Point * sample = nullptr) const;

  bool search(
    const VoxelKey & start, const VoxelKey & goal,
    std::vector<VoxelKey> & path, double * path_cost = nullptr) const;
  bool searchToAny(
    const VoxelKey & start, const std::vector<VoxelKey> & goals,
    std::vector<VoxelKey> & path, VoxelKey & reached_goal,
    double * path_cost = nullptr, SearchDiagnostics * diagnostics = nullptr,
    const MotionConstraints * motion = nullptr) const;
  bool segmentTraversable(
    const VoxelKey & from, const VoxelKey & to) const;
  // Terrain-only validation for cached routes; dynamic-obstacle grace is
  // handled separately and must not bypass support, slope, or step checks.
  bool segmentTerrainTraversable(
    const VoxelKey & from, const VoxelKey & to) const;
  bool refreshAndValidateTerrainPath(
    const VoxelKey & projected_start, std::vector<OccupyMap::Point> & path,
    TerrainValidation * failure = nullptr, bool check_obstacles = false) const;
  // Check the exact start-to-goal line independently of a side-selected start
  // projection. Only its initial configured blind prefix may lack support;
  // the goal, later terrain, obstacles, slope and steps must remain valid.
  bool tryDirectGroundPath(
    const OccupyMap::Point & from, const OccupyMap::Point & to,
    double maximum_initial_blind_length,
    std::vector<OccupyMap::Point> & output,
    TerrainValidation * failure = nullptr) const;
  // Refreshed route heights may lie in a different Z voxel from their retained
  // support-layer key. Resolve that key before checking collision clearance.
  OccupyMap::Traversability pathPointTraversability(
    const OccupyMap::Point & position) const;
  double groundObstacleCost(const OccupyMap::Point & position) const;
  std::vector<VoxelKey> simplifyPath(
    const std::vector<VoxelKey> & path) const;
  // Include exact endpoints in line-of-sight simplification. Only the bounded
  // initial blind bridge may lack support; later gaps cannot be shortcut.
  std::vector<OccupyMap::Point> simplifyGroundPath(
    const std::vector<OccupyMap::Point> & path,
    double maximum_initial_blind_length) const;
  std::vector<OccupyMap::Point> densifyPath(
    const std::vector<OccupyMap::Point> & path, double maximum_xy_spacing) const;

  int supportDepthCells() const {return support_depth_cells_;}
  int robotHeightCells() const {return robot_height_cells_;}
  int preferredGroundGapCells() const {return preferred_ground_gap_cells_;}
  int groundGapToleranceCells() const {return ground_gap_tolerance_cells_;}

private:
  struct Node
  {
    VoxelKey key;
    double g{0.0};
    double f{0.0};
    int parent{-1};
    bool closed{false};
    double yaw{0.0};
  };

  double heuristic(const VoxelKey & from, const VoxelKey & to) const;
  bool groundConnectionAllowed(
    double from_height, double to_height, double horizontal_distance) const;
  bool segmentTraversableImpl(
    const VoxelKey & from, const VoxelKey & to, bool check_obstacles) const;
  bool resolvePathGround(const OccupyMap::Point & position, VoxelKey & ground) const;
  bool validateGroundPath(
    const std::vector<OccupyMap::Point> & path,
    double maximum_initial_blind_length, bool check_obstacles,
    std::vector<OccupyMap::Point> * refreshed,
    TerrainValidation * failure,
    bool constrain_blind_ground_layer = false,
    double maximum_initial_bridge_length = 0.0) const;

  const OccupyMap & map_;
  Config config_;
  double resolution_{0.1};
  double max_slope_tangent_{1.1917536};
  int support_depth_cells_{6};
  int robot_height_cells_{5};
  int preferred_ground_gap_cells_{2};
  int ground_gap_tolerance_cells_{2};
};

}  // namespace astar_3d_planner

#endif  // ASTAR_3D_PLANNER__ASTAR_3D_HPP_
