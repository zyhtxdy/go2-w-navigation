#include "astar_3d_planner/astar_3d.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace astar_3d_planner
{

Astar3D::Astar3D(const OccupyMap & map, const Config & config)
: map_(map), config_(config)
{
  resolution_ = map_.keyToPos(VoxelKey{1, 0, 0})[0] -
    map_.keyToPos(VoxelKey{0, 0, 0})[0];
  constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
  max_slope_tangent_ = std::tan(config.max_slope_deg * degrees_to_radians);
  support_depth_cells_ = std::max(
    1, static_cast<int>(std::ceil(config.ground_check_depth / resolution_)));
  robot_height_cells_ = std::max(
    1, static_cast<int>(std::ceil(config.robot_height / resolution_)));
  preferred_ground_gap_cells_ = std::max(
    1, static_cast<int>(std::round(config.planning_height_offset / resolution_)));
  ground_gap_tolerance_cells_ = std::max(
    0, static_cast<int>(std::ceil(config.ground_gap_tolerance / resolution_)));
}

bool Astar3D::projectToSurface(
  double x, double y, double desired_ground_z, double horizontal_radius,
  double vertical_radius, VoxelKey & candidate) const
{
  const auto candidates = surfaceCandidates(
    x, y, desired_ground_z, horizontal_radius, vertical_radius);
  if (candidates.empty()) {
    return false;
  }
  candidate = candidates.front();
  return true;
}

bool Astar3D::projectStartToSurface(
  double x, double y, double desired_ground_z, double horizontal_radius,
  double vertical_radius, double target_x, double target_y,
  VoxelKey & candidate, ProjectionDiagnostics * diagnostics,
  const std::function<bool(const VoxelKey &)> & validate_connector) const
{
  if (diagnostics) {*diagnostics = ProjectionDiagnostics{};}
  const auto candidates = map_.surfaceCandidatesUnlocked(
    x, y, desired_ground_z, horizontal_radius, vertical_radius,
    support_depth_cells_, robot_height_cells_, preferred_ground_gap_cells_,
    ground_gap_tolerance_cells_, diagnostics ? &diagnostics->surface : nullptr, config_.robot_height);
  if (candidates.empty()) {
    if (diagnostics) {
      diagnostics->reason = diagnostics->surface.layers == 0 ? "no_support_xy" :
        diagnostics->surface.occupied > 0 ? "surface_clearance_blocked" :
        "support_outside_height_window";
      diagnostics->has_collision = diagnostics->surface.has_collision;
      diagnostics->collision = diagnostics->surface.collision;
      if (diagnostics->has_collision) {
        diagnostics->sample = map_.keyToPos(diagnostics->collision);
      }
    }
    return false;
  }

  struct StartOption
  {
    VoxelKey key;
    double ground_height_error;
    double horizontal_distance;
    double score;
  };
  std::vector<StartOption> options;
  options.reserve(candidates.size());
  double best_ground_height_error = std::numeric_limits<double>::infinity();

  for (const auto & key : candidates) {
    VoxelKey ground;
    if (!map_.findSurfaceGroundUnlocked(
        key, preferred_ground_gap_cells_, ground_gap_tolerance_cells_, ground))
    {
      continue;
    }
    VoxelKey collision;
    OccupyMap::Point sample;
    if (!startBridgeCollisionFree(
        x, y, desired_ground_z, key, diagnostics ? &collision : nullptr,
        diagnostics ? &sample : nullptr))
    {
      if (diagnostics) {
        ++diagnostics->bridge_rejected;
        const auto distance = [&](const OccupyMap::Point & p) {
            return std::hypot(p[0] - x, p[1] - y);
          };
        if (!diagnostics->has_collision || distance(sample) < distance(diagnostics->sample)) {
          diagnostics->has_collision = true;
          diagnostics->collision = collision; diagnostics->sample = sample;
        }
      }
      continue;
    }
    const auto position = map_.keyToPos(key);
    double ground_height;
    if (!map_.supportHeightUnlocked(ground, ground_height)) {
      continue;
    }
    const double ground_height_error =
      std::abs(ground_height - desired_ground_z);
    const double horizontal_distance =
      std::hypot(position[0] - x, position[1] - y);
    options.push_back(
      StartOption{key, ground_height_error, horizontal_distance, 0.0});
    best_ground_height_error =
      std::min(best_ground_height_error, ground_height_error);
  }
  if (options.empty()) {
    if (diagnostics) {
      diagnostics->reason = diagnostics->footprint_rejected > 0 ?
        "start_footprint_bridges_blocked" : diagnostics->bridge_rejected > 0 ?
        "start_bridge_blocked" : "support_height_unavailable";
    }
    return false;
  }

  // When the robot's own XY column already has safe observed ground, use it.
  // A goal-direction score must not move a supported start into another cell
  // simply because its centre lies slightly closer to the target direction.
  const auto exact_cell = map_.posToKey(x, y, desired_ground_z);
  const StartOption * exact_option = nullptr;
  for (const auto & option : options) {
    if (option.key.x == exact_cell.x && option.key.y == exact_cell.y &&
      option.ground_height_error <=
      best_ground_height_error + config_.start_projection_height_tolerance + 1e-9 &&
      (!exact_option || option.ground_height_error < exact_option->ground_height_error))
    {
      exact_option = &option;
    }
  }
  if (exact_option) {
    if (!validate_connector || validate_connector(exact_option->key)) {
      candidate = exact_option->key; return true;
    }
    if (diagnostics) {++diagnostics->footprint_rejected;}
  }

  const double target_dx = target_x - x;
  const double target_dy = target_y - y;
  const double target_distance = std::hypot(target_dx, target_dy);
  StartOption * best = nullptr;
  for (auto & option : options) {
    // Keep the start on the expected terrain layer. Small height differences
    // inside one noisy floor layer must not make the blind-zone projection
    // jump from one side of the vehicle to the other.
    if (option.ground_height_error >
      best_ground_height_error + config_.start_projection_height_tolerance + 1e-9)
    {
      continue;
    }
    const auto position = map_.keyToPos(option.key);
    double alignment = 1.0;
    if (target_distance > 1e-9 && option.horizontal_distance > 1e-9) {
      alignment = std::clamp(
        ((position[0] - x) * target_dx + (position[1] - y) * target_dy) /
        (option.horizontal_distance * target_distance), -1.0, 1.0);
    }
    option.score =
      option.horizontal_distance +
      config_.start_projection_direction_weight *
      option.horizontal_distance * (1.0 - alignment) +
      option.ground_height_error;

    constexpr double epsilon = 1e-9;
    if (best == nullptr || option.score < best->score - epsilon ||
      (std::abs(option.score - best->score) <= epsilon &&
      (option.horizontal_distance < best->horizontal_distance - epsilon ||
      (std::abs(option.horizontal_distance - best->horizontal_distance) <= epsilon &&
      (option.key.x < best->key.x ||
      (option.key.x == best->key.x &&
      (option.key.y < best->key.y ||
      (option.key.y == best->key.y && option.key.z < best->key.z))))))))
    {
      best = &option;
    }
  }
  if (best == nullptr) {
    if (diagnostics) {diagnostics->reason = "height_selection_failed";}
    return false;
  }
  if (validate_connector) {
    // Check connectors in preference order, stopping at the first safe one.
    // Do not sweep every nearby floor cell for an already supported start.
    std::vector<const StartOption *> ranked;
    for (const auto & option : options) {
      if (&option != exact_option && option.ground_height_error <=
        best_ground_height_error + config_.start_projection_height_tolerance + 1e-9)
      {ranked.push_back(&option);}
    }
    std::sort(ranked.begin(), ranked.end(), [](const StartOption * a, const StartOption * b) {
        if (a->score != b->score) {return a->score < b->score;}
        if (a->horizontal_distance != b->horizontal_distance) {
          return a->horizontal_distance < b->horizontal_distance;
        }
        if (a->key.x != b->key.x) {return a->key.x < b->key.x;}
        if (a->key.y != b->key.y) {return a->key.y < b->key.y;}
        return a->key.z < b->key.z;
      });
    for (const auto * option : ranked) {
      if (validate_connector(option->key)) {candidate = option->key; return true;}
      if (diagnostics) {++diagnostics->footprint_rejected;}
    }
    if (diagnostics) {diagnostics->reason = "start_footprint_bridges_blocked";}
    return false;
  }
  candidate = best->key;
  return true;
}

std::vector<VoxelKey> Astar3D::surfaceCandidates(
  double x, double y, double desired_ground_z, double horizontal_radius,
  double vertical_radius, OccupyMap::SurfaceDiagnostics * diagnostics) const
{
  return map_.surfaceCandidatesUnlocked(
    x, y, desired_ground_z, horizontal_radius, vertical_radius,
    support_depth_cells_, robot_height_cells_, preferred_ground_gap_cells_,
    ground_gap_tolerance_cells_, diagnostics, config_.robot_height);
}

bool Astar3D::startBridgeCollisionFree(
  double x, double y, double desired_ground_z,
  const VoxelKey & candidate, VoxelKey * collision,
  OccupyMap::Point * blocked_sample) const
{
  VoxelKey candidate_ground;
  if (!map_.findSurfaceGroundUnlocked(
      candidate, preferred_ground_gap_cells_,
      ground_gap_tolerance_cells_, candidate_ground))
  {
    return false;
  }
  const auto candidate_position = map_.keyToPos(candidate);
  double ground_height;
  if (!map_.supportHeightUnlocked(candidate_ground, ground_height)) {
    return false;
  }
  const double distance =
    std::hypot(candidate_position[0] - x, candidate_position[1] - y);
  const int samples = std::max(
    1, static_cast<int>(std::ceil(distance / resolution_)));
  for (int sample = 0; sample <= samples; ++sample) {
    const double ratio =
      static_cast<double>(sample) / static_cast<double>(samples);
    const double sample_x = x + ratio * (candidate_position[0] - x);
    const double sample_y = y + ratio * (candidate_position[1] - y);
    const double sample_ground_z =
      desired_ground_z + ratio * (ground_height - desired_ground_z);
    const VoxelKey ground = map_.posToKey(sample_x, sample_y, sample_ground_z);
    const int top = map_.posToKey(0, 0, sample_ground_z + config_.robot_height).z;
    for (int dz = 1; ground.z + dz <= top; ++dz) {
      if (map_.occupiedUnlocked(VoxelKey{ground.x, ground.y, ground.z + dz})) {
        if (collision) {*collision = VoxelKey{ground.x, ground.y, ground.z + dz};}
        if (blocked_sample) {*blocked_sample = {sample_x, sample_y, sample_ground_z};}
        return false;
      }
    }
  }
  return true;
}

bool Astar3D::search(
  const VoxelKey & start, const VoxelKey & goal,
  std::vector<VoxelKey> & path, double * path_cost) const
{
  VoxelKey reached_goal;
  return searchToAny(start, std::vector<VoxelKey>{goal}, path, reached_goal, path_cost);
}

bool Astar3D::searchToAny(
  const VoxelKey & start, const std::vector<VoxelKey> & goals,
  std::vector<VoxelKey> & path, VoxelKey & reached_goal,
  double * path_cost, SearchDiagnostics * diagnostics,
  const MotionConstraints * motion) const
{
  if (diagnostics) {*diagnostics = SearchDiagnostics{};}
  path.clear();
  if (!std::isfinite(config_.maximum_search_seconds) || config_.maximum_search_seconds < 0.0) {
    if (diagnostics) {diagnostics->reason = "invalid_search_budget";}
    return false;
  }
  if (motion && (!motion->transition || !std::isfinite(motion->start_yaw) ||
    !std::isfinite(motion->turn_cost_weight) || motion->turn_cost_weight < 0.0 ||
    !std::isfinite(motion->maximum_seconds) || motion->maximum_seconds <= 0.0 ||
    (!motion->include_heading_in_state && motion->turn_cost_weight != 0.0)))
  {
    if (diagnostics) {diagnostics->reason = "invalid_motion_constraints";}
    return false;
  }
  if (motion && motion->use_exact_start &&
    !std::all_of(motion->exact_start.begin(), motion->exact_start.end(),
      [](double value) {return std::isfinite(value);}))
  {
    if (diagnostics) {diagnostics->reason = "invalid_motion_constraints";}
    return false;
  }
  if (goals.empty()) {
    if (diagnostics) {diagnostics->reason = "no_goal_candidates";}
    return false;
  }
  VoxelKey start_ground;
  if (!map_.findSurfaceGroundUnlocked(
      start, preferred_ground_gap_cells_, ground_gap_tolerance_cells_, start_ground) ||
    !map_.traversableOnSurfaceUnlocked(
      start, start_ground, robot_height_cells_, preferred_ground_gap_cells_,
      ground_gap_tolerance_cells_, nullptr, config_.robot_height))
  {
    if (diagnostics) {diagnostics->reason = "projected_start_not_traversable";}
    return false;
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> goal_set;
  goal_set.reserve(goals.size());
  for (const auto & goal : goals) {
    goal_set.insert(goal);
  }

  const auto search_begin = std::chrono::steady_clock::now();
  const double maximum_seconds = motion ?
    (config_.maximum_search_seconds > 0.0 ?
    std::min(motion->maximum_seconds, config_.maximum_search_seconds) : motion->maximum_seconds) :
    config_.maximum_search_seconds;
  std::unordered_map<VoxelKey, double, VoxelKeyHash> terrain_lower_bounds;
  double unsettled_lower_bound = 0.0;
  if (motion) {
    // Like an obstacle-aware Hybrid-A* heuristic, solve the cheaper graph
    // without headings first. Keep all real support layers and directed edge
    // costs: a flat 2D projection could mix floors or block a valid underpass.
    struct ReverseEntry {VoxelKey key; double cost;};
    const auto greater = [](const ReverseEntry & a,const ReverseEntry & b) {
        return a.cost > b.cost;
      };
    std::priority_queue<ReverseEntry,std::vector<ReverseEntry>,decltype(greater)> reverse(greater);
    std::unordered_map<VoxelKey,double,VoxelKeyHash> tentative;
    for (const auto & key : goals) {tentative[key]=0.0;reverse.push({key,0.0});}
    const double budget = std::min(.07,maximum_seconds*.2);
    while (!reverse.empty() && std::chrono::duration<double>(
        std::chrono::steady_clock::now()-search_begin).count()<budget)
    {
      const auto current = reverse.top();reverse.pop();
      if (terrain_lower_bounds.count(current.key) || current.cost > tentative[current.key]+1e-9) {
        continue;
      }
      terrain_lower_bounds.emplace(current.key,current.cost);
      if (diagnostics) {++diagnostics->heuristic_expanded;}
      VoxelKey ground;
      double height;
      if (!map_.findSurfaceGroundUnlocked(current.key,preferred_ground_gap_cells_,
          ground_gap_tolerance_cells_,ground) || !map_.supportHeightUnlocked(ground,height))
      {continue;}
      const auto p=map_.keyToPos(current.key);
      const auto neighbors=map_.surfaceCandidatesUnlocked(p[0],p[1],height,
        config_.surface_connection_radius,config_.max_step_height+resolution_,
        support_depth_cells_,robot_height_cells_,preferred_ground_gap_cells_,ground_gap_tolerance_cells_,nullptr,config_.robot_height);
      const double destination_penalty=config_.obstacle_cost_weight*
        groundObstacleCost({p[0], p[1], height}) +
        config_.ground_gap_cost_weight*std::abs(static_cast<double>(
          current.key.z-ground.z-preferred_ground_gap_cells_));
      for (const auto & predecessor : neighbors) {
        if (predecessor==current.key || terrain_lower_bounds.count(predecessor)) {continue;}
        const auto q=map_.keyToPos(predecessor);
        const double distance=std::hypot(q[0]-p[0],q[1]-p[1]);
        if (distance<1e-9 || distance>config_.surface_connection_radius+1e-9) {continue;}
        VoxelKey previous_ground;
        double previous_height;
        if (!map_.findSurfaceGroundUnlocked(predecessor,preferred_ground_gap_cells_,
            ground_gap_tolerance_cells_,previous_ground) ||
          !map_.supportHeightUnlocked(previous_ground,previous_height) ||
          !groundConnectionAllowed(previous_height,height,distance) ||
          !segmentTraversable(predecessor,current.key)) {continue;}
        const double difference=std::abs(height-previous_height);
        const double value=current.cost+std::hypot(distance,difference)+
          destination_penalty+config_.slope_cost_weight*difference/distance;
        const auto known=tentative.find(predecessor);
        if (known==tentative.end() || value<known->second) {
          tentative[predecessor]=value;reverse.push({predecessor,value});
        }
      }
    }
    // A partial Dijkstra run must not use unfinalized (upper-bound) distances
    // as heuristics. Every unsettled node is at least the frontier minimum.
    if (!reverse.empty()) {unsettled_lower_bound=reverse.top().cost;}
  }
  const auto goal_heuristic = [&](const VoxelKey & key) {
      double best = std::numeric_limits<double>::infinity();
      for (const auto & goal : goals) {
        best = std::min(best, heuristic(key, goal));
      }
      const auto exact=terrain_lower_bounds.find(key);
      return std::max(best,exact==terrain_lower_bounds.end() ?
        unsettled_lower_bound : exact->second);
    };

  struct QueueEntry
  {
    double f;
    double g;
    int node_index;
    std::size_t insertion_order;
  };
  struct Greater
  {
    bool operator()(const QueueEntry & left, const QueueEntry & right) const
    {
      constexpr double epsilon = 1e-12;
      if (std::abs(left.f - right.f) > epsilon) {
        return left.f > right.f;
      }
      if (std::abs(left.g - right.g) > epsilon) {
        return left.g < right.g;
      }
      return left.insertion_order > right.insertion_order;
    }
  };

  std::vector<Node> nodes;
  nodes.reserve(std::min<std::size_t>(config_.max_search_nodes, 100000));
  struct State
  {
    VoxelKey key;
    double yaw;
    bool operator==(const State & other) const
    {return key == other.key && yaw == other.yaw;}
  };
  struct StateHash
  {
    std::size_t operator()(const State & state) const
    {
      std::size_t h = VoxelKeyHash{}(state.key);
      h ^= std::hash<double>{}(state.yaw) + 0x9e3779b9 + (h << 6) + (h >> 2);
      return h;
    }
  };
  std::unordered_map<State, int, StateHash> node_indices;
  node_indices.reserve(std::min<std::size_t>(config_.max_search_nodes, 100000));
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, Greater> open;
  std::size_t insertion_order = 0;

  nodes.push_back(Node{
      start, 0.0, config_.heuristic_weight * goal_heuristic(start), -1, false,
      motion ? motion->start_yaw : 0.0});
  node_indices.emplace(State{start,
    motion && motion->include_heading_in_state ? motion->start_yaw : 0.0}, 0);
  open.push(QueueEntry{nodes.front().f, 0.0, 0, insertion_order++});

  int reached_index = -1;
  while (!open.empty()) {
    if (maximum_seconds > 0.0 && std::chrono::duration<double>(
        std::chrono::steady_clock::now() - search_begin).count() > maximum_seconds)
    {
      if (diagnostics) {
        diagnostics->reason = motion ? "heading_search_time_limit" : "search_time_limit";
      }
      return false;
    }
    const QueueEntry entry = open.top();
    open.pop();
    Node & current = nodes[entry.node_index];
    if (current.closed || entry.g > current.g + 1e-9) {
      continue;
    }
    current.closed = true;
    if (diagnostics) {++diagnostics->expanded;}
    const VoxelKey current_key = current.key;
    const double current_g = current.g;
    const double current_yaw = current.yaw;

    if (goal_set.find(current_key) != goal_set.end()) {
      if (!motion || !motion->terminal || motion->terminal(current_key, current_yaw)) {
        reached_index = entry.node_index;
        reached_goal = current_key;
        break;
      }
      if (diagnostics) {++diagnostics->terminal_rejected;}
    }

    VoxelKey current_ground;
    if (!map_.findSurfaceGroundUnlocked(
        current_key, preferred_ground_gap_cells_,
        ground_gap_tolerance_cells_, current_ground))
    {
      continue;
    }
    const auto current_position = map_.keyToPos(current_key);
    double current_ground_height;
    if (!map_.supportHeightUnlocked(current_ground, current_ground_height)) {
      continue;
    }

    // Expand only to observed support patches. Z is selected from a real
    // surface layer; it is never an independently movable free-space axis.
    const auto surface_successors = map_.surfaceCandidatesUnlocked(
      current_position[0], current_position[1], current_ground_height,
      config_.surface_connection_radius, config_.max_step_height + resolution_,
      support_depth_cells_, robot_height_cells_, preferred_ground_gap_cells_,
      ground_gap_tolerance_cells_, nullptr, config_.robot_height);

    for (const VoxelKey & next : surface_successors) {
      if (next == current_key) {
        continue;
      }
      const int dx = next.x - current_key.x;
      const int dy = next.y - current_key.y;
      const double horizontal_distance =
        resolution_ * std::hypot(static_cast<double>(dx), static_cast<double>(dy));
      if (horizontal_distance < 1e-9 ||
        horizontal_distance > config_.surface_connection_radius + 1e-9)
      {
        if (diagnostics) {++diagnostics->outside_connection_radius;}
        continue;
      }

      VoxelKey ground;
      if (!map_.findSurfaceGroundUnlocked(
          next, preferred_ground_gap_cells_,
          ground_gap_tolerance_cells_, ground))
      {
        if (diagnostics) {++diagnostics->height_rejected;}
        continue;
      }
      double ground_height;
      if (!map_.supportHeightUnlocked(ground, ground_height) ||
        !groundConnectionAllowed(current_ground_height, ground_height, horizontal_distance))
      {
        if (diagnostics) {++diagnostics->height_rejected;}
        continue;
      }
      const double ground_height_change = ground_height - current_ground_height;
      const auto next_position = map_.keyToPos(next);
      const bool exact_start_edge = motion && motion->use_exact_start && entry.node_index == 0;
      const double next_yaw = exact_start_edge ?
        std::atan2(next_position[1] - motion->exact_start[1],
        next_position[0] - motion->exact_start[0]) :
        std::atan2(static_cast<double>(dy), static_cast<double>(dx));
      const double slope =
        std::abs(ground_height_change) / horizontal_distance;
      if (!segmentTraversable(current_key, next))
      {
        if (diagnostics) {++diagnostics->segment_rejected;}
        continue;
      }

      if (motion) {
        auto from = current_position;
        auto to = next_position;
        from[2] = current_ground_height;
        if (exact_start_edge) {from = motion->exact_start;}
        to[2] = ground_height;
        if (!motion->transition(from, to, current_yaw, next_yaw)) {
          if (diagnostics) {++diagnostics->motion_rejected;}
          continue;
        }
      }

      const double step_distance =
        std::hypot(horizontal_distance, ground_height_change);
      // Start at the observed ground's first body cell, matching rectangular
      // checks and path refinement. Low returns must not lose clearance cost
      // merely because the search node is at planning_height_offset.
      const double obstacle_cost =
        groundObstacleCost({next_position[0], next_position[1], ground_height});
      const double ground_gap_error = std::abs(
        static_cast<double>(next.z - ground.z - preferred_ground_gap_cells_));
      const double tentative_g =
        current_g + step_distance +
        config_.obstacle_cost_weight * obstacle_cost +
        config_.slope_cost_weight * slope +
        config_.ground_gap_cost_weight * ground_gap_error +
        (motion ? motion->turn_cost_weight *
        std::abs(std::remainder(next_yaw - current_yaw, 2.0 * std::acos(-1.0))) : 0.0);

      const State state{next, motion && motion->include_heading_in_state ? next_yaw : 0.0};
      const auto known = node_indices.find(state);
      int next_index = -1;
      if (known == node_indices.end()) {
        if (nodes.size() >= config_.max_search_nodes) {
          if (diagnostics) {diagnostics->reason = "search_node_limit";}
          return false;
        }
        next_index = static_cast<int>(nodes.size());
        const double f =
          tentative_g + config_.heuristic_weight * goal_heuristic(next);
        nodes.push_back(Node{next, tentative_g, f, entry.node_index, false, next_yaw});
        node_indices.emplace(state, next_index);
      } else {
        next_index = known->second;
        Node & known_node = nodes[next_index];
        if (known_node.closed || tentative_g >= known_node.g) {
          continue;
        }
        known_node.g = tentative_g;
        known_node.f =
          tentative_g + config_.heuristic_weight * goal_heuristic(next);
        known_node.parent = entry.node_index;
        known_node.yaw = next_yaw;
      }
      open.push(
        QueueEntry{
          nodes[next_index].f, nodes[next_index].g, next_index,
          insertion_order++});
    }
  }

  if (reached_index < 0) {
    if (diagnostics && !motion) {
      diagnostics->exhausted_component.reserve(nodes.size());
      for (const auto & node : nodes) {diagnostics->exhausted_component.push_back(node.key);}
    }
    return false;
  }

  if (path_cost != nullptr) {
    *path_cost = nodes[reached_index].g;
  }
  for (int index = reached_index; index >= 0; index = nodes[index].parent) {
    path.push_back(nodes[index].key);
  }
  std::reverse(path.begin(), path.end());
  if (diagnostics) {diagnostics->reason = "valid";}
  return true;
}

bool Astar3D::segmentTraversable(
  const VoxelKey & from, const VoxelKey & to) const
{
  return segmentTraversableImpl(from, to, true);
}

bool Astar3D::segmentTerrainTraversable(
  const VoxelKey & from, const VoxelKey & to) const
{
  return segmentTraversableImpl(from, to, false);
}

bool Astar3D::segmentTraversableImpl(
  const VoxelKey & from, const VoxelKey & to, bool check_obstacles) const
{
  std::vector<OccupyMap::Point> endpoints;
  for (const auto & key : {from, to}) {
    VoxelKey ground;
    if (!map_.findSurfaceGroundUnlocked(
        key, preferred_ground_gap_cells_, ground_gap_tolerance_cells_, ground))
    {
      if (!map_.findGroundUnlocked(key, support_depth_cells_, ground) ||
        std::abs(key.z - ground.z - preferred_ground_gap_cells_) >
        ground_gap_tolerance_cells_)
      {
        return false;
      }
    }
    double height;
    if (!map_.supportHeightUnlocked(ground, height)) {return false;}
    const auto point = map_.keyToPos(key);
    endpoints.push_back({point[0], point[1], height});
  }
  // Search edges, voxel shortcuts, exact-endpoint shortcuts and cached-route
  // checks must all visit the same world-coordinate columns. Rounding integer
  // keys along a line can visit another row at negative-coordinate half ties.
  return validateGroundPath(endpoints, 0.0, check_obstacles, nullptr, nullptr);
}


bool Astar3D::resolvePathGround(
  const OccupyMap::Point & position, VoxelKey & ground) const
{
  const double tolerance = std::max(
    config_.max_step_height,
    resolution_ * static_cast<double>(ground_gap_tolerance_cells_ + 1));
  VoxelKey exact_ground;
  const bool exact=map_.findSupportByHeightUnlocked(
    position[0],position[1],position[2],tolerance,exact_ground);
  if (exact && !map_.inferredSupportUnlocked(exact_ground)) {ground=exact_ground; return true;}
  auto candidate = map_.posToKey(position[0], position[1], position[2]);
  candidate.z += preferred_ground_gap_cells_;
  double height;
  if (map_.findMeasuredGroundUnlocked(candidate,support_depth_cells_,ground) &&
    map_.supportHeightUnlocked(ground,height) && std::abs(height-position[2])<=tolerance)
  {
    return true;
  }
  if (exact) {ground=exact_ground; return true;}
  return false;
}

bool Astar3D::validateGroundPath(
  const std::vector<OccupyMap::Point> & path,
  double maximum_initial_blind_length, bool check_obstacles,
  std::vector<OccupyMap::Point> * refreshed,
  TerrainValidation * failure, bool constrain_blind_ground_layer,
  double maximum_initial_bridge_length) const
{
  if (failure) {*failure = TerrainValidation{};}
  const auto fail = [&](const char * reason, std::size_t index,
      const OccupyMap::Point & position, const VoxelKey * collision = nullptr) {
      if (failure) {
        *failure = TerrainValidation{reason, index, position};
        if (collision) {failure->collision = *collision; failure->has_collision = true;}
      }
      return false;
    };
  if (path.empty()) {
    return fail("empty_path", 0, {});
  }
  for (std::size_t i = 0; i < path.size(); ++i) {
    for (double coordinate : path[i]) {
      if (!std::isfinite(coordinate)) {
        return fail("nonfinite_pose", i, path[i]);
      }
    }
  }
  auto updated = path;
  bool observed = false;
  VoxelKey previous_ground;
  double previous_height = 0.0;
  double first_height = 0.0;
  double travelled = 0.0;
  std::size_t first_supported_index = path.size();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto & from = path[i == 0 ? 0 : i - 1];
    const auto & to = path[i];
    const double length = std::hypot(to[0] - from[0], to[1] - from[1]);
    // Visit every positive-length XY cell interval of this segment. Fixed
    // sampling and checks at dense internal vertices can select a neighbouring
    // cell only at a floating-point grid-corner tie. Midpoints between exact
    // grid crossings give the same columns regardless of path densification.
    std::vector<double> cuts{0.0, 1.0};
    if (i > 0) {
      for (int axis : {0, 1}) {
        const double delta = to[axis] - from[axis];
        if (std::abs(delta) < 1e-12) {continue;}
        const double low = std::min(from[axis], to[axis]);
        const double high = std::max(from[axis], to[axis]);
        const int first = static_cast<int>(std::floor(low / resolution_)) + 1;
        const int last = static_cast<int>(std::ceil(high / resolution_)) - 1;
        for (int boundary = first; boundary <= last; ++boundary) {
          const double ratio = (boundary * resolution_ - from[axis]) / delta;
          if (ratio > 1e-10 && ratio < 1.0 - 1e-10) {cuts.push_back(ratio);}
        }
      }
    }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end(), [](double left, double right) {
          return std::abs(left - right) <= 1e-10;
        }), cuts.end());
    std::vector<double> ratios;
    if (i == 0) {
      ratios.push_back(0.0);
    } else {
      for (std::size_t j = 1; j < cuts.size(); ++j) {
        ratios.push_back(0.5 * (cuts[j - 1] + cuts[j]));
      }
      if (i + 1 == path.size()) {ratios.push_back(1.0);}
    }
    for (std::size_t sample_index = 0; sample_index < ratios.size(); ++sample_index) {
      const double ratio = ratios[sample_index];
      const OccupyMap::Point sample{
        from[0] + ratio * (to[0] - from[0]),
        from[1] + ratio * (to[1] - from[1]),
        from[2] + ratio * (to[2] - from[2])};
      VoxelKey ground;
      double height;
      const bool supported = resolvePathGround(sample, ground) &&
        map_.supportHeightUnlocked(ground, height);
      if (!supported) {
        const double interval_end = i == 0 ? 0.0 :
          (sample_index + 1 < cuts.size() ? cuts[sample_index + 1] : 1.0);
        const double interval_distance = travelled + interval_end * length;
        const bool within_selected_bridge = maximum_initial_bridge_length > 0.0 &&
          interval_distance <= maximum_initial_bridge_length + 1e-9;
        if ((!within_selected_bridge && observed) || interval_distance >
          maximum_initial_blind_length + 1e-9)
        {
          return fail("missing_ground_support", i, sample);
        }
        // Only the original bounded blind prefix or its explicitly selected
        // start connector may lack ground. An incidental radius-neighbour
        // observation does not end that connector early. Every observation
        // still participates in slope/step tests and collisions remain active.
        ground = map_.posToKey(sample[0], sample[1], sample[2]);
        height = sample[2];
      }
      const auto observed_cell = map_.posToKey(sample[0], sample[1], height);
      const bool selected_bridge_sample = maximum_initial_bridge_length > 0.0 &&
        travelled + ratio * length <= maximum_initial_bridge_length + 1e-9;
      if (check_obstacles || !supported || selected_bridge_sample) {
        for (int z = std::min(ground.z, observed_cell.z) + 1;
          z <= map_.posToKey(0, 0, height + config_.robot_height).z; ++z)
        {
          if (map_.occupiedUnlocked(VoxelKey{observed_cell.x, observed_cell.y, z})) {
            const VoxelKey collision{observed_cell.x, observed_cell.y, z};
            return fail("occupied_body_column", i, sample, &collision);
          }
        }
      }
      if (!supported) {continue;}
      if (observed) {
        const double distance = resolution_ * std::hypot(
          static_cast<double>(ground.x - previous_ground.x),
          static_cast<double>(ground.y - previous_ground.y));
        if (!groundConnectionAllowed(previous_height, height, distance)) {
          fail(
            std::abs(height - previous_height) > config_.max_step_height + 1e-6 ?
            "step_limit" : "slope_limit", i, sample);
          if (failure) {
            failure->has_support_edge = true;
            failure->previous_support = previous_ground;
            failure->current_support = ground;
            failure->previous_height = previous_height;
            failure->current_height = height;
            failure->horizontal_distance = distance;
          }
          return false;
        }
      } else {
        // A blind prefix is only a near-field connection to the current
        // ground layer. Without this gate, a line ending on an isolated table
        // top inside the projection radius could appear valid despite having
        // no observed terrain connecting the robot to that higher surface.
        if (constrain_blind_ground_layer && travelled + ratio * length > 1e-9 &&
          std::abs(height - path.front()[2]) > config_.max_step_height + 1e-6)
        {
          return fail("blind_ground_height_mismatch", i, sample);
        }
        first_height = height;
        first_supported_index = i;
      }
      observed = true;
      previous_ground = ground;
      previous_height = height;
      if (sample_index + 1 == ratios.size()) {updated[i][2] = height;}
    }
    travelled += length;
  }
  if (!observed) {
    return fail("no_route_support", path.size() - 1, path.back());
  }
  for (std::size_t i = 0; i < first_supported_index; ++i) {
    updated[i][2] = first_height;
  }
  if (refreshed) {*refreshed = std::move(updated);}
  return true;
}

bool Astar3D::refreshAndValidateTerrainPath(
  const VoxelKey & projected_start, std::vector<OccupyMap::Point> & path,
  TerrainValidation * failure, bool check_obstacles) const
{
  // A new best projection can lie beside an already supported straight route.
  // Validate the earliest support along the route itself, rather than requiring
  // the route to visit that newly selected centre or reach support sooner than
  // that unrelated centre's distance. The configured radius is a hard bound.
  // Normally every sample after first support must retain it; the opt-in case
  // below retains only the concrete straight connector to its observed start
  // endpoint. Slope and step limits remain active in both cases.
  double bridge_length = 0.0;
  if (config_.retain_start_projection_bridge && !path.empty()) {
    const auto selected = map_.keyToPos(projected_start);
    VoxelKey selected_ground;
    double selected_height;
    // This is an exemption for the concrete start projection, not a radius
    // dilation or permission to cross any hole in the first N metres. Cached
    // routes not visiting this selected endpoint retain the strict rule.
    if (map_.findSurfaceGroundUnlocked(
        projected_start, preferred_ground_gap_cells_, ground_gap_tolerance_cells_,
        selected_ground) && map_.supportHeightUnlocked(selected_ground, selected_height) &&
      std::abs(selected_height - path.front()[2]) <= config_.max_step_height + 1e-9 &&
      std::hypot(selected[0] - path.front()[0], selected[1] - path.front()[1]) <=
      config_.start_projection_xy_radius + 1e-9)
    {
      double travelled = 0.0;
      const double dx = selected[0] - path.front()[0];
      const double dy = selected[1] - path.front()[1];
      const double distance = std::hypot(dx, dy);
      double previous_progress = 0.0;
      for (std::size_t i = 0; i < path.size(); ++i) {
        if (distance <= 1e-9) {break;}
        const double sample_dx = path[i][0] - path.front()[0];
        const double sample_dy = path[i][1] - path.front()[1];
        const double progress = (sample_dx * dx + sample_dy * dy) / distance;
        const double cross_track = std::abs(sample_dx * dy - sample_dy * dx) / distance;
        if (cross_track > 1e-7 || progress < previous_progress - 1e-7 ||
          progress > distance + 1e-7)
        {
          break;
        }
        previous_progress = progress;
        if (i > 0) {
          travelled += std::hypot(
            path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
        }
        if (travelled > config_.start_projection_xy_radius + 1e-9) {break;}
        if (std::hypot(path[i][0] - selected[0], path[i][1] - selected[1]) <= 1e-7 &&
          std::abs(path[i][2] - selected_height) <= config_.max_step_height + 1e-9)
        {
          // A supported robot column must retain the ordinary no-gap policy.
          // Only projection from a column without its own observed support is
          // eligible for the near-field connector assumption.
          VoxelKey exact_ground;
          if (!map_.findSupportByHeightUnlocked(
              path.front()[0], path.front()[1], path.front()[2],
              config_.max_step_height, exact_ground))
          {
            bridge_length = travelled;
          }
          break;
        }
      }
    }
  }
  return validateGroundPath(
    path, config_.start_projection_xy_radius, check_obstacles, &path, failure,
    bridge_length > 0.0, bridge_length);
}

bool Astar3D::tryDirectGroundPath(
  const OccupyMap::Point & from, const OccupyMap::Point & to,
  double maximum_initial_blind_length,
  std::vector<OccupyMap::Point> & output, TerrainValidation * failure) const
{
  if (failure) {*failure = TerrainValidation{};}
  for (const auto & endpoint : {from, to}) {
    for (double coordinate : endpoint) {
      if (!std::isfinite(coordinate)) {
        if (failure) {*failure = TerrainValidation{"nonfinite_pose", 0, endpoint};}
        return false;
      }
    }
  }
  if (!std::isfinite(maximum_initial_blind_length) || maximum_initial_blind_length < 0.0) {
    if (failure) {*failure = TerrainValidation{"invalid_blind_length", 0, from};}
    return false;
  }
  const double height_tolerance = std::max(
    config_.max_step_height,
    resolution_ * static_cast<double>(ground_gap_tolerance_cells_ + 1));
  VoxelKey goal_ground;
  if (!map_.findSupportByHeightUnlocked(
      to[0], to[1], to[2], height_tolerance, goal_ground))
  {
    if (failure) {*failure = TerrainValidation{"missing_goal_ground_support", 1, to};}
    return false;
  }
  std::vector<OccupyMap::Point> refreshed;
  if (!validateGroundPath(
      {from, to}, std::min(maximum_initial_blind_length, config_.start_projection_xy_radius),
      true, &refreshed, failure, true))
  {
    return false;
  }
  output = std::move(refreshed);
  return true;
}

std::vector<OccupyMap::Point> Astar3D::simplifyGroundPath(
  const std::vector<OccupyMap::Point> & path,
  double maximum_initial_blind_length) const
{
  if (path.size() < 3) {return path;}
  std::vector<OccupyMap::Point> simplified{path.front()};
  std::size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    std::size_t next = anchor + 1;
    for (std::size_t candidate = path.size() - 1; candidate > anchor + 1; --candidate) {
      if (validateGroundPath(
          {path[anchor], path[candidate]},
          anchor == 0 ? maximum_initial_blind_length : 0.0, true, nullptr, nullptr))
      {
        next = candidate;
        break;
      }
    }
    simplified.push_back(path[next]);
    anchor = next;
  }
  return simplified;
}

double Astar3D::groundObstacleCost(const OccupyMap::Point & position) const
{
  if (!std::all_of(position.begin(), position.end(), [](double v) {return std::isfinite(v);})) {
    return std::numeric_limits<double>::infinity();
  }
  auto first = map_.posToKey(position[0], position[1], position[2]);
  ++first.z;
  const int top = map_.posToKey(0, 0, position[2] + config_.robot_height).z;
  return top < first.z ? 0.0 : map_.obstacleCostUnlocked(first, top - first.z);
}

OccupyMap::Traversability Astar3D::pathPointTraversability(
  const OccupyMap::Point & position) const
{
  if (!std::isfinite(position[0]) || !std::isfinite(position[1]) ||
    !std::isfinite(position[2]))
  {
    return OccupyMap::Traversability::kNoGroundSupport;
  }
  VoxelKey candidate = map_.posToKey(position[0], position[1], position[2]);
  candidate.z += preferred_ground_gap_cells_;
  VoxelKey ground;
  const double height_match_tolerance = std::max(
    config_.max_step_height,
    resolution_ * static_cast<double>(ground_gap_tolerance_cells_ + 1));
  if (resolvePathGround(position,ground))
  {
    candidate.z = ground.z + preferred_ground_gap_cells_;
    if (candidate.x==ground.x && candidate.y==ground.y) {
      return map_.traversableOnSurfaceUnlocked(
        candidate,ground,robot_height_cells_,preferred_ground_gap_cells_,
        ground_gap_tolerance_cells_, nullptr, config_.robot_height) ? OccupyMap::Traversability::kTraversable :
        OccupyMap::Traversability::kOccupied;
    }
    return map_.traversabilityUnlocked(candidate,support_depth_cells_,robot_height_cells_,
      preferred_ground_gap_cells_,ground_gap_tolerance_cells_,nullptr,config_.robot_height);
  }

  // Preserve the legacy height-mismatch diagnostic after the shared resolver
  // has rejected support. No second interpolation attempt at a cell centre.
  if (!map_.findMeasuredGroundUnlocked(candidate, support_depth_cells_, ground)) {
    return OccupyMap::Traversability::kNoGroundSupport;
  }
  double height;
  if (!map_.supportHeightUnlocked(ground, height)) {
    return OccupyMap::Traversability::kNoGroundSupport;
  }
  if (std::abs(height - position[2]) > height_match_tolerance + 1e-9) {
    return OccupyMap::Traversability::kGroundGapMismatch;
  }
  return OccupyMap::Traversability::kNoGroundSupport;
}

std::vector<VoxelKey> Astar3D::simplifyPath(
  const std::vector<VoxelKey> & path) const
{
  if (path.size() <= 2) {
    return path;
  }
  std::vector<VoxelKey> simplified;
  simplified.reserve(path.size());
  std::size_t anchor = 0;
  simplified.push_back(path.front());
  while (anchor + 1 < path.size()) {
    std::size_t next = anchor + 1;
    for (std::size_t candidate = path.size() - 1;
      candidate > anchor + 1; --candidate)
    {
      if (segmentTraversable(path[anchor], path[candidate])) {
        next = candidate;
        break;
      }
    }
    simplified.push_back(path[next]);
    anchor = next;
  }
  return simplified;
}

std::vector<OccupyMap::Point> Astar3D::densifyPath(
  const std::vector<OccupyMap::Point> & path, double maximum_xy_spacing) const
{
  if (path.size() <= 1 || maximum_xy_spacing <= 0.0) {
    return path;
  }

  std::vector<OccupyMap::Point> dense;
  dense.push_back(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & from = path[i - 1];
    const auto & to = path[i];
    const double dx = to[0] - from[0];
    const double dy = to[1] - from[1];
    const double dz = to[2] - from[2];
    const double horizontal_distance = std::hypot(dx, dy);
    const int steps = std::max(
      1, static_cast<int>(std::ceil(horizontal_distance / maximum_xy_spacing)));
    for (int step = 1; step <= steps; ++step) {
      const double ratio =
        static_cast<double>(step) / static_cast<double>(steps);
      dense.push_back(
        OccupyMap::Point{
          from[0] + ratio * dx,
          from[1] + ratio * dy,
          from[2] + ratio * dz});
    }
  }
  return dense;
}

double Astar3D::heuristic(const VoxelKey & from, const VoxelKey & to) const
{
  const double dx = static_cast<double>(from.x - to.x);
  const double dy = static_cast<double>(from.y - to.y);
  VoxelKey from_ground;
  VoxelKey to_ground;
  double from_height;
  double to_height;
  double height_change = 0.0;
  if (map_.findSurfaceGroundUnlocked(
      from, preferred_ground_gap_cells_, ground_gap_tolerance_cells_, from_ground) &&
    map_.findSurfaceGroundUnlocked(
      to, preferred_ground_gap_cells_, ground_gap_tolerance_cells_, to_ground) &&
    map_.supportHeightUnlocked(from_ground, from_height) &&
    map_.supportHeightUnlocked(to_ground, to_height))
  {
    height_change = from_height - to_height;
  }
  return std::hypot(resolution_ * std::hypot(dx, dy), height_change);
}

bool Astar3D::groundConnectionAllowed(
  double from_height, double to_height, double horizontal_distance) const
{
  const double height_change = std::abs(to_height - from_height);
  if (horizontal_distance < 1e-9) {
    return height_change <= 1e-9;
  }
  // The inputs are observed heights, so neither the step nor the slope limit
  // needs a voxel-size allowance. Use the same rule for A* edges and shortcuts.
  return height_change <= config_.max_step_height + 1e-9 &&
         height_change / horizontal_distance <= max_slope_tangent_ + 1e-9;
}

}  // namespace astar_3d_planner
