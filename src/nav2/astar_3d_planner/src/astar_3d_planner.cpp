#include "astar_3d_planner/astar_3d_planner.hpp"
#include "astar_3d_planner/cloud_observations.hpp"
#include "astar_3d_planner/path_refiner.hpp"
#include "astar_3d_planner/goal_connections.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace astar_3d_planner
{

LocalizationHeartbeatStatus localizationHeartbeatStatus(
  bool available, std::int64_t received_ns, std::int64_t now_ns)
{
  if (received_ns == 0) {return {false, "localization_heartbeat_missing", -1.0};}
  const bool clock_valid = received_ns > 0 && now_ns >= received_ns;
  const double age = clock_valid ? (now_ns - received_ns) * 1e-9 : -1.0;
  // An explicit negative availability is never downgraded to a recoverable
  // heartbeat delay, even after its last reception has aged out.
  if (!available && received_ns > 0) {return {false, "localization_unavailable", age};}
  if (!clock_valid) {return {false, "localization_heartbeat_clock_invalid", age};}
  if (now_ns - received_ns >= kLocalizationHeartbeatTimeoutNs) {
    return {false, "localization_heartbeat_stale", age};
  }
  return {true, "ready", age};
}

bool liveObstacleStampFresh(
  const builtin_interfaces::msg::Time & stamp, std::int64_t now_ns, double maximum_age)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u ||
    !std::isfinite(maximum_age) || maximum_age <= 0 || maximum_age > 3600) {return false;}
  const auto stamp_ns = static_cast<std::int64_t>(stamp.sec) * 1000000000ll + stamp.nanosec;
  return stamp_ns > 0 && now_ns >= stamp_ns && now_ns - stamp_ns <= static_cast<std::int64_t>(std::llround(maximum_age * 1e9));
}

using namespace std::chrono_literals;

namespace
{

bool planarYaw(const geometry_msgs::msg::Quaternion & q, double & yaw)
{
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {return false;}
  yaw = std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
  return std::isfinite(yaw);
}

void populateGroundPath(
  nav_msgs::msg::Path & path, const std::vector<OccupyMap::Point> & points,
  const geometry_msgs::msg::Quaternion & goal_orientation)
{
  path.poses.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto & pose = path.poses[i];
    pose.header = path.header;
    pose.pose.position.x = points[i][0];
    pose.pose.position.y = points[i][1];
    pose.pose.position.z = points[i][2];
    double yaw = 0.0;
    if (i + 1 < points.size()) {
      yaw = std::atan2(points[i + 1][1] - points[i][1], points[i + 1][0] - points[i][0]);
    } else if (i > 0) {
      yaw = std::atan2(points[i][1] - points[i - 1][1], points[i][0] - points[i - 1][0]);
    }
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(orientation);
  }
  if (!path.poses.empty()) {path.poses.back().pose.orientation = goal_orientation;}
}

}  // namespace

void TerrainInputBuffer::push(Queue & queue, const Cloud & cloud)
{
  const auto stamp = rclcpp::Time(cloud->header.stamp).nanoseconds();
  if (stamp <= last_pair_stamp_) {return;}
  queue[stamp] = cloud;
  while (queue.size() > 10) {queue.erase(queue.begin());}
}

void TerrainInputBuffer::pushSupport(const Cloud & cloud) {push(support_, cloud);}
void TerrainInputBuffer::pushObstacles(const Cloud & cloud) {push(obstacles_, cloud);}

bool TerrainInputBuffer::pairStamp(std::int64_t & stamp) const
{
  auto support = support_.begin();
  auto obstacle = obstacles_.begin();
  while (support != support_.end() && obstacle != obstacles_.end()) {
    if (support->first == obstacle->first) {stamp = support->first; return true;}
    if (support->first < obstacle->first) {++support;} else {++obstacle;}
  }
  return false;
}

bool TerrainInputBuffer::hasPair() const
{
  std::int64_t stamp;
  return pairStamp(stamp);
}

bool TerrainInputBuffer::popPair(Cloud & support, Cloud & obstacles)
{
  std::int64_t stamp;
  if (!pairStamp(stamp)) {return false;}
  support = std::move(support_.at(stamp));
  obstacles = std::move(obstacles_.at(stamp));
  support_.erase(support_.begin(), support_.upper_bound(stamp));
  obstacles_.erase(obstacles_.begin(), obstacles_.upper_bound(stamp));
  last_pair_stamp_ = stamp;
  return true;
}

void TerrainInputBuffer::clear()
{
  support_.clear(); obstacles_.clear(); last_pair_stamp_ = -1;
}

bool selectStablePathSuffix(
  const nav_msgs::msg::Path & cached_path,
  double robot_x, double robot_y,
  double maximum_distance, double anchor_behind_distance,
  double & progress_distance,
  nav_msgs::msg::Path & output_path,
  geometry_msgs::msg::PoseStamped * projection_out)
{
  if (cached_path.poses.empty() || maximum_distance <= 0.0 ||
    anchor_behind_distance < 0.0 || progress_distance < 0.0)
  {
    return false;
  }

  // Project onto the continuous polyline rather than selecting a sampled pose.
  // The accumulated path distance is the sole cache-progress state.  Restricting
  // every later projection to distances at or after this value prevents a
  // nearby old branch (especially at a crossing or a hairpin) from becoming
  // the beginning of a replanned path.
  std::vector<double> accumulated_distance(cached_path.poses.size(), 0.0);
  for (std::size_t i = 1; i < cached_path.poses.size(); ++i) {
    const auto & previous = cached_path.poses[i - 1].pose.position;
    const auto & current = cached_path.poses[i].pose.position;
    accumulated_distance[i] = accumulated_distance[i - 1] +
      std::hypot(current.x - previous.x, current.y - previous.y);
  }
  const double total_distance = accumulated_distance.back();
  progress_distance = std::min(progress_distance, total_distance);

  std::size_t best_segment = cached_path.poses.size();
  double best_ratio = 0.0;
  double best_path_distance = progress_distance;
  double best_distance_sq = maximum_distance * maximum_distance;
  constexpr double comparison_epsilon = 1e-12;

  if (cached_path.poses.size() == 1) {
    const auto & endpoint = cached_path.poses.front().pose.position;
    const double dx = endpoint.x - robot_x;
    const double dy = endpoint.y - robot_y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq > best_distance_sq) {
      return false;
    }
    best_segment = 0;
    best_ratio = 1.0;
    best_path_distance = 0.0;
    best_distance_sq = distance_sq;
  } else {
    for (std::size_t i = 0; i + 1 < cached_path.poses.size(); ++i) {
      const auto & from = cached_path.poses[i].pose.position;
      const auto & to = cached_path.poses[i + 1].pose.position;
      const double dx = to.x - from.x;
      const double dy = to.y - from.y;
      const double length_sq = dx * dx + dy * dy;
      const double length = std::sqrt(length_sq);
      const double segment_begin = accumulated_distance[i];
      const double segment_end = accumulated_distance[i + 1];
      if (segment_end + comparison_epsilon < progress_distance ||
        length < comparison_epsilon)
      {
        continue;
      }

      const double minimum_ratio = std::clamp(
        (progress_distance - segment_begin) / length, 0.0, 1.0);
      const double unconstrained_ratio =
        ((robot_x - from.x) * dx + (robot_y - from.y) * dy) / length_sq;
      const double ratio = std::clamp(unconstrained_ratio, minimum_ratio, 1.0);
      const double projection_x = from.x + ratio * dx;
      const double projection_y = from.y + ratio * dy;
      const double offset_x = projection_x - robot_x;
      const double offset_y = projection_y - robot_y;
      const double distance_sq = offset_x * offset_x + offset_y * offset_y;
      const double path_distance = segment_begin + ratio * length;

      // Keep the earlier path location on an exact tie.  Once the robot has
      // actually reached a later branch, progress_distance already excludes
      // the earlier one.
      if (distance_sq + comparison_epsilon < best_distance_sq) {
        best_segment = i;
        best_ratio = ratio;
        best_path_distance = path_distance;
        best_distance_sq = distance_sq;
      }
    }
  }
  if (best_segment == cached_path.poses.size()) {
    return false;
  }

  progress_distance = std::max(progress_distance, best_path_distance);
  output_path.header = cached_path.header;
  output_path.poses.clear();

  geometry_msgs::msg::PoseStamped projection = cached_path.poses[best_segment];
  if (cached_path.poses.size() > 1) {
    const auto & from = cached_path.poses[best_segment].pose.position;
    const auto & to = cached_path.poses[best_segment + 1].pose.position;
    projection.pose.position.x = from.x + best_ratio * (to.x - from.x);
    projection.pose.position.y = from.y + best_ratio * (to.y - from.y);
    projection.pose.position.z = from.z + best_ratio * (to.z - from.z);
  }

  if (projection_out) {*projection_out = projection;}

  // Nav2's global-plan contract expects the first pose to represent the
  // current planning start.  Never expose a cached sampled point behind the
  // robot as pose zero.
  auto current = projection;
  current.pose.position.x = robot_x;
  current.pose.position.y = robot_y;
  output_path.poses.push_back(current);

  const double reconnect_distance = std::sqrt(best_distance_sq);
  if (reconnect_distance > 1e-3) {
    output_path.poses.push_back(projection);
  }

  const std::size_t first_future_index =
    cached_path.poses.size() == 1 ? cached_path.poses.size() :
    best_segment + 1;
  for (std::size_t i = first_future_index; i < cached_path.poses.size(); ++i) {
    const auto & candidate = cached_path.poses[i];
    const auto & previous = output_path.poses.back().pose.position;
    const auto & position = candidate.pose.position;
    if (std::hypot(position.x - previous.x, position.y - previous.y) > 1e-6 ||
      std::abs(position.z - previous.z) > 1e-6)
    {
      output_path.poses.push_back(candidate);
    }
  }

  // A single endpoint has no tangent for Rotation Shim or MPPI.  If the robot
  // has passed or drifted away from it, retain a two-pose return path.
  if (output_path.poses.size() == 1 && reconnect_distance > 1e-3) {
    output_path.poses.push_back(cached_path.poses.back());
  }

  for (std::size_t i = 0; i + 1 < output_path.poses.size(); ++i) {
    const auto & position = output_path.poses[i].pose.position;
    const auto & next = output_path.poses[i + 1].pose.position;
    const double dx = next.x - position.x;
    const double dy = next.y - position.y;
    if (std::hypot(dx, dy) <= 1e-9) {
      continue;
    }
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, std::atan2(dy, dx));
    output_path.poses[i].pose.orientation = tf2::toMsg(orientation);
  }

  return !output_path.poses.empty();
}

namespace
{

struct GroundDiagnosticOnExit
{
  std::function<void()> publish;
  ~GroundDiagnosticOnExit() noexcept
  {
    // Diagnostics must not turn a valid path or an original planner exception
    // into a different result when DDS/lifecycle publication is unavailable.
    try {publish();} catch (const std::exception &) {}
  }
};

template<typename T>
void declareAndGet(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & parameter_name, const T & default_value, T & output)
{
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_name, rclcpp::ParameterValue(default_value));
  node->get_parameter(parameter_name, output);
}

}  // namespace

Astar3DPlanner::~Astar3DPlanner()
{
  map_audit_timer_.reset();
  stopWorker();
  support_map_publisher_.reset();
}

void Astar3DPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = std::move(tf);
  name_ = std::move(name);
  global_frame_ = costmap_ros->getGlobalFrameID();
  robot_base_frame_ = costmap_ros->getBaseFrameID();

  const auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("Astar3DPlanner failed to lock its parent node");
  }
  const std::string prefix = name_ + ".";

  OccupyMap::Config map_config;
  declareAndGet(node, prefix + "use_static_map", false, use_static_map_);
  declareAndGet(node, prefix + "verify_static_map_identity", false, verify_static_map_identity_);
  declareAndGet(node, prefix + "static_map_id_topic",
    std::string("/offline_map/id"), static_map_id_topic_);
  std::string expected_map_id, expected_support, expected_obstacles;
  declareAndGet(node, prefix + "expected_map_id", std::string{}, expected_map_id);
  declareAndGet(node, prefix + "expected_support_sha256", std::string{}, expected_support);
  declareAndGet(node, prefix + "expected_static_obstacles_sha256", std::string{}, expected_obstacles);
  fixed_map_guard_.reset();
  last_map_guard_reason_.clear();
  if (verify_static_map_identity_) {
    if (!use_static_map_) {
      throw nav2_core::PlannerException("Map identity checks require use_static_map=true");
    }
    fixed_map_guard_ = std::make_unique<FixedMapGuard>(
      global_frame_, expected_map_id, expected_support, expected_obstacles);
    if (fixed_map_guard_->faulted()) {
      throw nav2_core::PlannerException("Invalid fixed-map identity configuration: " +
        fixed_map_guard_->reason());
    }
  }
  declareAndGet(node, prefix + "static_obstacle_topic",
    std::string("/offline_map/obstacles"), static_obstacle_topic_);
  map_config.require_static_obstacles = use_static_map_;
  declareAndGet(
    node, prefix + "support_topic",
    std::string("/terrain/support"), support_topic_);
  declareAndGet(
    node, prefix + "obstacle_topic",
    std::string("/terrain/obstacles_global"), obstacle_topic_);
  declareAndGet(node,prefix+"live_obstacle_max_observation_age",0.6,
    live_obstacle_max_observation_age_);
  if(!std::isfinite(live_obstacle_max_observation_age_) ||
    live_obstacle_max_observation_age_<=0 || live_obstacle_max_observation_age_>3600) {
    throw nav2_core::PlannerException("Invalid live obstacle observation retention");
  }
  declareAndGet(node, prefix + "live_obstacle_freshness_timeout", 2.0,
    live_obstacle_freshness_timeout_);
  live_obstacle_health_.setTimeout(live_obstacle_freshness_timeout_);
  declareAndGet(node, prefix + "lio_odom_topic", std::string("/Odometry"), lio_odom_topic_);
  declareAndGet(
    node, prefix + "path_validity_service",
    std::string("/terrain/is_path_valid"), path_validity_service_);
  declareAndGet(node, prefix + "use_lio_start_z", true, use_lio_start_z_);
  declareAndGet(
    node, prefix + "goal_z_zero_is_unspecified", true, goal_z_zero_is_unspecified_);
  declareAndGet(node, prefix + "lio_height_offset", 0.0, lio_height_offset_);
  declareAndGet(node, prefix + "path_pose_spacing", 0.05, path_pose_spacing_);
  declareAndGet(node, prefix + "reuse_previous_path", true, reuse_previous_path_);
  declareAndGet(
    node, prefix + "path_reuse_start_tolerance", 0.25,
    path_reuse_start_tolerance_);
  declareAndGet(
    node, prefix + "path_reuse_anchor_behind", 0.0,
    path_reuse_anchor_behind_);
  declareAndGet(
    node, prefix + "cached_goal_obstacle_grace_period", 5.0,
    cached_goal_obstacle_grace_period_);
  declareAndGet(
    node, prefix + "cached_goal_obstacle_ignore_radius", 0.4,
    cached_goal_obstacle_ignore_radius_);
  declareAndGet(
    node, prefix + "path_validity_start_tolerance", 0.75,
    path_validity_start_tolerance_);
  declareAndGet(
    node, prefix + "path_validity_ignore_near_robot_radius", 0.6,
    path_validity_ignore_near_robot_radius_);
  declareAndGet(
    node, prefix + "path_validity_min_blocked_length", 0.25,
    path_validity_min_blocked_length_);
  declareAndGet(node, prefix + "local_detour_max_deviation", 0.75,
    local_detour_max_deviation_);
  declareAndGet(node, prefix + "local_detour_timeout", 12.0,
    local_detour_budget_config_.max_duration_s);
  declareAndGet(node, prefix + "local_detour_progress_timeout", 4.0,
    local_detour_budget_config_.no_progress_timeout_s);
  local_detour_budget_ = LocalDetourBudget(local_detour_budget_config_);
  declareAndGet(node, prefix + "point_stride", 1, point_stride_);
  point_stride_ = std::max(1, point_stride_);

  declareAndGet(node, prefix + "resolution", 0.1, map_config.resolution);
  declareAndGet(node, prefix + "inflate_radius", 0.3, map_config.inflate_radius);
  declareAndGet(node, prefix + "cost_inflate_cells", 2, map_config.cost_inflate_cells);
  declareAndGet(node, prefix + "support_radius", 0.15, map_config.support_radius);
  declareAndGet(node, prefix + "support_inference_enabled", false,
    map_config.support_inference_enabled);
  declareAndGet(node, prefix + "support_inference_radius", 0.25,
    map_config.support_inference_radius);
  declareAndGet(node, prefix + "support_inference_max_height_span", 0.06,
    map_config.support_inference_max_height_span);
  declareAndGet(node, prefix + "support_inference_max_residual", 0.01,
    map_config.support_inference_max_residual);
  declareAndGet(node, prefix + "support_inference_max_slope_deg", 10.0,
    map_config.support_inference_max_slope_deg);
  declareAndGet(node, prefix + "support_inference_max_columns", 32768,
    map_config.support_inference_max_columns);
  declareAndGet(
    node, prefix + "support_merge_height", 0.15,
    map_config.support_merge_height);
  declareAndGet(
    node, prefix + "compensate_height_drift", false,
    map_config.compensate_height_drift);
  declareAndGet(
    node, prefix + "height_drift_min_overlap", 30,
    map_config.height_drift_min_overlap);
  declareAndGet(
    node, prefix + "height_drift_max_update", 0.02,
    map_config.height_drift_max_update);
  declareAndGet(
    node, prefix + "height_drift_inlier_tolerance", 0.008,
    map_config.height_drift_inlier_tolerance);
  declareAndGet(
    node, prefix + "height_drift_min_consensus", 0.7,
    map_config.height_drift_min_consensus);
  declareAndGet(
    node, prefix + "height_drift_min_xy_span", 0.6,
    map_config.height_drift_min_xy_span);
  declareAndGet(
    node, prefix + "height_drift_min_change", 0.001,
    map_config.height_drift_min_change);
  declareAndGet(
    node, prefix + "obstacle_update_radius", 4.5,
    map_config.obstacle_update_radius);
  declareAndGet(
    node, prefix + "obstacle_clear_after_updates", 3,
    map_config.obstacle_clear_after_updates);
  declareAndGet(
    node, prefix + "synthetic_obstacle_clearance_radius", 0.0,
    map_config.synthetic_obstacle_clearance_radius);

  declareAndGet(
    node, prefix + "heuristic_weight", 1.0, astar_config_.heuristic_weight);
  declareAndGet(
    node, prefix + "obstacle_cost_weight", 2.0, astar_config_.obstacle_cost_weight);
  declareAndGet(
    node, prefix + "slope_cost_weight", 0.2, astar_config_.slope_cost_weight);
  declareAndGet(
    node, prefix + "ground_gap_cost_weight", 0.1,
    astar_config_.ground_gap_cost_weight);
  declareAndGet(node, prefix + "heading_aware_search", true, heading_aware_search_);
  declareAndGet(node, prefix + "path_refinement_enabled", true, path_refinement_enabled_);
  declareAndGet(node, prefix + "turn_cost_weight", 0.15, turn_cost_weight_);
  if (!std::isfinite(turn_cost_weight_) || turn_cost_weight_ < 0.0) {
    throw nav2_core::PlannerException("turn_cost_weight must be finite and nonnegative");
  }
  declareAndGet(node, prefix + "max_slope_deg", 50.0, astar_config_.max_slope_deg);
  declareAndGet(
    node, prefix + "max_step_height", 0.18,
    astar_config_.max_step_height);
  declareAndGet(
    node, prefix + "surface_connection_radius", 0.25,
    astar_config_.surface_connection_radius);
  declareAndGet(
    node, prefix + "ground_check_depth", 0.6, astar_config_.ground_check_depth);
  declareAndGet(node, prefix + "robot_height", 0.45, astar_config_.robot_height);
  declareAndGet(
    node, prefix + "planning_height_offset", 0.2,
    astar_config_.planning_height_offset);
  declareAndGet(
    node, prefix + "ground_gap_tolerance", 0.12,
    astar_config_.ground_gap_tolerance);
  declareAndGet(
    node, prefix + "start_projection_xy_radius", 1.5,
    astar_config_.start_projection_xy_radius);
  declareAndGet(
    node, prefix + "start_projection_height_tolerance", 0.2,
    astar_config_.start_projection_height_tolerance);
  declareAndGet(
    node, prefix + "start_projection_direction_weight", 2.0,
    astar_config_.start_projection_direction_weight);
  declareAndGet(
    node, prefix + "retain_start_projection_bridge", false,
    astar_config_.retain_start_projection_bridge);
  declareAndGet(
    node, prefix + "goal_projection_xy_radius", 0.5,
    astar_config_.goal_projection_xy_radius);
  declareAndGet(
    node, prefix + "projection_z_radius", 1.0,
    astar_config_.projection_z_radius);
  declareAndGet(
    node, prefix + "unspecified_goal_projection_z_radius", 10.0,
    astar_config_.unspecified_goal_projection_z_radius);
  int max_search_nodes = 500000;
  declareAndGet(node, prefix + "max_search_nodes", 500000, max_search_nodes);
  astar_config_.max_search_nodes =
    static_cast<std::size_t>(std::max(1, max_search_nodes));

  if (map_config.resolution <= 0.0 || path_pose_spacing_ <= 0.0 ||
    astar_config_.max_step_height < 0.0 ||
    astar_config_.start_projection_height_tolerance < 0.0 ||
    astar_config_.start_projection_direction_weight < 0.0 ||
    astar_config_.surface_connection_radius <= map_config.resolution ||
    map_config.support_merge_height < 0.0 ||
    path_reuse_start_tolerance_ <= 0.0 ||
    path_reuse_anchor_behind_ < 0.0 ||
    cached_goal_obstacle_grace_period_ < 0.0 ||
    cached_goal_obstacle_ignore_radius_ < 0.0 ||
    path_validity_start_tolerance_ <= 0.0 ||
    path_validity_ignore_near_robot_radius_ < 0.0 ||
    path_validity_min_blocked_length_ <= 0.0 ||
    !std::isfinite(local_detour_max_deviation_) ||
    local_detour_max_deviation_ < path_validity_start_tolerance_ ||
    map_config.obstacle_update_radius <= 0.0 ||
    map_config.obstacle_clear_after_updates < 1)
  {
    throw nav2_core::PlannerException(
            "Astar3DPlanner map resolution, path spacing/reuse tolerance, goal "
            "obstacle grace settings, obstacle update radius and clear count are invalid");
  }
  declareAndGet(node, prefix + "footprint_check_enabled", true, footprint_check_enabled_);
  int footprint_max_samples = 20000;
  declareAndGet(node, prefix + "footprint_max_samples", 20000, footprint_max_samples);
  declareAndGet(node, prefix + "near_start_static_refinement_radius", 0.0,
    footprint_config_.near_start_static_refinement_radius);
  declareAndGet(node, prefix + "near_start_static_obstacle_margin", 0.02,
    footprint_config_.near_start_static_obstacle_margin);
  declareAndGet(node, prefix + "refine_static_obstacle_extents", false,
    footprint_config_.refine_static_obstacle_extents);
  declareAndGet(node, prefix + "static_obstacle_extent_margin", 0.02,
    footprint_config_.static_obstacle_extent_margin);
  declareAndGet(node, prefix + "near_start_measured_refinement_radius", 0.0,
    footprint_config_.near_start_measured_refinement_radius);
  declareAndGet(node, prefix + "near_start_measured_obstacle_margin", 0.03,
    footprint_config_.near_start_measured_obstacle_margin);
  declareAndGet(node, prefix + "check_in_place_turn_sweeps", true,
    footprint_config_.check_in_place_turn_sweeps);
  if (footprint_check_enabled_) {
    const auto footprint = costmap_ros->getRobotFootprint();
    if (footprint.size() < 3 || footprint_max_samples < 1) {
      throw nav2_core::PlannerException("A valid Nav2 footprint is required for rectangular checks");
    }
    footprint_config_.min_x = footprint_config_.min_y = std::numeric_limits<double>::infinity();
    footprint_config_.max_x = footprint_config_.max_y = -std::numeric_limits<double>::infinity();
    for (const auto & point : footprint) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        throw nav2_core::PlannerException("Nav2 footprint must contain finite coordinates");
      }
      footprint_config_.min_x = std::min(footprint_config_.min_x, point.x);
      footprint_config_.max_x = std::max(footprint_config_.max_x, point.x);
      footprint_config_.min_y = std::min(footprint_config_.min_y, point.y);
      footprint_config_.max_y = std::max(footprint_config_.max_y, point.y);
    }
    footprint_config_.robot_height = astar_config_.robot_height;
    footprint_config_.max_samples = static_cast<std::size_t>(footprint_max_samples);
    // A rejection must be necessary for the padded rectangular envelope at
    // every orientation, including exact positions away from a voxel centre.
    map_config.footprint_inscribed_radius = std::max(0.0, std::min({
      -footprint_config_.min_x, footprint_config_.max_x,
      -footprint_config_.min_y, footprint_config_.max_y}));
  }
  // The early hard mask may only reject collisions that the final checker
  // must also reject. Refined point bounds invalidate a full-voxel proof.
  map_config.refine_static_obstacle_extents = footprint_check_enabled_ &&
    (footprint_config_.refine_static_obstacle_extents ||
    footprint_config_.near_start_static_refinement_radius > 0.0);
  map_config.refine_measured_obstacle_extents = footprint_check_enabled_ &&
    footprint_config_.near_start_measured_refinement_radius > 0.0;
  double prefilter_margin = std::numeric_limits<double>::infinity();
  if (footprint_config_.refine_static_obstacle_extents) {
    prefilter_margin = std::min(prefilter_margin, footprint_config_.static_obstacle_extent_margin);
  }
  if (footprint_config_.near_start_static_refinement_radius > 0.0) {
    prefilter_margin = std::min(prefilter_margin, footprint_config_.near_start_static_obstacle_margin);
  }
  if (map_config.refine_measured_obstacle_extents) {
    prefilter_margin = std::min(prefilter_margin, footprint_config_.near_start_measured_obstacle_margin);
  }
  map_config.obstacle_extent_prefilter_margin = std::isfinite(prefilter_margin) ? prefilter_margin : 0.0;
  map_ = std::make_unique<OccupyMap>(map_config);
  if (footprint_check_enabled_) {
    footprint_checker_ = std::make_unique<SweptFootprint>(*map_, footprint_config_);
    RCLCPP_INFO(node->get_logger(),
      "Rectangular path check: padded footprint x=[%.3f,%.3f], y=[%.3f,%.3f], height=%.3f; raw obstacles",
      footprint_config_.min_x, footprint_config_.max_x, footprint_config_.min_y,
      footprint_config_.max_y, footprint_config_.robot_height);
    RCLCPP_INFO(node->get_logger(),
      "Near-start saved-point fallback: radius=%.2f m, XY margin=%.2f m",
      footprint_config_.near_start_static_refinement_radius,
      footprint_config_.near_start_static_obstacle_margin);
    RCLCPP_INFO(node->get_logger(),
      "Global saved-point extents=%s margin=%.2f m; near-start measured extents radius=%.2f m margin=%.2f m",
      footprint_config_.refine_static_obstacle_extents ? "true" : "false",
      footprint_config_.static_obstacle_extent_margin,
      footprint_config_.near_start_measured_refinement_radius,
      footprint_config_.near_start_measured_obstacle_margin);
    RCLCPP_INFO(node->get_logger(), "Global footprint policy: %s",
      footprint_config_.check_in_place_turn_sweeps ?
      "complete turn-in-place maneuver sweeps" :
      "guidance corridors and endpoint poses; local controller validates turns");
    RCLCPP_INFO(node->get_logger(),
      "Necessary rectangular prefilter: inscribed radius=%.3f m, hard radius=%.3f m; "
      "configured circle %.3f m remains clearance preference",
      *map_config.footprint_inscribed_radius, map_->hardPrefilterRadius(), map_config.inflate_radius);
  } else {footprint_checker_.reset();}
  if (use_static_map_ && map_config.compensate_height_drift) {
    throw nav2_core::PlannerException("Offline maps require compensate_height_drift=false");
  }
  support_map_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/planner/support_map", use_static_map_ ? rclcpp::QoS(1).reliable().transient_local() :
    rclcpp::QoS(rclcpp::SensorDataQoS().keep_last(1)));
  last_support_map_publish_ = std::chrono::steady_clock::time_point{};
  const auto diagnostic_qos = rclcpp::QoS(1).reliable().transient_local();
  ground_query_markers_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planner/ground_queries", diagnostic_qos);
  direct_path_markers_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planner/direct_path_markers", diagnostic_qos);
  direct_path_status_ = node->create_publisher<std_msgs::msg::String>(
    "/planner/direct_path_status", diagnostic_qos);
  last_ground_query_publish_ = std::chrono::steady_clock::time_point{};
  if (fixed_map_guard_) {
    // A normal publisher keeps input diagnostics visible while the plugin is
    // configuring or inactive. It does not authorize robot commands.
    map_status_publisher_ = rclcpp::create_publisher<std_msgs::msg::String>(
      node, "/planner/map_status", diagnostic_qos);
    static_map_id_sub_ = node->create_subscription<std_msgs::msg::String>(
      static_map_id_topic_, diagnostic_qos,
      [this](std_msgs::msg::String::SharedPtr identity) {
        if (fixed_map_guard_) {fixed_map_guard_->observeIdentity(identity->data);}
      });
  }
  start_failure_publisher_ = node->create_publisher<std_msgs::msg::String>(
    "/planner/start_failure", diagnostic_qos);
  start_failure_markers_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planner/start_failure_markers", diagnostic_qos);
  {
    std::lock_guard<std::mutex> lock(path_validity_mutex_);
    validity_path_initialized_ = false;
    validity_progress_distance_ = 0.0;
    reference_goal_.reset();
    local_detour_budget_.reset();
  }

  // Clear the previous lifecycle's stop flag before subscriptions can accept a
  // once-only fixed map. A validated first frame must not be discarded here.
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    stop_worker_ = false;
  }
  support_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    support_topic_, use_static_map_ ? rclcpp::QoS(1).reliable().transient_local() :
    rclcpp::QoS(rclcpp::SensorDataQoS()),
    std::bind(&Astar3DPlanner::supportCallback, this, std::placeholders::_1));
  obstacle_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    obstacle_topic_, rclcpp::SensorDataQoS(),
    std::bind(&Astar3DPlanner::obstacleCallback, this, std::placeholders::_1));
  if (use_static_map_) {
    offline_localization_ready_.store(false);
    offline_alignment_ready_.store(false);
    offline_ready_wall_ns_.store(0);
    offline_alignment_wall_ns_.store(0);
    live_obstacle_health_.reset();
    live_obstacle_status_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(
      node, "/planner/live_obstacle_status", diagnostic_qos);
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".localization_ready_topic",
      rclcpp::ParameterValue(std::string("/offline_localization/ready")));
    localization_ready_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      node->get_parameter(name_ + ".localization_ready_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr ready) {
        offline_ready_wall_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        offline_localization_ready_.store(ready->data);
        if (!ready->data) {clearPathCache();}
      });
    // Independent of the final motion gate: input processing must be able to
    // bootstrap/recover while navigation is waiting for a fresh obstacle map.
    alignment_ready_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/offline_localization/localization_available", diagnostic_qos,
      [this](std_msgs::msg::Bool::SharedPtr ready) {
        offline_alignment_wall_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        offline_alignment_ready_.store(ready->data);
        if (!ready->data) {
          std::lock_guard<std::mutex> lock(pending_mutex_);
          pending_obstacle_cloud_.reset();
          if (map_) {map_->clearDynamicObstacles(true);}
          live_obstacle_health_.reset();
        }
      });
    static_obstacle_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      static_obstacle_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
        if (fixed_map_guard_ && fixed_map_guard_->validateCloud(
            FixedMapGuard::Layer::Obstacles, *cloud) != FixedMapGuard::Decision::Accept)
        {return;}
        if (cloud->header.frame_id != global_frame_) {
          const auto node = node_.lock();
          if (node) {RCLCPP_ERROR(node->get_logger(), "Static obstacles must be in map frame");}
          return;
        }
        {std::lock_guard<std::mutex> lock(pending_mutex_);
          if (stop_worker_) {return;}
          pending_static_obstacle_cloud_ = std::move(cloud);}
        pending_cv_.notify_one();
      });
  }
  lio_odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    lio_odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&Astar3DPlanner::lioOdomCallback, this, std::placeholders::_1));
  terrain_path_validity_service_ =
    node->create_service<nav2_msgs::srv::IsPathValid>(
    path_validity_service_,
    std::bind(
      &Astar3DPlanner::terrainPathValidityCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  detailed_terrain_path_validity_service_ =
    node->create_service<terrain_navigation_interfaces::srv::ValidateTerrainPath>(
    path_validity_service_ + "_detailed",
    std::bind(&Astar3DPlanner::detailedTerrainPathValidityCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  // Reference-path arbitration may temporarily leave a nearby live obstacle
  // to the local controller. Recovery/safety callers keep the strict service.
  reference_terrain_path_validity_service_ =
    node->create_service<terrain_navigation_interfaces::srv::ValidateTerrainPath>(
    path_validity_service_ + "_reference",
    std::bind(&Astar3DPlanner::referenceTerrainPathValidityCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  map_worker_ = std::thread(&Astar3DPlanner::mapWorker, this);
  if (use_static_map_) {
    live_obstacle_status_timer_ = node->create_wall_timer(100ms,
      [this]() {publishLiveObstacleHealth();});
  }
  if (fixed_map_guard_) {
    map_audit_timer_ = node->create_wall_timer(500ms, [this]() {auditFixedMap();});
    auditFixedMap();
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Configured %s: support=%s, obstacles=%s, global_frame=%s, resolution=%.3f m, "
    "obstacle refresh radius=%.2f m, clear after=%d updates, path check=%s "
    "(configured local ignore %.2f m, effective LiDAR blind bridge %.2f m)",
    name_.c_str(), support_topic_.c_str(), obstacle_topic_.c_str(),
    global_frame_.c_str(), map_config.resolution,
    map_config.obstacle_update_radius, map_config.obstacle_clear_after_updates,
    path_validity_service_.c_str(),
    path_validity_ignore_near_robot_radius_,
    std::max(
      path_validity_ignore_near_robot_radius_,
      astar_config_.start_projection_xy_radius + path_pose_spacing_));
  RCLCPP_INFO(node->get_logger(),
    "Reference route check=%s_reference: local deviation %.2f m, progress budget %.1f s, "
    "total detour budget %.1f s; actual footprint and strict recovery stay collision checked",
    path_validity_service_.c_str(), local_detour_max_deviation_,
    local_detour_budget_config_.no_progress_timeout_s,
    local_detour_budget_config_.max_duration_s);
}

void Astar3DPlanner::cleanup()
{
  const auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "Cleaning up %s", name_.c_str());
  }
  map_audit_timer_.reset();
  live_obstacle_status_timer_.reset();
  static_map_id_sub_.reset();
  support_sub_.reset();
  obstacle_sub_.reset();
  static_obstacle_sub_.reset();
  localization_ready_sub_.reset();
  alignment_ready_sub_.reset();
  lio_odom_sub_.reset();
  terrain_path_validity_service_.reset();
  detailed_terrain_path_validity_service_.reset();
  reference_terrain_path_validity_service_.reset();
  stopWorker();
  live_obstacle_status_pub_.reset();
  live_obstacle_health_.reset();
  map_status_publisher_.reset();
  fixed_map_guard_.reset();
  support_map_publisher_.reset();
  start_failure_publisher_.reset();
  start_failure_markers_.reset();
  {
    std::lock_guard<std::mutex> lock(ground_diagnostics_mutex_);
    ground_query_markers_.reset(); direct_path_markers_.reset(); direct_path_status_.reset();
  }
  clearPathCache();
  {
    std::lock_guard<std::mutex> lock(path_validity_mutex_);
    validity_path_initialized_ = false;
    validity_progress_distance_ = 0.0;
  }
  footprint_checker_.reset();
  map_.reset();
  tf_.reset();
}

void Astar3DPlanner::activate()
{
  {
    std::lock_guard<std::mutex> lock(support_snapshot_mutex_);
    if (support_map_publisher_) {support_map_publisher_->on_activate();}
    active_.store(true);
  }
  if (start_failure_publisher_) {start_failure_publisher_->on_activate();}
  if (start_failure_markers_) {start_failure_markers_->on_activate();}
  {
    std::lock_guard<std::mutex> lock(ground_diagnostics_mutex_);
    if (ground_query_markers_) {ground_query_markers_->on_activate();}
    if (direct_path_markers_) {direct_path_markers_->on_activate();}
    if (direct_path_status_) {direct_path_status_->on_activate();}
  }
  // The fixed support can arrive while the plugin is still inactive and is
  // published only once upstream. Publish the loaded snapshot on activation.
  if (use_static_map_) {publishSupportSnapshot(true);}
  clearStartFailure();
  const auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "Activating %s", name_.c_str());
  }
}

void Astar3DPlanner::deactivate()
{
  clearStartFailure();
  {
    std::lock_guard<std::mutex> lock(support_snapshot_mutex_);
    active_.store(false);
    if (support_map_publisher_) {support_map_publisher_->on_deactivate();}
  }
  if (start_failure_publisher_) {start_failure_publisher_->on_deactivate();}
  if (start_failure_markers_) {start_failure_markers_->on_deactivate();}
  {
    std::lock_guard<std::mutex> lock(ground_diagnostics_mutex_);
    if (ground_query_markers_) {ground_query_markers_->on_deactivate();}
    if (direct_path_markers_) {direct_path_markers_->on_deactivate();}
    if (direct_path_status_) {direct_path_status_->on_deactivate();}
  }
  const auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "Deactivating %s", name_.c_str());
  }
}

bool Astar3DPlanner::validateFootprintPath(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Quaternion & start_orientation,
  const geometry_msgs::msg::Quaternion & goal_orientation,
  SweptFootprint::Failure * failure, bool report_failure,
  const SweptFootprint * checker, const OccupyMap * query_map)
{
  if (!footprint_check_enabled_) {return true;}
  std::unique_ptr<SweptFootprint> local_checker;
  if (!checker) {
    auto config = footprint_config_;
    if (!path.poses.empty()) {
      const auto & p = path.poses.front().pose.position;
      config.near_start_center = std::array<double, 2>{p.x, p.y};
    }
    const auto * source = query_map ? query_map : map_.get();
    if (source) {local_checker = std::make_unique<SweptFootprint>(*source, config);}
    checker = local_checker.get();
  }
  SweptFootprint::Failure result;
  double start_yaw = 0.0, goal_yaw = 0.0;
  bool valid = false;
  if (!checker || !planarYaw(start_orientation, start_yaw) ||
    !planarYaw(goal_orientation, goal_yaw))
  {result.reason = "footprint_invalid_orientation";}
  else {
    std::vector<OccupyMap::Point> points;
    points.reserve(path.poses.size());
    for (const auto & pose : path.poses) {
      const auto & p = pose.pose.position;
      points.push_back({p.x, p.y, p.z});
    }
    valid = checker->validatePath(points, start_yaw, goal_yaw, &result);
  }
  if (failure) {*failure = result;}
  if (!valid && report_failure) {reportFootprintFailure(path, result, query_map);}
  return valid;
}

void Astar3DPlanner::reportFootprintFailure(
  const nav_msgs::msg::Path & path, const SweptFootprint::Failure & result,
  const OccupyMap * query_map)
{
  Astar3D::ProjectionDiagnostics diagnostic;
  diagnostic.stage = "footprint"; diagnostic.reason = result.reason;
  diagnostic.has_collision = result.has_collision;
  diagnostic.collision = result.collision; diagnostic.sample = result.position;
  const auto & p = path.poses.empty() ? geometry_msgs::msg::Point{} :
    path.poses.front().pose.position;
  publishStartFailure({p.x, p.y, p.z}, diagnostic, nullptr, nullptr, nullptr, query_map);
  if (const auto node = node_.lock()) {
    RCLCPP_WARN(node->get_logger(),
      "Rectangular path rejected: %s at pose=%zu near (%.3f,%.3f,%.3f), yaw=%.3f, samples=%zu",
      result.reason, result.index, result.position[0], result.position[1], result.position[2],
      result.yaw, result.samples);
  }
}

nav_msgs::msg::Path Astar3DPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const auto node = node_.lock();
  if (!node || !map_) {
    throw nav2_core::PlannerException("Astar3DPlanner is not configured");
  }
  if (fixed_map_guard_ && !fixed_map_guard_->ready()) {
    throw nav2_core::PlannerException("Fixed-map input is not verified: " +
      fixed_map_guard_->reason());
  }
  if (!localizationReady()) {
    throw nav2_core::PlannerException("Known-map navigation gate is not ready; alignment=" +
      std::to_string(alignmentReady()) + ", motion=" +
      std::to_string(offline_localization_ready_.load()));
  }
  if (!liveObstaclesReady()) {
    throw nav2_core::PlannerException(std::string("Global live obstacle map is not ready: ") +
      live_obstacle_health_.statusNow([&node]() {return node->now().nanoseconds();}).reason);
  }
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
            "Astar3DPlanner start and goal must be in " + global_frame_);
  }
  double requested_goal_yaw;
  if (!planarYaw(goal.pose.orientation, requested_goal_yaw)) {
    throw nav2_core::PlannerException("Astar3DPlanner invalid requested goal orientation");
  }
  {
    std::lock_guard<std::mutex> lock(path_validity_mutex_);
    reference_goal_ = std::array<double, 4>{goal.pose.position.x, goal.pose.position.y,
      goal.pose.position.z, requested_goal_yaw};
  }

  const auto robot_pose = [&]() {
      const auto transform = tf_->lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero, 50ms);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return pose;
    };
  bool advance_robot_start = false;
  try {advance_robot_start = planningStartMatchesRobot(start, robot_pose());}
  catch (const tf2::TransformException &) {
    // A caller can deliberately plan from an explicit pose without a live
    // base transform. Such requests retain the GlobalPlanner start contract.
  }

  auto query_footprint = footprint_config_;
  query_footprint.near_start_center = std::array<double, 2>{
    start.pose.position.x, start.pose.position.y};
  std::unique_ptr<OccupyMap> snapshot;
  VoxelKey checked_start;
  nav_msgs::msg::Path path;
  if (use_static_map_) {
    // Search can take longer than a live cloud's freshness window. Holding
    // the live read lock would block the worker renewing that same window.
    snapshot = map_->planningSnapshotAfterExpiry(node->now().nanoseconds(),
      static_cast<std::int64_t>(std::llround(live_obstacle_max_observation_age_ * 1e9)));
    const auto checker = footprint_check_enabled_ ?
      std::make_unique<SweptFootprint>(*snapshot, query_footprint) : nullptr;
    path = computePlan(start, goal, *snapshot, checker.get(), checked_start);
  } else {
    const auto read = map_->lockForReading();
    const auto checker = footprint_check_enabled_ ?
      std::make_unique<SweptFootprint>(*map_, query_footprint) : nullptr;
    path = computePlan(start, goal, *map_, checker.get(), checked_start);
  }
  const auto computed_path = path;

  // A request's start was sampled BEFORE search. While a replacement route
  // is being computed the robot can still be following the previous route.
  // Refresh geometry as well as the timestamp: Shim samples from pose zero,
  // so a newly stamped old prefix can otherwise make it turn toward the rear.
  auto final_start = start;
  if (advance_robot_start) {
    try {final_start = robot_pose();}
    catch (const tf2::TransformException & error) {
      clearPathCache();
      throw nav2_core::PlannerException(
              std::string("Cannot refresh robot start after search: ") + error.what());
    }
  }
  query_footprint.near_start_center = std::array<double, 2>{
    final_start.pose.position.x, final_start.pose.position.y};

  // A newly arrived obstacle must still veto a snapshot result. This final
  // validation is short and uses the live map under a read lock, without
  // rerunning search or extending any health timeout.
  const auto read = use_static_map_ ?
    map_->lockForReadingAfterExpiry(node->now().nanoseconds(),
      static_cast<std::int64_t>(std::llround(live_obstacle_max_observation_age_ * 1e9))) :
    map_->lockForReading();
  const auto health = live_obstacle_health_.statusNow([&node]() {return node->now().nanoseconds();});
  std::string input_failure;
  if (fixed_map_guard_ && !fixed_map_guard_->ready()) {
    input_failure = "fixed_map_" + fixed_map_guard_->reason();
  } else if (use_static_map_ && !active_.load()) {input_failure = "planner_inactive";}
  else if (!localizationReady()) {input_failure = "localization_not_ready";}
  else if (snapshot && !map_->planningStateMatchesUnlocked(*snapshot)) {input_failure = "map_generation_changed";}
  else if (use_static_map_ && !health.ready) {input_failure = "live_obstacle_" + std::string(health.reason);}
  if (!input_failure.empty()) {
    clearPathCache();
    throw nav2_core::PlannerException("Planning inputs unavailable after search: " + input_failure);
  }
  Astar3D current(*map_, astar_config_);
  const auto current_checker = footprint_check_enabled_ ?
    std::make_unique<SweptFootprint>(*map_, query_footprint) : nullptr;
  Astar3D::TerrainValidation failure;
  SweptFootprint::Failure footprint_failure;
  const auto validate = [&](nav_msgs::msg::Path & candidate, const VoxelKey & actual_start) {
      failure = Astar3D::TerrainValidation{};
      footprint_failure = SweptFootprint::Failure{};
      return refreshAndValidateTerrainPath(current, actual_start, candidate, &failure) &&
        validateFootprintPath(candidate, final_start.pose.orientation, goal.pose.orientation,
          &footprint_failure, false, current_checker.get());
    };
  const double start_shift = std::hypot(final_start.pose.position.x-start.pose.position.x,
      final_start.pose.position.y-start.pose.position.y);
  if (advance_robot_start && start_shift > 1e-3) {
    nav_msgs::msg::Path refreshed;
    VoxelKey refreshed_start;
    const auto reconnect = [&](nav_msgs::msg::Path & candidate) {
        VoxelKey actual_start;
        if (!current.projectStartToSurface(final_start.pose.position.x, final_start.pose.position.y,
            candidate.poses.front().pose.position.z, astar_config_.start_projection_xy_radius,
            astar_config_.projection_z_radius, goal.pose.position.x, goal.pose.position.y, actual_start))
        {return false;}
        std::vector<OccupyMap::Point> points;
        for (const auto & pose : candidate.poses) {
          const auto & p = pose.pose.position; points.push_back({p.x, p.y, p.z});
        }
        const auto orientation = candidate.poses.back().pose.orientation;
        populateGroundPath(candidate, current.densifyPath(points, path_pose_spacing_), orientation);
        if (!validate(candidate, actual_start)) {return false;}
        refreshed_start = actual_start;
        return true;
      };
    if (!reanchorPublishedPath(path, final_start, path_reuse_start_tolerance_,
        footprint_check_enabled_ ? .6 : 0.0, reconnect, refreshed))
    {
      clearPathCache();
      throw nav2_core::PlannerException(
              "Cannot safely reconnect planned route to the robot's current position");
    }
    path = std::move(refreshed);
    checked_start = refreshed_start;
    RCLCPP_INFO(node->get_logger(),
      "%s refreshed published path start after robot moved %.3f m during search",
      name_.c_str(), start_shift);
  } else if (!validate(path, checked_start)) {
    clearPathCache();
    if (std::string(failure.reason) != "valid") {
      publishDirectDiagnostic(failure);
      throw nav2_core::PlannerException(std::string("Latest map rejected planned terrain: ") + failure.reason);
    }
    reportFootprintFailure(path, footprint_failure);
    throw nav2_core::PlannerException("Latest map rejected planned rectangular footprint");
  }
  path.header.stamp = node->now();
  for (auto & pose : path.poses) {pose.header = path.header;}
  {
    std::lock_guard<std::mutex> lock(path_cache_mutex_);
    // Retain the selected start-layer witness for the actual returned message.
    // Do not resurrect a cache invalidated by a concurrent map update.
    if (cached_path_valid_ && cached_output_path_ == computed_path) {
      cached_output_path_ = path;
      cached_progress_distance_ = 0.0;
      cached_start_key_ = checked_start;
      cached_start_valid_ = use_static_map_;
    }
  }
  return path;
}

nav_msgs::msg::Path Astar3DPlanner::computePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal, const OccupyMap & planning_map,
  const SweptFootprint * checker, VoxelKey & checked_start)
{
  const auto node = node_.lock();
  const auto begin = std::chrono::steady_clock::now();
  clearStartFailure();
  // Queries and their diagnostics must refer to the same request snapshot.
  GroundDiagnosticOnExit ground_diagnostics{[this, &planning_map]() {
      publishGroundQueries(&planning_map);
    }};
  if (!planning_map.readyUnlocked()) {
    Astar3D::ProjectionDiagnostics diagnostics;
    diagnostics.reason = "map_has_no_support";
    publishStartFailure(
      {start.pose.position.x, start.pose.position.y, startHeightInGlobalFrame(start)},
      diagnostics, nullptr, nullptr, nullptr, &planning_map);
    throw nav2_core::PlannerException(
            "Astar3DPlanner has not received a point-cloud map yet");
  }

  Astar3D astar(planning_map, astar_config_);
  const double start_ground_z = startHeightInGlobalFrame(start);
  std::function<bool(const VoxelKey &)> validate_start_connector;
  SweptFootprint::Failure connector_failure;
  bool have_connector_failure = false;
  if (use_static_map_ && footprint_check_enabled_) {
    double start_yaw;
    if (!checker || !planarYaw(start.pose.orientation, start_yaw)) {
      throw nav2_core::PlannerException("Astar3DPlanner invalid offline start orientation");
    }
    validate_start_connector = [&, start_yaw](const VoxelKey & key) {
        VoxelKey ground;
        double height;
        if (!planning_map.findSurfaceGroundUnlocked(key, astar.preferredGroundGapCells(),
            astar.groundGapToleranceCells(), ground) ||
          !planning_map.supportHeightUnlocked(ground, height)) {return false;}
        const OccupyMap::Point actual{start.pose.position.x, start.pose.position.y, height};
        const auto exact_key = planning_map.posToKey(actual[0], actual[1], height);
        if (key.x == exact_key.x && key.y == exact_key.y) {
          VoxelKey collision;
          const bool valid = checker->poseCollisionFree(actual, start_yaw, &collision);
          if (!valid && !have_connector_failure) {
            connector_failure.has_collision = true; connector_failure.collision = collision;
            connector_failure.position = actual; connector_failure.yaw = start_yaw;
            have_connector_failure = true;
          }
          return valid;
        }
        const auto position = planning_map.keyToPos(key);
        const double heading = std::atan2(position[1] - actual[1], position[0] - actual[0]);
        SweptFootprint::Failure failure;
        const bool valid = checker->validatePath(
          {actual, {position[0], position[1], height}}, start_yaw, heading, &failure);
        if (!valid && !have_connector_failure) {
          connector_failure = failure; have_connector_failure = true;
        }
        return valid;
      };
  }
  VoxelKey start_key;
  if (!astar.projectStartToSurface(
      start.pose.position.x, start.pose.position.y, start_ground_z,
      astar_config_.start_projection_xy_radius, astar_config_.projection_z_radius,
      goal.pose.position.x, goal.pose.position.y, start_key, nullptr, validate_start_connector))
  {
    // Re-query the same locked map solely for diagnostic details. Normal
    // successful requests pay no candidate/evidence collection cost.
    Astar3D::ProjectionDiagnostics diagnostics;
    astar.projectStartToSurface(
      start.pose.position.x, start.pose.position.y, start_ground_z,
      astar_config_.start_projection_xy_radius, astar_config_.projection_z_radius,
      goal.pose.position.x, goal.pose.position.y, start_key, &diagnostics, validate_start_connector);
    if (have_connector_failure && !diagnostics.has_collision) {
      diagnostics.has_collision = connector_failure.has_collision;
      diagnostics.collision = connector_failure.collision;
      diagnostics.sample = connector_failure.position;
    }
    publishStartFailure(
      {start.pose.position.x, start.pose.position.y, start_ground_z}, diagnostics, nullptr, nullptr, nullptr, &planning_map);
    throw nav2_core::PlannerException(
            std::string("Astar3DPlanner start projection failed: ") + diagnostics.reason);
  }

  VoxelKey start_support;
  planning_map.findSurfaceGroundUnlocked(
    start_key, astar.preferredGroundGapCells(),
    astar.groundGapToleranceCells(), start_support);
  double detected_start_ground_z;
  if (!planning_map.supportHeightUnlocked(start_support, detected_start_ground_z)) {
    throw nav2_core::PlannerException("Astar3DPlanner start support has no observed height");
  }
  const bool unspecified_goal_z =
    goal_z_zero_is_unspecified_ && std::abs(goal.pose.position.z) < 1e-6;
  const double desired_goal_z =
    unspecified_goal_z ? detected_start_ground_z : goal.pose.position.z;
  const double goal_vertical_radius =
    unspecified_goal_z ? astar_config_.unspecified_goal_projection_z_radius :
    astar_config_.projection_z_radius;

  const auto projected_goal_candidates = astar.surfaceCandidates(
    goal.pose.position.x, goal.pose.position.y, desired_goal_z,
    astar_config_.goal_projection_xy_radius, goal_vertical_radius);

  struct GoalOption
  {
    VoxelKey key;
    double x;
    double y;
    bool preserves_exact_xy;
    OccupyMap::Traversability exact_traversability;
    std::vector<OccupyMap::Point> connector;
  };

  // Build one option for every observed support level near the requested goal.
  // Do not select a level yet: a table top can be locally planar and therefore
  // appear in the support cloud, but it is not reachable from the robot's
  // ground component.  The multi-goal A* below selects only a support option
  // connected to the robot through valid ground transitions.
  std::vector<GoalOption> goal_options;
  std::vector<GoalOption> deferred_goal_options;
  std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> option_indices;
  std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash> option_groups;
  std::vector<VoxelKey> goal_keys;
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied_exact_goals;
  // A centre-clear goal can still put the rectangular front/rear corner into
  // a measured obstacle. Check the requested final orientation before exact-XY
  // deduplication; otherwise all nearby support alternatives collapse onto the
  // same impossible endpoint. Online selection keeps its existing behaviour.
  double terminal_yaw = 0.0;
  const bool filter_terminal_footprint = use_static_map_ && footprint_check_enabled_;
  if (filter_terminal_footprint &&
    (!checker || !planarYaw(goal.pose.orientation, terminal_yaw)))
  {
    throw nav2_core::PlannerException("Astar3DPlanner invalid offline goal orientation");
  }
  std::size_t terminal_footprint_rejections = 0;
  VoxelKey first_terminal_collision;
  OccupyMap::Point first_terminal_sample{};
  bool have_terminal_collision = false;
  const auto terminal_pose_clear = [&](const VoxelKey & key, double x, double y) {
      if (!filter_terminal_footprint) {return true;}
      VoxelKey support, collision;
      double height;
      if (!planning_map.findSurfaceGroundUnlocked(
          key, astar.preferredGroundGapCells(), astar.groundGapToleranceCells(), support) ||
        !planning_map.supportHeightUnlocked(support, height)) {return false;}
      if (checker->poseCollisionFree({x, y, height}, terminal_yaw, &collision)) {
        return true;
      }
      ++terminal_footprint_rejections;
      if (!have_terminal_collision) {
        first_terminal_collision = collision;
        first_terminal_sample = {x, y, height};
        have_terminal_collision = true;
      }
      return false;
    };
  for (const auto & projected_key : projected_goal_candidates) {
    const auto projected_position = planning_map.keyToPos(projected_key);
    const VoxelKey exact_key = planning_map.posToKey(
      goal.pose.position.x, goal.pose.position.y, projected_position[2]);
    VoxelKey exact_ground;
    auto exact_traversability = OccupyMap::Traversability::kNoGroundSupport;
    if (planning_map.findSurfaceGroundUnlocked(
        exact_key, astar.preferredGroundGapCells(),
        astar.groundGapToleranceCells(), exact_ground))
    {
      exact_traversability = planning_map.traversableOnSurfaceUnlocked(
        exact_key, exact_ground, astar.robotHeightCells(),
        astar.preferredGroundGapCells(), astar.groundGapToleranceCells(),
        nullptr, astar_config_.robot_height) ?
        OccupyMap::Traversability::kTraversable :
        OccupyMap::Traversability::kOccupied;
    }

    GoalOption option{
      projected_key, projected_position[0], projected_position[1], false,
      exact_traversability, {}};
    if (exact_traversability == OccupyMap::Traversability::kTraversable &&
      terminal_pose_clear(exact_key, goal.pose.position.x, goal.pose.position.y))
    {
      option.key = exact_key;
      option.x = goal.pose.position.x;
      option.y = goal.pose.position.y;
      option.preserves_exact_xy = true;
    } else if (exact_traversability == OccupyMap::Traversability::kOccupied) {
      occupied_exact_goals.insert(exact_key);
    }

    // An exact terminal pose can fit while its complete approach/turn cannot.
    // Keep distinct projected endpoints for recovery, without making a normal
    // successful exact-goal request terminate early at a nearby cell.
    if (option.preserves_exact_xy && !(projected_key == option.key)) {
      deferred_goal_options.push_back(GoalOption{
        projected_key, projected_position[0], projected_position[1], false, exact_traversability, {}});
    }
    if (!option.preserves_exact_xy && filter_terminal_footprint &&
      std::hypot(option.x - goal.pose.position.x, option.y - goal.pose.position.y) >
      astar_config_.goal_projection_xy_radius + 1e-9) {continue;}
    if (!option.preserves_exact_xy &&
      !terminal_pose_clear(option.key, option.x, option.y)) {continue;}
    const auto inserted = option_indices.emplace(option.key, goal_options.size());
    if (inserted.second) {
      goal_options.push_back(option);
      goal_keys.push_back(option.key);
    } else if (option.preserves_exact_xy) {
      goal_options[inserted.first->second] = option;
    }
  }
  bool subcell_goals_attempted = false;
  const auto activate_subcell_goals = [&]() {
      if (subcell_goals_attempted || !filter_terminal_footprint) {return false;}
      subcell_goals_attempted = true;
      GoalConnectionDiagnostics diagnostics;
      std::vector<GoalPose> terminal_poses;
      const double connection_length = std::min(1.5, std::max(
        astar_config_.surface_connection_radius,
        2.0 * (footprint_config_.max_x - footprint_config_.min_x)));
      auto connections = subcellGoalConnections(planning_map, astar, *checker,
        goal.pose.position.x, goal.pose.position.y, desired_goal_z, terminal_yaw,
        astar_config_.goal_projection_xy_radius, goal_vertical_radius,
        connection_length, &diagnostics, &terminal_poses);
      const auto add_option = [&](GoalOption option) {
          const std::size_t index = goal_options.size();
          auto & group = option_groups[option.key];
          const auto existing = option_indices.find(option.key);
          if (group.empty() && existing != option_indices.end()) {group.push_back(existing->second);}
          const auto key = option.key;
          goal_options.push_back(std::move(option));
          group.push_back(index);
          if (option_indices.emplace(key, index).second) {goal_keys.push_back(key);}
        };
      // A clear terminal pose is independent of its approach. Ordinary search
      // and continuous path refinement may find an approach not represented
      // by the bounded analytic connectors. Neither may skip final checks.
      for (const auto & pose : terminal_poses) {
        add_option(GoalOption{pose.key, pose.point[0], pose.point[1], false,
          OccupyMap::Traversability::kTraversable, {}});
      }
      for (auto & connection : connections) {
        const auto endpoint = connection.points.back();
        add_option(GoalOption{connection.anchor, endpoint[0], endpoint[1], false,
          OccupyMap::Traversability::kTraversable, std::move(connection.points)});
      }
      RCLCPP_INFO(node->get_logger(),
        "%s subcell terminals: poses=%zu clear=%zu connector_checks=%zu accepted=%zu limited=%s",
        name_.c_str(), diagnostics.poses_checked, diagnostics.poses_clear,
        diagnostics.connectors_checked, connections.size(), diagnostics.limit_reached ? "true" : "false");
      return !terminal_poses.empty();
    };
  if (goal_options.empty()) {activate_subcell_goals();}
  if (goal_options.empty()) {
    if (projected_goal_candidates.empty()) {
      Astar3D::ProjectionDiagnostics diagnostic;
      diagnostic.stage = "goal_projection";
      astar.surfaceCandidates(goal.pose.position.x, goal.pose.position.y, desired_goal_z,
        astar_config_.goal_projection_xy_radius, goal_vertical_radius, &diagnostic.surface);
      diagnostic.reason = diagnostic.surface.layers == 0 ? "goal_no_support_xy" :
        diagnostic.surface.occupied > 0 ? "goal_body_clearance_blocked" :
        "goal_support_outside_height_window";
      diagnostic.has_collision = diagnostic.surface.has_collision;
      diagnostic.collision = diagnostic.surface.collision;
      diagnostic.sample = {goal.pose.position.x, goal.pose.position.y, desired_goal_z};
      publishStartFailure(
        {start.pose.position.x, start.pose.position.y, start_ground_z}, diagnostic, nullptr, nullptr, nullptr, &planning_map);
      throw nav2_core::PlannerException(
              "Astar3DPlanner found no safe ground surface within the goal tolerance");
    }
    if (have_terminal_collision) {
      Astar3D::ProjectionDiagnostics diagnostics;
      diagnostics.stage = "goal_footprint";
      diagnostics.reason = "goal_orientation_footprint_collision";
      diagnostics.has_collision = true;
      diagnostics.collision = first_terminal_collision;
      diagnostics.sample = first_terminal_sample;
      publishStartFailure(
        {start.pose.position.x, start.pose.position.y, start_ground_z}, diagnostics, nullptr, nullptr, nullptr, &planning_map);
      throw nav2_core::PlannerException(
              "Astar3DPlanner found no rectangular-footprint-safe goal within the goal tolerance");
    }
    throw nav2_core::PlannerException(
            "Astar3DPlanner found no traversable goal option");
  }
  if (terminal_footprint_rejections > 0) {
    RCLCPP_DEBUG(
      node->get_logger(), "%s retained %zu offline goal options after %zu terminal footprint rejects",
      name_.c_str(), goal_options.size(), terminal_footprint_rejections);
  }

  std::vector<VoxelKey> voxel_path;
  nav_msgs::msg::Path path;
  double path_cost = 0.0;
  // A near-field blind zone can put the best start projection to one side.
  // Try the actual start-to-goal line before committing search or cache reuse
  // to that side cell. The checker still requires real goal support, observes
  // every later ground column and checks all body-clearance/slope/step gates.
  std::vector<OccupyMap::Point> direct_points;
  const GoalOption * direct_option = nullptr;
  double direct_distance = std::numeric_limits<double>::infinity();
  Astar3D::TerrainValidation first_direct_failure;
  bool have_direct_failure = false;
  std::set<std::tuple<double, double, double>> direct_endpoints;
  for (const auto & option : goal_options) {
    const double distance = std::hypot(
      option.x - start.pose.position.x, option.y - start.pose.position.y);
    // Geometry for a dominated endpoint cannot alter the selection below.
    // In particular, a valid exact-XY path outranks every tolerated endpoint.
    // Keep ties in their existing order, including separate support layers.
    if (direct_option != nullptr &&
      ((!option.preserves_exact_xy && direct_option->preserves_exact_xy) ||
      (option.preserves_exact_xy == direct_option->preserves_exact_xy &&
      distance >= direct_distance - 1e-9))) {continue;}
    VoxelKey support;
    double height;
    if (!planning_map.findSurfaceGroundUnlocked(
        option.key, astar.preferredGroundGapCells(), astar.groundGapToleranceCells(),
        support) || !planning_map.supportHeightUnlocked(support, height))
    {
      continue;
    }
    if (!option.connector.empty()) {height = option.connector.back()[2];}
    if (!direct_endpoints.emplace(option.x, option.y, height).second) {continue;}
    std::vector<OccupyMap::Point> candidate;
    Astar3D::TerrainValidation validation;
    if (!astar.tryDirectGroundPath(
        {start.pose.position.x, start.pose.position.y, start_ground_z},
        {option.x, option.y, height}, astar_config_.start_projection_xy_radius,
        candidate, &validation))
    {
      if (!have_direct_failure) {first_direct_failure = validation; have_direct_failure = true;}
      RCLCPP_DEBUG(
        node->get_logger(), "Direct ground candidate rejected: %s near (%.3f, %.3f, %.3f)",
        validation.reason,
        validation.position[0], validation.position[1], validation.position[2]);
      continue;
    }
    nav_msgs::msg::Path footprint_candidate;
    populateGroundPath(footprint_candidate, candidate, goal.pose.orientation);
    SweptFootprint::Failure footprint_failure;
    if (!validateFootprintPath(footprint_candidate, start.pose.orientation,
        goal.pose.orientation, &footprint_failure, false, checker, &planning_map))
    {
      if (!have_direct_failure) {
        first_direct_failure.reason = footprint_failure.reason;
        first_direct_failure.position = footprint_failure.position;
        first_direct_failure.has_collision = footprint_failure.has_collision;
        first_direct_failure.collision = footprint_failure.collision;
        have_direct_failure = true;
      }
      continue;
    }
    if (direct_option == nullptr ||
      (option.preserves_exact_xy && !direct_option->preserves_exact_xy) ||
      (option.preserves_exact_xy == direct_option->preserves_exact_xy &&
      distance < direct_distance - 1e-9))
    {
      direct_option = &option;
      direct_distance = distance;
      direct_points = std::move(candidate);
    }
  }
  if (direct_option != nullptr) {
    auto dense = astar.densifyPath(direct_points, path_pose_spacing_);
    Astar3D::TerrainValidation validation;
    const bool valid_terrain = astar.refreshAndValidateTerrainPath(start_key, dense, &validation);
    populateGroundPath(path, dense, goal.pose.orientation);
    if (valid_terrain && validateFootprintPath(
        path, start.pose.orientation, goal.pose.orientation, nullptr, false, checker, &planning_map))
    {
      path.header.frame_id = global_frame_;
      path.header.stamp = node->now();
      populateGroundPath(path, dense, goal.pose.orientation);
      cachePath(
        direct_option->key, goal.pose.position.x, goal.pose.position.y, path,
        use_static_map_ ? &start_key : nullptr);
      if (!direct_option->preserves_exact_xy) {
        RCLCPP_WARN(
          node->get_logger(),
          "%s moved requested goal (%.2f, %.2f) to directly reachable ground (%.2f, %.2f), "
          "offset %.2f m",
          name_.c_str(), goal.pose.position.x, goal.pose.position.y,
          direct_option->x, direct_option->y,
          std::hypot(
            direct_option->x - goal.pose.position.x, direct_option->y - goal.pose.position.y));
      }
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();
      RCLCPP_INFO(
        node->get_logger(),
        "%s found %zu-point direct ground path, xy_length=%.3f straight=%.3f, "
        "initial_blind_limit=%.2f m, using %zu occupied voxels in %.3f s",
        name_.c_str(), path.poses.size(), direct_distance, direct_distance,
        astar_config_.start_projection_xy_radius, planning_map.rawVoxelCountUnlocked(), elapsed);
      checked_start = start_key;
      return path;
    }
    if (!valid_terrain && !have_direct_failure) {
      first_direct_failure = validation; have_direct_failure = true;
    }
  }
  if (have_direct_failure) {publishDirectDiagnostic(first_direct_failure, &planning_map);}
  // A previously valid exact goal can be covered by one noisy dynamic obstacle
  // update.  Prefer its recently validated cached route before moving the goal.
  for (const auto & exact_goal_key : occupied_exact_goals) {
    double goal_valid_age = 0.0;
    if (tryReuseCachedPath(
        astar, start_key, exact_goal_key,
        start.pose.position.x, start.pose.position.y,
        goal.pose.position.x, goal.pose.position.y,
        start.pose.orientation, goal.pose.orientation,
        true, &goal_valid_age, path, checker, &planning_map))
    {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();
      RCLCPP_WARN(
        node->get_logger(),
        "%s kept a safe cached path through a transient goal-cell obstacle "
        "(last valid %.2f s ago, ignoring only the final %.2f m); "
        "the remaining path is still collision checked",
        name_.c_str(), goal_valid_age, cached_goal_obstacle_ignore_radius_);
      RCLCPP_INFO(
        node->get_logger(),
        "%s reused stable %zu-point ground path, using %zu occupied voxels in %.3f s",
        name_.c_str(), path.poses.size(), planning_map.rawVoxelCountUnlocked(), elapsed);
      checked_start = start_key;
      return path;
    }
  }

  for (const auto & option : goal_options) {
    if (!tryReuseCachedPath(
        astar, start_key, option.key,
        start.pose.position.x, start.pose.position.y,
        goal.pose.position.x, goal.pose.position.y,
        start.pose.orientation, goal.pose.orientation,
        false, nullptr, path, checker, &planning_map))
    {
      continue;
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - begin).count();
    RCLCPP_INFO(
      node->get_logger(),
      "%s reused stable %zu-point ground path, using %zu occupied voxels in %.3f s",
      name_.c_str(), path.poses.size(), planning_map.rawVoxelCountUnlocked(), elapsed);
    checked_start = start_key;
    return path;
  }

  VoxelKey goal_key;
  Astar3D::SearchDiagnostics search_diagnostics;
  Astar3D::MotionConstraints motion;
  const bool constrain_motion = heading_aware_search_ && footprint_check_enabled_;
  if (constrain_motion) {
    double start_yaw, goal_yaw;
    if (!checker || !planarYaw(start.pose.orientation, start_yaw) ||
      !planarYaw(goal.pose.orientation, goal_yaw))
    {throw nav2_core::PlannerException("Heading-aware search requires valid orientations");}
    const auto projected = planning_map.keyToPos(start_key);
    const double dx = projected[0] - start.pose.position.x;
    const double dy = projected[1] - start.pose.position.y;
    motion.start_yaw = std::hypot(dx, dy) > 1e-3 ? std::atan2(dy, dx) : start_yaw;
    motion.include_heading_in_state = footprint_config_.check_in_place_turn_sweeps;
    motion.turn_cost_weight = motion.include_heading_in_state ? turn_cost_weight_ : 0.0;
    // Within this immutable planning view, an edge's translation is identical
    // for all arrivals. Reuse that expensive sweep, but still check each
    // arrival heading's distinct turn. Never retain the cache across updates.
    auto translation_cache = std::make_shared<
      std::map<std::pair<OccupyMap::Point, OccupyMap::Point>, bool>>();
    motion.transition = [checker, translation_cache, check_turns=motion.include_heading_in_state](
        const OccupyMap::Point & from, const OccupyMap::Point & to,
        double incoming_yaw, double outgoing_yaw) {
        const auto inserted = translation_cache->try_emplace(std::make_pair(from, to), false);
        if (inserted.second) {
          inserted.first->second = checker->validatePath(
            {from, to}, outgoing_yaw, outgoing_yaw);
        }
        return inserted.first->second && (!check_turns ||
               checker->validatePath({from}, incoming_yaw, outgoing_yaw));
      };
    motion.terminal = [&, goal_yaw](const VoxelKey & key, double incoming_yaw) {
        const auto found = option_indices.find(key);
        if (found == option_indices.end()) {return false;}
        std::vector<std::size_t> indices{found->second};
        const auto group = option_groups.find(key);
        if (group != option_groups.end()) {
          for (auto index : group->second) {if (index != found->second) {indices.push_back(index);}}
        }
        for (auto index : indices) {
          const auto & option = goal_options[index];
          if (!option.connector.empty()) {
            // This connection was checked under the configured footprint
            // policy in the same immutable map view. Strict mode also checks
            // the arrival turn; guidance mode checks the relevant poses.
            // Final validation uses the same policy on the newest live map.
            const auto & from = option.connector.front();
            const auto & to = option.connector.back();
            const double approach = std::atan2(to[1]-from[1], to[0]-from[0]);
            if (checker->validatePath({from}, incoming_yaw, approach)) {
              option_indices[key] = index;
              return true;
            }
            continue;
          }
          auto connector = option.connector;
          if (connector.empty()) {
            VoxelKey ground; double height;
            if (!planning_map.findSurfaceGroundUnlocked(key, astar.preferredGroundGapCells(),
                astar.groundGapToleranceCells(), ground) ||
              !planning_map.supportHeightUnlocked(ground, height)) {continue;}
            auto from = planning_map.keyToPos(key); from[2] = height;
            if (!astar.tryDirectGroundPath(from, {option.x, option.y, height}, 0.0, connector)) {continue;}
          }
          if (checker->validatePath(connector, incoming_yaw, goal_yaw)) {
            option_indices[key] = index;
            return true;
          }
        }
        return false;
      };
  }
  const auto configure_motion_start = [&](Astar3D::MotionConstraints & constraints,
      const VoxelKey & key) {
      const auto projected = planning_map.keyToPos(key);
      const auto actual = planning_map.posToKey(start.pose.position.x, start.pose.position.y, 0);
      constraints.use_exact_start = key.x == actual.x && key.y == actual.y;
      if (constraints.use_exact_start) {
        planarYaw(start.pose.orientation, constraints.start_yaw);
        constraints.exact_start = {start.pose.position.x, start.pose.position.y,
          detected_start_ground_z};
      } else {
        constraints.start_yaw = std::atan2(projected[1]-start.pose.position.y,
          projected[0]-start.pose.position.x);
      }
    };
  if (constrain_motion) {configure_motion_start(motion, start_key);}

  // Share one bounded pool across disconnected-search and footprint retries.
  // A locally clear first projection need not belong to the goal's component.
  std::optional<std::chrono::steady_clock::time_point> start_retry_begin;
  std::vector<VoxelKey> alternative_starts;
  std::unordered_set<VoxelKey, VoxelKeyHash> attempted_starts{start_key};
  std::unordered_set<VoxelKey, VoxelKeyHash> disconnected_starts;
  std::size_t start_retry_attempts = 0;
  bool alternatives_initialized = false;
  const auto start_retry_remaining = [&]() {
      return start_retry_begin ? .25 - std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *start_retry_begin).count() : .25;
    };
  const auto remember_disconnected = [&](const Astar3D::SearchDiagnostics & diagnostic) {
      disconnected_starts.insert(
        diagnostic.exhausted_component.begin(), diagnostic.exhausted_component.end());
    };
  const auto try_alternate_starts = [&](auto && try_candidate) {
      if (!start_retry_begin) {start_retry_begin = std::chrono::steady_clock::now();}
      if (!alternatives_initialized) {
        alternatives_initialized = true;
        alternative_starts = astar.surfaceCandidates(
          start.pose.position.x, start.pose.position.y, detected_start_ground_z,
          astar_config_.start_projection_xy_radius, astar_config_.projection_z_radius);
        double yaw = 0.0;
        planarYaw(start.pose.orientation, yaw);
        const auto score = [&](const VoxelKey & key) {
            const auto p = planning_map.keyToPos(key);
            const double dx = p[0] - start.pose.position.x, dy = p[1] - start.pose.position.y;
            const double length = std::hypot(dx, dy);
            const double forward = length > 1e-9 ?
              (dx * std::cos(yaw) + dy * std::sin(yaw)) / length : 1.0;
            return length * (2.0 - forward);
          };
        std::stable_sort(alternative_starts.begin(), alternative_starts.end(),
          [&](const VoxelKey & a, const VoxelKey & b) {return score(a) < score(b);});
      }
      for (const auto & alternative : alternative_starts) {
        if (attempted_starts.count(alternative) || disconnected_starts.count(alternative)) {continue;}
        if (start_retry_attempts >= 8 || start_retry_remaining() <= 0.0) {break;}
        attempted_starts.insert(alternative);
        ++start_retry_attempts;
        VoxelKey ground;
        double height;
        const auto p = planning_map.keyToPos(alternative);
        if (!planning_map.findSurfaceGroundUnlocked(alternative, astar.preferredGroundGapCells(),
            astar.groundGapToleranceCells(), ground) ||
          !planning_map.supportHeightUnlocked(ground, height) ||
          std::abs(height - detected_start_ground_z) > astar_config_.max_step_height + 1e-9 ||
          (validate_start_connector && !validate_start_connector(alternative))) {continue;}
        std::vector<OccupyMap::Point> connector{
          {start.pose.position.x, start.pose.position.y, detected_start_ground_z},
          {p[0], p[1], height}};
        connector = astar.densifyPath(connector, path_pose_spacing_);
        // Only the existing selected-start blind-zone policy may bridge missing
        // data. Supported starts cannot jump gaps; all obstacles stay active.
        if (!astar.refreshAndValidateTerrainPath(alternative, connector, nullptr, true)) {continue;}
        if (footprint_check_enabled_) {
          double yaw;
          if (!checker || !planarYaw(start.pose.orientation, yaw) ||
            !checker->validatePath(connector, yaw,
              std::atan2(p[1] - start.pose.position.y, p[0] - start.pose.position.x))) {continue;}
        }
        const double seconds = start_retry_remaining();
        if (seconds <= 0.0) {break;}
        auto retry_config = astar_config_;
        retry_config.max_search_nodes = std::min<std::size_t>(retry_config.max_search_nodes, 50000);
        retry_config.maximum_search_seconds = seconds;
        Astar3D retry_astar(planning_map, retry_config);
        if (try_candidate(retry_astar, alternative, connector)) {return true;}
      }
      return false;
    };
  const auto activate_deferred_goals = [&]() {
      bool expanded = false;
      for (const auto & option : deferred_goal_options) {
        if (option_indices.count(option.key) ||
          std::hypot(option.x - goal.pose.position.x, option.y - goal.pose.position.y) >
          astar_config_.goal_projection_xy_radius + 1e-9 ||
          !terminal_pose_clear(option.key, option.x, option.y)) {continue;}
        option_indices.emplace(option.key, goal_options.size());
        goal_options.push_back(option);
        goal_keys.push_back(option.key);
        expanded = true;
      }
      deferred_goal_options.clear();
      if (expanded) {
        // Reachability proved against the old, smaller goal set cannot prune
        // retries against this expanded set. Keep the common time/count budget.
        disconnected_starts.clear();
        attempted_starts.clear(); attempted_starts.insert(start_key);
        RCLCPP_INFO(node->get_logger(), "%s retained %zu tolerated endpoints after exact-goal rejection",
          name_.c_str(), goal_keys.size());
      }
      return expanded;
    };
  // Most routes need no heading lattice. Keep the fast terrain search and
  // only invoke the larger maneuver search if its complete footprint fails.
  bool used_heading_search = false;
  const auto search_alternative = [&](const Astar3D & retry_astar,
      const VoxelKey & alternative, const std::vector<OccupyMap::Point> &) {
      std::vector<VoxelKey> route;
      VoxelKey reached;
      Astar3D::SearchDiagnostics diagnostic;
      double cost = 0.0;
      if (!retry_astar.searchToAny(alternative, goal_keys, route, reached, &cost, &diagnostic)) {
        remember_disconnected(diagnostic);
        return false;
      }
      start_key = alternative; goal_key = reached; voxel_path = std::move(route);
      path_cost = cost; search_diagnostics = diagnostic;
      if (constrain_motion) {configure_motion_start(motion, start_key);}
      RCLCPP_INFO(node->get_logger(),
        "%s recovered disconnected preferred start after %zu alternate attempts; goals=%zu",
        name_.c_str(), start_retry_attempts, goal_keys.size());
      return true;
    };
  bool found_route = astar.searchToAny(
    start_key, goal_keys, voxel_path, goal_key, &path_cost, &search_diagnostics);
  if (!found_route) {
    remember_disconnected(search_diagnostics);
    // Keep the original endpoint preference while trying safe start bridges.
    // Expanding the goal disk first can stop short even when exact XY is reachable.
    found_route = try_alternate_starts(search_alternative);
  }
  if (!found_route && activate_deferred_goals()) {
    if (!start_retry_begin) {start_retry_begin = std::chrono::steady_clock::now();}
    const double seconds = start_retry_remaining();
    if (seconds > 0.0) {
      auto config = astar_config_;
      config.max_search_nodes = std::min<std::size_t>(config.max_search_nodes, 50000);
      config.maximum_search_seconds = seconds;
      found_route = Astar3D(planning_map, config).searchToAny(
        start_key, goal_keys, voxel_path, goal_key, &path_cost, &search_diagnostics);
      if (!found_route) {remember_disconnected(search_diagnostics);}
    }
    if (!found_route) {found_route = try_alternate_starts(search_alternative);}
  }
  if (!found_route && activate_subcell_goals()) {
    auto config = astar_config_;
    config.max_search_nodes = std::min<std::size_t>(config.max_search_nodes, 50000);
    config.maximum_search_seconds = .35;
    found_route = Astar3D(planning_map, config).searchToAny(
      start_key, goal_keys, voxel_path, goal_key, &path_cost, &search_diagnostics);
  }
  if (!found_route)
  {
    const auto start_position = planning_map.keyToPos(start_key);
    Astar3D::ProjectionDiagnostics diagnostics;
    VoxelKey ignored;
    astar.projectStartToSurface(
      start.pose.position.x, start.pose.position.y, start_ground_z,
      astar_config_.start_projection_xy_radius, astar_config_.projection_z_radius,
      goal.pose.position.x, goal.pose.position.y, ignored, &diagnostics, validate_start_connector);
    diagnostics.reason = start_retry_attempts >= 8 ? "alternate_start_attempt_limit" :
      (start_retry_remaining() <= 0.0 ? "alternate_start_time_limit" : search_diagnostics.reason);
    diagnostics.stage = "search";
    diagnostics.has_collision = have_direct_failure && first_direct_failure.has_collision;
    diagnostics.collision = first_direct_failure.collision;
    diagnostics.sample = first_direct_failure.position;
    publishStartFailure(
      {start.pose.position.x, start.pose.position.y, start_ground_z}, diagnostics,
      have_direct_failure ? &first_direct_failure : nullptr, &search_diagnostics, &start_position, &planning_map);
    const auto first_goal_position = planning_map.keyToPos(goal_keys.front());
    RCLCPP_ERROR(
      node->get_logger(),
      "%s unsuccessful search: start_key=(%d,%d,%d) pos=(%.2f,%.2f,%.2f), "
      "goal_options=%zu first_key=(%d,%d,%d) pos=(%.2f,%.2f,%.2f)",
      name_.c_str(), start_key.x, start_key.y, start_key.z,
      start_position[0], start_position[1], start_position[2],
      goal_keys.size(), goal_keys.front().x, goal_keys.front().y,
      goal_keys.front().z, first_goal_position[0], first_goal_position[1],
      first_goal_position[2]);
    clearPathCache();
    throw nav2_core::PlannerException(
            std::string("Astar3DPlanner found no ground-connected route: ") + diagnostics.reason);
  }
  const auto selected_option = option_indices.find(goal_key);
  if (selected_option == option_indices.end()) {
    clearPathCache();
    throw nav2_core::PlannerException(
            "Astar3DPlanner reached an unknown goal option");
  }
  const auto initial_goal_option = goal_options[selected_option->second];
  const double selected_goal_x = initial_goal_option.x;
  const double selected_goal_y = initial_goal_option.y;
  // Retain the actual searched route for one bounded, fully checked fallback.
  // A centreline shortcut can fail the later rectangular angular sweep even
  // when the original route went around that obstacle.
  const bool refine_maneuver = path_refinement_enabled_ && footprint_check_enabled_;
  const auto searched_voxel_path = use_static_map_ || refine_maneuver ?
    voxel_path : std::vector<VoxelKey>{};
  voxel_path = astar.simplifyPath(voxel_path);

  path.header.frame_id = global_frame_;
  path.header.stamp = node->now();
  std::vector<OccupyMap::Point> output_points;
  output_points.reserve(voxel_path.size() + 2);
  const double start_path_z = detected_start_ground_z;
  output_points.push_back(
    OccupyMap::Point{
      start.pose.position.x, start.pose.position.y, start_path_z});

  // Keep the searched start centre until continuous endpoint simplification.
  // Removing it before checking the exact-start connection can cut through a
  // different column than the edge actually validated during A* search.
  const auto start_key_position = planning_map.keyToPos(start_key);
  for (const auto & voxel : voxel_path) {
    const auto position = planning_map.keyToPos(voxel);
    VoxelKey support;
    planning_map.findSurfaceGroundUnlocked(
      voxel, astar.preferredGroundGapCells(),
      astar.groundGapToleranceCells(), support);
    double ground_height;
    if (!planning_map.supportHeightUnlocked(support, ground_height)) {
      throw nav2_core::PlannerException("Astar3DPlanner path support has no observed height");
    }
    const OccupyMap::Point point{position[0], position[1], ground_height};
    const auto & previous = output_points.back();
    if (std::hypot(point[0] - previous[0], point[1] - previous[1]) > 1e-3 ||
      std::abs(point[2] - previous[2]) > 1e-3)
    {
      output_points.push_back(point);
    }
  }

  VoxelKey goal_support;
  planning_map.findSurfaceGroundUnlocked(
    goal_key, astar.preferredGroundGapCells(),
    astar.groundGapToleranceCells(), goal_support);
  double goal_path_z;
  if (!planning_map.supportHeightUnlocked(goal_support, goal_path_z)) {
    throw nav2_core::PlannerException("Astar3DPlanner goal support has no observed height");
  }
  if (!initial_goal_option.connector.empty()) {
    goal_path_z = initial_goal_option.connector.back()[2];
    output_points.insert(output_points.end(), initial_goal_option.connector.begin()+1,
      initial_goal_option.connector.end());
  }
  OccupyMap::Point selected_goal{
    selected_goal_x, selected_goal_y, goal_path_z};
  const auto & previous = output_points.back();
  if (std::hypot(
      selected_goal[0] - previous[0],
      selected_goal[1] - previous[1]) > 1e-3 ||
    std::abs(selected_goal[2] - previous[2]) > 1e-3)
  {
    output_points.push_back(selected_goal);
  } else {
    output_points.back() = selected_goal;
  }
  std::size_t original_corners = output_points.size();
  const auto actual_start_cell = planning_map.posToKey(
    start.pose.position.x, start.pose.position.y, start_path_z);
  const bool selected_start_is_same_column =
    actual_start_cell.x == start_key.x && actual_start_cell.y == start_key.y;
  const double resolution = planning_map.keyToPos(VoxelKey{1, 0, 0})[0] -
    planning_map.keyToPos(VoxelKey{0, 0, 0})[0];
  const double initial_blind_length = std::min(
    astar_config_.start_projection_xy_radius,
    std::hypot(
      start_key_position[0] - start.pose.position.x,
      start_key_position[1] - start.pose.position.y) + 2.0 * resolution);
  const bool retain_connector = use_static_map_ && astar_config_.retain_start_projection_bridge &&
    !selected_start_is_same_column;
  PathRefiner::Diagnostics refinement;
  bool used_search_seed=false;
  PathRefiner::Config refinement_config;
  refinement_config.turn_weight=turn_cost_weight_;
  refinement_config.clearance_weight=astar_config_.obstacle_cost_weight;
  refinement_config.grid_resolution=resolution;
  const auto clearance_cost=[&](const OccupyMap::Point & point) {
      return astar.groundObstacleCost(point);
    };
  if (use_static_map_ && astar_config_.retain_start_projection_bridge &&
    !selected_start_is_same_column &&
    output_points.size() > 2 &&
    std::hypot(output_points[1][0] - start_key_position[0],
      output_points[1][1] - start_key_position[1]) < 1e-6)
  {
    // A supported exact start does not need a detour through its voxel centre.
    // Such centimetre-scale detours introduce artificial headings and turns.
    // Retain exactly the start connector selected and collision-checked during
    // projection. Simplifying across its endpoint loses the bounded bridge
    // witness needed by final continuous terrain validation.
    std::vector<OccupyMap::Point> tail(output_points.begin() + 1, output_points.end());
    tail = astar.simplifyGroundPath(tail, 0.0);
    output_points.resize(1);
    output_points.insert(output_points.end(), tail.begin(), tail.end());
  } else {
    output_points = astar.simplifyGroundPath(output_points, initial_blind_length);
  }
  if (refine_maneuver) {
    double start_yaw, goal_yaw;
    if (!planarYaw(start.pose.orientation, start_yaw) ||
      !planarYaw(goal.pose.orientation, goal_yaw))
    {throw nav2_core::PlannerException("Path refinement requires valid orientations");}
    const auto validate=[&](std::vector<OccupyMap::Point> & candidate) {
        return astar.refreshAndValidateTerrainPath(start_key, candidate, nullptr, true) &&
               checker->validatePath(candidate, start_yaw, goal_yaw);
      };
    PathRefiner refiner(refinement_config);
    // Legacy centreline simplification may already have discarded the search
    // route's clearance. Keep its fully checked unsimplified witness as an
    // alternative seed, instead of optimizing only the already cut corner.
    auto legacy=output_points;
    const bool legacy_valid=validate(legacy);
    std::vector<OccupyMap::Point> searched{
      {start.pose.position.x,start.pose.position.y,start_path_z}};
    bool have_heights=true;
    for(const auto & key:searched_voxel_path) {
      VoxelKey ground;double height;
      if(!planning_map.findSurfaceGroundUnlocked(key,astar.preferredGroundGapCells(),
          astar.groundGapToleranceCells(),ground) ||
        !planning_map.supportHeightUnlocked(ground,height)) {have_heights=false;break;}
      auto p=planning_map.keyToPos(key);p[2]=height;
      if(std::hypot(p[0]-searched.back()[0],p[1]-searched.back()[1])>1e-3 ||
        std::abs(p[2]-searched.back()[2])>1e-3) {searched.push_back(p);}
    }
    if(have_heights) {
      if(std::hypot(searched.back()[0]-selected_goal[0],
          searched.back()[1]-selected_goal[1])>1e-3) {searched.push_back(selected_goal);}
      else {searched.back()=selected_goal;}
      if(validate(searched) && (!legacy_valid ||
        refiner.score(searched,start_yaw,goal_yaw,clearance_cost)<
        refiner.score(legacy,start_yaw,goal_yaw,clearance_cost)-1e-9))
      {output_points=std::move(searched);used_search_seed=true;}
    }
    output_points = refiner.refine(output_points,start_yaw,goal_yaw,
      retain_connector,validate,&refinement,clearance_cost);
  }
  std::size_t retained_corners = output_points.size();
  double horizontal_length = 0.0;
  for (std::size_t i = 1; i < output_points.size(); ++i) {
    horizontal_length += std::hypot(
      output_points[i][0] - output_points[i - 1][0],
      output_points[i][1] - output_points[i - 1][1]);
  }
  double straight_distance = std::hypot(
    output_points.back()[0] - output_points.front()[0],
    output_points.back()[1] - output_points.front()[1]);
  output_points = astar.densifyPath(output_points, path_pose_spacing_);

  Astar3D::TerrainValidation validation;
  if (!astar.refreshAndValidateTerrainPath(start_key, output_points, &validation)) {
    RCLCPP_WARN(
      node->get_logger(), "Final ground path rejected: %s at pose=%zu near (%.3f, %.3f, %.3f)",
      validation.reason, validation.index,
      validation.position[0], validation.position[1], validation.position[2]);
    clearPathCache();
    throw nav2_core::PlannerException("Astar3DPlanner final route failed continuous terrain validation");
  }

  populateGroundPath(path, output_points, goal.pose.orientation);
  SweptFootprint::Failure shortcut_footprint_failure;
  if (!validateFootprintPath(path, start.pose.orientation, goal.pose.orientation,
      &shortcut_footprint_failure, false, checker, &planning_map))
  {
    activate_deferred_goals();
    bool recovered_searched_route = false;
    if (use_static_map_ && !searched_voxel_path.empty()) {
      std::vector<OccupyMap::Point> searched_points;
      searched_points.reserve(searched_voxel_path.size() + 2);
      searched_points.push_back(
        {start.pose.position.x, start.pose.position.y, start_path_z});
      bool have_heights = true;
      for (const auto & key : searched_voxel_path) {
        const auto position = planning_map.keyToPos(key);
        VoxelKey support;
        double height;
        if (!planning_map.findSurfaceGroundUnlocked(
            key, astar.preferredGroundGapCells(), astar.groundGapToleranceCells(), support) ||
          !planning_map.supportHeightUnlocked(support, height))
        {
          have_heights = false;
          break;
        }
        const auto & previous = searched_points.back();
        if (std::hypot(position[0] - previous[0], position[1] - previous[1]) > 1e-3 ||
          std::abs(height - previous[2]) > 1e-3)
        {
          searched_points.push_back({position[0], position[1], height});
        }
      }
      if (have_heights) {
        if (!initial_goal_option.connector.empty()) {
          searched_points.insert(searched_points.end(), initial_goal_option.connector.begin()+1,
            initial_goal_option.connector.end());
        }
        const auto & previous = searched_points.back();
        if (std::hypot(selected_goal[0] - previous[0], selected_goal[1] - previous[1]) > 1e-3 ||
          std::abs(selected_goal[2] - previous[2]) > 1e-3)
        {
          searched_points.push_back(selected_goal);
        } else {searched_points.back() = selected_goal;}
        const auto searched_corners = searched_points.size();
        searched_points = astar.densifyPath(searched_points, path_pose_spacing_);
        Astar3D::TerrainValidation searched_validation;
        nav_msgs::msg::Path searched_path;
        searched_path.header = path.header;
        if (astar.refreshAndValidateTerrainPath(start_key, searched_points, &searched_validation)) {
          populateGroundPath(searched_path, searched_points, goal.pose.orientation);
          if (validateFootprintPath(searched_path, start.pose.orientation,
              goal.pose.orientation, nullptr, false, checker, &planning_map))
          {
            recovered_searched_route = true;
            output_points = std::move(searched_points);
            path = std::move(searched_path);
            retained_corners = searched_corners;
            horizontal_length = 0.0;
            for (std::size_t i = 1; i < output_points.size(); ++i) {
              horizontal_length += std::hypot(output_points[i][0] - output_points[i - 1][0],
                output_points[i][1] - output_points[i - 1][1]);
            }
            RCLCPP_INFO(node->get_logger(),
              "%s kept the original searched route after rectangular shortcut rejection; "
              "terrain, obstacles and configured footprint checks revalidated", name_.c_str());
          }
        }
      }
    }
    const auto accept_recovery = [&](const VoxelKey & alternative,
        const std::vector<VoxelKey> & route, const VoxelKey & reached,
        double cost, const Astar3D::SearchDiagnostics & diagnostic, bool refine) {
        const auto selected = option_indices.find(reached);
        if (selected == option_indices.end()) {return false;}
        const auto try_endpoint = [&](std::size_t index) {
          VoxelKey ground;
          double height;
          if (!planning_map.findSurfaceGroundUnlocked(reached, astar.preferredGroundGapCells(),
              astar.groundGapToleranceCells(), ground) ||
            !planning_map.supportHeightUnlocked(ground, height)) {return false;}
          const auto & option = goal_options[index];
          if (!option.connector.empty()) {height = option.connector.back()[2];}
          const OccupyMap::Point target{option.x, option.y, height};
          std::vector<OccupyMap::Point> points{
            {start.pose.position.x, start.pose.position.y, start_path_z}};
          const bool exact_start = alternative.x == actual_start_cell.x &&
            alternative.y == actual_start_cell.y;
          for (std::size_t i = exact_start ? 1 : 0; i < route.size(); ++i) {
            if (!planning_map.findSurfaceGroundUnlocked(route[i], astar.preferredGroundGapCells(),
                astar.groundGapToleranceCells(), ground) ||
              !planning_map.supportHeightUnlocked(ground, height)) {return false;}
            auto q = planning_map.keyToPos(route[i]); q[2] = height;
            if (std::hypot(q[0] - points.back()[0], q[1] - points.back()[1]) > 1e-3 ||
              std::abs(q[2] - points.back()[2]) > 1e-3) {points.push_back(q);}
          }
          if (!option.connector.empty()) {
            points.insert(points.end(), option.connector.begin()+1, option.connector.end());
          }
          if (std::hypot(points.back()[0] - target[0], points.back()[1] - target[1]) > 1e-3 ||
            std::abs(points.back()[2] - target[2]) > 1e-3) {points.push_back(target);}
          else {points.back() = target;}
          double start_yaw, goal_yaw;
          if (!planarYaw(start.pose.orientation, start_yaw) ||
            !planarYaw(goal.pose.orientation, goal_yaw)) {return false;}
          const auto candidate_original_corners = points.size();
          auto candidate_refinement = refinement;
          if (refine && refine_maneuver) {
            const bool retain = use_static_map_ && astar_config_.retain_start_projection_bridge &&
              !exact_start;
            points = PathRefiner(refinement_config).refine(points, start_yaw, goal_yaw, retain,
              [&](auto & candidate) {
                return astar.refreshAndValidateTerrainPath(alternative, candidate, nullptr, true) &&
                       checker->validatePath(candidate, start_yaw, goal_yaw);
              }, &candidate_refinement, clearance_cost);
          }
          const auto corners = points.size();
          points = astar.densifyPath(points, path_pose_spacing_);
          if (!astar.refreshAndValidateTerrainPath(alternative, points, nullptr, true)) {return false;}
          nav_msgs::msg::Path candidate; candidate.header = path.header;
          populateGroundPath(candidate, points, goal.pose.orientation);
          if (!validateFootprintPath(candidate, start.pose.orientation, goal.pose.orientation,
              nullptr, false, checker, &planning_map)) {return false;}
          // Commit all endpoint metadata together. Otherwise a safe alternate
          // endpoint can be cached/logged under the old, rejected goal key.
          option_indices[reached] = index;
          path = std::move(candidate); output_points = std::move(points);
          start_key = alternative; goal_key = reached; selected_goal = target; goal_path_z = target[2];
          path_cost = cost; search_diagnostics = diagnostic; retained_corners = corners;
          refinement = refine ? candidate_refinement : PathRefiner::Diagnostics{};
          used_search_seed = false;
          original_corners = candidate_original_corners;
          horizontal_length = 0.0;
          for (std::size_t i = 1; i < output_points.size(); ++i) {
            horizontal_length += std::hypot(output_points[i][0] - output_points[i-1][0],
              output_points[i][1] - output_points[i-1][1]);
          }
          return true;
        };
        const auto preferred = selected->second;
        if (try_endpoint(preferred)) {return true;}
        const auto group = option_groups.find(reached);
        if (group != option_groups.end()) {
          for (auto index : group->second) {
            if (index != preferred && try_endpoint(index)) {return true;}
          }
        }
        return false;
      };
    const auto retry_endpoints = [&](const VoxelKey & alternative, bool omit_rejected_goal) {
        if (!start_retry_begin) {start_retry_begin = std::chrono::steady_clock::now();}
        auto remaining_goals = goal_keys;
        const auto group = option_groups.find(goal_key);
        if (omit_rejected_goal && (group == option_groups.end() || group->second.size() < 2)) {
          remaining_goals.erase(std::remove(remaining_goals.begin(), remaining_goals.end(), goal_key),
            remaining_goals.end());
        }
        while (!remaining_goals.empty() && start_retry_remaining() > 0.0) {
          auto config = astar_config_;
          config.max_search_nodes = std::min<std::size_t>(config.max_search_nodes, 50000);
          config.maximum_search_seconds = start_retry_remaining();
          if (config.maximum_search_seconds <= 0.0) {break;}
          std::vector<VoxelKey> route;
          VoxelKey reached;
          Astar3D::SearchDiagnostics diagnostic;
          double cost = 0.0;
          if (!Astar3D(planning_map, config).searchToAny(alternative, remaining_goals, route, reached,
              &cost, &diagnostic))
          {
            // A component exhausted for only a subset of goals must not be
            // excluded from retries that restore the full goal set.
            if (remaining_goals.size() == goal_keys.size()) {remember_disconnected(diagnostic);}
            break;
          }
          if (accept_recovery(alternative, route, reached, cost, diagnostic, false)) {return true;}
          remaining_goals.erase(std::remove(remaining_goals.begin(), remaining_goals.end(), reached),
            remaining_goals.end());
        }
        return false;
      };
    if (!recovered_searched_route && goal_keys.size() > 1) {
      recovered_searched_route = retry_endpoints(start_key, true);
    }
    if (!recovered_searched_route) {
      recovered_searched_route = try_alternate_starts([&](const Astar3D &,
          const VoxelKey & alternative, const std::vector<OccupyMap::Point> &) {
          if (!retry_endpoints(alternative, false)) {return false;}
          RCLCPP_INFO(node->get_logger(),
            "%s retained a fully checked alternate start after %zu attempts; goals=%zu",
            name_.c_str(), start_retry_attempts, goal_keys.size());
          return true;
        });
    }
    if (!recovered_searched_route && constrain_motion) {
      used_heading_search = true;
      configure_motion_start(motion, start_key);
      auto remaining_goals = goal_keys;
      const auto maneuver_begin = std::chrono::steady_clock::now();
      const double maneuver_budget = motion.maximum_seconds;
      Astar3D::SearchDiagnostics diagnostic;
      // Preserve every goal through arrival-heading and terminal-turn checks.
      // If final continuous validation still rejects one, try the remaining
      // endpoints within the same total maneuver budget.
      while (!remaining_goals.empty()) {
        auto attempt_motion = motion;
        attempt_motion.maximum_seconds = maneuver_budget - std::chrono::duration<double>(
          std::chrono::steady_clock::now() - maneuver_begin).count();
        if (attempt_motion.maximum_seconds <= 0.0) {break;}
        std::vector<VoxelKey> route;
        VoxelKey reached;
        double cost = 0.0;
        if (!astar.searchToAny(start_key, remaining_goals, route, reached,
            &cost, &diagnostic, &attempt_motion)) {break;}
        if (accept_recovery(start_key, route, reached, cost, diagnostic, true)) {
          recovered_searched_route = true;
          break;
        }
        remaining_goals.erase(std::remove(remaining_goals.begin(), remaining_goals.end(), reached),
          remaining_goals.end());
      }
      if (!recovered_searched_route) {
        RCLCPP_INFO(node->get_logger(), "%s maneuver fallback did not yield a valid route: "
          "reason=%s expanded=%zu motion_rejected=%zu remaining_goals=%zu", name_.c_str(),
          diagnostic.reason, diagnostic.expanded, diagnostic.motion_rejected, remaining_goals.size());
      }
    }
    if (!recovered_searched_route && activate_subcell_goals()) {
      // Retain the established coarse-endpoint preference. Only after those
      // complete maneuvers fail, search the newly checked terminal anchors.
      std::vector<VoxelKey> remaining;
      for (const auto & key : goal_keys) {
        if (option_groups.count(key)) {remaining.push_back(key);}
      }
      const auto retry_begin = std::chrono::steady_clock::now();
      while (!remaining.empty()) {
        auto config = astar_config_;
        config.max_search_nodes = std::min<std::size_t>(config.max_search_nodes, 50000);
        config.maximum_search_seconds = .35 - std::chrono::duration<double>(
          std::chrono::steady_clock::now() - retry_begin).count();
        if (config.maximum_search_seconds <= 0) {break;}
        std::vector<VoxelKey> route; VoxelKey reached;
        double cost = 0; Astar3D::SearchDiagnostics diagnostic;
        auto constraints = motion;
        configure_motion_start(constraints, start_key);
        constraints.maximum_seconds = config.maximum_search_seconds;
        if (!Astar3D(planning_map, config).searchToAny(start_key, remaining, route, reached,
            &cost, &diagnostic, constrain_motion ? &constraints : nullptr)) {break;}
        if (accept_recovery(start_key, route, reached, cost, diagnostic, true)) {
          recovered_searched_route = true; break;
        }
        remaining.erase(std::remove(remaining.begin(), remaining.end(), reached), remaining.end());
      }
    }
    if (!recovered_searched_route) {
      // Report the original rejection only after the safe alternative failed.
      validateFootprintPath(path, start.pose.orientation, goal.pose.orientation, nullptr, true, checker, &planning_map);
      clearPathCache();
      throw nav2_core::PlannerException("Astar3DPlanner final route failed rectangular footprint validation");
    }
  }
  const auto & final_goal_option = goal_options.at(option_indices.at(goal_key));
  if (!final_goal_option.preserves_exact_xy) {
    RCLCPP_WARN(node->get_logger(),
      "%s moved requested goal (%.2f, %.2f) to fully checked ground (%.2f, %.2f), offset %.2f m",
      name_.c_str(), goal.pose.position.x, goal.pose.position.y,
      selected_goal[0], selected_goal[1],
      std::hypot(selected_goal[0] - goal.pose.position.x, selected_goal[1] - goal.pose.position.y));
  }
  straight_distance = std::hypot(
    output_points.back()[0] - output_points.front()[0],
    output_points.back()[1] - output_points.front()[1]);
  cachePath(
    goal_key, goal.pose.position.x, goal.pose.position.y,
    path, use_static_map_ ? &start_key : nullptr);

  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - begin).count();
  double minimum_path_z = output_points.front()[2];
  double maximum_path_z = output_points.front()[2];
  for (const auto & point : output_points) {
    minimum_path_z = std::min(minimum_path_z, point[2]);
    maximum_path_z = std::max(maximum_path_z, point[2]);
  }
  RCLCPP_INFO(
    node->get_logger(),
    "%s found %zu-point ground path, cost %.3f, z=[%.2f, %.2f], "
    "goal_z=%.2f, candidates=%zu, using %zu occupied voxels in %.3f s; "
    "xy_length=%.3f straight=%.3f vertices=%zu->%zu",
    name_.c_str(), path.poses.size(), path_cost, minimum_path_z, maximum_path_z,
    goal_path_z, goal_options.size(), planning_map.rawVoxelCountUnlocked(), elapsed,
    horizontal_length, straight_distance, original_corners, retained_corners);
  RCLCPP_INFO(node->get_logger(),
    "%s maneuver search: heading_aware=%s expanded=%zu heuristic_expanded=%zu motion_rejected=%zu "
    "terminal_rejected=%zu refinement_checks=%zu shortcuts=%zu bevels=%zu budget_exhausted=%s "
    "search_seed=%s clearance=%.4f->%.4f clearance_rejected=%zu",
    name_.c_str(), used_heading_search && footprint_config_.check_in_place_turn_sweeps ?
    "true" : "false", search_diagnostics.expanded,
    search_diagnostics.heuristic_expanded,
    search_diagnostics.motion_rejected, search_diagnostics.terminal_rejected,
    refinement.checks, refinement.shortcuts, refinement.bevels,
    refinement.budget_exhausted ? "true" : "false",used_search_seed ? "true" : "false",
    refinement.clearance_before,refinement.clearance_after,refinement.clearance_rejected);
  checked_start = start_key;
  return path;
}

void Astar3DPlanner::supportCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (fixed_map_guard_ && fixed_map_guard_->validateCloud(
      FixedMapGuard::Layer::Support, *msg) != FixedMapGuard::Decision::Accept)
  {return;}
  if (use_static_map_ && msg->header.frame_id != global_frame_) {return;}
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (stop_worker_) {return;}
    if (map_ && map_->heightCompensationEnabled()) {
      pending_terrain_pairs_.pushSupport(msg);
    } else {pending_support_cloud_ = msg;}
  }
  pending_cv_.notify_one();
}

void Astar3DPlanner::obstacleCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (use_static_map_) {
    const auto node = node_.lock();
    if (!node || !liveObstacleStampFresh(
        msg->header.stamp, node->now().nanoseconds(), live_obstacle_freshness_timeout_)) {
      if (node) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
          "Skipping offline live obstacles with invalid, future or older-than-%.2fs stamp",
          live_obstacle_freshness_timeout_);
      }
      return;
    }
  }
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (stop_worker_) {return;}
    if (map_ && map_->heightCompensationEnabled()) {
      pending_terrain_pairs_.pushObstacles(msg);
    } else {pending_obstacle_cloud_ = msg;}
  }
  pending_cv_.notify_one();
}

void Astar3DPlanner::lioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(lio_mutex_);
  latest_lio_odom_ = msg;
}

void Astar3DPlanner::terrainPathValidityCallback(
  const nav2_msgs::srv::IsPathValid::Request::SharedPtr request,
  nav2_msgs::srv::IsPathValid::Response::SharedPtr response)
{
  auto detailed_request = std::make_shared<terrain_navigation_interfaces::srv::ValidateTerrainPath::Request>();
  detailed_request->path = request->path;
  auto result = std::make_shared<terrain_navigation_interfaces::srv::ValidateTerrainPath::Response>();
  detailedTerrainPathValidityCallback(detailed_request, result);
  response->is_valid = result->is_valid;
  response->invalid_pose_indices = result->invalid_pose_indices;
}

void Astar3DPlanner::detailedTerrainPathValidityCallback(
  const terrain_navigation_interfaces::srv::ValidateTerrainPath::Request::SharedPtr request,
  terrain_navigation_interfaces::srv::ValidateTerrainPath::Response::SharedPtr response)
{
  try {evaluateTerrainPath(request->path, *response);}
  catch (const tf2::TransformException & error) {
    response->is_valid = false; response->status = response->TF_UNAVAILABLE;
    response->reason = error.what();
  } catch (const std::exception & error) {
    response->is_valid = false; response->status = response->INTERNAL_ERROR;
    response->reason = error.what();
  }
}

void Astar3DPlanner::referenceTerrainPathValidityCallback(
  const terrain_navigation_interfaces::srv::ValidateTerrainPath::Request::SharedPtr request,
  terrain_navigation_interfaces::srv::ValidateTerrainPath::Response::SharedPtr response)
{
  try {evaluateTerrainPath(request->path, *response, true);}
  catch (const tf2::TransformException & error) {
    response->is_valid = false; response->status = response->TF_UNAVAILABLE;
    response->reason = error.what();
  } catch (const std::exception & error) {
    response->is_valid = false; response->status = response->INTERNAL_ERROR;
    response->reason = error.what();
  }
}

void Astar3DPlanner::evaluateTerrainPath(const nav_msgs::msg::Path & path,
  terrain_navigation_interfaces::srv::ValidateTerrainPath::Response & result,
  bool reference_path)
{
  using Result = terrain_navigation_interfaces::srv::ValidateTerrainPath::Response;
  result.is_valid = false;
  result.status = Result::MAP_UNAVAILABLE;
  result.reason = "map_not_ready";
  result.invalid_pose_indices.clear();
  if (fixed_map_guard_ && !fixed_map_guard_->ready()) {result.reason = fixed_map_guard_->reason(); return;}
  if (!localizationReady()) {
    result.status = Result::LOCALIZATION_UNAVAILABLE;
    result.reason = "localization_not_ready"; return;
  }
  const auto node = node_.lock();
  if (!node || !map_ || !active_.load() || path.poses.size() < 2) {
    result.reason = "planner_inactive_or_map_missing";
    if (path.poses.size() < 2) {result.status = Result::INVALID_REQUEST; result.reason = "path_too_short";}
    return;
  }
  {
    const auto read = map_->lockForReading();
    if (!map_->readyUnlocked()) {return;}
  }
  if (!liveObstaclesReady()) {
    result.status = Result::OBSTACLES_UNAVAILABLE;
    result.reason = live_obstacle_health_.statusNow([this]() {
      const auto node = node_.lock();
      return node ? node->now().nanoseconds() : 0;}).reason;
    if (result.reason == "ready") {result.reason = "obstacle_map_not_ready";}
    return;
  }
  if (!path.header.frame_id.empty() &&
    path.header.frame_id != global_frame_)
  {
    result.status = Result::INVALID_REQUEST; result.reason = "path_frame_mismatch";
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "Cannot validate terrain path in frame '%s'; expected '%s'",
      path.header.frame_id.c_str(), global_frame_.c_str());
    return;
  }

  geometry_msgs::msg::TransformStamped robot_transform;
  try {
    robot_transform = tf_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero, 100ms);
  } catch (const tf2::TransformException & exception) {
    result.status = Result::TF_UNAVAILABLE; result.reason = exception.what();
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "Cannot validate terrain path without %s->%s transform: %s",
      global_frame_.c_str(), robot_base_frame_.c_str(), exception.what());
    return;
  }

  nav_msgs::msg::Path remaining_path;
  geometry_msgs::msg::PoseStamped projection;
  std::optional<std::array<double, 4>> checked_goal;
  {
    std::lock_guard<std::mutex> lock(path_validity_mutex_);
    checked_goal = reference_goal_;
    const auto & first = path.poses.front().pose.position;
    const auto & last = path.poses.back().pose.position;
    const std::int64_t stamp_ns = rclcpp::Time(path.header.stamp).nanoseconds();
    const bool same_path =
      validity_path_initialized_ &&
      validity_path_stamp_ns_ == stamp_ns &&
      validity_path_size_ == path.poses.size() &&
      std::hypot(first.x - validity_path_first_x_, first.y - validity_path_first_y_) < 1e-3 &&
      std::hypot(last.x - validity_path_last_x_, last.y - validity_path_last_y_) < 1e-3;
    if (!same_path) {
      validity_path_initialized_ = true;
      validity_path_stamp_ns_ = stamp_ns;
      validity_path_size_ = path.poses.size();
      validity_path_first_x_ = first.x;
      validity_path_first_y_ = first.y;
      validity_path_last_x_ = last.x;
      validity_path_last_y_ = last.y;
      validity_progress_distance_ = 0.0;
    }

    double candidate_progress = validity_progress_distance_;
    if (!selectStablePathSuffix(
        path,
        robot_transform.transform.translation.x,
        robot_transform.transform.translation.y,
        reference_path ? local_detour_max_deviation_ : path_validity_start_tolerance_, 0.0,
        candidate_progress, remaining_path, &projection))
    {
      result.status = Result::PATH_MISMATCH; result.reason = "robot_not_near_remaining_path";
      return;
    }
    validity_progress_distance_ = candidate_progress;
  }

  const auto map_lock = use_static_map_ ? map_->lockForReadingAfterExpiry(
    node->now().nanoseconds(),static_cast<std::int64_t>(std::llround(
      live_obstacle_max_observation_age_*1e9))) : map_->lockForReading();
  if (!map_->readyUnlocked() || remaining_path.poses.size() < 2) {
    return;
  }
  const auto inputs_still_ready = [&]() {
      if (!active_.load() || (fixed_map_guard_ && !fixed_map_guard_->ready())) {
        result.is_valid = false; result.status = Result::MAP_UNAVAILABLE;
        result.reason = "planning_inputs_changed_during_validation"; return false;
      }
      if (!localizationReady()) {
        result.is_valid = false; result.status = Result::LOCALIZATION_UNAVAILABLE;
        result.reason = "localization_not_ready"; return false;
      }
      if (use_static_map_) {
        const auto health = live_obstacle_health_.statusNow(
          [&node]() {return node->now().nanoseconds();});
        if (!health.ready) {
          result.is_valid = false; result.status = Result::OBSTACLES_UNAVAILABLE;
          result.reason = health.reason; return false;
        }
      }
      return true;
    };
  if (!reference_path) {
    validateTerrainPathGeometry(path, remaining_path, projection, robot_transform,
      *map_, result, true);
    if (result.is_valid) {inputs_still_ready();}
    return;
  }

  const double robot_x = robot_transform.transform.translation.x;
  const double robot_y = robot_transform.transform.translation.y;
  double robot_yaw, goal_yaw;
  if (!planarYaw(robot_transform.transform.rotation, robot_yaw) ||
    !planarYaw(path.poses.back().pose.orientation, goal_yaw))
  {
    result.status = Result::INVALID_REQUEST; result.reason = "invalid_path_orientation";
    return;
  }

  // A reference line may pass through a local obstacle while MPPI goes around
  // it. The actual current body never receives that reference-line exemption.
  if (footprint_check_enabled_) {
    auto config = footprint_config_;
    config.near_start_center = std::array<double, 2>{robot_x, robot_y};
    const SweptFootprint current_body(*map_, config);
    geometry_msgs::msg::PoseStamped current_ground;
    current_ground.header.frame_id = global_frame_;
    current_ground.pose.position.x = robot_x;
    current_ground.pose.position.y = robot_y;
    current_ground.pose.position.z = projection.pose.position.z;
    double ground_z = startHeightInGlobalFrame(current_ground);
    VoxelKey ground;
    if (map_->findSupportByHeightUnlocked(robot_x, robot_y, ground_z,
        std::max(astar_config_.start_projection_height_tolerance,
        astar_config_.ground_gap_tolerance), ground))
    {
      map_->supportHeightUnlocked(ground, ground_z);
    }
    if (!current_body.poseCollisionFree({robot_x, robot_y, ground_z}, robot_yaw)) {
      result.status = Result::COLLISION; result.reason = "current_footprint_collision";
      return;
    }
  }

  validateTerrainPathGeometry(path, remaining_path, projection, robot_transform,
    *map_, result, false);
  bool detour_required = std::hypot(robot_x - projection.pose.position.x,
    robot_y - projection.pose.position.y) > path_validity_start_tolerance_;
  if (!result.is_valid) {
    if (result.status != Result::COLLISION && result.status != Result::TERRAIN_INVALID) {
      return;
    }
    std::size_t ignored = 0;
    const double radius = std::max(path_validity_ignore_near_robot_radius_,
      astar_config_.start_projection_xy_radius + path_pose_spacing_);
    const auto reference_map = map_->referenceSnapshotUnlocked(robot_x, robot_y, radius, &ignored);
    if (ignored == 0) {return;}
    // This private view keeps static, unknown, synthetic and distant evidence.
    // The live costmap, initial planner and strict recovery service are intact.
    validateTerrainPathGeometry(path, remaining_path, projection, robot_transform,
      *reference_map, result, false);
    if (!result.is_valid) {return;}
    detour_required = true;
  }

  if (!inputs_still_ready()) {return;}
  std::lock_guard<std::mutex> lock(path_validity_mutex_);
  if (checked_goal != reference_goal_) {
    result.is_valid = false; result.status = Result::PATH_MISMATCH;
    result.reason = "reference_goal_changed"; return;
  }
  const auto goal = checked_goal.value_or(std::array<double, 4>{
    path.poses.back().pose.position.x, path.poses.back().pose.position.y,
    path.poses.back().pose.position.z, goal_yaw});
  const auto now_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto decision = local_detour_budget_.update(
    goal, now_s, {robot_x, robot_y, robot_yaw}, detour_required);
  result.is_valid = decision.allowed();
  result.status = result.is_valid ? Result::VALID : Result::PATH_MISMATCH;
  result.reason = decision.reason();
}

void Astar3DPlanner::validateTerrainPathGeometry(
  const nav_msgs::msg::Path & original_path, nav_msgs::msg::Path remaining_path,
  const geometry_msgs::msg::PoseStamped & projection,
  const geometry_msgs::msg::TransformStamped & robot_transform,
  const OccupyMap & query_map,
  terrain_navigation_interfaces::srv::ValidateTerrainPath::Response & result,
  bool report_failure)
{
  using Result = terrain_navigation_interfaces::srv::ValidateTerrainPath::Response;
  result.is_valid = false;
  result.status = Result::TERRAIN_INVALID;
  result.reason = "terrain_path_not_validated";
  result.invalid_pose_indices.clear();
  const auto node = node_.lock();
  if (!node) {result.status = Result::INTERNAL_ERROR; return;}
  Astar3D astar(query_map, astar_config_);

  // Support can retain its voxel key while its continuous height is corrected.
  // A cached route must still satisfy the current actual slope and step limits.
  // The initial LiDAR-blind bridge retains its existing projection rule.
  double blocked_length = 0.0;
  std::size_t blocked_begin = 0;
  bool have_previous_position = false;
  OccupyMap::Point previous_position{};
  const auto goal = remaining_path.poses.back().pose.position;
  const double robot_x = robot_transform.transform.translation.x;
  const double robot_y = robot_transform.transform.translation.y;
  geometry_msgs::msg::PoseStamped terrain_start;
  terrain_start.header.frame_id = global_frame_;
  terrain_start.pose.position.x = robot_x;
  terrain_start.pose.position.y = robot_y;
  terrain_start.pose.position.z = remaining_path.poses.front().pose.position.z;
  VoxelKey projected_start;
  bool retained_verified_connector = false;
  if (use_static_map_ && astar_config_.retain_start_projection_bridge) {
    std::lock_guard<std::mutex> cache_lock(path_cache_mutex_);
    // Only an exact copy of our most recently validated output may retain its
    // chosen connector. An arbitrary caller's path gains no new exemption.
    if (cached_path_valid_ && cached_start_valid_ && cached_output_path_ == original_path &&
      astar.startBridgeCollisionFree(robot_x, robot_y, startHeightInGlobalFrame(terrain_start),
        cached_start_key_))
    {
      projected_start = cached_start_key_;
      retained_verified_connector = true;
    }
  }
  if (!retained_verified_connector && !astar.projectStartToSurface(
      robot_x, robot_y, startHeightInGlobalFrame(terrain_start),
      astar_config_.start_projection_xy_radius, astar_config_.projection_z_radius,
      goal.x, goal.y, projected_start))
  {
    result.status = Result::TERRAIN_INVALID; result.reason = "start_projection_failed";
    Astar3D::ProjectionDiagnostics diagnostics;
    astar.projectStartToSurface(
      robot_x, robot_y, startHeightInGlobalFrame(terrain_start),
      astar_config_.start_projection_xy_radius, astar_config_.projection_z_radius,
      goal.x, goal.y, projected_start, &diagnostics);
    result.status = diagnostics.has_collision ? Result::COLLISION : Result::TERRAIN_INVALID;
    result.reason = diagnostics.reason;
    if (report_failure) {
      publishStartFailure({robot_x, robot_y, startHeightInGlobalFrame(terrain_start)},
        diagnostics, nullptr, nullptr, nullptr, &query_map);
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "Terrain path invalid: start_projection_failed at robot=(%.3f, %.3f), "
        "ground_reference=%.3f, radius=%.3f",
        robot_x, robot_y, startHeightInGlobalFrame(terrain_start),
        astar_config_.start_projection_xy_radius);
    }
    return;
  }
  Astar3D::TerrainValidation failure;
  SweptFootprint::Failure footprint_failure;
  auto remaining_footprint = footprint_config_;
  remaining_footprint.near_start_center = std::array<double, 2>{robot_x, robot_y};
  const auto remaining_checker = footprint_check_enabled_ ?
    std::make_unique<SweptFootprint>(query_map, remaining_footprint) : nullptr;
  if (!reconnectAndValidateSuffix(astar, projected_start, projection,
      robot_transform.transform.rotation, remaining_path, &failure, &footprint_failure,
      report_failure, remaining_checker.get(), &query_map))
  {
    result.status = failure.has_collision ? Result::COLLISION : Result::TERRAIN_INVALID;
    result.reason = failure.reason;
    if (std::string(failure.reason) == "valid") {
      result.status = Result::COLLISION; result.reason = footprint_failure.reason;
      result.invalid_pose_indices.push_back(static_cast<std::int32_t>(
          std::min(footprint_failure.index, remaining_path.poses.size() - 1)));
      return;
    }
    if (report_failure) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "Terrain path invalid: %s at pose=%zu near (%.3f, %.3f, %.3f)",
        failure.reason, failure.index,
        failure.position[0], failure.position[1], failure.position[2]);
      if (failure.has_support_edge) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 1000,
          "Terrain rejected support edge: from=(%d,%d,%d) h=%.6f "
          "to=(%d,%d,%d) h=%.6f xy_distance=%.6f slope_limit_deg=%.3f step_limit=%.3f",
          failure.previous_support.x, failure.previous_support.y, failure.previous_support.z,
          failure.previous_height,
          failure.current_support.x, failure.current_support.y, failure.current_support.z,
          failure.current_height, failure.horizontal_distance,
          astar_config_.max_slope_deg, astar_config_.max_step_height);
      }
    }
    return;
  }
  const double effective_near_robot_ignore_radius = std::max(
    path_validity_ignore_near_robot_radius_,
    astar_config_.start_projection_xy_radius + path_pose_spacing_);
  for (std::size_t i = 1; i < remaining_path.poses.size(); ++i) {
    const auto & position = remaining_path.poses[i].pose.position;
    if (std::hypot(position.x - robot_x, position.y - robot_y) <=
      effective_near_robot_ignore_radius)
    {
      continue;
    }
    if (std::hypot(position.x - goal.x, position.y - goal.y) <=
      cached_goal_obstacle_ignore_radius_)
    {
      break;
    }
    const OccupyMap::Point current_position{
      position.x, position.y, position.z};
    const double sample_length = have_previous_position ?
      std::hypot(
      current_position[0] - previous_position[0],
      current_position[1] - previous_position[1]) :
      path_pose_spacing_;
    previous_position = current_position;
    have_previous_position = true;

    const auto traversability = astar.pathPointTraversability(current_position);
    if (traversability == OccupyMap::Traversability::kNoGroundSupport ||
      traversability == OccupyMap::Traversability::kGroundGapMismatch)
    {
      result.status = Result::TERRAIN_INVALID; result.reason = "ground_support_invalid";
      result.invalid_pose_indices.push_back(static_cast<std::int32_t>(i));
      return;
    }
    if (traversability == OccupyMap::Traversability::kOccupied) {
      if (blocked_length <= 1e-9) {
        blocked_begin = i;
      }
      blocked_length += sample_length;
      if (blocked_length + 1e-9 >= path_validity_min_blocked_length_) {
        result.status = Result::COLLISION; result.reason = "remaining_corridor_occupied";
        result.invalid_pose_indices.push_back(
          static_cast<std::int32_t>(blocked_begin));
        if (report_failure) {
          RCLCPP_WARN_THROTTLE(
            node->get_logger(), *node->get_clock(), 1000,
            "Remaining terrain corridor is occupied for %.2f m near "
            "(%.2f, %.2f, %.2f)",
            blocked_length, position.x, position.y, position.z);
        }
        return;
      }
    } else {
      blocked_length = 0.0;
    }
  }
  result.is_valid = true; result.status = Result::VALID; result.reason = "valid";
}

void Astar3DPlanner::mapWorker()
{
  while (true) {
    sensor_msgs::msg::PointCloud2::SharedPtr support_cloud;
    sensor_msgs::msg::PointCloud2::SharedPtr obstacle_cloud;
    sensor_msgs::msg::PointCloud2::SharedPtr static_obstacle_cloud;
    std::optional<std::uint64_t> obstacle_epoch;
    {
      std::unique_lock<std::mutex> lock(pending_mutex_);
      const bool ready = pending_cv_.wait_for(
        lock, 2s, [this]() {
          if (stop_worker_) {return true;}
          if (map_ && map_->heightCompensationEnabled()) {
            return pending_terrain_pairs_.hasPair();
          }
          return pending_support_cloud_ != nullptr || pending_obstacle_cloud_ != nullptr ||
                 pending_static_obstacle_cloud_ != nullptr;
        });
      if (stop_worker_) {
        return;
      }
      if (!ready) {
        const auto support_count = pending_terrain_pairs_.supportSize();
        const auto obstacle_count = pending_terrain_pairs_.obstacleSize();
        lock.unlock();
        const auto node = node_.lock();
        if (node && (support_count > 0 || obstacle_count > 0)) {
          RCLCPP_WARN(
            node->get_logger(),
            "Waiting for matching terrain stamps: buffered support=%zu obstacles=%zu; "
            "height compensation requires both outputs from the same terrain frame",
            support_count, obstacle_count);
        }
        continue;
      }
      if (map_ && map_->heightCompensationEnabled()) {
        pending_terrain_pairs_.popPair(support_cloud, obstacle_cloud);
      } else {
        support_cloud = std::move(pending_support_cloud_);
        obstacle_cloud = std::move(pending_obstacle_cloud_);
      }
      static_obstacle_cloud = std::move(pending_static_obstacle_cloud_);
      if (use_static_map_ && obstacle_cloud && map_) {
        obstacle_epoch = map_->dynamicObstacleEpoch();
      }
    }

    try {
      if (static_obstacle_cloud && map_) {
        std::vector<std::uint8_t> sources;
        const auto points = transformCloud(*static_obstacle_cloud, &sources, nullptr, true);
        map_->setStaticObstacles(points,
          rclcpp::Time(static_obstacle_cloud->header.stamp).nanoseconds(), sources);
        clearPathCache();
      }
      if (support_cloud && map_) {
        double before;
        {
          const auto read = map_->lockForReading(); before = map_->heightDatumOffsetUnlocked();
        }
        map_->updateSupport(
          transformCloud(*support_cloud, nullptr, nullptr, use_static_map_),
          rclcpp::Time(support_cloud->header.stamp).nanoseconds());
        const auto node = node_.lock();
        {
          const auto read = map_->lockForReading();
          const double after = map_->heightDatumOffsetUnlocked();
          if (node && std::abs(after - before) > 1e-6) {
            RCLCPP_INFO_THROTTLE(
              node->get_logger(), *node->get_clock(), 2000,
              "Planning height datum: update=%.4f m total=%.4f m", after - before, after);
          }
        }
        publishSupportSnapshot();
      }
      if (obstacle_cloud && map_ && alignmentReady()) {
        OccupyMap::Point observer;
        if (obstacleObserverInGlobalFrame(observer)) {
          std::vector<std::uint8_t> sources;
          std::vector<std::int64_t> observed_stamps_ns;
          const auto points = transformCloud(*obstacle_cloud, &sources, &observed_stamps_ns);
          // The conversion may wait for TF while the replay clock rewinds or
          // advances. Reject stale DDS deliveries again before they can move
          // the new stream's monotonic timestamp baseline into the future.
          if (use_static_map_) {
            const auto node = node_.lock();
            if (!node || !liveObstacleStampFresh(
                obstacle_cloud->header.stamp, node->now().nanoseconds(),
                live_obstacle_freshness_timeout_)) {continue;}
          }
          if (use_static_map_) {
            // Serialize commit+health against alignment invalidation. A cloud
            // whose TF conversion crossed an epoch change must not revive it.
            std::lock_guard<std::mutex> lock(pending_mutex_);
            const auto node = node_.lock();
            if (!node || !alignmentReady() || !liveObstacleStampFresh(
                obstacle_cloud->header.stamp, node->now().nanoseconds(),
                live_obstacle_freshness_timeout_)) {continue;}
            if (map_->updateObstacles(
                points, observer[0], observer[1],
                rclcpp::Time(obstacle_cloud->header.stamp).nanoseconds(), sources, true,
                obstacle_epoch, observed_stamps_ns))
            {
              live_obstacle_health_.consumed(
                rclcpp::Time(obstacle_cloud->header.stamp).nanoseconds(),
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now().time_since_epoch()).count());
            }
          } else {
            map_->updateObstacles(points, observer[0], observer[1],
              rclcpp::Time(obstacle_cloud->header.stamp).nanoseconds(), sources, false,
              obstacle_epoch, observed_stamps_ns);
          }
        } else {
          const auto node = node_.lock();
          if (node) {
            RCLCPP_WARN_THROTTLE(
              node->get_logger(), *node->get_clock(), 2000,
              "Skipping global obstacle refresh: no transformable LIO observer pose");
          }
        }
      }
    } catch (const tf2::TransformException & exception) {
      const auto node = node_.lock();
      if (node) {
        RCLCPP_WARN(
          node->get_logger(), "Skipping terrain input: cannot transform to %s: %s",
          global_frame_.c_str(), exception.what());
      }
    } catch (const std::exception & exception) {
      const auto node = node_.lock();
      if (node) {
        RCLCPP_WARN(node->get_logger(), "Skipping invalid cloud: %s", exception.what());
      }
    }
  }
}

void Astar3DPlanner::publishSupportSnapshot(bool force)
{
  // Lifecycle publication must stay serialized with activate/deactivate, but
  // the map lock is released before handing the message to DDS.
  std::lock_guard<std::mutex> publish_lock(support_snapshot_mutex_);
  const auto node = node_.lock();
  const auto now = std::chrono::steady_clock::now();
  if (!node || !map_ || !active_.load() || !support_map_publisher_ ||
    (!force && now - last_support_map_publish_ < std::chrono::seconds(1)))
  {
    return;
  }
  std::vector<OccupyMap::Point> points;
  {
    const auto read = map_->lockForReading();
    points = map_->supportSnapshotUnlocked();
  }
  if (points.empty()) {return;}
  sensor_msgs::msg::PointCloud2 snapshot;
  snapshot.header.frame_id = global_frame_;
  snapshot.header.stamp = node->now();
  sensor_msgs::PointCloud2Modifier modifier(snapshot);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(snapshot, "x"), y(snapshot, "y"), z(snapshot, "z");
  for (const auto & point : points) {
    *x = point[0]; *y = point[1]; *z = point[2]; ++x; ++y; ++z;
  }
  support_map_publisher_->publish(snapshot);
  last_support_map_publish_ = now;
}

std::vector<OccupyMap::Point> Astar3DPlanner::transformCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  std::vector<std::uint8_t> * obstacle_sources,
  std::vector<std::int64_t> * observed_stamps_ns, bool fixed_layer) const
{
  if (obstacle_sources) {obstacle_sources->clear();}
  if (observed_stamps_ns) {observed_stamps_ns->clear();}
  // An empty obstacle map is valid. Humble's point-cloud iterators use
  // data.front()/back() during construction, so never construct one here.
  const bool empty = cloud.width == 0 || cloud.height == 0;
  if (empty && !cloud.data.empty()) {throw std::invalid_argument("Empty point cloud has a payload");}
  if (!empty && cloud.data.empty()) {throw std::invalid_argument("Nonempty point cloud has no data");}
  if (cloud.header.frame_id.empty()) {throw std::invalid_argument("Point cloud frame is missing");}
  tf2::Transform transform;
  transform.setIdentity();
  if (cloud.header.frame_id != global_frame_) {
    const auto stamped = tf_->lookupTransform(
      global_frame_, cloud.header.frame_id,
      use_static_map_ ? rclcpp::Time(0, 0, RCL_ROS_TIME) :
      rclcpp::Time(cloud.header.stamp, RCL_ROS_TIME), 200ms);
    tf2::fromMsg(stamped.transform, transform);
  }
  // An empty snapshot is a legitimate clearance observation, but its frame
  // must still belong to the current transform tree before clearing the map.
  return transformCloudObservations(
    cloud, transform, fixed_layer ? 1 : point_stride_, obstacle_sources, observed_stamps_ns);
}

bool Astar3DPlanner::obstacleObserverInGlobalFrame(
  OccupyMap::Point & observer) const
{
  nav_msgs::msg::Odometry::SharedPtr odometry;
  {
    std::lock_guard<std::mutex> lock(lio_mutex_);
    odometry = latest_lio_odom_;
  }
  if (!odometry) {
    return false;
  }

  geometry_msgs::msg::PointStamped source;
  source.header = odometry->header;
  // Use the chassis center for near-field policy, matching terrain analysis.
  // The LIO pose belongs to the offset body IMU origin.
  tf2::Transform odom_from_body;
  tf2::fromMsg(odometry->pose.pose, odom_from_body);
  const auto body_from_base_msg = tf_->lookupTransform(
    odometry->child_frame_id, robot_base_frame_, tf2::TimePointZero, 200ms);
  tf2::Transform body_from_base;
  tf2::fromMsg(body_from_base_msg.transform, body_from_base);
  const auto base_position = (odom_from_body * body_from_base).getOrigin();
  source.point.x = base_position.x(); source.point.y = base_position.y();
  source.point.z = base_position.z();
  geometry_msgs::msg::PointStamped target;
  if (source.header.frame_id == global_frame_) {
    target = source;
  } else {
    const auto transform = tf_->lookupTransform(
      global_frame_, source.header.frame_id,
      use_static_map_ ? rclcpp::Time(0, 0, RCL_ROS_TIME) :
      rclcpp::Time(source.header.stamp, RCL_ROS_TIME), 200ms);
    tf2::doTransform(source, target, transform);
  }
  observer = OccupyMap::Point{
    target.point.x, target.point.y, target.point.z};
  return true;
}

bool Astar3DPlanner::refreshAndValidateTerrainPath(
  const Astar3D & astar, const VoxelKey & start,
  nav_msgs::msg::Path & path, Astar3D::TerrainValidation * failure) const
{
  std::vector<OccupyMap::Point> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    const auto & position = pose.pose.position;
    points.push_back({position.x, position.y, position.z});
  }
  if (!astar.refreshAndValidateTerrainPath(start, points, failure)) {
    return false;
  }
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    path.poses[i].pose.position.z = points[i][2];
  }
  return true;
}

bool Astar3DPlanner::reconnectAndValidateSuffix(
  const Astar3D & astar, const VoxelKey & start,
  const geometry_msgs::msg::PoseStamped & projection,
  const geometry_msgs::msg::Quaternion & start_orientation,
  nav_msgs::msg::Path & path,
  Astar3D::TerrainValidation * terrain_failure,
  SweptFootprint::Failure * footprint_failure,
  bool report_failure, const SweptFootprint * checker, const OccupyMap * query_map)
{
  Astar3D::TerrainValidation terrain;
  SweptFootprint::Failure footprint;
  nav_msgs::msg::Path rejected, output;
  const auto validate = [&](nav_msgs::msg::Path & candidate) {
      terrain = Astar3D::TerrainValidation{};
      footprint = SweptFootprint::Failure{};
      const auto goal_orientation = candidate.poses.back().pose.orientation;
      std::vector<OccupyMap::Point> points;
      points.reserve(candidate.poses.size());
      for (const auto & pose : candidate.poses) {
        const auto & p = pose.pose.position; points.push_back({p.x, p.y, p.z});
      }
      // MPPI receives the same spacing as a freshly planned path, including
      // the new prefix. Keep exact start/goal and all retained path corners.
      populateGroundPath(candidate, astar.densifyPath(points, path_pose_spacing_), goal_orientation);
      if (!refreshAndValidateTerrainPath(astar, start, candidate, &terrain) ||
        !validateFootprintPath(candidate, start_orientation, goal_orientation, &footprint, false, checker, query_map))
      {rejected = candidate; return false;}
      return true;
    };
  PathReconnectDiagnostics diagnostic;
  // Disabled footprint checks retain the old connector policy. The new
  // alternatives require complete swept-body validation, never centre only.
  const bool valid = reconnectPathSuffix(path, projection,
    footprint_check_enabled_ ? .6 : 0.0, validate, output, &diagnostic);
  if (!valid && rejected.poses.empty()) {terrain.reason = "invalid_reconnection_input";}
  if (terrain_failure) {*terrain_failure = terrain;}
  if (footprint_failure) {*footprint_failure = footprint;}
  if (valid) {
    path = std::move(output);
    const auto node = node_.lock();
    if (node && diagnostic.used_forward) {
      RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
        "Safe forward path reconnect: %.3f m, attempts=%zu",
        diagnostic.forward_distance, diagnostic.attempts);
    }
  } else {
    if (!rejected.poses.empty()) {path = std::move(rejected);}
    if (report_failure && std::string(footprint.reason) != "valid") {
      reportFootprintFailure(path, footprint, query_map);
    }
  }
  return valid;
}

bool Astar3DPlanner::tryReuseCachedPath(
  const Astar3D & astar, const VoxelKey & start, const VoxelKey & goal,
  double exact_start_x, double exact_start_y,
  double exact_goal_x, double exact_goal_y,
  const geometry_msgs::msg::Quaternion & start_orientation,
  const geometry_msgs::msg::Quaternion & goal_orientation,
  bool allow_goal_obstacle_grace,
  double * goal_valid_age,
  nav_msgs::msg::Path & path, const SweptFootprint * checker, const OccupyMap * query_map)
{
  if (!reuse_previous_path_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(path_cache_mutex_);
  if (!cached_path_valid_ || cached_output_path_.poses.empty() ||
    !(cached_goal_key_ == goal) ||
    std::hypot(
      cached_exact_goal_x_ - exact_goal_x,
      cached_exact_goal_y_ - exact_goal_y) > 0.02)
  {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  if (allow_goal_obstacle_grace) {
    const double age = std::chrono::duration<double>(
      now - cached_goal_last_valid_).count();
    if (age > cached_goal_obstacle_grace_period_) {
      return false;
    }
    if (goal_valid_age != nullptr) {
      *goal_valid_age = age;
    }
  }

  double candidate_progress = cached_progress_distance_;
  geometry_msgs::msg::PoseStamped projection;
  if (!selectStablePathSuffix(
      cached_output_path_, exact_start_x, exact_start_y,
      path_reuse_start_tolerance_, path_reuse_anchor_behind_,
      candidate_progress, path, &projection))
  {
    return false;
  }

  path.poses.back().pose.orientation = goal_orientation;
  if (!reconnectAndValidateSuffix(astar, start, projection, start_orientation, path,
      nullptr, nullptr, false, checker, query_map)) {
    return false;
  }

  // Validate exactly the same dense suffix that will be returned.  Previously
  // a sparse raw-path index and a separate dense-output index could advance
  // independently, allowing validation and trimming to refer to different
  // portions of the route.
  double blocked_length = 0.0;
  bool have_previous_position = false;
  OccupyMap::Point previous_position{};
  const double effective_near_robot_ignore_radius = std::max(
    path_validity_ignore_near_robot_radius_,
    astar_config_.start_projection_xy_radius + path_pose_spacing_);
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & position = path.poses[i].pose.position;
    if (std::hypot(position.x - exact_start_x, position.y - exact_start_y) <=
      effective_near_robot_ignore_radius)
    {
      continue;
    }
    if (allow_goal_obstacle_grace &&
      std::hypot(position.x - exact_goal_x, position.y - exact_goal_y) <=
      cached_goal_obstacle_ignore_radius_)
    {
      break;
    }
    const OccupyMap::Point current_position{
      position.x, position.y, position.z};
    const double sample_length = have_previous_position ?
      std::hypot(
      current_position[0] - previous_position[0],
      current_position[1] - previous_position[1]) :
      path_pose_spacing_;
    previous_position = current_position;
    have_previous_position = true;
    const auto traversability = astar.pathPointTraversability(current_position);
    if (traversability == OccupyMap::Traversability::kNoGroundSupport ||
      traversability == OccupyMap::Traversability::kGroundGapMismatch)
    {
      return false;
    }
    if (traversability == OccupyMap::Traversability::kOccupied)
    {
      blocked_length += sample_length;
      if (blocked_length + 1e-9 >= path_validity_min_blocked_length_) {
        return false;
      }
    } else {
      blocked_length = 0.0;
    }
  }

  const auto node = node_.lock();
  if (!node) {return false;}
  path.header.stamp = node->now();
  for (auto & pose : path.poses) {pose.header = path.header;}
  cached_output_path_ = path;
  // The cache is now this new suffix, so its arc-length cursor starts at zero.
  cached_progress_distance_ = 0.0;
  cached_start_valid_ = use_static_map_;
  cached_start_key_ = start;
  if (!allow_goal_obstacle_grace) {cached_goal_last_valid_ = now;}
  return true;
}

void Astar3DPlanner::cachePath(
  const VoxelKey & goal, double exact_goal_x, double exact_goal_y,
  const nav_msgs::msg::Path & output_path, const VoxelKey * verified_start)
{
  std::lock_guard<std::mutex> lock(path_cache_mutex_);
  cached_goal_key_ = goal;
  cached_exact_goal_x_ = exact_goal_x;
  cached_exact_goal_y_ = exact_goal_y;
  cached_progress_distance_ = 0.0;
  cached_output_path_ = output_path;
  cached_start_valid_ = verified_start != nullptr;
  if (verified_start) {cached_start_key_ = *verified_start;}
  cached_path_valid_ = !output_path.poses.empty();
  cached_goal_last_valid_ = std::chrono::steady_clock::now();
}

void Astar3DPlanner::clearPathCache()
{
  std::lock_guard<std::mutex> lock(path_cache_mutex_);
  cached_path_valid_ = false;
  cached_progress_distance_ = 0.0;
  cached_output_path_.poses.clear();
  cached_start_valid_ = false;
  cached_goal_last_valid_ = std::chrono::steady_clock::time_point{};
}

double Astar3DPlanner::startHeightInGlobalFrame(
  const geometry_msgs::msg::PoseStamped & nav2_start) const
{
  if (!use_lio_start_z_) {
    return nav2_start.pose.position.z;
  }

  nav_msgs::msg::Odometry::SharedPtr odometry;
  {
    std::lock_guard<std::mutex> lock(lio_mutex_);
    odometry = latest_lio_odom_;
  }
  if (!odometry) {
    return nav2_start.pose.position.z;
  }

  try {
    geometry_msgs::msg::PointStamped source;
    source.header = odometry->header;
    source.point = odometry->pose.pose.position;
    geometry_msgs::msg::PointStamped target;
    if (source.header.frame_id == global_frame_) {
      target = source;
    } else {
      const auto transform = tf_->lookupTransform(
        global_frame_, source.header.frame_id,
        use_static_map_ ? rclcpp::Time(0, 0, RCL_ROS_TIME) :
        rclcpp::Time(source.header.stamp, RCL_ROS_TIME), 200ms);
      tf2::doTransform(source, target, transform);
    }
    return target.point.z + lio_height_offset_;
  } catch (const tf2::TransformException & exception) {
    const auto node = node_.lock();
    if (node) {
      RCLCPP_WARN(
        node->get_logger(),
        "Cannot transform latest LIO height into %s, using Nav2 start Z: %s",
        global_frame_.c_str(), exception.what());
    }
    return nav2_start.pose.position.z;
  }
}

void Astar3DPlanner::stopWorker()
{
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    stop_worker_ = true;
    pending_support_cloud_.reset();
    pending_obstacle_cloud_.reset();
    pending_static_obstacle_cloud_.reset();
    pending_terrain_pairs_.clear();
  }
  pending_cv_.notify_all();
  if (map_worker_.joinable()) {
    map_worker_.join();
  }
}

void Astar3DPlanner::auditFixedMap()
{
  const auto node = node_.lock();
  if (!node || !fixed_map_guard_ || !map_status_publisher_) {return;}
  const auto sources = [&node](const std::string & topic) {
      std::vector<std::string> names;
      for (const auto & endpoint : node->get_publishers_info_by_topic(topic)) {
        const auto & ns = endpoint.node_namespace();
        names.push_back(ns + (ns.empty() || ns.back() != '/' ? "/" : "") + endpoint.node_name());
      }
      return names;
    };
  try {
    fixed_map_guard_->observePublishers(
      sources(support_sub_->get_topic_name()), sources(static_obstacle_sub_->get_topic_name()),
      sources(static_map_id_sub_->get_topic_name()));
  } catch (const std::exception & error) {
    fixed_map_guard_->observePublishers({}, {}, {});
    RCLCPP_WARN(node->get_logger(), "Cannot inspect fixed-map publishers: %s", error.what());
  }
  const auto reason = fixed_map_guard_->reason();
  if (reason != last_map_guard_reason_) {
    if (fixed_map_guard_->faulted()) {
      RCLCPP_ERROR(node->get_logger(),
        "Fixed-map input rejected: %s; restart the offline stack after resolving the conflict",
        reason.c_str());
    } else {
      RCLCPP_INFO(node->get_logger(), "Fixed-map verification: %s", reason.c_str());
    }
    last_map_guard_reason_ = reason;
  }
  std_msgs::msg::String status;
  status.data = fixed_map_guard_->report();
  map_status_publisher_->publish(status);
}

bool Astar3DPlanner::localizationReady() const
{
  if (!use_static_map_) {return true;}
  const auto received = offline_ready_wall_ns_.load();
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  return alignmentReady() &&
         localizationHeartbeatStatus(offline_localization_ready_.load(), received, now).ready;
}

bool Astar3DPlanner::alignmentReady() const
{
  return alignmentHeartbeatStatus().ready;
}

LocalizationHeartbeatStatus Astar3DPlanner::alignmentHeartbeatStatus() const
{
  if (!use_static_map_) {return {true, "ready", -1.0};}
  const auto received = offline_alignment_wall_ns_.load();
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  return localizationHeartbeatStatus(offline_alignment_ready_.load(), received, now);
}

bool Astar3DPlanner::liveObstaclesReady() const
{
  if (!use_static_map_) {return true;}
  const auto node = node_.lock();
  if (!node || !map_) {return false;}
  const auto read = map_->lockForReading();
  return map_->readyUnlocked() && active_.load() && alignmentReady() &&
         live_obstacle_health_.statusNow([&node]() {return node->now().nanoseconds();}).ready;
}

void Astar3DPlanner::publishLiveObstacleHealth()
{
  const auto node = node_.lock();
  if (!node || !live_obstacle_status_pub_) {return;}
  bool map_ready = false;
  if (map_) {
    const auto read = map_->lockForReading();
    map_ready = map_->readyUnlocked();
  }
  const auto status = live_obstacle_health_.statusNow([&node]() {return node->now().nanoseconds();});
  const auto alignment = alignmentHeartbeatStatus();
  // The ready bit and timestamp must describe the same consumption snapshot.
  const bool ready = status.ready && map_ready && active_.load() && alignment.ready &&
    (!fixed_map_guard_ || fixed_map_guard_->ready());
  const char * reason = status.reason;
  if (!active_.load()) {reason = "planner_inactive";}
  else if (!alignment.ready) {reason = alignment.reason;}
  else if (fixed_map_guard_ && !fixed_map_guard_->ready()) {reason = "fixed_map_not_verified";}
  else if (!ready && status.ready) {reason = "waiting_for_map_or_fresh_input";}
  std_msgs::msg::String message;
  message.data = std::string("{\"ready\":") + (ready ? "true" : "false") +
    ",\"reason\":\"" + reason + "\",\"consumed_stamp_ns\":" + std::to_string(status.stamp_ns) +
    ",\"processed_age_s\":" + std::to_string(
    status.processed_steady_ns > 0 ?
    (status.checked_steady_ns - status.processed_steady_ns) * 1e-9 : -1.0) +
    ",\"freshness_timeout_s\":" + std::to_string(live_obstacle_freshness_timeout_) +
    ",\"alignment_heartbeat_age_s\":" + std::to_string(alignment.age_s) +
    ",\"localization_heartbeat_timeout_s\":" +
    std::to_string(kLocalizationHeartbeatTimeoutNs * 1e-9) +
    ",\"successful_updates\":" + std::to_string(status.updates) + "}";
  live_obstacle_status_pub_->publish(message);
}

}  // namespace astar_3d_planner

PLUGINLIB_EXPORT_CLASS(
  astar_3d_planner::Astar3DPlanner, nav2_core::GlobalPlanner)
