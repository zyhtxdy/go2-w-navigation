#include "astar_3d_planner/astar_3d_planner.hpp"

#include <iomanip>
#include <map>
#include <locale>
#include <sstream>

namespace astar_3d_planner
{
namespace
{
const char * sourceName(std::uint8_t sources)
{
  if (sources == OccupyMap::kMeasured) {return "measured_return";}
  if (sources == OccupyMap::kSynthetic) {return "synthetic_terrain_marker";}
  if (sources == OccupyMap::kUnknown) {return "unknown";}
  if (sources == OccupyMap::kStaticMap) {return "static_map_hazard";}
  return "mixed_voxel";
}

void writePoint(std::ostream & stream, const OccupyMap::Point & point)
{
  stream << '[' << point[0] << ',' << point[1] << ',' << point[2] << ']';
}

void writeKey(std::ostream & stream, const VoxelKey & key)
{
  stream << '[' << key.x << ',' << key.y << ',' << key.z << ']';
}
}  // namespace

void Astar3DPlanner::clearStartFailure()
{
  if (!active_.load() || !start_failure_markers_ || !start_failure_publisher_) {return;}
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  start_failure_markers_->publish(markers);
  std_msgs::msg::String status;
  status.data = "{\"event\":\"cleared\"}";
  start_failure_publisher_->publish(status);
  {
    std::lock_guard<std::mutex> lock(ground_diagnostics_mutex_);
    if (direct_path_markers_) {direct_path_markers_->publish(markers);}
    if (direct_path_status_) {direct_path_status_->publish(status);}
  }
}

void Astar3DPlanner::publishStartFailure(
  const OccupyMap::Point & start, const Astar3D::ProjectionDiagnostics & diagnostics,
  const Astar3D::TerrainValidation * direct_failure,
  const Astar3D::SearchDiagnostics * search,
  const OccupyMap::Point * projected_start, const OccupyMap * query_map)
{
  const auto * map = query_map ? query_map : map_.get();
  const auto node = node_.lock();
  if (!node || !map || !active_.load()) {return;}
  // The caller holds the live read lock or owns an immutable planning snapshot.
  OccupyMap::ObstacleEvidence evidence;
  const bool has_evidence = diagnostics.has_collision &&
    map->obstacleEvidenceUnlocked(diagnostics.collision, evidence);
  const auto & surface = diagnostics.surface;
  std::ostringstream json;
  json.imbue(std::locale::classic()); json << std::setprecision(17);
  json << "{\"event\":\"planning_failed\",\"stage\":" << std::quoted(diagnostics.stage)
       << ",\"node\":" << std::quoted(
    std::string(node->get_namespace()) +
    (std::string(node->get_namespace()) == "/" ? "" : "/") + node->get_name())
       << ",\"reason\":"
       << std::quoted(diagnostics.reason) << ",\"frame_id\":" << std::quoted(global_frame_)
       << ",\"request_stamp_ns\":" << node->now().nanoseconds() << ",\"start\":";
  writePoint(json, start);
  json << ",\"support_stamp_ns\":" << map->supportStampUnlocked()
       << ",\"obstacle_stamp_ns\":" << map->obstacleStampUnlocked()
       << ",\"height_datum_offset\":" << map->heightDatumOffsetUnlocked()
       << ",\"resolution\":" << map->resolution()
       << ",\"inflation_cells\":" << map->inflationCells()
       << ",\"prefilter_mode\":" << std::quoted(
    map->rectangularPrefilter() ? "rectangle_necessary" : "legacy_circle")
       << ",\"hard_prefilter_radius_m\":" << map->hardPrefilterRadius()
       << ",\"footprint_check_enabled\":" << (footprint_check_enabled_ ? "true" : "false")
       << ",\"footprint_bounds\":[" << footprint_config_.min_x << ',' << footprint_config_.max_x
       << ',' << footprint_config_.min_y << ',' << footprint_config_.max_y << ']'
       << ",\"synthetic_clearance_radius\":" << map->syntheticClearanceRadius()
       << ",\"robot_height_cells\":"
       << Astar3D(*map, astar_config_).robotHeightCells()
       << ",\"support_columns\":" << surface.columns
       << ",\"support_layers\":" << surface.layers
       << ",\"outside_height_layers\":" << surface.outside_height
       << ",\"occupied_candidates\":" << surface.occupied
       << ",\"safe_candidates\":" << surface.accepted
       << ",\"bridge_rejected\":" << diagnostics.bridge_rejected
       << ",\"footprint_bridges_rejected\":" << diagnostics.footprint_rejected
       << ",\"collision\":";
  if (diagnostics.has_collision) {
    json << "{\"key\":"; writeKey(json, diagnostics.collision);
    json << ",\"cell_center\":"; writePoint(json, map->keyToPos(diagnostics.collision));
    json << ",\"tested_sample\":"; writePoint(json, diagnostics.sample);
    json << '}';
  } else {json << "null";}
  json << ",\"surface_candidates\":[";
  bool first_candidate = true;
  for (const auto & candidate : surface.candidates) {
    if (!first_candidate) {json << ',';}
    first_candidate = false;
    json << "{\"ground_key\":"; writeKey(json, candidate.ground);
    double height = 0.0;
    const bool has_height = map->supportHeightUnlocked(candidate.ground, height);
    json << ",\"ground_height\":";
    if (has_height) {json << height;} else {json << "null";}
    json << ",\"reason\":" << std::quoted(candidate.reason) << ",\"collision_key\":";
    if (candidate.has_collision) {writeKey(json, candidate.collision);} else {json << "null";}
    json << ",\"support_source\":" << std::quoted(
      map->inferredSupportUnlocked(candidate.ground) ? "inferred" : "measured") << '}';
  }
  json << "],\"surface_candidates_omitted\":" << surface.omitted_candidates;
  json << ",\"obstacle\":";
  if (has_evidence) {
    json << "{\"source_topic\":" << std::quoted(
      evidence.sources == OccupyMap::kStaticMap ? static_obstacle_topic_ : obstacle_topic_)
         << ",\"classification\":" << std::quoted(sourceName(evidence.sources))
         << ",\"source_mask\":" << static_cast<int>(evidence.sources)
         << ",\"source_cloud_stamp_ns\":" << evidence.stamp_ns
         << ",\"source_observation_stamp_ns\":" << evidence.observed_stamp_ns
         << ",\"key\":"; writeKey(json, evidence.key);
    json << ",\"point_current_datum\":"; writePoint(json, evidence.point);
    json << ",\"point_observed_z\":" << evidence.observed_z << '}';
  } else {json << "null";}
  json << ",\"direct_reject\":";
  if (direct_failure) {
    json << "{\"reason\":" << std::quoted(direct_failure->reason)
         << ",\"index\":" << direct_failure->index << ",\"position\":";
    writePoint(json, direct_failure->position);
    json << ",\"support_edge\":";
    if (direct_failure->has_support_edge) {
      json << "{\"from_key\":"; writeKey(json, direct_failure->previous_support);
      json << ",\"to_key\":"; writeKey(json, direct_failure->current_support);
      json << ",\"from_height\":" << direct_failure->previous_height
           << ",\"to_height\":" << direct_failure->current_height
           << ",\"horizontal_distance\":" << direct_failure->horizontal_distance
           << ",\"max_slope_degrees\":" << astar_config_.max_slope_deg
           << ",\"max_step_height\":" << astar_config_.max_step_height << '}';
    } else {json << "null";}
    json << '}';
  } else {json << "null";}
  json << ",\"search\":";
  if (search) {
    json << "{\"expanded\":" << search->expanded
         << ",\"outside_connection_radius\":" << search->outside_connection_radius
         << ",\"height_rejected\":" << search->height_rejected
         << ",\"segment_rejected\":" << search->segment_rejected
         << ",\"motion_rejected\":" << search->motion_rejected
         << ",\"terminal_rejected\":" << search->terminal_rejected
         << ",\"heuristic_expanded\":" << search->heuristic_expanded
         << ",\"connection_radius\":" << astar_config_.surface_connection_radius << '}';
  } else {json << "null";}
  json << ",\"projected_start\":";
  if (projected_start) {writePoint(json, *projected_start);} else {json << "null";}
  json << ",\"collision_context\":"
       << std::quoted(std::string(diagnostics.stage) == "goal_footprint" ?
      "terminal_rectangular_footprint" : (std::string(diagnostics.stage) == "footprint" ?
      "rectangular_footprint" : (direct_failure ? "direct_candidate_only" : "start_projection")))
       << '}';
  std_msgs::msg::String status;
  status.data = json.str();
  if (start_failure_publisher_) {start_failure_publisher_->publish(status);}
  RCLCPP_WARN(node->get_logger(), "%s start diagnostic: %s", name_.c_str(), status.data.c_str());

  if (!start_failure_markers_) {return;}
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  int next_id = 0;
  const auto marker = [&](int type, const OccupyMap::Point & point) {
      visualization_msgs::msg::Marker result;
      result.header.frame_id = global_frame_; result.header.stamp = node->now();
      result.ns = name_ + "/start_failure"; result.id = next_id++;
      result.type = type; result.action = visualization_msgs::msg::Marker::ADD;
      result.pose.position.x = point[0]; result.pose.position.y = point[1];
      result.pose.position.z = point[2]; result.pose.orientation.w = 1.0;
      result.scale.x = result.scale.y = result.scale.z = 0.12;
      result.color.a = 1.0;
      return result;
    };
  auto start_marker = marker(visualization_msgs::msg::Marker::SPHERE, start);
  start_marker.color.b = 1.0; start_marker.color.g = 0.5;
  markers.markers.push_back(start_marker);
  if (projected_start) {
    auto selected = marker(visualization_msgs::msg::Marker::SPHERE, *projected_start);
    selected.color.g = 1.0; markers.markers.push_back(selected);
  }
  if (direct_failure) {
    auto rejected = marker(visualization_msgs::msg::Marker::SPHERE, direct_failure->position);
    rejected.color.r = 1.0; rejected.color.g = 1.0;
    markers.markers.push_back(rejected);
  }
  if (diagnostics.has_collision) {
    auto blocked = marker(
      visualization_msgs::msg::Marker::CUBE, map->keyToPos(diagnostics.collision));
    blocked.scale.x = blocked.scale.y = blocked.scale.z = map->resolution();
    blocked.color.r = 1.0;
    markers.markers.push_back(blocked);
  }
  if (has_evidence) {
    auto origin = marker(visualization_msgs::msg::Marker::SPHERE, evidence.point);
    origin.color.r = origin.color.b = 1.0;
    markers.markers.push_back(origin);
    auto inflated = marker(visualization_msgs::msg::Marker::CUBE_LIST, {0, 0, 0});
    inflated.scale.x = inflated.scale.y = inflated.scale.z = map->resolution();
    inflated.color.r = 1.0; inflated.color.g = 0.5; inflated.color.a = 0.25;
    for (const auto & point : map->inflationSnapshotUnlocked(evidence.key)) {
      geometry_msgs::msg::Point p;
      p.x = point[0]; p.y = point[1]; p.z = point[2]; inflated.points.push_back(p);
    }
    markers.markers.push_back(inflated);
  }
  auto label_position = start; label_position[2] += 1.0;
  auto label = marker(visualization_msgs::msg::Marker::TEXT_VIEW_FACING, label_position);
  label.scale.z = 0.12; label.color.r = label.color.g = label.color.b = 1.0;
  std::ostringstream text;
  text << diagnostics.reason << "\nsafe=" << surface.accepted
       << " occupied=" << surface.occupied << " bridge=" << diagnostics.bridge_rejected;
  if (has_evidence) {text << "\n" << sourceName(evidence.sources);}
  if (direct_failure) {text << "\ndirect: " << direct_failure->reason;}
  label.text = text.str(); markers.markers.push_back(label);
  start_failure_markers_->publish(markers);
}

void Astar3DPlanner::publishGroundQueries(const OccupyMap * query_map)
{
  const auto * map = query_map ? query_map : map_.get();
  std::lock_guard<std::mutex> lock(ground_diagnostics_mutex_);
  const auto node = node_.lock();
  const auto now = std::chrono::steady_clock::now();
  if (!node || !map || !active_.load() || !ground_query_markers_ ||
    now - last_ground_query_publish_ < std::chrono::seconds(1)) {return;}
  visualization_msgs::msg::MarkerArray output;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  output.markers.push_back(clear);
  std::map<std::string, visualization_msgs::msg::Marker> groups;
  for (const auto & query : map->groundQueriesUnlocked()) {
    const std::string category = query.inferred ? "inferred_ground" : query.reason;
    auto & marker = groups[category];
    marker.header.frame_id = global_frame_; marker.header.stamp = node->now();
    marker.ns = category; marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = map->resolution() * .8;
    marker.scale.z = .02;
    marker.color.r = query.inferred ? 0.0 : 1.0;
    marker.color.g = query.inferred ? .8 : .45;
    marker.color.b = query.inferred ? 1.0 : 0.0;
    marker.color.a = .75;
    geometry_msgs::msg::Point point;
    point.x = query.point[0]; point.y = query.point[1]; point.z = query.point[2];
    marker.points.push_back(point);
  }
  for (auto & group : groups) {output.markers.push_back(std::move(group.second));}
  ground_query_markers_->publish(output);
  last_ground_query_publish_ = now;
}

void Astar3DPlanner::publishDirectDiagnostic(const Astar3D::TerrainValidation & failure,
  const OccupyMap * query_map)
{
  const auto * map = query_map ? query_map : map_.get();
  std::lock_guard<std::mutex> lock(ground_diagnostics_mutex_);
  const auto node = node_.lock();
  if (!node || !map || !active_.load() || !direct_path_status_ || !direct_path_markers_) {return;}
  OccupyMap::ObstacleEvidence evidence;
  const bool has_evidence = failure.has_collision &&
    map->obstacleEvidenceUnlocked(failure.collision, evidence);
  std::ostringstream json;
  json.imbue(std::locale::classic()); json << std::setprecision(17);
  json << "{\"event\":\"direct_candidate_rejected\","
       << "\"scope\":\"first_rejected_goal_option\",\"reason\":"
       << std::quoted(failure.reason) << ",\"frame_id\":" << std::quoted(global_frame_)
       << ",\"stamp_ns\":" << node->now().nanoseconds() << ",\"position\":";
  writePoint(json, failure.position);
  json << ",\"collision_key\":";
  if (failure.has_collision) {writeKey(json, failure.collision);} else {json << "null";}
  json << ",\"obstacle_source\":";
  if (has_evidence) {json << std::quoted(sourceName(evidence.sources));} else {json << "null";}
  json << ",\"support_stamp_ns\":" << map->supportStampUnlocked()
       << ",\"obstacle_stamp_ns\":" << map->obstacleStampUnlocked()
       << ",\"has_support_edge\":" << (failure.has_support_edge ? "true" : "false")
       << ",\"previous_height\":" << failure.previous_height
       << ",\"current_height\":" << failure.current_height
       << ",\"horizontal_distance\":" << failure.horizontal_distance << '}';
  std_msgs::msg::String status; status.data = json.str();
  direct_path_status_->publish(status);
  RCLCPP_INFO(node->get_logger(), "%s direct diagnostic: %s", name_.c_str(), status.data.c_str());
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker point;
  point.header.frame_id = global_frame_; point.header.stamp = node->now();
  point.ns = "direct_rejection"; point.id = 0;
  point.type = visualization_msgs::msg::Marker::SPHERE;
  point.action = visualization_msgs::msg::Marker::ADD;
  point.pose.orientation.w = 1;
  point.pose.position.x = failure.position[0]; point.pose.position.y = failure.position[1];
  point.pose.position.z = failure.position[2];
  point.scale.x = point.scale.y = point.scale.z = .12;
  point.color.r = 1; point.color.g = 1; point.color.a = 1;
  markers.markers.push_back(point);
  point.id = 1; point.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  point.pose.position.z += .3; point.scale.z = .12;
  point.text = std::string("direct candidate: ") + failure.reason;
  markers.markers.push_back(point);
  direct_path_markers_->publish(markers);
}
}  // namespace astar_3d_planner
