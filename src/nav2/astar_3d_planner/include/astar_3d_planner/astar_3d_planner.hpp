#include "terrain_navigation_interfaces/srv/validate_terrain_path.hpp"
#ifndef ASTAR_3D_PLANNER__ASTAR_3D_PLANNER_HPP_
#define ASTAR_3D_PLANNER__ASTAR_3D_PLANNER_HPP_

#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"

#include "astar_3d_planner/astar_3d.hpp"
#include "astar_3d_planner/occupy_map.hpp"
#include "astar_3d_planner/swept_footprint.hpp"
#include "astar_3d_planner/fixed_map_guard.hpp"
#include "astar_3d_planner/live_obstacle_health.hpp"
#include "astar_3d_planner/local_detour_budget.hpp"

namespace astar_3d_planner
{

// Both offline readiness gates share one steady-clock heartbeat budget. This
// bounds participant/callback loss independently of localization source age.
inline constexpr std::int64_t kLocalizationHeartbeatTimeoutNs = 1000000000LL;

struct LocalizationHeartbeatStatus
{
  bool ready{false};
  const char * reason{"localization_heartbeat_missing"};
  double age_s{-1.0};
};

LocalizationHeartbeatStatus localizationHeartbeatStatus(
  bool available, std::int64_t received_ns, std::int64_t now_ns);

// Offline global input uses source time with its own configured deadline.
// The fixed map is immutable and deliberately does not use this age limit.
bool liveObstacleStampFresh(
  const builtin_interfaces::msg::Time & stamp, std::int64_t now_ns, double maximum_age);

bool selectStablePathSuffix(
  const nav_msgs::msg::Path & cached_path,
  double robot_x, double robot_y,
  double maximum_distance, double anchor_behind_distance,
  double & progress_distance,
  nav_msgs::msg::Path & output_path,
  geometry_msgs::msg::PoseStamped * projection_out = nullptr);

struct PathReconnectDiagnostics
{
  std::size_t attempts{0};
  double forward_distance{0.0};
  bool used_forward{false};
};

// Operates after stable projection; lookahead never changes the progress cursor.
// Every accepted candidate includes the actual start and the entire old tail.
bool reconnectPathSuffix(
  const nav_msgs::msg::Path & projected_suffix,
  const geometry_msgs::msg::PoseStamped & projection,
  double maximum_forward_distance,
  const std::function<bool(nav_msgs::msg::Path &)> & validate,
  nav_msgs::msg::Path & output,
  PathReconnectDiagnostics * diagnostics = nullptr);

// Explicit-start planning requests retain their start. Only a request which
// matched the robot at search entry may be advanced to its latest pose.
bool planningStartMatchesRobot(
  const geometry_msgs::msg::PoseStamped & requested,
  const geometry_msgs::msg::PoseStamped & robot,
  double xy_tolerance = 0.10, double yaw_tolerance = 0.20);

// Trim progress made while search was running, then validate a complete new
// connection and the retained tail. This does not remove legitimate rear goals.
bool reanchorPublishedPath(
  const nav_msgs::msg::Path & planned_path,
  const geometry_msgs::msg::PoseStamped & robot,
  double maximum_distance, double maximum_forward_distance,
  const std::function<bool(nav_msgs::msg::Path &)> & validate,
  nav_msgs::msg::Path & output);

// Terrain publishes support and obstacles for one measurement stamp. Retain
// bounded queues so independent callback ordering cannot starve their pairing.
// The caller holds pending_mutex_; this buffer does not own another lock.
class TerrainInputBuffer
{
public:
  using Cloud = sensor_msgs::msg::PointCloud2::SharedPtr;
  void pushSupport(const Cloud & cloud);
  void pushObstacles(const Cloud & cloud);
  bool hasPair() const;
  bool popPair(Cloud & support, Cloud & obstacles);
  void clear();
  std::size_t supportSize() const {return support_.size();}
  std::size_t obstacleSize() const {return obstacles_.size();}

private:
  using Queue = std::map<std::int64_t, Cloud>;
  void push(Queue & queue, const Cloud & cloud);
  bool pairStamp(std::int64_t & stamp) const;
  Queue support_;
  Queue obstacles_;
  std::int64_t last_pair_stamp_{-1};
};

class Astar3DPlanner : public nav2_core::GlobalPlanner
{
public:
  Astar3DPlanner() = default;
  ~Astar3DPlanner() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  nav_msgs::msg::Path computePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, const OccupyMap & planning_map,
    const SweptFootprint * checker, VoxelKey & checked_start);
  void supportCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void obstacleCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void lioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void terrainPathValidityCallback(
    const nav2_msgs::srv::IsPathValid::Request::SharedPtr request,
    nav2_msgs::srv::IsPathValid::Response::SharedPtr response);
  void detailedTerrainPathValidityCallback(
    const terrain_navigation_interfaces::srv::ValidateTerrainPath::Request::SharedPtr request,
    terrain_navigation_interfaces::srv::ValidateTerrainPath::Response::SharedPtr response);
  void referenceTerrainPathValidityCallback(
    const terrain_navigation_interfaces::srv::ValidateTerrainPath::Request::SharedPtr request,
    terrain_navigation_interfaces::srv::ValidateTerrainPath::Response::SharedPtr response);
  void evaluateTerrainPath(const nav_msgs::msg::Path & path,
    terrain_navigation_interfaces::srv::ValidateTerrainPath::Response & result,
    bool reference_path = false);
  void validateTerrainPathGeometry(
    const nav_msgs::msg::Path & original_path, nav_msgs::msg::Path remaining_path,
    const geometry_msgs::msg::PoseStamped & projection,
    const geometry_msgs::msg::TransformStamped & robot_transform,
    const OccupyMap & query_map,
    terrain_navigation_interfaces::srv::ValidateTerrainPath::Response & result,
    bool report_failure);
  void mapWorker();
  void publishSupportSnapshot(bool force = false);
  // Called with the map read lock; never takes support_snapshot_mutex_.
  void publishGroundQueries(const OccupyMap * query_map = nullptr);
  void publishDirectDiagnostic(const Astar3D::TerrainValidation & failure,
    const OccupyMap * query_map = nullptr);
  std::vector<OccupyMap::Point> transformCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    std::vector<std::uint8_t> * obstacle_sources = nullptr,
    std::vector<std::int64_t> * observed_stamps_ns = nullptr,
    bool fixed_layer = false) const;
  void clearStartFailure();
  void publishStartFailure(
    const OccupyMap::Point & start, const Astar3D::ProjectionDiagnostics & diagnostics,
    const Astar3D::TerrainValidation * direct_failure = nullptr,
    const Astar3D::SearchDiagnostics * search = nullptr,
    const OccupyMap::Point * projected_start = nullptr,
    const OccupyMap * query_map = nullptr);
  bool obstacleObserverInGlobalFrame(OccupyMap::Point & observer) const;
  bool tryReuseCachedPath(
    const Astar3D & astar, const VoxelKey & start, const VoxelKey & goal,
    double exact_start_x, double exact_start_y,
    double exact_goal_x, double exact_goal_y,
    const geometry_msgs::msg::Quaternion & start_orientation,
    const geometry_msgs::msg::Quaternion & goal_orientation,
    bool allow_goal_obstacle_grace,
    double * goal_valid_age,
    nav_msgs::msg::Path & path, const SweptFootprint * checker = nullptr,
    const OccupyMap * query_map = nullptr);
  bool refreshAndValidateTerrainPath(
    const Astar3D & astar, const VoxelKey & start,
    nav_msgs::msg::Path & path,
    Astar3D::TerrainValidation * failure = nullptr) const;
  bool reconnectAndValidateSuffix(
    const Astar3D & astar, const VoxelKey & start,
    const geometry_msgs::msg::PoseStamped & projection,
    const geometry_msgs::msg::Quaternion & start_orientation,
    nav_msgs::msg::Path & path,
    Astar3D::TerrainValidation * terrain_failure = nullptr,
    SweptFootprint::Failure * footprint_failure = nullptr,
    bool report_failure = false, const SweptFootprint * checker = nullptr,
    const OccupyMap * query_map = nullptr);
  void reportFootprintFailure(
    const nav_msgs::msg::Path & path, const SweptFootprint::Failure & failure,
    const OccupyMap * query_map = nullptr);
  void cachePath(
    const VoxelKey & goal, double exact_goal_x, double exact_goal_y,
    const nav_msgs::msg::Path & output_path, const VoxelKey * verified_start = nullptr);
  void clearPathCache();
  double startHeightInGlobalFrame(
    const geometry_msgs::msg::PoseStamped & nav2_start) const;
  void stopWorker();
  bool localizationReady() const;
  bool alignmentReady() const;
  LocalizationHeartbeatStatus alignmentHeartbeatStatus() const;
  bool liveObstaclesReady() const;
  void publishLiveObstacleHealth();
  void auditFixedMap();
  bool validateFootprintPath(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Quaternion & start_orientation,
    const geometry_msgs::msg::Quaternion & goal_orientation,
    SweptFootprint::Failure * failure = nullptr, bool report_failure = true,
    const SweptFootprint * checker = nullptr, const OccupyMap * query_map = nullptr);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string support_topic_;
  std::string obstacle_topic_;
  std::string static_obstacle_topic_;
  bool use_static_map_{false};
  bool verify_static_map_identity_{false};
  std::string static_map_id_topic_;
  std::unique_ptr<FixedMapGuard> fixed_map_guard_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr static_map_id_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_status_publisher_;
  rclcpp::TimerBase::SharedPtr map_audit_timer_;
  std::string last_map_guard_reason_;
  std::atomic_bool offline_localization_ready_{false};
  std::atomic<std::int64_t> offline_ready_wall_ns_{0};
  std::atomic_bool offline_alignment_ready_{false};
  std::atomic<std::int64_t> offline_alignment_wall_ns_{0};
  LiveObstacleHealth live_obstacle_health_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alignment_ready_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr live_obstacle_status_pub_;
  rclcpp::TimerBase::SharedPtr live_obstacle_status_timer_;
  std::string lio_odom_topic_;
  std::string path_validity_service_;
  bool use_lio_start_z_{true};
  bool goal_z_zero_is_unspecified_{true};
  double lio_height_offset_{0.0};
  double path_pose_spacing_{0.05};
  bool reuse_previous_path_{true};
  double path_reuse_start_tolerance_{0.75};
  double path_reuse_anchor_behind_{0.0};
  double cached_goal_obstacle_grace_period_{5.0};
  double cached_goal_obstacle_ignore_radius_{0.4};
  double path_validity_start_tolerance_{0.75};
  double path_validity_ignore_near_robot_radius_{0.6};
  double path_validity_min_blocked_length_{0.25};
  double local_detour_max_deviation_{0.75};
  LocalDetourBudget::Config local_detour_budget_config_;
  LocalDetourBudget local_detour_budget_;
  int point_stride_{1};
  std::atomic_bool active_{false};

  std::unique_ptr<OccupyMap> map_;
  Astar3D::Config astar_config_;
  bool heading_aware_search_{true};
  bool path_refinement_enabled_{true};
  double turn_cost_weight_{0.15};
  double live_obstacle_max_observation_age_{0.6};
  double live_obstacle_freshness_timeout_{2.0};
  bool footprint_check_enabled_{true};
  SweptFootprint::Config footprint_config_;
  std::unique_ptr<SweptFootprint> footprint_checker_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr support_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr static_obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_ready_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    support_map_publisher_;
  // Serialize publication with lifecycle transitions. Never hold the map lock
  // while taking this mutex; the worker calls after finishing its update.
  std::mutex support_snapshot_mutex_;
  std::chrono::steady_clock::time_point last_support_map_publish_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
    start_failure_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    start_failure_markers_;
  std::mutex ground_diagnostics_mutex_;
  std::chrono::steady_clock::time_point last_ground_query_publish_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    ground_query_markers_, direct_path_markers_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr direct_path_status_;
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr
    terrain_path_validity_service_;
  rclcpp::Service<terrain_navigation_interfaces::srv::ValidateTerrainPath>::SharedPtr
    detailed_terrain_path_validity_service_;
  rclcpp::Service<terrain_navigation_interfaces::srv::ValidateTerrainPath>::SharedPtr
    reference_terrain_path_validity_service_;

  mutable std::mutex lio_mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_lio_odom_;

  std::mutex path_cache_mutex_;
  bool cached_path_valid_{false};
  VoxelKey cached_goal_key_;
  double cached_exact_goal_x_{0.0};
  double cached_exact_goal_y_{0.0};
  double cached_progress_distance_{0.0};
  nav_msgs::msg::Path cached_output_path_;
  bool cached_start_valid_{false};
  VoxelKey cached_start_key_;
  std::chrono::steady_clock::time_point cached_goal_last_valid_;

  std::mutex path_validity_mutex_;
  std::optional<std::array<double, 4>> reference_goal_;
  bool validity_path_initialized_{false};
  std::int64_t validity_path_stamp_ns_{0};
  std::size_t validity_path_size_{0};
  double validity_path_first_x_{0.0};
  double validity_path_first_y_{0.0};
  double validity_path_last_x_{0.0};
  double validity_path_last_y_{0.0};
  double validity_progress_distance_{0.0};

  std::mutex pending_mutex_;
  std::condition_variable pending_cv_;
  sensor_msgs::msg::PointCloud2::SharedPtr pending_support_cloud_;
  sensor_msgs::msg::PointCloud2::SharedPtr pending_obstacle_cloud_;
  sensor_msgs::msg::PointCloud2::SharedPtr pending_static_obstacle_cloud_;
  TerrainInputBuffer pending_terrain_pairs_;
  bool stop_worker_{false};
  std::thread map_worker_;
};

}  // namespace astar_3d_planner

#endif  // ASTAR_3D_PLANNER__ASTAR_3D_PLANNER_HPP_
