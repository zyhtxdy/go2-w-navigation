#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <sstream>
#include <memory>
#include <thread>
#include <tuple>
#include <vector>

#include "astar_3d_planner/astar_3d_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rcl_interfaces/msg/log.hpp"

namespace astar_3d_planner
{
namespace
{
using namespace std::chrono_literals;

sensor_msgs::msg::PointCloud2 cloud(
  const std::vector<OccupyMap::Point> & points, const rclcpp::Time & stamp,
  const std::vector<uint8_t> & sources = {})
{
  sensor_msgs::msg::PointCloud2 result;
  result.header.frame_id = "map";
  result.header.stamp = stamp;
  sensor_msgs::PointCloud2Modifier modifier(result);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  if (points.empty()) {return result;}
  sensor_msgs::PointCloud2Iterator<float> x(result, "x"), y(result, "y"), z(result, "z");
  for (const auto & point : points) {
    *x = point[0]; *y = point[1]; *z = point[2]; ++x; ++y; ++z;
  }
  if (!sources.empty()) {
    if (sources.size() != points.size()) {throw std::invalid_argument("Source count mismatch");}
    sensor_msgs::msg::PointField source;
    source.name = "obstacle_source"; source.offset = 12;
    source.datatype = sensor_msgs::msg::PointField::UINT8; source.count = 1;
    result.fields.push_back(source);
    for (size_t i = 0; i < sources.size(); ++i) {result.data[i * result.point_step + 12] = sources[i];}
  }
  return result;
}

geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.position.z = .01;
  result.pose.orientation.w = 1.0;
  return result;
}

std::vector<OccupyMap::Point> savedPoints(const char * name,
  const std::string & directory = TEST_MAP_DIRECTORY)
{
  std::ifstream input(directory + "/" + name);
  std::vector<OccupyMap::Point> points;
  std::string line;
  std::getline(input, line);
  while (std::getline(input, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    OccupyMap::Point point;
    if (row >> point[0] >> point[1] >> point[2]) {points.push_back(point);}
  }
  return points;
}

// Test-only member access keeps production class layout/API unchanged while
// verifying that the public returned message retains its selected connector.
template<class Tag, typename Tag::type Member>
struct PlannerMemberAccess {friend typename Tag::type member(Tag) {return Member;}};
struct CachedPathTag {
  using type = nav_msgs::msg::Path Astar3DPlanner::*;
  friend type member(CachedPathTag);
};
struct CachedStartTag {
  using type = bool Astar3DPlanner::*;
  friend type member(CachedStartTag);
};
struct CacheMutexTag {
  using type = std::mutex Astar3DPlanner::*;
  friend type member(CacheMutexTag);
};
template struct PlannerMemberAccess<CachedPathTag, &Astar3DPlanner::cached_output_path_>;
template struct PlannerMemberAccess<CachedStartTag, &Astar3DPlanner::cached_start_valid_>;
template struct PlannerMemberAccess<CacheMutexTag, &Astar3DPlanner::path_cache_mutex_>;

// Exercise the real public planner, including callbacks, fixed-layer loading,
// exact-goal selection, search, terrain refresh, and rectangular final sweep.
class OfflineGoalSelection : public ::testing::Test
{
protected:
  virtual double footprintPadding() const {return 0.0;}
  virtual bool supportInference() const {return false;}
  virtual double supportInferenceRadius() const {return .25;}
  virtual bool fixedMap() const {return true;}
  virtual double mapResolution() const {return .1;}
  virtual double goalRadius() const {return .2;}
  virtual double obstacleWeight() const {return 2.0;}
  virtual bool headingSearch() const {return true;}
  virtual bool footprintEnabled() const {return true;}
  virtual bool reusePaths() const {return false;}
  virtual double startRefinementRadius() const {return 0.0;}
  virtual bool refineStaticExtents() const {return false;}
  virtual double measuredRefinementRadius() const {return 0.0;}
  virtual bool checkTurnSweeps() const {return true;}
  virtual int pointStride() const {return 1;}
  void SetUp() override
  {
    context_ = rclcpp::contexts::get_global_default_context();
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(147);
    context_->init(0, nullptr, init_options);
    rclcpp::NodeOptions options;
    options.context(context_);
    options.parameter_overrides({
      {"global_frame", "map"}, {"robot_base_frame", "base_link"},
      {"plugins", std::vector<std::string>{}}, {"filters", std::vector<std::string>{}},
      {"footprint_padding", footprintPadding()},
      {"footprint", "[[0.4,0.25],[0.4,-0.25],[-0.4,-0.25],[-0.4,0.25]]"}});
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    ASSERT_EQ(costmap_->on_configure(rclcpp_lifecycle::State()),
      nav2_util::CallbackReturn::SUCCESS);
    rclcpp::NodeOptions parent_options;
    parent_options.context(context_);
    parent_options.parameter_overrides({
      {"GridBased.use_static_map", fixedMap()}, {"GridBased.use_lio_start_z", false},
      {"GridBased.support_topic", "/offline_goal_test/support"},
      {"GridBased.static_obstacle_topic", "/offline_goal_test/static_obstacles"},
      {"GridBased.obstacle_topic", "/offline_goal_test/live_obstacles"},
      {"GridBased.resolution", mapResolution()}, {"GridBased.inflate_radius", .24},
      {"GridBased.point_stride", pointStride()},
      {"GridBased.obstacle_cost_weight", obstacleWeight()},
      {"GridBased.heading_aware_search", headingSearch()},
      {"GridBased.footprint_check_enabled", footprintEnabled()},
      {"GridBased.near_start_static_refinement_radius", startRefinementRadius()},
      {"GridBased.refine_static_obstacle_extents", refineStaticExtents()},
      {"GridBased.near_start_measured_refinement_radius", measuredRefinementRadius()},
      {"GridBased.check_in_place_turn_sweeps", checkTurnSweeps()},
      {"GridBased.support_inference_enabled", supportInference()},
      {"GridBased.support_inference_radius", supportInferenceRadius()},
      {"GridBased.surface_connection_radius", .3},
      {"GridBased.robot_height", .75}, {"GridBased.max_slope_deg", 10.0},
      {"GridBased.max_step_height", .08},
      {"GridBased.start_projection_xy_radius", 2.0},
      {"GridBased.retain_start_projection_bridge", fixedMap()},
      {"GridBased.goal_projection_xy_radius", goalRadius()},
      {"GridBased.reuse_previous_path", reusePaths()}});
    parent_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "offline_goal_selection_parent", parent_options);
    tf_ = std::make_shared<tf2_ros::Buffer>(parent_->get_clock());
    tf_->setUsingDedicatedThread(true);
    planner_ = std::make_unique<Astar3DPlanner>();
    planner_->configure(parent_, "GridBased", tf_, costmap_);
    planner_->activate();
    input_ = std::make_shared<rclcpp::Node>("offline_goal_selection_input",
      rclcpp::NodeOptions().context(context_));
    const auto fixed_qos = rclcpp::QoS(1).reliable().transient_local();
    support_pub_ = input_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/offline_goal_test/support", fixed_qos);
    obstacle_pub_ = input_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/offline_goal_test/static_obstacles", fixed_qos);
    ready_pub_ = input_->create_publisher<std_msgs::msg::Bool>(
      "/offline_localization/ready", fixed_qos);
    alignment_pub_ = input_->create_publisher<std_msgs::msg::Bool>(
      "/offline_localization/localization_available", fixed_qos);
    live_pub_ = input_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/offline_goal_test/live_obstacles", rclcpp::SensorDataQoS());
    observer_pub_ = input_->create_publisher<nav_msgs::msg::Odometry>("/Odometry", rclcpp::SensorDataQoS());
    ready_timer_ = input_->create_wall_timer(25ms, [this]() {
        std_msgs::msg::Bool ready; ready.data = true; ready_pub_->publish(ready);
        alignment_pub_->publish(ready);
        nav_msgs::msg::Odometry observer;
        observer.header.frame_id = "map"; observer.child_frame_id = "base_link";
        observer.header.stamp = input_->now(); observer.pose.pose.orientation.w = 1.0;
        observer_pub_->publish(observer);
        live_pub_->publish(cloud({}, input_->now()));
      });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(parent_->get_node_base_interface());
    executor_->add_node(input_);
    executor_thread_ = std::thread([this]() {executor_->spin();});
    for (int x = -10; x <= 30; ++x) {
      for (int y = -10; y <= 10; ++y) {
        floor_.push_back({x * .1 + .05, y * .1 + .05, .01});
      }
    }
  }

  void TearDown() override
  {
    if (executor_) {executor_->cancel();}
    if (executor_thread_.joinable()) {executor_thread_.join();}
    if (planner_) {planner_->deactivate(); planner_->cleanup();}
    planner_.reset();
    if (costmap_) {costmap_->on_cleanup(rclcpp_lifecycle::State());}
    costmap_.reset();
    context_->shutdown("test finished");
  }

  virtual std::vector<uint8_t> obstacleSources(const std::vector<OccupyMap::Point> & obstacles) const
  {return std::vector<uint8_t>(obstacles.size(), 1);}

  void expectReturnedPathCached(const nav_msgs::msg::Path & path)
  {
    std::lock_guard<std::mutex> lock(planner_.get()->*member(CacheMutexTag{}));
    EXPECT_EQ(planner_.get()->*member(CachedPathTag{}), path);
    EXPECT_TRUE(planner_.get()->*member(CachedStartTag{}));
  }

  void load(const std::vector<OccupyMap::Point> & obstacles)
  {
    const auto stamp = input_->now();
    support_pub_->publish(cloud(floor_, stamp));
    obstacle_pub_->publish(cloud(obstacles, stamp, obstacleSources(obstacles)));
    // Allow the public subscription and map worker to load the immutable map.
    std::this_thread::sleep_for(200ms);
  }

  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<rclcpp::Node> input_;
  std::unique_ptr<Astar3DPlanner> planner_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr support_pub_, obstacle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alignment_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr live_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr observer_pub_;
  rclcpp::TimerBase::SharedPtr ready_timer_;
  std::vector<OccupyMap::Point> floor_;
};

TEST_F(OfflineGoalSelection, DirectPlanRetainsWitnessForExactReturnedStampedMessage)
{
  load({});
  const auto before = input_->now();
  const auto path = planner_->createPlan(pose(.05,.05), pose(2.05,.05));
  ASSERT_GT(rclcpp::Time(path.header.stamp).nanoseconds(), before.nanoseconds());
  ASSERT_GE(path.poses.size(), 2u);
  expectReturnedPathCached(path);
  for (const auto & waypoint : path.poses) {EXPECT_EQ(waypoint.header, path.header);}
}

class FixedMapWithLiveStride : public OfflineGoalSelection
{
protected:
  int pointStride() const override {return 3;}
};
TEST_F(FixedMapWithLiveStride, FixedMapStillLoadsEverySignedObstacle)
{
  // A runtime point decimation setting must not discard a signed fixed hazard.
  load({{10.,10.,.25}, {.05,.05,.25}});
  EXPECT_THROW(planner_->createPlan(pose(.05,.05),pose(2.05,.05)), nav2_core::PlannerException);
}

TEST_F(OfflineGoalSelection, BlockedExactRectangleRetainsSafeNearbyGoalAndSweepsWholeRoute)
{
  // This return is outside the inflated centre circle, but inside the
  // forward-left footprint corner of the exact requested goal.
  const OccupyMap::Point obstacle{1.425, .275, .225};
  load({obstacle});
  const auto start = pose(.05, .05), goal = pose(1.05, .05);
  const auto path = planner_->createPlan(start, goal);
  ASSERT_GE(path.poses.size(), 2u);
  const auto & end = path.poses.back().pose.position;
  const double goal_offset = std::hypot(end.x - goal.pose.position.x,
    end.y - goal.pose.position.y);
  EXPECT_GT(goal_offset, .01);
  EXPECT_LE(goal_offset, .2 + 1e-6);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.orientation.w, 1.0);

  OccupyMap::Config config;
  config.resolution = .1; config.inflate_radius = .24;
  OccupyMap collision_map(config);
  collision_map.setStaticObstacles({obstacle});
  SweptFootprint checker(collision_map, SweptFootprint::Config{});
  std::vector<OccupyMap::Point> points;
  for (const auto & p : path.poses) {
    points.push_back({p.pose.position.x, p.pose.position.y, p.pose.position.z});
  }
  const auto lock = collision_map.lockForReading();
  ASSERT_FALSE(checker.poseCollisionFree({1.05, .05, .01}, 0));
  ASSERT_TRUE(checker.poseCollisionFree({end.x, end.y, end.z}, 0));
  EXPECT_TRUE(checker.validatePath(points, 0, 0));
}

TEST_F(OfflineGoalSelection, RejectsWhenEveryTerminalFootprintIsBlocked)
{
  // The long front edge spans this wall for all goals within 0.2 m. Some
  // candidate centres remain clear, so centre-only selection is insufficient.
  std::vector<OccupyMap::Point> wall;
  for (int y = -7; y <= 7; ++y) {wall.push_back({1.225, y * .1 + .025, .225});}
  load(wall);
  EXPECT_THROW(planner_->createPlan(pose(.05, .05), pose(1.05, .05)),
    nav2_core::PlannerException);
}

TEST_F(OfflineGoalSelection, RectangleUnsafeShortcutRetriesValidatedSearchedRoute)
{
  const std::vector<OccupyMap::Point> obstacles{
    {1.425, .725, .225}, {1.225, .325, .225},
    {2.525, -.175, .225}, {2.325, -.875, .225}};
  load(obstacles);
  const auto start = pose(.05, .05), goal = pose(2.05, .05);
  const auto path = planner_->createPlan(start, goal);
  ASSERT_GE(path.poses.size(), 2u);

  OccupyMap::Config config;
  config.resolution = .1; config.inflate_radius = .24;
  OccupyMap baseline_map(config);
  baseline_map.updateSupport(floor_);
  baseline_map.setStaticObstacles(obstacles);
  const auto lock = baseline_map.lockForReading();
  Astar3D::Config planner_config;
  planner_config.robot_height = .75;
  planner_config.max_slope_deg = 10;
  planner_config.max_step_height = .08;
  planner_config.surface_connection_radius = .3;
  Astar3D baseline(baseline_map, planner_config);
  VoxelKey start_key, goal_key;
  ASSERT_TRUE(baseline.projectToSurface(.05, .05, .01, .01, 1., start_key));
  ASSERT_TRUE(baseline.projectToSurface(2.05, .05, .01, .01, 1., goal_key));
  std::vector<VoxelKey> searched;
  ASSERT_TRUE(baseline.search(start_key, goal_key, searched));
  const auto ground_points = [&](const std::vector<VoxelKey> & keys) {
      std::vector<OccupyMap::Point> points;
      for (const auto & key : keys) {
        const auto p = baseline_map.keyToPos(key);
        points.push_back({p[0], p[1], .01});
      }
      return points;
    };
  const auto shortcut = baseline.densifyPath(baseline.simplifyGroundPath(
      ground_points(baseline.simplifyPath(searched)), 0), .05);
  const auto original = baseline.densifyPath(ground_points(searched), .05);
  SweptFootprint checker(baseline_map, SweptFootprint::Config{});
  // Same obstacle map: centre-based shortcut is unsafe, original search is
  // executable. The actual public planner must return a fully safe alternative.
  ASSERT_FALSE(checker.validatePath(shortcut, 0, 0));
  ASSERT_TRUE(checker.validatePath(original, 0, 0));
  std::vector<OccupyMap::Point> output;
  for (const auto & p : path.poses) {
    output.push_back({p.pose.position.x, p.pose.position.y, p.pose.position.z});
  }
  EXPECT_TRUE(checker.validatePath(output, 0, 0));
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, goal.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, goal.pose.position.y);
}

TEST_F(OfflineGoalSelection, SavedSparseMapRetainsSelectedStartConnector)
{
  floor_.clear();
  std::ifstream input(std::string(TEST_MAP_DIRECTORY) + "/support.csv");
  ASSERT_TRUE(input.is_open());
  std::string line;
  std::getline(input, line);
  while (std::getline(input, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    OccupyMap::Point point;
    ASSERT_TRUE(static_cast<bool>(row >> point[0] >> point[1] >> point[2]));
    floor_.push_back(point);
  }
  ASSERT_EQ(floor_.size(), 891u);
  // Replay the logged sparse near-start geometry in the real plugin. Only
  // support is loaded here: historical live obstacle clouds were not saved,
  // so this specifically verifies connector construction and final validation.
  load({});
  auto start = pose(3.7990988, -1.7080307), goal = pose(4.51, -.16);
  start.pose.position.z = -.36337194;
  goal.pose.position.z = -.38;
  const auto path = planner_->createPlan(start, goal);
  ASSERT_GE(path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, start.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.y, start.pose.position.y);

  expectReturnedPathCached(path);
  // A freshly published plan must also pass the actual Nav2 BT validity
  // service as the robot makes a small straight advance on that connector.
  auto client = input_->create_client<nav2_msgs::srv::IsPathValid>("/terrain/is_path_valid");
  ASSERT_TRUE(client->wait_for_service(1s));
  for (double progress : {0.0, .02}) {
    const auto & next = path.poses[1].pose.position;
    const double length = std::hypot(next.x - start.pose.position.x,
      next.y - start.pose.position.y);
    ASSERT_GT(length, 0.0);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.header.stamp = input_->now();
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = start.pose.position.x +
      progress * (next.x - start.pose.position.x) / length;
    transform.transform.translation.y = start.pose.position.y +
      progress * (next.y - start.pose.position.y) / length;
    transform.transform.translation.z = start.pose.position.z;
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_->setTransform(transform, "regression_test", true));
    auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
    request->path = path;
    auto response = client->async_send_request(request);
    ASSERT_EQ(response.wait_for(1s), std::future_status::ready);
    EXPECT_TRUE(response.get()->is_valid) << "connector progress " << progress;
  }
}

TEST_F(OfflineGoalSelection, LoggedOffCentreGoalFindsSupportWithinMetricTolerance)
{
  floor_.clear();
  for (int x = -5; x <= 10; ++x) {
    for (int y = -12; y <= 2; ++y) {
      floor_.push_back({x * .1 + .05, y * .1 + .05, -.34});
    }
  }
  floor_.push_back({.20, -1.40, -.34});
  load({});
  auto start = pose(.60, -.36), goal = pose(.18, -1.51);
  start.pose.position.z = goal.pose.position.z = -.34;
  const auto path = planner_->createPlan(start, goal);
  ASSERT_GE(path.poses.size(), 2u);
  const auto & end = path.poses.back().pose.position;
  EXPECT_LE(std::hypot(end.x - .18, end.y + 1.51), .20 + 1e-9);
}

TEST_F(OfflineGoalSelection, ClearExactGoalKeepsRequestedCoordinates)
{
  load({});
  const auto goal = pose(1.05, .05);
  const auto path = planner_->createPlan(pose(.05, .05), goal);
  ASSERT_GE(path.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, goal.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, goal.pose.position.y);
}

TEST_F(OfflineGoalSelection, LoggedSupportedStartDoesNotRequireTurnTowardVoxelCentre)
{
  floor_ = savedPoints("support.csv");
  const auto obstacles = savedPoints("obstacles.csv");
  ASSERT_EQ(floor_.size(), 891u);
  ASSERT_EQ(obstacles.size(), 982u);
  load(obstacles);
  auto start = pose(3.0182126074, -.1925728639);
  start.pose.position.z = -.390362829;
  const double start_yaw = .07072944;
  start.pose.orientation.z = std::sin(start_yaw / 2);
  start.pose.orientation.w = std::cos(start_yaw / 2);
  auto goal = pose(1.45606, -.703491);
  goal.pose.position.z = 0;
  goal.pose.orientation.z = std::sin(-.786121 / 2);
  goal.pose.orientation.w = std::cos(-.786121 / 2);
  EXPECT_NO_THROW({
    const auto path = planner_->createPlan(start, goal);
    EXPECT_GE(path.poses.size(), 2u);
  });
}

class PaddedOfflineGoalSelection : public OfflineGoalSelection
{
protected:
  double footprintPadding() const override {return .01;}
};

TEST_F(PaddedOfflineGoalSelection, LoggedNearWallTurnRetriesAndKeepsNavigationValidity)
{
  floor_ = savedPoints("support.csv");
  load(savedPoints("obstacles.csv"));
  auto start = pose(2.1556672867, .0059842000), goal = pose(1.45606, -.703491);
  start.pose.position.z = -.3719151914;
  const double yaw = -.0748548346;
  start.pose.orientation.z = std::sin(yaw / 2);
  start.pose.orientation.w = std::cos(yaw / 2);
  goal.pose.position.z = 0;
  goal.pose.orientation.z = std::sin(-.786121 / 2);
  goal.pose.orientation.w = std::cos(-.786121 / 2);
  const auto path = planner_->createPlan(start, goal);
  ASSERT_GE(path.poses.size(), 2u);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map"; transform.header.stamp = input_->now();
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = start.pose.position.x;
  transform.transform.translation.y = start.pose.position.y;
  transform.transform.translation.z = start.pose.position.z;
  transform.transform.rotation = start.pose.orientation;
  ASSERT_TRUE(tf_->setTransform(transform, "regression_test", true));
  auto client = input_->create_client<nav2_msgs::srv::IsPathValid>("/terrain/is_path_valid");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
  request->path = path;
  auto result = client->async_send_request(request);
  ASSERT_EQ(result.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(result.get()->is_valid);
}

TEST_F(OfflineGoalSelection, MissingGoalGroundPublishesStructuredCandidateEvidence)
{
  std::promise<std::string> diagnostic;
  bool received = false;
  auto subscription = input_->create_subscription<std_msgs::msg::String>(
    "/planner/start_failure", 10, [&](std_msgs::msg::String::ConstSharedPtr message) {
      if (!received && message->data.find("\"stage\":\"goal_projection\"") != std::string::npos) {
        received = true; diagnostic.set_value(message->data);
      }
    });
  auto future = diagnostic.get_future();
  load({});
  EXPECT_THROW(planner_->createPlan(pose(.05, .05), pose(20, 20)), nav2_core::PlannerException);
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  const auto text = future.get();
  EXPECT_NE(text.find("\"reason\":\"goal_no_support_xy\""), std::string::npos);
  EXPECT_NE(text.find("\"surface_candidates\":[]"), std::string::npos);
}
class OfflineGroundInference : public OfflineGoalSelection
{
protected:
  bool supportInference() const override {return true;}
};

TEST_F(OfflineGoalSelection, DisconnectedPreferredStartRetriesSafeConnectedComponent)
{
  floor_.clear();
  floor_.push_back({.45,.05,.01});  // Attractive, isolated start projection.
  for (int x=0;x<=20;++x) {floor_.push_back({x*.1+.05,.65,.01});}
  for (int y=0;y<=6;++y) {floor_.push_back({2.05,y*.1+.05,.01});}
  load({});
  const auto path=planner_->createPlan(pose(.05,.05),pose(2.05,.05));
  ASSERT_GE(path.poses.size(),2u);
  bool used_upper_component=false;
  for (const auto & p:path.poses) {used_upper_component |= p.pose.position.y>.5;}
  EXPECT_TRUE(used_upper_component);
  EXPECT_NEAR(path.poses.back().pose.position.x,2.05,1e-6);
  EXPECT_NEAR(path.poses.back().pose.position.y,.05,1e-6);
}

TEST_F(OfflineGoalSelection, DoesNotSpendRetryPoolInsideSameDisconnectedPatch)
{
  floor_.clear();
  // More than eight attractive candidates belong to the same dead component.
  for(int x=4;x<=8;++x) {
    for(int y=-1;y<=1;++y) {floor_.push_back({x*.1+.05,y*.1+.05,.01});}
  }
  for(int x=0;x<=20;++x) {floor_.push_back({x*.1+.05,.85,.01});}
  for(int y=0;y<=8;++y) {floor_.push_back({2.05,y*.1+.05,.01});}
  load({});
  const auto path=planner_->createPlan(pose(.05,.05),pose(2.05,.05));
  ASSERT_GE(path.poses.size(),2u);
  bool reached_component=false;
  for(const auto & p:path.poses) {reached_component |= p.pose.position.y>.7;}
  EXPECT_TRUE(reached_component);
}

TEST_F(OfflineGoalSelection, SupportedStartCannotSkipGroundGapByChangingProjection)
{
  floor_.clear(); floor_.push_back({.05,.05,.01});
  for(int x=10;x<=25;++x) {floor_.push_back({x*.1+.05,.05,.01});}
  load({});
  EXPECT_THROW(planner_->createPlan(pose(.05,.05),pose(2.05,.05)),nav2_core::PlannerException);
}

TEST_F(OfflineGoalSelection, AlternateStartCannotBridgeSolidWall)
{
  floor_.clear(); floor_.push_back({.45,.05,.01});
  for (int x=0;x<=20;++x) {floor_.push_back({x*.1+.05,.65,.01});}
  for (int y=0;y<=6;++y) {floor_.push_back({2.05,y*.1+.05,.01});}
  std::vector<OccupyMap::Point> wall;
  for(int y=-10;y<=10;++y) {wall.push_back({1.05,y*.1+.05,.21});}
  load(wall);
  EXPECT_THROW(planner_->createPlan(pose(.05,.05),pose(2.05,.05)),nav2_core::PlannerException);
}

TEST_F(OfflineGoalSelection, AlternateStartCannotJumpToElevatedFloor)
{
  floor_.clear(); floor_.push_back({.45,.05,.01});
  for (int x=0;x<=20;++x) {floor_.push_back({x*.1+.05,.65,.31});}
  for (int y=0;y<=6;++y) {floor_.push_back({2.05,y*.1+.05,.31});}
  load({}); auto goal=pose(2.05,.05); goal.pose.position.z=.31;
  EXPECT_THROW(planner_->createPlan(pose(.05,.05),goal),nav2_core::PlannerException);
}

class RectangularPrefilterPlanning : public OfflineGoalSelection
{
protected:
  double footprintPadding() const override {return .01;}

  void loadCorridor(double lower_wall=-.31)
  {
    floor_.clear();
    std::vector<OccupyMap::Point> walls;
    for(int x=-10;x<=30;++x) {
      floor_.push_back({x*.1+.01,.01,.01});
      for(int z=1;z<=7;++z) {
        walls.push_back({x*.1+.01,lower_wall,z*.1+.01});
        walls.push_back({x*.1+.01,.31,z*.1+.01});
      }
    }
    load(walls);
  }
};

TEST_F(RectangularPrefilterPlanning, ExactSubcellCorridorUsesCompleteRectangleInsteadOfRoundedCircle)
{
  loadCorridor();
  const auto start=pose(.01,.01),goal=pose(2.01,.01);
  const auto path=planner_->createPlan(start,goal);
  ASSERT_GE(path.poses.size(),2u);
  EXPECT_NEAR(path.poses.front().pose.position.x,.01,1e-6);
  EXPECT_NEAR(path.poses.back().pose.position.x,2.01,1e-6);
  for(const auto & p:path.poses) {EXPECT_NEAR(p.pose.position.y,.01,1e-6);}
  OccupyMap map(OccupyMap::Config{});
  std::vector<OccupyMap::Point> walls;
  for(int x=-10;x<=30;++x) {
    for(int z=1;z<=7;++z) {
      walls.push_back({x*.1+.01,-.31,z*.1+.01});
      walls.push_back({x*.1+.01,.31,z*.1+.01});
    }
  }
  map.setStaticObstacles(walls);
  const auto lock=map.lockForReading();
  SweptFootprint::Config config;config.min_x=-.41;config.max_x=.41;
  config.min_y=-.26;config.max_y=.26;
  SweptFootprint checker(map,config);
  std::vector<OccupyMap::Point> output;
  for(const auto & p:path.poses) {output.push_back({p.pose.position.x,p.pose.position.y,p.pose.position.z});}
  EXPECT_TRUE(checker.validatePath(output,0,0));
  // The grid-centred pose really collides. A relaxed prefilter is not a
  // declaration that every position in this candidate cell is safe.
  EXPECT_FALSE(checker.poseCollisionFree({.05,.05,.01},0));
}

TEST_F(RectangularPrefilterPlanning, PhysicallyNarrowerThanPaddedBodyIsStillRejected)
{
  loadCorridor(-.21);
  EXPECT_THROW(planner_->createPlan(pose(.01,.01),pose(2.01,.01)),nav2_core::PlannerException);
}

TEST_F(RectangularPrefilterPlanning, ClearTerminalPoseCannotBypassBlockedHalfTurn)
{
  loadCorridor(); auto goal=pose(2.01,.01);
  goal.pose.orientation.w=0;goal.pose.orientation.z=1;
  EXPECT_THROW(planner_->createPlan(pose(.01,.01),goal),nav2_core::PlannerException);
}

class LegacyCirclePlanning : public RectangularPrefilterPlanning
{
protected:
  bool footprintEnabled() const override {return false;}
};

TEST_F(LegacyCirclePlanning, DisabledFullFootprintKeepsLegacyCircleRejection)
{
  loadCorridor();
  EXPECT_THROW(planner_->createPlan(pose(.01,.01),pose(2.01,.01)),nav2_core::PlannerException);
}

class PathSuffixReconnection : public OfflineGoalSelection
{
protected:
  double mapResolution() const override {return .05;}
  double footprintPadding() const override {return .01;}
  bool reusePaths() const override {return true;}

  void loadStraightCorridor()
  {
    floor_.clear(); walls_.clear();
    for(int x=-20;x<=60;++x) {
      for(int y=-6;y<=6;++y) {floor_.push_back({x*.05+.025,y*.05+.025,.01});}
      walls_.push_back({x*.05+.025,-.375,.225});
      walls_.push_back({x*.05+.025,.375,.225});
    }
    load(walls_);
  }

  void robotAt(double x,double y,double yaw=0)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id="map";transform.child_frame_id="base_link";
    transform.header.stamp=input_->now();
    transform.transform.translation.x=x;transform.transform.translation.y=y;
    transform.transform.rotation.z=std::sin(yaw/2);transform.transform.rotation.w=std::cos(yaw/2);
    ASSERT_TRUE(tf_->setTransform(transform,"reconnect_test",true));
  }

  bool valid(const nav_msgs::msg::Path & path)
  {
    auto client=input_->create_client<nav2_msgs::srv::IsPathValid>("/terrain/is_path_valid");
    if(!client->wait_for_service(1s)) {ADD_FAILURE()<<"No path validity service";return false;}
    auto request=std::make_shared<nav2_msgs::srv::IsPathValid::Request>();request->path=path;
    auto future=client->async_send_request(request);
    if(future.wait_for(1s)!=std::future_status::ready) {ADD_FAILURE()<<"No validity response";return false;}
    return future.get()->is_valid;
  }

  terrain_navigation_interfaces::srv::ValidateTerrainPath::Response detailed(
    const nav_msgs::msg::Path & path)
  {
    using Service = terrain_navigation_interfaces::srv::ValidateTerrainPath;
    auto client=input_->create_client<Service>("/terrain/is_path_valid_detailed");
    Service::Response fallback; fallback.status=Service::Response::INTERNAL_ERROR;
    if(!client->wait_for_service(1s)) {ADD_FAILURE()<<"No detailed service";return fallback;}
    auto request=std::make_shared<Service::Request>(); request->path=path;
    auto future=client->async_send_request(request);
    if(future.wait_for(1s)!=std::future_status::ready) {ADD_FAILURE()<<"No detailed response";return fallback;}
    return *future.get();
  }

  std::vector<OccupyMap::Point> walls_;
};

using ValidityResult = terrain_navigation_interfaces::srv::ValidateTerrainPath::Response;

TEST_F(PathSuffixReconnection, DetailedValiditySeparatesValidAndRealCollision)
{
  loadStraightCorridor();
  auto path=planner_->createPlan(pose(.025,.025),pose(2.025,.025));
  robotAt(.325,.045);
  auto result=detailed(path); EXPECT_TRUE(result.is_valid); EXPECT_EQ(result.status,ValidityResult::VALID);
  EXPECT_EQ(valid(path),result.is_valid);
  robotAt(.325,.135);
  result=detailed(path); EXPECT_FALSE(result.is_valid); EXPECT_EQ(result.status,ValidityResult::COLLISION);
  EXPECT_FALSE(result.reason.empty()); EXPECT_EQ(valid(path),result.is_valid);
}

TEST_F(PathSuffixReconnection, DetailedValidityReportsMissingTransform)
{
  loadStraightCorridor();
  auto path=planner_->createPlan(pose(.025,.025),pose(2.025,.025));
  auto result=detailed(path); EXPECT_FALSE(result.is_valid); EXPECT_EQ(result.status,ValidityResult::TF_UNAVAILABLE);
}

TEST_F(PathSuffixReconnection, DetailedValidityReportsLocalizationUnavailable)
{
  loadStraightCorridor();
  auto path=planner_->createPlan(pose(.025,.025),pose(2.025,.025));
  ready_timer_->cancel(); std_msgs::msg::Bool missing; missing.data=false;
  ready_pub_->publish(missing); std::this_thread::sleep_for(80ms);
  auto result=detailed(path); EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.status,ValidityResult::LOCALIZATION_UNAVAILABLE);
}

TEST_F(PathSuffixReconnection, DetailedValidityReportsStaleObstacleSource)
{
  loadStraightCorridor();
  auto path=planner_->createPlan(pose(.025,.025),pose(2.025,.025));
  ready_timer_->cancel();
  ready_timer_=input_->create_wall_timer(25ms,[this]() {
    std_msgs::msg::Bool ready;ready.data=true;ready_pub_->publish(ready);alignment_pub_->publish(ready);
  });
  robotAt(.025,.025);
  std::this_thread::sleep_for(1150ms);
  auto result=detailed(path);
  EXPECT_TRUE(result.is_valid);  // Within the configured two-second global budget.
  std::this_thread::sleep_for(1000ms);
  result=detailed(path); EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.status,ValidityResult::OBSTACLES_UNAVAILABLE);
  EXPECT_NE(result.reason.find("stale"),std::string::npos);
}

TEST_F(PathSuffixReconnection, DetailedValidityReportsMapNotReady)
{
  std::this_thread::sleep_for(100ms);
  nav_msgs::msg::Path path;path.header.frame_id="map";path.poses={pose(0,0),pose(1,0)};
  auto result=detailed(path); EXPECT_FALSE(result.is_valid);EXPECT_EQ(result.status,ValidityResult::MAP_UNAVAILABLE);
}

TEST_F(PathSuffixReconnection, DetailedValidityReportsWrongFrame)
{
  loadStraightCorridor();
  auto path=planner_->createPlan(pose(.025,.025),pose(2.025,.025));path.header.frame_id="wrong_frame";
  auto result=detailed(path);EXPECT_FALSE(result.is_valid);EXPECT_EQ(result.status,ValidityResult::INVALID_REQUEST);
}

TEST_F(PathSuffixReconnection, ValidityAcceptsSafeForwardConnectionInsteadOfLateralTurn)
{
  loadStraightCorridor();
  const auto path=planner_->createPlan(pose(.025,.025),pose(2.025,.025));
  ASSERT_GE(path.poses.size(),2u);
  // Establish the previous false rejection with exactly the same raw walls.
  nav_msgs::msg::Path lateral;double progress=0;
  ASSERT_TRUE(selectStablePathSuffix(path,.325,.045,.3,0,progress,lateral));
  OccupyMap::Config config;config.resolution=.05;
  OccupyMap map(config);map.setStaticObstacles(walls_);
  const auto lock=map.lockForReading();
  SweptFootprint::Config body;body.min_x=-.41;body.max_x=.41;body.min_y=-.26;body.max_y=.26;
  SweptFootprint checker(map,body);
  std::vector<OccupyMap::Point> points;
  for(const auto & p:lateral.poses) {points.push_back({p.pose.position.x,p.pose.position.y,p.pose.position.z});}
  ASSERT_FALSE(checker.validatePath(points,0,0));
  robotAt(.325,.045);
  EXPECT_TRUE(valid(path));
  // Repeated checks of the unchanged physical pose must not consume lookahead.
  EXPECT_TRUE(valid(path));
}

TEST_F(PathSuffixReconnection, ValidityStillRejectsActualBodyCollision)
{
  loadStraightCorridor();
  const auto path=planner_->createPlan(pose(.025,.025),pose(2.025,.025));
  robotAt(.325,.135);
  EXPECT_FALSE(valid(path));
}

TEST_F(PathSuffixReconnection, ValidityChecksRetainedTailBeyondForwardConnector)
{
  loadStraightCorridor();
  const auto path=planner_->createPlan(pose(.025,.025),pose(2.025,.025));
  walls_.push_back({1.425,.025,.225});load(walls_);
  robotAt(.325,.045);
  EXPECT_FALSE(valid(path));
}

TEST_F(PathSuffixReconnection, CachedRouteUsesSafeForwardConnectionAfterLateralDrift)
{
  floor_.clear(); walls_.clear();
  for (int x=-10;x<=30;++x) {
    for (int y=-6;y<=20;++y) {floor_.push_back({x*.1+.025,y*.1+.025,.01});}
  }
  // A finite corridor opens into room for the required turn toward the goal.
  for (int x=-20;x<=20;++x) {
    walls_.push_back({x*.05+.025,-.425,.225});
    walls_.push_back({x*.05+.025,.425,.225});
  }
  load(walls_);
  const auto goal=pose(2.025,1.025);
  const auto original=planner_->createPlan(pose(.025,.025),goal);
  ASSERT_GE(original.poses.size(),3u);

  nav_msgs::msg::Path lateral;
  double progress=0;
  ASSERT_TRUE(selectStablePathSuffix(original,.325,.045,.3,0,progress,lateral));
  OccupyMap::Config config;config.resolution=.05;
  OccupyMap map(config);map.setStaticObstacles(walls_);
  const auto lock=map.lockForReading();
  SweptFootprint::Config body;
  body.min_x=-.41;body.max_x=.41;body.min_y=-.26;body.max_y=.26;
  SweptFootprint checker(map,body);
  const auto points=[](const nav_msgs::msg::Path & path) {
      std::vector<OccupyMap::Point> result;
      for(const auto & p:path.poses) {
        result.push_back({p.pose.position.x,p.pose.position.y,p.pose.position.z});
      }
      return result;
    };
  ASSERT_FALSE(checker.validatePath(points(lateral),0,0));
  ASSERT_FALSE(checker.validatePath({{.325,.045,.01},{2.025,1.025,.01}},0,0));

  auto reused_log=std::make_shared<std::promise<std::string>>();
  auto reused_future=reused_log->get_future();
  auto reported=std::make_shared<std::atomic<bool>>(false);
  auto subscription=input_->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout",rclcpp::QoS(1000).reliable().transient_local(),
    [reused_log,reported](rcl_interfaces::msg::Log::ConstSharedPtr message) {
      if(message->name=="offline_goal_selection_parent" &&
        message->msg.find("reused stable")!=std::string::npos && !reported->exchange(true))
      {reused_log->set_value(message->msg);}
    });
  auto updated_goal=goal;
  updated_goal.pose.orientation.z=std::sin(.1);
  updated_goal.pose.orientation.w=std::cos(.1);
  const auto reused=planner_->createPlan(pose(.325,.045),updated_goal);
  ASSERT_EQ(reused_future.wait_for(1s),std::future_status::ready);
  EXPECT_NE(reused_future.get().find("reused stable"),std::string::npos);
  ASSERT_GE(reused.poses.size(),3u);
  expectReturnedPathCached(reused);
  EXPECT_DOUBLE_EQ(reused.poses.front().pose.position.x,.325);
  EXPECT_DOUBLE_EQ(reused.poses.front().pose.position.y,.045);
  EXPECT_NEAR(reused.poses.back().pose.position.x,goal.pose.position.x,1e-6);
  EXPECT_NEAR(reused.poses.back().pose.position.y,goal.pose.position.y,1e-6);
  EXPECT_EQ(reused.poses.back().pose.orientation,updated_goal.pose.orientation);
  const auto & a=reused.poses[0].pose.position;
  const auto & b=reused.poses[1].pose.position;
  EXPECT_GT(b.x,a.x);
  EXPECT_LT(std::abs(std::atan2(b.y-a.y,b.x-a.x)),.1);
  EXPECT_TRUE(checker.validatePath(points(reused),0,.2));
  // Forward reconnection must retain every old vertex beyond the local join.
  for(const auto & old:original.poses) {
    if(old.pose.position.x<1.2) {continue;}
    bool found=false;
    for(const auto & p:reused.poses) {
      found |= std::hypot(p.pose.position.x-old.pose.position.x,
        p.pose.position.y-old.pose.position.y)<1e-6;
    }
    EXPECT_TRUE(found);
  }
}

class MultiGoalManeuver : public OfflineGoalSelection
{
protected:
  double mapResolution() const override {return .05;}
  double goalRadius() const override {return .48;}
  double obstacleWeight() const override {return 0.0;}

  void loadBentScene()
  {
    floor_.clear();
    for(int x=0;x<=12;++x) {floor_.push_back({.025+x*.1,.025,.01});}
    for(int y=1;y<=3;++y) {floor_.push_back({.825,.025+y*.1,.01});}
    for(int y=-6;y<0;++y) {floor_.push_back({.025,.025+y*.1,.01});}
    load({{1.125,.625,.21}});
  }

  void verifyOtherEndpoint()
  {
    loadBentScene();
    const auto goal=pose(1.265,.495);
    const auto path=planner_->createPlan(pose(.025,-.575),goal);
    ASSERT_GE(path.poses.size(),2u);
    const auto & end=path.poses.back().pose.position;
    EXPECT_NEAR(end.x,1.225,1e-5);
    EXPECT_NEAR(end.y,.025,1e-5);
    EXPECT_LE(std::hypot(end.x-goal.pose.position.x,end.y-goal.pose.position.y),.48+1e-6);
    EXPECT_DOUBLE_EQ(path.poses.back().pose.orientation.w,1.0);
    OccupyMap::Config config;
    config.resolution=.05; config.inflate_radius=.24;
    OccupyMap map(config); map.updateSupport(floor_); map.setStaticObstacles({{1.125,.625,.21}});
    const auto lock=map.lockForReading();
    SweptFootprint checker(map,SweptFootprint::Config{});
    std::vector<OccupyMap::Point> output;
    for(const auto & p:path.poses) {output.push_back({p.pose.position.x,p.pose.position.y,p.pose.position.z});}
    EXPECT_TRUE(checker.validatePath(output,0,0));
  }
};

TEST_F(MultiGoalManeuver, PublicPlannerKeepsOtherEndpointAfterFirstManeuverFails)
{
  verifyOtherEndpoint();
}

TEST_F(MultiGoalManeuver, FittingExactPoseDoesNotDiscardNearbyManeuverOptions)
{
  loadBentScene();
  // This exact endpoint has observed ground and fits the final orientation,
  // but the required intervening rectangle rotation intersects the obstacle.
  const auto requested=pose(.825,.325);
  const auto path=planner_->createPlan(pose(.025,-.575),requested);
  ASSERT_GE(path.poses.size(),2u);
  const auto & end=path.poses.back().pose.position;
  const double offset=std::hypot(end.x-requested.pose.position.x,end.y-requested.pose.position.y);
  EXPECT_GT(offset,.01);
  EXPECT_LE(offset,.48+1e-6);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.orientation.w,1.0);
  OccupyMap::Config config; config.resolution=.05; config.inflate_radius=.24;
  OccupyMap map(config); map.updateSupport(floor_); map.setStaticObstacles({{1.125,.625,.21}});
  const auto lock=map.lockForReading(); SweptFootprint checker(map,SweptFootprint::Config{});
  ASSERT_TRUE(checker.poseCollisionFree({.825,.325,.01},0));
  std::vector<OccupyMap::Point> output;
  for(const auto & p:path.poses) {output.push_back({p.pose.position.x,p.pose.position.y,p.pose.position.z});}
  EXPECT_TRUE(checker.validatePath(output,0,0));
}

class MultiGoalWithoutHeading : public MultiGoalManeuver
{
protected:
  bool headingSearch() const override {return false;}
};

TEST_F(MultiGoalWithoutHeading, EndpointRetryDoesNotDependOnHeadingFallback)
{
  verifyOtherEndpoint();
}

class OnlineStartRecovery : public OfflineGoalSelection
{
protected:
  bool fixedMap() const override {return false;}
};

TEST_F(OnlineStartRecovery, ConnectedStartRetryAlsoWorksWithoutOfflineBlindBridge)
{
  floor_.clear(); floor_.push_back({.45,.05,.01});
  for(int x=0;x<=20;++x) {floor_.push_back({x*.1+.05,.65,.01});}
  for(int y=0;y<=6;++y) {floor_.push_back({2.05,y*.1+.05,.01});}
  load({});
  const auto path=planner_->createPlan(pose(.05,.05),pose(2.05,.05));
  ASSERT_GE(path.poses.size(),2u);
  EXPECT_NEAR(path.poses.back().pose.position.x,2.05,1e-5);
  EXPECT_NEAR(path.poses.back().pose.position.y,.05,1e-5);
}

TEST_F(OfflineGroundInference, PublicPluginUsesSmallEnclosedHolesAndPublishesProvenance)
{
  // Sparse 20 cm terrain output projected onto the planner's 10 cm grid.
  floor_.clear();
  for (int x=-5;x<=15;++x) {
    for(int y=-5;y<=5;++y) {floor_.push_back({x*.2+.05,y*.2+.05,.01});}
  }
  std::promise<bool> shown;
  auto future=shown.get_future(); bool received=false;
  auto subscription=input_->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/planner/ground_queries",rclcpp::QoS(1).reliable().transient_local(),
    [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr message) {
      for(const auto & marker:message->markers) {
        if(!received && marker.ns=="inferred_ground" && !marker.points.empty()) {
          received=true; shown.set_value(true);
        }
      }
    });
  load({});
  const auto path=planner_->createPlan(pose(.05,.15),pose(1.05,.15));
  ASSERT_GE(path.poses.size(),2u);
  for(const auto & p:path.poses) {EXPECT_NEAR(p.pose.position.y,.15,1e-5);}
  ASSERT_EQ(future.wait_for(1s),std::future_status::ready);
  EXPECT_TRUE(future.get());
}

TEST_F(OfflineGroundInference, SuccessfulDetourRetainsDirectRejectionDiagnostic)
{
  std::promise<std::string> shown;
  auto future=shown.get_future(); bool received=false;
  auto subscription=input_->create_subscription<std_msgs::msg::String>(
    "/planner/direct_path_status",rclcpp::QoS(1).reliable().transient_local(),
    [&](std_msgs::msg::String::ConstSharedPtr message) {
      if(!received && message->data.find("direct_candidate_rejected")!=std::string::npos) {
        received=true; shown.set_value(message->data);
      }
    });
  // This real obstacle is in the direct corridor; it must remain present.
  load({{1.05,.05,.21}});
  const auto path=planner_->createPlan(pose(.05,.05),pose(2.05,.05));
  ASSERT_GE(path.poses.size(),2u);
  bool detoured=false;
  for(const auto & p:path.poses) {detoured |= std::abs(p.pose.position.y-.05)>.25;}
  EXPECT_TRUE(detoured);
  ASSERT_EQ(future.wait_for(1s),std::future_status::ready);
  const auto status=future.get();
  EXPECT_NE(status.find("occupied_body_column"),std::string::npos);
  EXPECT_NE(status.find("static_map_hazard"),std::string::npos);
}

TEST_F(OfflineGroundInference, LargeInteriorGapStillRejectsRoute)
{
  floor_.erase(std::remove_if(floor_.begin(),floor_.end(),[](const OccupyMap::Point & p) {
      return p[0]>.6 && p[0]<1.8;
    }),floor_.end());
  load({});
  EXPECT_THROW(planner_->createPlan(pose(.05,.05),pose(2.55,.05)),nav2_core::PlannerException);
}

// Full public-plugin comparison on the saved map, retaining all fixed hazards
// and the complete padded-rectangle sweep. Recorded start/goal XY are used;
// historical yaw/live obstacles were not saved, so yaw is an explicit fixture.
class SparseGroundRegression : public PaddedOfflineGoalSelection,
  public ::testing::WithParamInterface<bool>
{
protected:
  bool supportInference() const override {return GetParam();}
  double supportInferenceRadius() const override {return .4;}
};

TEST_P(SparseGroundRegression, LoggedGoalsProduceCollisionCheckedPaths)
{
  floor_ = savedPoints("support.csv", TEST_SPARSE_MAP_DIRECTORY);
  const auto obstacles = savedPoints("obstacles.csv", TEST_SPARSE_MAP_DIRECTORY);
  ASSERT_EQ(floor_.size(), 1300u);
  ASSERT_EQ(obstacles.size(), 1089u);
  load(obstacles);
  std::size_t index = 0;
  for (const auto & query : std::vector<std::array<double, 4>>{
      {2.07, -.20, 1.67, -.67}, {2.56, -.39, .04, .02},
      {2.07, -.20, 1.85, -.75}, {2.56, -.39, .05, -.15}})
  {
    auto start = pose(query[0], query[1]), goal = pose(query[2], query[3]);
    start.pose.position.z = -.388;
    goal.pose.position.z = 0;
    const auto begin = std::chrono::steady_clock::now();
    const auto path = planner_->createPlan(start, goal);
    ASSERT_GE(path.poses.size(), 2u);
    if (GetParam() || index>=2) {
      EXPECT_NEAR(path.poses.back().pose.position.x,query[2],1e-6);
      EXPECT_NEAR(path.poses.back().pose.position.y,query[3],1e-6);
    }
    double length = 0;
    std::size_t turns = 0;
    double previous_heading = 0;
    std::vector<OccupyMap::Point> points;
    for (std::size_t i = 0; i < path.poses.size(); ++i) {
      const auto & p = path.poses[i].pose.position;
      points.push_back({p.x, p.y, p.z});
      if (i == 0) {continue;}
      const auto & a = points[i-1];
      const double distance = std::hypot(p.x-a[0], p.y-a[1]);
      const double heading = std::atan2(p.y-a[1], p.x-a[0]);
      length += distance;
      if (i > 1 && std::abs(std::remainder(heading-previous_heading, 2*M_PI)) > .05) {++turns;}
      previous_heading = heading;
    }
    OccupyMap::Config config;
    config.inflate_radius = .24;
    OccupyMap collision_map(config);
    auto transported_obstacles = obstacles;
    for (auto & p : transported_obstacles) {
      for (auto & coordinate : p) {coordinate = static_cast<float>(coordinate);}
    }
    collision_map.setStaticObstacles(transported_obstacles);
    SweptFootprint::Config body;
    body.min_x = -.41; body.max_x = .41;
    body.min_y = -.26; body.max_y = .26;
    body.robot_height = .75;
    SweptFootprint sweep(collision_map, body);
    const auto read = collision_map.lockForReading();
    EXPECT_TRUE(sweep.validatePath(points, 0, 0));
    if (GetParam()) {
      const auto & a=points.front(); const auto & b=points.back();
      const double chord=std::hypot(b[0]-a[0],b[1]-a[1]);
      EXPECT_NEAR(length,chord,1e-5);
      for(const auto & p:points) {
        EXPECT_NEAR((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]),0.,1e-5);
      }
    }
    if (!GetParam()) {
      EXPECT_LT(length, index % 2 == 0 ? .8 : 3.2) << "Sparse-support baseline must not regain the long detour";
    }
    std::cout << "FROZEN_COMPARISON {\"query\":" << ++index
      << ",\"inference\":" << (GetParam() ? "true" : "false")
      << ",\"length_m\":" << length << ",\"turns\":" << turns
      << ",\"elapsed_ms\":" << std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now()-begin).count() << ",\"path\":[";
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (i) {std::cout << ',';}
      std::cout << '[' << points[i][0] << ',' << points[i][1] << ',' << points[i][2] << ']';
    }
    std::cout << "]}" << std::endl;
  }
}

INSTANTIATE_TEST_SUITE_P(FrozenMap, SparseGroundRegression, ::testing::Bool());

class InterpolatedScenario : public PaddedOfflineGoalSelection
{
protected:
  bool supportInference() const override {return true;}
  double supportInferenceRadius() const override {return .4;}
  double goalRadius() const override {return .02;}

  void sparseFloor()
  {
    floor_.clear();
    for(int x=-10;x<=20;++x) {for(int y=-10;y<=10;++y) {
      floor_.push_back({x*.2+.05,y*.2+.05,.01});
    }}
  }

  void checkCompleteSweep(const nav_msgs::msg::Path & path,
    std::vector<OccupyMap::Point> obstacles,double start_yaw,double goal_yaw)
  {
    ASSERT_GE(path.poses.size(),2u);
    for(auto & p:obstacles) {for(auto & v:p) {v=static_cast<float>(v);}}
    OccupyMap::Config c; c.inflate_radius=.24;
    OccupyMap collision(c); collision.setStaticObstacles(obstacles);
    SweptFootprint::Config body;
    body.min_x=-.41; body.max_x=.41; body.min_y=-.26; body.max_y=.26;
    SweptFootprint sweep(collision,body);
    std::vector<OccupyMap::Point> points;
    for(const auto & p:path.poses) {
      EXPECT_TRUE(std::isfinite(p.pose.position.x) && std::isfinite(p.pose.position.y));
      points.push_back({p.pose.position.x,p.pose.position.y,p.pose.position.z});
    }
    const auto read=collision.lockForReading();
    EXPECT_TRUE(sweep.validatePath(points,start_yaw,goal_yaw));
  }
};

class OpenHeadingMatrix : public InterpolatedScenario,
  public ::testing::WithParamInterface<std::tuple<double,double>> {};

TEST_P(OpenHeadingMatrix, OpenFlatRouteStaysStraightAndPreservesFinalYaw)
{
  sparseFloor(); load({});
  auto start=pose(.05,.05),goal=pose(2.05,.05);
  const double a=std::get<0>(GetParam()),b=std::get<1>(GetParam());
  RecordProperty("start_yaw_rad",std::to_string(a));
  RecordProperty("goal_yaw_rad",std::to_string(b));
  start.pose.orientation.z=std::sin(a/2); start.pose.orientation.w=std::cos(a/2);
  goal.pose.orientation.z=std::sin(b/2); goal.pose.orientation.w=std::cos(b/2);
  const auto path=planner_->createPlan(start,goal);
  ASSERT_GE(path.poses.size(),2u);
  EXPECT_NEAR(path.poses.front().pose.position.x,.05,1e-6);
  EXPECT_NEAR(path.poses.back().pose.position.x,2.05,1e-6);
  double length=0;
  for(std::size_t i=0;i<path.poses.size();++i) {
    const auto & p=path.poses[i].pose.position;
    EXPECT_NEAR(p.y,.05,1e-6);
    if(i) {
      const auto & previous=path.poses[i-1].pose.position;
      EXPECT_GE(p.x+1e-6,previous.x);
      length+=std::hypot(p.x-previous.x,p.y-previous.y);
    }
  }
  EXPECT_NEAR(length,2.,1e-5);
  RecordProperty("xy_length_m",std::to_string(length));
  EXPECT_NEAR(path.poses.back().pose.orientation.z,goal.pose.orientation.z,1e-6);
  EXPECT_NEAR(path.poses.back().pose.orientation.w,goal.pose.orientation.w,1e-6);
  checkCompleteSweep(path,{},a,b);
}

INSTANTIATE_TEST_SUITE_P(CardinalHeadings,OpenHeadingMatrix,::testing::Combine(
  ::testing::Values(0.,M_PI/2,M_PI,-M_PI/2),
  ::testing::Values(0.,M_PI/2,M_PI,-M_PI/2)));

class CorridorWidthMatrix : public InterpolatedScenario,
  public ::testing::WithParamInterface<std::tuple<int,double>> {};

TEST_P(CorridorWidthMatrix, RespectsPaddedBodyAndFinalRotation)
{
  sparseFloor();
  const int cells=std::get<0>(GetParam());
  const double goal_yaw=std::get<1>(GetParam());
  const double lower=-static_cast<double>(cells/2)*.1;
  const double upper=lower+cells*.1;
  const double centre=(lower+upper)/2;
  std::vector<OccupyMap::Point> walls;
  for(int x=-15;x<=35;++x) {for(int z=1;z<=7;++z) {
    walls.push_back({x*.1+.05,lower-.05,z*.1+.01});
    walls.push_back({x*.1+.05,upper+.05,z*.1+.01});
  }}
  load(walls);
  auto start=pose(.05,centre),goal=pose(2.05,centre);
  goal.pose.orientation.z=std::sin(goal_yaw/2); goal.pose.orientation.w=std::cos(goal_yaw/2);
  // The current padded body is 0.52m wide and 0.82m long. These are
  // software-model cases, not measured physical Go2W clearance.
  const bool possible=cells>=6 && (goal_yaw==0. || cells>=11);
  RecordProperty("model_clearance_m",std::to_string(cells*.1));
  RecordProperty("goal_yaw_rad",std::to_string(goal_yaw));
  RecordProperty("expected_success",possible ? "true" : "false");
  if(!possible) {
    try {
      planner_->createPlan(start,goal);
      FAIL()<<"Impossible body/terminal orientation was accepted";
    } catch(const nav2_core::PlannerException & error) {
      const std::string reason=error.what();
      EXPECT_TRUE(reason.find("start")!=std::string::npos ||
        reason.find("goal")!=std::string::npos || reason.find("footprint")!=std::string::npos)
        <<"Expected geometry refusal, not an input outage: "<<reason;
    }
  } else {
    const auto path=planner_->createPlan(start,goal);
    ASSERT_GE(path.poses.size(),2u);
    EXPECT_NEAR(path.poses.back().pose.position.x,2.05,1e-6);
    EXPECT_NEAR(path.poses.back().pose.position.y,centre,1e-6);
    for(const auto & p:path.poses) {EXPECT_NEAR(p.pose.position.y,centre,1e-6);}
    checkCompleteSweep(path,walls,0,goal_yaw);
  }
}

INSTANTIATE_TEST_SUITE_P(ClearanceAndTurn,CorridorWidthMatrix,::testing::Combine(
  ::testing::Values(4,5,6,7,11),::testing::Values(0.,M_PI/2)));

TEST_F(InterpolatedScenario, RealObstacleMustDeflectOtherwiseStraightRoute)
{
  sparseFloor();
  const std::vector<OccupyMap::Point> obstacle{{1.05,.05,.21}};
  load(obstacle);
  const auto path=planner_->createPlan(pose(.05,.05),pose(2.05,.05));
  ASSERT_GE(path.poses.size(),2u);
  bool detour=false;
  for(const auto & p:path.poses) {detour|=std::abs(p.pose.position.y-.05)>.26;}
  EXPECT_TRUE(detour);
  checkCompleteSweep(path,obstacle,0,0);
}
class RecordedTerminalScenario : public InterpolatedScenario
{
protected:
  double goalRadius() const override {return .2;}
  std::vector<uint8_t> obstacleSources(const std::vector<OccupyMap::Point> & obstacles) const override
  {
    std::ifstream input(std::string(TEST_TERMINAL_MAP_DIRECTORY) + "/hazard_sources.csv");
    if (!input) {throw std::runtime_error("Recorded hazard provenance is missing");}
    std::map<std::array<float,3>, uint8_t> labels;
    std::string line; std::getline(input,line);
    while (std::getline(input,line)) {
      std::replace(line.begin(),line.end(),',',' ');
      std::istringstream row(line); std::array<float,3> key; std::string name;
      if (!(row >> key[0] >> key[1] >> key[2] >> name)) {throw std::runtime_error("Invalid source row");}
      const uint8_t source = name == "measured" ? 1 : name == "synthetic" ? 2 : 4;
      const auto found = labels.find(key);
      labels[key] = found == labels.end() || found->second == source ? source : 4;
    }
    std::vector<uint8_t> result;
    for (const auto & p : obstacles) {
      const auto found = labels.find({float(p[0]),float(p[1]),float(p[2])});
      result.push_back(found == labels.end() ? 4 : found->second);
    }
    return result;
  }
};

class RecordedTerminalRegression : public RecordedTerminalScenario,
  public ::testing::WithParamInterface<int> {};

TEST_P(RecordedTerminalRegression, RecoveredTerminalStillRequiresCompleteValidManeuver)
{
  floor_ = savedPoints("support.csv", TEST_TERMINAL_MAP_DIRECTORY);
  const auto obstacles = savedPoints("obstacles.csv", TEST_TERMINAL_MAP_DIRECTORY);
  load(obstacles);
  struct Case {double sx, sy, sz, yaw, gx, gy, gyaw;};
  const Case cases[]{
    {3.037082430073947,-.19346096438069257,-.3935408195072222,.07045651936752503,1.5,.4,0},
    {3.1832451856968786,-.1819669439765902,-.39717346322071173,.06892566459966407,1.5,.4,3.141592653589793},
    {4.544948884471809,-.12467627995010788,-.41227587299600704,-.10091256459806851,2.5,.4,0},
    {4.652574033460416,-.16500615799712753,-.4228932992984243,-.25513805339928297,2.5,.4,3.141592653589793},
    {.008868051850360104,-.005799404456559076,-.35096475481987,1.0175995616646936,4.5,-.6,0},
    {.14705676054223948,.11867022398506255,-.3574434220790863,.22837274300147734,4.5,-.6,0}};
  const auto & test=cases[GetParam()];
  auto start=pose(test.sx,test.sy),goal=pose(test.gx,test.gy);
  start.pose.position.z=test.sz;goal.pose.position.z=0;
  start.pose.orientation.z=std::sin(test.yaw/2);start.pose.orientation.w=std::cos(test.yaw/2);
  goal.pose.orientation.z=std::sin(test.gyaw/2);goal.pose.orientation.w=std::cos(test.gyaw/2);
  nav_msgs::msg::Path path;
  try {
    path=planner_->createPlan(start,goal);
  } catch (const nav2_core::PlannerException & error) {
    // Cases 0, 4 and 5 have verified complete maneuvers in the fixed map.
    // For the other historical starts, a clear terminal pose is not evidence
    // that the robot can rotate/approach it. Preserve a safe motion rejection,
    // while regressing the erroneous "no safe terminal" classification.
    ASSERT_TRUE(GetParam()>=1 && GetParam()<=3) << error.what();
    EXPECT_NE(std::string(error.what()).find("final route failed rectangular footprint validation"),
      std::string::npos) << error.what();
    return;
  }
  ASSERT_GE(path.poses.size(),2u);
  const auto & end=path.poses.back().pose;
  EXPECT_LE(std::hypot(end.position.x-test.gx,end.position.y-test.gy),.2+1e-9);
  EXPECT_DOUBLE_EQ(end.orientation.z,goal.pose.orientation.z);
  EXPECT_DOUBLE_EQ(end.orientation.w,goal.pose.orientation.w);
  EXPECT_NEAR(path.poses.front().pose.position.x,test.sx,1e-9);
  EXPECT_NEAR(path.poses.front().pose.position.y,test.sy,1e-9);
  checkCompleteSweep(path,obstacles,test.yaw,test.gyaw);
}
INSTANTIATE_TEST_SUITE_P(RecordedMap,RecordedTerminalRegression,::testing::Range(0,6));

TEST_F(RecordedTerminalScenario, RecordedOccupiedTerminalCannotBeFreedBySubcellSampling)
{
  floor_ = savedPoints("support.csv", TEST_TERMINAL_MAP_DIRECTORY);
  load(savedPoints("obstacles.csv", TEST_TERMINAL_MAP_DIRECTORY));
  auto start=pose(3.0,-.2),goal=pose(3.5,.4);
  start.pose.position.z=-.39;goal.pose.position.z=0;
  EXPECT_THROW(planner_->createPlan(start,goal),nav2_core::PlannerException);
}

// Original saved lab map and the 16 recorded successful detours. No invented floor.
class RecordedStartRefinement : public RecordedTerminalScenario,
  public ::testing::WithParamInterface<std::tuple<int, bool>>
{
protected:
  double startRefinementRadius() const override {return .6;}
  bool refineStaticExtents() const override {return std::get<1>(GetParam());}
  double measuredRefinementRadius() const override {return std::get<1>(GetParam()) ? .8 : 0.;}
  virtual const char * comparisonScope() const {return "optional_global";}
  void checkFrozenRecordedQuery();
};
void RecordedStartRefinement::checkFrozenRecordedQuery()
{
  struct Case {int index; double sx, sy, ground, yaw, gx, gy, gyaw;};
  const Case cases[]{
    {19, 3.09289964092378, -0.18626644056223754, -0.39036282896995544, 0.07213122929705731, 1.5, -0.1, -1.5707963267948966},
    {24, 4.200827871788875, -0.12176705605982655, -0.38520297408103943, 0.07330465016021402, 2.5, -0.6, 0.0},
    {25, 4.284464134298576, -0.1161600704941598, -0.38520297408103943, 0.06831381154425342, 2.5, -0.6, 1.5707963267948966},
    {90, 1.097030100929693, -1.167728840082031, -0.3467933237552643, 2.990276636405318, 1.5, 0.4, 0.0},
    {127, 0.7598731561852854, 0.10982832632569445, -0.3576991558074951, -0.03682929293664158, 4.5, 0.4, 1.5707963267948966},
    {130, 1.435660940601925, 0.06173572791736992, -0.3666265308856964, -0.06195446980238103, 0.04, 0.02, 0.0},
    {131, 1.5504875401820422, 0.05015051928815539, -0.3666265308856964, -0.06516294502843477, 0.04, 0.02, 1.5707963267948966},
    {132, 1.658434655772517, 0.05248811212248908, -0.36649978160858154, -0.06944944176784378, 1.67, -0.67, 0.0},
    {133, 1.7320515003446306, 0.037898724802502504, -0.36649978160858154, -0.069173493203639, 1.67, -0.67, 1.5707963267948966},
    {134, 1.814602296972639, 0.03523624535027778, -0.3696146309375763, -0.06897963638148896, 1.85, -0.75, 0.0},
    {135, 1.8840407696558814, 0.02627846435932324, -0.3696146309375763, -0.0679134971176986, 1.85, -0.75, 1.5707963267948966},
    {140, 2.218689722068616, -0.0016681046569836377, -0.37759628891944885, -0.07571308349921237, 0.5, -0.6, 0.0},
    {152, 3.5336283445113796, -0.12102658015229371, -0.38176066746219356, 0.07179238387458378, 1.5, -0.6, 0.0},
    {153, 3.828338151710085, -0.10298992919952779, -0.381570041179657, 0.07025042093421789, 1.5, -0.6, 1.5707963267948966},
    {154, 4.229865061702239, -0.09149005850190595, -0.38520297408103943, 0.016931616858258138, 1.5, -0.6, 3.141592653589793},
    {160, 4.642196129663811, -0.09798116310440246, -0.38644126057624817, -0.19054423105561158, 1.5, 0.4, 0.0},
  };
  const auto & c = cases[std::get<0>(GetParam())];
  const bool refined = std::get<1>(GetParam());
  floor_ = savedPoints("support.csv", TEST_TERMINAL_MAP_DIRECTORY);
  auto obstacles = savedPoints("obstacles.csv", TEST_TERMINAL_MAP_DIRECTORY);
  load(obstacles);
  auto start = pose(c.sx, c.sy), goal = pose(c.gx, c.gy);
  start.pose.position.z = c.ground; goal.pose.position.z = 0;
  start.pose.orientation.z = std::sin(c.yaw/2); start.pose.orientation.w = std::cos(c.yaw/2);
  goal.pose.orientation.z = std::sin(c.gyaw/2); goal.pose.orientation.w = std::cos(c.gyaw/2);
  const auto begin = std::chrono::steady_clock::now();
  nav_msgs::msg::Path path;
  try {path = planner_->createPlan(start, goal);}
  catch (const nav2_core::PlannerException & error) {
    std::cout << "START_REFINEMENT {\"index\":" << c.index
      << ",\"scope\":\"" << comparisonScope() << "\""
      << ",\"refined\":" << (refined ? "true" : "false")
      << ",\"success\":false,\"reason\":\"" << error.what() << "\"}" << std::endl;
    if (std::string(comparisonScope()) == "guidance" && refined) {
      FAIL() << "Previously valid recorded guide was rejected: " << error.what();
    }
    // The frozen input intentionally excludes all historical live returns.
    // Preserve failure rather than silently substituting a successful goal.
    return;
  }
  ASSERT_GE(path.poses.size(), 2u);
  std::vector<OccupyMap::Point> points; double length = 0.;
  for (const auto & pose : path.poses) {
    const auto & p = pose.pose.position;
    if (!points.empty()) {length += std::hypot(p.x-points.back()[0], p.y-points.back()[1]);}
    points.push_back({p.x,p.y,p.z});
  }
  EXPECT_LE(std::hypot(points.back()[0]-c.gx, points.back()[1]-c.gy), .2+1e-9);
  EXPECT_NEAR(path.poses.back().pose.orientation.z, goal.pose.orientation.z, 1e-9);
  for (auto & p : obstacles) {for (auto & v : p) {v = static_cast<float>(v);}}
  OccupyMap::Config map_config; map_config.inflate_radius = .24;
  OccupyMap check_map(map_config); check_map.setStaticObstacles(obstacles, 0, obstacleSources(obstacles));
  SweptFootprint::Config body;
  body.min_x=-.41; body.max_x=.41; body.min_y=-.26; body.max_y=.26;
  body.near_start_static_refinement_radius = startRefinementRadius();
  body.refine_static_obstacle_extents = refineStaticExtents();
  body.near_start_measured_refinement_radius = measuredRefinementRadius();
  body.check_in_place_turn_sweeps = checkTurnSweeps();
  body.near_start_center = std::array<double,2>{c.sx,c.sy};
  SweptFootprint check(check_map,body);
  const auto read = check_map.lockForReading();
  EXPECT_TRUE(check.validatePath(points,c.yaw,c.gyaw));
  std::cout << "START_REFINEMENT {\"index\":" << c.index
    << ",\"scope\":\"" << comparisonScope() << "\""
    << ",\"refined\":" << (refined ? "true" : "false")
    << ",\"success\":true,\"length_m\":" << length
    << ",\"straight_m\":" << std::hypot(points.back()[0]-points.front()[0],points.back()[1]-points.front()[1])
    << ",\"elapsed_ms\":" << std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-begin).count() << "}" << std::endl;
}
TEST_P(RecordedStartRefinement, FrozenRecordedDetourRetainsCompleteBodyValidation)
{checkFrozenRecordedQuery();}
INSTANTIATE_TEST_SUITE_P(OriginalDetours,RecordedStartRefinement,
  ::testing::Combine(::testing::Range(0,16),::testing::Bool()));

class RefinedObstacleExtentScenario : public InterpolatedScenario
{
protected:
  double startRefinementRadius() const override {return .6;}
  bool refineStaticExtents() const override {return true;}
  double measuredRefinementRadius() const override {return .8;}
};

TEST_F(RefinedObstacleExtentScenario, RealObstacleStillDeflectsOtherwiseStraightRoute)
{
  sparseFloor();
  const std::vector<OccupyMap::Point> obstacle{{1.05,.05,.21}};
  load(obstacle);
  const auto path=planner_->createPlan(pose(.05,.05),pose(2.05,.05));
  ASSERT_GE(path.poses.size(),2u);
  bool detour=false;
  for(const auto & p:path.poses) {detour|=std::abs(p.pose.position.y-.05)>.26;}
  EXPECT_TRUE(detour);
  // The actual central obstacle remains inside its voxel at every relevant
  // heading; this negative control must also pass the original full sweep.
  checkCompleteSweep(path,obstacle,0,0);
}

TEST_F(RefinedObstacleExtentScenario, ActualStartBodyCollisionCannotBeIgnored)
{
  sparseFloor(); load({{.05,.05,.21}});
  EXPECT_THROW(planner_->createPlan(pose(.05,.05),pose(2.05,.05)),nav2_core::PlannerException);
}

TEST_F(RefinedObstacleExtentScenario, OccupiedWallAtGoalCannotBeFreedByExtentRefinement)
{
  sparseFloor();
  std::vector<OccupyMap::Point> wall;
  for(int y=-12;y<=12;++y) {wall.push_back({2.05,y*.1+.05,.21});}
  load(wall);
  EXPECT_THROW(planner_->createPlan(pose(.05,.05),pose(2.05,.05)),nav2_core::PlannerException);
}

class RecordedGuidanceRegression : public RecordedStartRefinement
{
protected:
  bool refineStaticExtents() const override {return std::get<1>(GetParam());}
  double measuredRefinementRadius() const override {return .8;}
  bool checkTurnSweeps() const override {return !std::get<1>(GetParam());}
  const char * comparisonScope() const override {return "guidance";}
};
TEST_P(RecordedGuidanceRegression, RecordedDetourStillChecksEveryBodyCorridorAndEndpoint)
{checkFrozenRecordedQuery();}
INSTANTIATE_TEST_SUITE_P(RecordedGuidance,RecordedGuidanceRegression,
  ::testing::Combine(::testing::Range(0,16),::testing::Bool()));

}  // namespace
}  // namespace astar_3d_planner
