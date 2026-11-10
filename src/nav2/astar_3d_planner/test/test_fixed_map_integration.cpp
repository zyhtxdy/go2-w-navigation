#include <gtest/gtest.h>

#include <chrono>
#include <atomic>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "astar_3d_planner/astar_3d_planner.hpp"
#include "astar_3d_planner/fixed_map_guard.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace astar_3d_planner
{
namespace
{
using namespace std::chrono_literals;

sensor_msgs::msg::PointCloud2 packedCloud(
  const std::vector<OccupyMap::Point> & points, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 result;
  result.header.frame_id = "map";
  result.header.stamp = stamp;
  result.height = 1;
  result.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(result);
  modifier.setPointCloud2Fields(3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());
  if (points.empty()) {return result;}
  sensor_msgs::PointCloud2Iterator<float> x(result, "x"), y(result, "y"), z(result, "z");
  for (const auto & point : points) {
    *x = point[0]; *y = point[1]; *z = point[2]; ++x; ++y; ++z;
  }
  return result;
}

sensor_msgs::msg::PointCloud2 observedCloud(
  const std::vector<OccupyMap::Point> & points,
  const std::vector<std::uint8_t> & sources, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 result;
  result.header.frame_id = "map";
  result.header.stamp = stamp;
  result.height = 1;
  result.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(result);
  modifier.setPointCloud2Fields(7,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "obstacle_source", 1, sensor_msgs::msg::PointField::UINT8,
    "_padding", 3, sensor_msgs::msg::PointField::UINT8,
    "observation_sec", 1, sensor_msgs::msg::PointField::INT32,
    "observation_nanosec", 1, sensor_msgs::msg::PointField::UINT32);
  modifier.resize(points.size());
  if (points.empty()) {return result;}
  sensor_msgs::PointCloud2Iterator<float> x(result, "x"), y(result, "y"), z(result, "z");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> source(result, "obstacle_source");
  sensor_msgs::PointCloud2Iterator<std::int32_t> seconds(result, "observation_sec");
  sensor_msgs::PointCloud2Iterator<std::uint32_t> nanoseconds(result, "observation_nanosec");
  for (std::size_t i = 0; i < points.size(); ++i) {
    *x = points[i][0]; *y = points[i][1]; *z = points[i][2];
    *source = sources.at(i);
    *seconds = result.header.stamp.sec; *nanoseconds = result.header.stamp.nanosec;
    ++x; ++y; ++z; ++source; ++seconds; ++nanoseconds;
  }
  return result;
}

geometry_msgs::msg::PoseStamped mapPose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = .01;
  pose.pose.orientation.w = 1.0;
  return pose;
}

class FixedMapIntegration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = rclcpp::contexts::get_global_default_context();
    rclcpp::InitOptions init;
    init.set_domain_id(151);
    context_->init(0, nullptr, init);
    rclcpp::NodeOptions costmap_options;
    costmap_options.context(context_);
    costmap_options.parameter_overrides({
      {"global_frame", "map"}, {"robot_base_frame", "base_link"},
      {"plugins", std::vector<std::string>{}}, {"filters", std::vector<std::string>{}},
      {"footprint_padding", 0.0},
      {"footprint", "[[0.4,0.25],[0.4,-0.25],[-0.4,-0.25],[-0.4,0.25]]"}});
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_options);
    ASSERT_EQ(costmap_->on_configure(rclcpp_lifecycle::State()),
      nav2_util::CallbackReturn::SUCCESS);
    input_ = std::make_shared<rclcpp::Node>("fixed_map_test_source",
      rclcpp::NodeOptions().context(context_));
    for (int x = -10; x <= 30; ++x) {
      for (int y = -10; y <= 10; ++y) {
        floor_.push_back({x * .1 + .05, y * .1 + .05, .01});
      }
    }
    support_ = packedCloud(floor_, input_->now());
    obstacles_ = packedCloud(fixed_points_, input_->now());
    ASSERT_EQ(support_.point_step, 12u);
    ASSERT_EQ(obstacles_.height, 1u);
    rclcpp::NodeOptions parent_options;
    parent_options.context(context_);
    if (remap_topics_) {
      parent_options.arguments({"--ros-args",
        "-r", "/fixed_map_test/support:=/fixed_map_remapped/support",
        "-r", "/fixed_map_test/obstacles:=/fixed_map_remapped/obstacles",
        "-r", "/fixed_map_test/id:=/fixed_map_remapped/id"});
    }
    parent_options.parameter_overrides({
      {"GridBased.use_static_map", true}, {"GridBased.verify_static_map_identity", true},
      {"GridBased.expected_map_id", map_id_},
      {"GridBased.expected_support_sha256", payloadSha256(support_.data)},
      {"GridBased.expected_static_obstacles_sha256", payloadSha256(obstacles_.data)},
      {"GridBased.static_map_id_topic", "/fixed_map_test/id"},
      {"GridBased.support_topic", "/fixed_map_test/support"},
      {"GridBased.static_obstacle_topic", "/fixed_map_test/obstacles"},
      {"GridBased.obstacle_topic", "/fixed_map_test/live_obstacles"},
      {"GridBased.use_lio_start_z", false}, {"GridBased.compensate_height_drift", false},
      {"GridBased.reuse_previous_path", false}, {"GridBased.resolution", .1},
      {"GridBased.path_validity_start_tolerance", .30},
      {"GridBased.path_validity_ignore_near_robot_radius", 1.6},
      {"GridBased.inflate_radius", .24}, {"GridBased.surface_connection_radius", .3}});
    if (separate_planning_gate_) {
      parent_options.append_parameter_override("GridBased.localization_ready_topic",
        std::string("/offline_localization/planning_ready"));
    }
    parent_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "fixed_map_test_parent", parent_options);
    tf_ = std::make_shared<tf2_ros::Buffer>(parent_->get_clock());
    tf_->setUsingDedicatedThread(true);
    geometry_msgs::msg::TransformStamped robot_tf;
    robot_tf.header.frame_id = "map"; robot_tf.child_frame_id = "base_link";
    robot_tf.transform.translation.x = .05; robot_tf.transform.translation.y = .05;
    robot_tf.transform.translation.z = .01; robot_tf.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_->setTransform(robot_tf, "fixed_map_fixture", true));
    planner_ = std::make_unique<Astar3DPlanner>();
    planner_->configure(parent_, "GridBased", tf_, costmap_);
    planner_->activate();
    const auto fixed_qos = rclcpp::QoS(1).reliable().transient_local();
    const std::string map_prefix = remap_topics_ ? "/fixed_map_remapped/" : "/fixed_map_test/";
    // All three map endpoints intentionally belong to exactly the same node.
    support_pub_ = input_->create_publisher<sensor_msgs::msg::PointCloud2>(
      map_prefix + "support", fixed_qos);
    obstacles_pub_ = input_->create_publisher<sensor_msgs::msg::PointCloud2>(
      map_prefix + "obstacles", fixed_qos);
    identity_pub_ = input_->create_publisher<std_msgs::msg::String>(
      map_prefix + "id", fixed_qos);
    ready_pub_ = input_->create_publisher<std_msgs::msg::Bool>(
      separate_planning_gate_ ? "/offline_localization/planning_ready" :
      "/offline_localization/ready", fixed_qos);
    alignment_pub_ = input_->create_publisher<std_msgs::msg::Bool>(
      "/offline_localization/localization_available", fixed_qos);
    live_pub_ = input_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/fixed_map_test/live_obstacles", rclcpp::SensorDataQoS());
    observer_pub_ = input_->create_publisher<nav_msgs::msg::Odometry>("/Odometry", rclcpp::SensorDataQoS());
    ready_timer_ = input_->create_wall_timer(25ms, [this]() {
        std_msgs::msg::Bool message; message.data = ready_available_.load(); ready_pub_->publish(message);
        message.data = alignment_available_.load();
        alignment_pub_->publish(message);
        nav_msgs::msg::Odometry observer;
        observer.header.frame_id = "map"; observer.child_frame_id = "base_link";
        observer.header.stamp = input_->now(); observer.pose.pose.orientation.w = 1.0;
        observer_pub_->publish(observer);
        if (live_mode_.load() != 1) {
          const auto stamp = live_mode_.load() == 2 ?
            rclcpp::Time(frozen_stamp_ns_.load()) : input_->now();
          sensor_msgs::msg::PointCloud2 live;
          {
            std::lock_guard<std::mutex> lock(live_points_mutex_);
            live = observedCloud(live_points_, live_sources_, stamp);
          }
          if (live_mode_.load() == 3) {live.header.frame_id = "unavailable_obstacle_sensor";}
          live_pub_->publish(live);
        }
      });
    status_sub_ = input_->create_subscription<std_msgs::msg::String>(
      "/planner/map_status", fixed_qos, [this](std_msgs::msg::String::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(observations_mutex_);
        status_ = message->data;
      });
    live_status_sub_ = input_->create_subscription<std_msgs::msg::String>(
      "/planner/live_obstacle_status", fixed_qos,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(observations_mutex_);
        live_status_ = message->data;
      });
    snapshot_sub_ = input_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/planner/support_map", fixed_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(observations_mutex_);
        snapshots_.push_back(*message);
      });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(parent_->get_node_base_interface());
    executor_->add_node(input_);
    executor_thread_ = std::thread([this]() {executor_->spin();});
    ASSERT_TRUE(wait([this]() {
        return support_pub_->get_subscription_count() == 1 &&
               obstacles_pub_->get_subscription_count() == 1 &&
               identity_pub_->get_subscription_count() == 1;
      }));
  }

  void TearDown() override
  {
    if (executor_) {executor_->cancel();}
    if (executor_thread_.joinable()) {executor_thread_.join();}
    if (planner_) {planner_->deactivate(); planner_->cleanup();}
    planner_.reset();
    ready_timer_.reset();
    executor_.reset();
    status_sub_.reset(); live_status_sub_.reset(); snapshot_sub_.reset();
    support_pub_.reset(); obstacles_pub_.reset(); identity_pub_.reset(); ready_pub_.reset();
    input_.reset(); parent_.reset();
    if (costmap_) {costmap_->on_cleanup(rclcpp_lifecycle::State());}
    costmap_.reset();
    if (context_ && context_->is_valid()) {context_->shutdown("fixed map integration finished");}
  }

  bool wait(const std::function<bool()> & predicate)
  {
    const auto end = std::chrono::steady_clock::now() + 4s;
    while (std::chrono::steady_clock::now() < end) {
      if (predicate()) {return true;}
      std::this_thread::sleep_for(5ms);
    }
    return predicate();
  }

  bool statusContains(const std::string & value)
  {
    std::lock_guard<std::mutex> lock(observations_mutex_);
    return status_.find(value) != std::string::npos;
  }

  bool liveStatusContains(const std::string & value)
  {
    std::lock_guard<std::mutex> lock(observations_mutex_);
    return live_status_.find(value) != std::string::npos;
  }

  std::string layerReport(const std::string & layer)
  {
    std::lock_guard<std::mutex> lock(observations_mutex_);
    const auto begin = status_.find('"' + layer + "\":{");
    if (begin == std::string::npos) {return {};}
    return status_.substr(begin, status_.find('}', begin) - begin + 1);
  }

  void publishClouds()
  {
    support_.header.stamp = input_->now();
    obstacles_.header.stamp = input_->now();
    support_pub_->publish(support_);
    obstacles_pub_->publish(obstacles_);
  }

  void publishIdentity(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    identity_pub_->publish(message);
  }

  void becomeReady()
  {
    publishClouds();
    publishIdentity(map_id_);
    ASSERT_TRUE(wait([this]() {return statusContains("\"ready\":true");}));
    ASSERT_TRUE(wait([this]() {
      try {return planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)).poses.size() >= 2;}
      catch (const nav2_core::PlannerException &) {return false;}
    }));
  }

  void publishVerifiedInputs()
  {
    publishClouds();
    publishIdentity(map_id_);
    ASSERT_TRUE(wait([this]() {return statusContains("\"ready\":true") &&
        liveStatusContains("\"ready\":true");}));
  }

  void setLivePoints(const std::vector<OccupyMap::Point> & points,
    const std::vector<std::uint8_t> & sources)
  {
    ASSERT_EQ(points.size(), sources.size());
    std::lock_guard<std::mutex> lock(live_points_mutex_);
    live_points_ = points;
    live_sources_ = sources;
  }

  nav_msgs::msg::Path referenceLine()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = input_->now();
    for (int i = 0; i <= 25; ++i) {
      auto pose = mapPose(.05 + i * .1, .05);
      pose.header = path.header;
      path.poses.push_back(pose);
    }
    return path;
  }

  std::shared_ptr<terrain_navigation_interfaces::srv::ValidateTerrainPath::Response>
  queryPath(const nav_msgs::msg::Path & path, bool reference)
  {
    using Service = terrain_navigation_interfaces::srv::ValidateTerrainPath;
    auto client = input_->create_client<Service>(reference ?
      "/terrain/is_path_valid_reference" : "/terrain/is_path_valid_detailed");
    if (!client->wait_for_service(1s)) {return {};}
    auto request = std::make_shared<Service::Request>();
    request->path = path;
    auto future = client->async_send_request(request);
    if (future.wait_for(2s) != std::future_status::ready) {return {};}
    return future.get();
  }

  std::size_t snapshotCount()
  {
    std::lock_guard<std::mutex> lock(observations_mutex_);
    return snapshots_.size();
  }

  sensor_msgs::msg::PointCloud2 lastSnapshot()
  {
    std::lock_guard<std::mutex> lock(observations_mutex_);
    return snapshots_.empty() ? sensor_msgs::msg::PointCloud2{} : snapshots_.back();
  }

  std::atomic<int> live_mode_{0};
  std::atomic_bool ready_available_{true};
  std::atomic_bool alignment_available_{true};
  std::atomic<std::int64_t> frozen_stamp_ns_{0};
  const std::string map_id_ = std::string(64, 'a');
  bool remap_topics_{false};
  bool separate_planning_gate_{false};
  std::vector<OccupyMap::Point> floor_;
  std::vector<OccupyMap::Point> fixed_points_, live_points_;
  std::vector<std::uint8_t> live_sources_;
  std::mutex live_points_mutex_;
  sensor_msgs::msg::PointCloud2 support_, obstacles_;
  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_;
  rclcpp::Node::SharedPtr input_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<Astar3DPlanner> planner_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr support_pub_, obstacles_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr identity_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alignment_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr live_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr observer_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr live_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr snapshot_sub_;
  rclcpp::TimerBase::SharedPtr ready_timer_;
  std::mutex observations_mutex_;
  std::string status_;
  std::string live_status_;
  std::vector<sensor_msgs::msg::PointCloud2> snapshots_;
};

TEST_F(FixedMapIntegration, ExplicitLocalizationFalseBlocksPlanningAndIsReportedSeparately)
{
  becomeReady();
  ASSERT_TRUE(wait([this]() {return liveStatusContains("\"ready\":true");}));
  EXPECT_TRUE(liveStatusContains("\"alignment_heartbeat_age_s\":"));
  EXPECT_TRUE(liveStatusContains("\"localization_heartbeat_timeout_s\":1.000000"));
  // Planning-ready and live-cloud publishers continue. An explicit alignment
  // revocation alone must stop planning and cannot become a heartbeat timeout.
  alignment_available_.store(false);
  ASSERT_TRUE(wait([this]() {
      return liveStatusContains("\"reason\":\"localization_unavailable\"");
    }));
  EXPECT_TRUE(liveStatusContains("\"ready\":false"));
  EXPECT_THROW(planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)),
    nav2_core::PlannerException);
  alignment_available_.store(true);
  ASSERT_TRUE(wait([this]() {
      if (!liveStatusContains("\"ready\":true")) {return false;}
      try {return planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)).poses.size() >= 2;}
      catch (const nav2_core::PlannerException &) {return false;}
    }));
}

TEST_F(FixedMapIntegration, LiveInputOutageDuplicateStampsAndTransformFailuresRejectPlanning)
{
  becomeReady();
  auto validity = input_->create_client<nav2_msgs::srv::IsPathValid>("/terrain/is_path_valid");
  ASSERT_TRUE(validity->wait_for_service(2s));
  const auto cached = planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05));
  const auto check_validity = [&validity, &cached, this]() {
      auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
      request->path = cached;
      auto future = validity->async_send_request(request);
      if (!wait([&future]() {return future.wait_for(0ms) == std::future_status::ready;})) {
        return true;  // A missing service must fail the outage assertion below.
      }
      return future.get()->is_valid;
    };
  ASSERT_TRUE(check_validity());
  const auto can_plan = [this]() {
      try {
        return planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)).poses.size() >= 2;
      } catch (const nav2_core::PlannerException &) {return false;}
    };
  for (int mode : {1, 2, 3}) {
    frozen_stamp_ns_.store(input_->now().nanoseconds());
    live_mode_.store(mode);
    ASSERT_TRUE(wait([&can_plan]() {return !can_plan();}));
    // Fixed map and localization heartbeats still run. Only obstacle
    // consumption is interrupted; an old snapshot must not authorize a plan.
    EXPECT_TRUE(statusContains("\"ready\":true"));
    EXPECT_FALSE(can_plan());
    EXPECT_FALSE(check_validity());
    live_mode_.store(0);
    ASSERT_TRUE(wait(can_plan));  // Real advancing empty scans recover without a dependency cycle.
  }
}

TEST_F(FixedMapIntegration, ReferenceRouteMayBeLocallyBlockedWhileStrictValidationRejectsIt)
{
  publishVerifiedInputs();
  const auto path = referenceLine();
  const auto clear = queryPath(path, false);
  ASSERT_NE(clear, nullptr);
  ASSERT_TRUE(clear->is_valid) << clear->reason;
  setLivePoints({{.85, .05, .25}}, {OccupyMap::kMeasured});
  ASSERT_TRUE(wait([&]() {
      const auto result = queryPath(path, false);
      return result && !result->is_valid &&
        result->status == result->COLLISION;
    }));
  const auto reference = queryPath(path, true);
  ASSERT_NE(reference, nullptr);
  EXPECT_TRUE(reference->is_valid) << reference->reason;
  EXPECT_EQ(reference->status, reference->VALID);
  EXPECT_EQ(reference->reason, "local_detour_active");
  // Reference validation must not mutate the real map or relax the strict
  // service used when a stopped localization gate authorizes recovery.
  const auto strict = queryPath(path, false);
  ASSERT_NE(strict, nullptr);
  EXPECT_FALSE(strict->is_valid);
  EXPECT_EQ(strict->status, strict->COLLISION);
}

TEST_F(FixedMapIntegration, ReferenceExemptionNeverIncludesTheCurrentRobotFootprint)
{
  publishVerifiedInputs();
  const auto path = referenceLine();
  setLivePoints({{.15, .05, .25}}, {OccupyMap::kMeasured});
  ASSERT_TRUE(wait([&]() {
      const auto result = queryPath(path, true);
      return result && !result->is_valid &&
        result->reason == "current_footprint_collision";
    }));
  const auto strict = queryPath(path, false);
  ASSERT_NE(strict, nullptr);
  EXPECT_FALSE(strict->is_valid);
}

TEST_F(FixedMapIntegration, ReferenceDoesNotExemptUnknownMixedOrDistantMeasuredObstacles)
{
  publishVerifiedInputs();
  const auto path = referenceLine();
  struct Scenario
  {
    std::vector<OccupyMap::Point> points;
    std::vector<std::uint8_t> sources;
    const char * name;
  };
  const std::vector<Scenario> scenarios{
    {{{.85, .05, .25}}, {OccupyMap::kUnknown}, "unknown"},
    {{{.85, .05, .25}, {.86, .05, .25}},
      {OccupyMap::kMeasured, OccupyMap::kSynthetic}, "mixed"},
    {{{2.15, .05, .25}}, {OccupyMap::kMeasured}, "distant_measured"}};
  for (const auto & scenario : scenarios) {
    SCOPED_TRACE(scenario.name);
    setLivePoints({}, {});
    ASSERT_TRUE(wait([&]() {
        const auto strict = queryPath(path, false);
        const auto reference = queryPath(path, true);
        return strict && reference && strict->is_valid && reference->is_valid;
      }));
    setLivePoints(scenario.points, scenario.sources);
    ASSERT_TRUE(wait([&]() {
        const auto strict = queryPath(path, false);
        return strict && !strict->is_valid && strict->status == strict->COLLISION;
      }));
    const auto reference = queryPath(path, true);
    ASSERT_NE(reference, nullptr);
    EXPECT_FALSE(reference->is_valid) << reference->reason;
    EXPECT_EQ(reference->status, reference->COLLISION);
  }
}

TEST_F(FixedMapIntegration, ReferenceTemporarilyToleratesBoundedDeviationButStrictDoesNot)
{
  publishVerifiedInputs();
  const auto path = referenceLine();
  geometry_msgs::msg::TransformStamped robot_tf;
  robot_tf.header.frame_id = "map"; robot_tf.child_frame_id = "base_link";
  robot_tf.transform.translation.x = .05; robot_tf.transform.translation.y = .50;
  robot_tf.transform.translation.z = .01; robot_tf.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_->setTransform(robot_tf, "fixed_map_fixture", true));
  const auto strict = queryPath(path, false);
  ASSERT_NE(strict, nullptr);
  EXPECT_FALSE(strict->is_valid);
  EXPECT_EQ(strict->status, strict->PATH_MISMATCH);
  const auto reference = queryPath(path, true);
  ASSERT_NE(reference, nullptr);
  EXPECT_TRUE(reference->is_valid) << reference->reason;
  EXPECT_EQ(reference->reason, "local_detour_active");
  robot_tf.transform.translation.y = .90;
  ASSERT_TRUE(tf_->setTransform(robot_tf, "fixed_map_fixture", true));
  const auto outside = queryPath(path, true);
  ASSERT_NE(outside, nullptr);
  EXPECT_FALSE(outside->is_valid);
  EXPECT_EQ(outside->status, outside->PATH_MISMATCH);
}

TEST_F(FixedMapIntegration, ReferenceAndStrictValidationBothRejectRevokedLocalizationAndStaleInputs)
{
  publishVerifiedInputs();
  const auto path = referenceLine();
  ready_available_.store(false);
  ASSERT_TRUE(wait([&]() {
      const auto result = queryPath(path, false);
      return result && !result->is_valid && result->status == result->LOCALIZATION_UNAVAILABLE;
    }));
  const auto no_localization = queryPath(path, true);
  ASSERT_NE(no_localization, nullptr);
  EXPECT_FALSE(no_localization->is_valid);
  EXPECT_EQ(no_localization->status, no_localization->LOCALIZATION_UNAVAILABLE);
  ready_available_.store(true);
  ASSERT_TRUE(wait([&]() {
      const auto result = queryPath(path, false);
      return result && result->is_valid;
    }));
  frozen_stamp_ns_.store(input_->now().nanoseconds());
  live_mode_.store(2);
  ASSERT_TRUE(wait([&]() {
      const auto result = queryPath(path, false);
      return result && !result->is_valid && result->status == result->OBSTACLES_UNAVAILABLE;
    }));
  const auto stale = queryPath(path, true);
  ASSERT_NE(stale, nullptr);
  EXPECT_FALSE(stale->is_valid);
  EXPECT_EQ(stale->status, stale->OBSTACLES_UNAVAILABLE);
}

class ReferenceFixedObstacleIntegration : public FixedMapIntegration
{
protected:
  void SetUp() override
  {
    fixed_points_ = {{.85, .05, .25}};
    FixedMapIntegration::SetUp();
  }
};

TEST_F(ReferenceFixedObstacleIntegration, ReferenceNeverExemptsNearbySavedMapObstacles)
{
  publishVerifiedInputs();
  const auto path = referenceLine();
  // Include a coincident pure measured return: dynamic metadata must not
  // erase the independently owned fixed obstacle in the reference snapshot.
  setLivePoints({{.85, .05, .25}}, {OccupyMap::kMeasured});
  ASSERT_TRUE(wait([&]() {
      const auto result = queryPath(path, false);
      return result && !result->is_valid && result->status == result->COLLISION;
    }));
  const auto reference = queryPath(path, true);
  ASSERT_NE(reference, nullptr);
  EXPECT_FALSE(reference->is_valid);
  EXPECT_EQ(reference->status, reference->COLLISION);
}

TEST_F(FixedMapIntegration, CloudsMayPrecedeIdentityButChangedPayloadCannotMutateMap)
{
  publishClouds();
  ASSERT_TRUE(wait([this]() {return statusContains("waiting_for_map_id") &&
      layerReport("support").find("\"verified\":true") != std::string::npos &&
      layerReport("obstacles").find("\"verified\":true") != std::string::npos;}));
  EXPECT_FALSE(statusContains("\"faulted\":true"));
  EXPECT_THROW(planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)),
    nav2_core::PlannerException);
  publishIdentity(map_id_);
  ASSERT_TRUE(wait([this]() {return statusContains("\"ready\":true");}));
  EXPECT_GE(planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)).poses.size(), 2u);
  ASSERT_TRUE(wait([this]() {return snapshotCount() > 0;}));
  const auto original = lastSnapshot();
  const auto snapshots_before_duplicate = snapshotCount();
  // Header stamp changes do not change immutable-map identity or reload it.
  support_.header.stamp = input_->now();
  support_pub_->publish(support_);
  ASSERT_TRUE(wait([this]() {return layerReport("support").find("\"duplicates\":1") !=
      std::string::npos;}));
  EXPECT_EQ(snapshotCount(), snapshots_before_duplicate);
  EXPECT_GE(planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)).poses.size(), 2u);
  auto changed = floor_;
  changed.push_back({50.05, 50.05, .01});
  support_pub_->publish(packedCloud(changed, input_->now()));
  ASSERT_TRUE(wait([this]() {return statusContains("support_payload_sha256_mismatch");}));
  EXPECT_THROW(planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)),
    nav2_core::PlannerException);
  // Re-activation forces a public snapshot of the actual internal map, proving
  // the rejected cloud was never merged, while keeping the fault latched.
  planner_->deactivate();
  planner_->activate();
  ASSERT_TRUE(wait([this, snapshots_before_duplicate]() {
      return snapshotCount() > snapshots_before_duplicate;
    }));
  EXPECT_EQ(lastSnapshot().width, original.width);
  EXPECT_EQ(lastSnapshot().data, original.data);
  EXPECT_EQ(lastSnapshot().width, floor_.size());
  EXPECT_TRUE(statusContains("\"faulted\":true"));
}

TEST_F(FixedMapIntegration, SecondSupportPublisherIsRejectedWithoutPublishingACloud)
{
  becomeReady();
  // Even an idle second endpoint conflicts with the single-source map contract.
  auto second = input_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/fixed_map_test/support", rclcpp::QoS(1).reliable().transient_local());
  ASSERT_TRUE(wait([this]() {return statusContains("multiple_fixed_map_publishers");}));
  EXPECT_TRUE(statusContains("\"faulted\":true"));
  EXPECT_THROW(planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)),
    nav2_core::PlannerException);
}

TEST_F(FixedMapIntegration, DifferentMapIdentityRejectsOtherwiseCorrectClouds)
{
  publishClouds();
  publishIdentity(std::string(64, 'b'));
  ASSERT_TRUE(wait([this]() {return statusContains("map_id_mismatch");}));
  EXPECT_TRUE(statusContains("\"faulted\":true"));
  EXPECT_THROW(planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)),
    nav2_core::PlannerException);
  // Correcting only the last ID cannot clear a previously latched conflict.
  publishIdentity(map_id_);
  std::this_thread::sleep_for(600ms);
  EXPECT_TRUE(statusContains("map_id_mismatch"));
  EXPECT_THROW(planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)),
    nav2_core::PlannerException);
}

TEST_F(FixedMapIntegration, CleanupAndConfigureReloadRetainedMapWithoutUpstreamRepublish)
{
  becomeReady();
  ASSERT_TRUE(wait([this]() {return snapshotCount() > 0;}));
  const auto previous_snapshot_count = snapshotCount();
  const auto original = lastSnapshot();
  auto reloading = std::make_shared<std::atomic_bool>(false);
  auto reloaded = std::make_shared<std::atomic_bool>(false);
  auto reload_error = std::make_shared<std::exception_ptr>();
  // Execute the lifecycle calls on the same single executor as subscriptions,
  // mirroring lifecycle service serialization without racing in-flight callbacks.
  auto reload_timer = input_->create_wall_timer(20ms, [this, reloading, reloaded, reload_error]() {
      if (reloading->exchange(true)) {return;}
      try {
        planner_->deactivate();
        planner_->cleanup();
        tf_ = std::make_shared<tf2_ros::Buffer>(parent_->get_clock());
        tf_->setUsingDedicatedThread(true);
        geometry_msgs::msg::TransformStamped robot_tf;
        robot_tf.header.frame_id = "map"; robot_tf.child_frame_id = "base_link";
        robot_tf.transform.translation.x = .05; robot_tf.transform.translation.y = .05;
        robot_tf.transform.translation.z = .01; robot_tf.transform.rotation.w = 1.0;
        if (!tf_->setTransform(robot_tf, "fixed_map_fixture", true)) {
          throw std::runtime_error("Cannot seed fixture transform");
        }
        planner_->configure(parent_, "GridBased", tf_, costmap_);
        planner_->activate();
      } catch (...) {
        *reload_error = std::current_exception();
      }
      reloaded->store(true);
    });
  ASSERT_TRUE(wait([reloaded]() {return reloaded->load();}));
  reload_timer->cancel();
  if (*reload_error) {std::rethrow_exception(*reload_error);}
  // No publishClouds()/publishIdentity() here: new subscriptions must consume
  // the retained original messages, each as the new guard's first valid frame.
  ASSERT_TRUE(wait([this, previous_snapshot_count]() {
      return snapshotCount() > previous_snapshot_count && statusContains("\"ready\":true");
    }));
  EXPECT_EQ(lastSnapshot().width, original.width);
  EXPECT_EQ(lastSnapshot().data, original.data);
  EXPECT_GE(planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)).poses.size(), 2u);
  EXPECT_FALSE(statusContains("\"faulted\":true"));
  EXPECT_NE(layerReport("support").find("\"accepted\":1"), std::string::npos);
}

class FixedMapRemapIntegration : public FixedMapIntegration
{
protected:
  void SetUp() override
  {
    remap_topics_ = true;
    FixedMapIntegration::SetUp();
  }
};

TEST_F(FixedMapRemapIntegration, ActualRemappedSubscriptionTopicsPassPublisherAuditAndPlanning)
{
  becomeReady();
  EXPECT_TRUE(statusContains("\"graph_healthy\":true"));
  EXPECT_TRUE(statusContains("\"ready\":true"));
  EXPECT_FALSE(statusContains("\"faulted\":true"));
  EXPECT_GE(planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05)).poses.size(), 2u);
}

class SeparatePlanningGate : public FixedMapIntegration
{
protected:
  void SetUp() override
  {
    separate_planning_gate_ = true;
    FixedMapIntegration::SetUp();
  }
};

TEST_F(SeparatePlanningGate, CanValidateWhileTheFinalVelocityGateIsClosed)
{
  auto motion = input_->create_publisher<std_msgs::msg::Bool>(
    "/offline_localization/ready", rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::Bool stopped; stopped.data = false;
  motion->publish(stopped);
  becomeReady();
  const auto path = planner_->createPlan(mapPose(.05, .05), mapPose(1.05, .05));
  auto client = input_->create_client<terrain_navigation_interfaces::srv::ValidateTerrainPath>(
    "/terrain/is_path_valid_detailed");
  ASSERT_TRUE(client->wait_for_service(1s));
  auto request = std::make_shared<terrain_navigation_interfaces::srv::ValidateTerrainPath::Request>();
  request->path = path;
  auto future = client->async_send_request(request);
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(future.get()->is_valid);
}

}  // namespace
}  // namespace astar_3d_planner
