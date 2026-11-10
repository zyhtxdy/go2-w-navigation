#include "terrain_costmap/terrain_analyzer.hpp"
#include "terrain_costmap/terrain_hazard_stabilizer.hpp"
#include "terrain_costmap/evidence_clearing.hpp"
#include "terrain_costmap/near_obstacle_detector.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace terrain_costmap
{
namespace
{

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1.0e-9;
}

sensor_msgs::msg::PointCloud2 makeCloud(
  const std::vector<TerrainPoint> & points,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    8,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "obstacle_source", 1, sensor_msgs::msg::PointField::UINT8,
    "_padding", 3, sensor_msgs::msg::PointField::UINT8,
    "observation_sec", 1, sensor_msgs::msg::PointField::INT32,
    "observation_nanosec", 1, sensor_msgs::msg::PointField::UINT32);
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
  sensor_msgs::PointCloud2Iterator<std::uint8_t> source(cloud, "obstacle_source");
  sensor_msgs::PointCloud2Iterator<std::int32_t> observed_sec(cloud, "observation_sec");
  sensor_msgs::PointCloud2Iterator<std::uint32_t> observed_nsec(cloud, "observation_nanosec");
  for (const TerrainPoint & point : points) {
    *x = static_cast<float>(point.point.x);
    *y = static_cast<float>(point.point.y);
    *z = static_cast<float>(point.point.z);
    *intensity = static_cast<float>(point.relative_height);
    *source = point.synthetic ? 2 : 1;
    *observed_sec = static_cast<std::int32_t>(point.point.observation_stamp_ns / 1000000000LL);
    *observed_nsec = static_cast<std::uint32_t>(point.point.observation_stamp_ns % 1000000000LL);
    ++x;
    ++y;
    ++z;
    ++intensity;
    ++source;
    ++observed_sec;
    ++observed_nsec;
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 makeCloud(
  const std::vector<Point> & points,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (const Point & point : points) {
    *x = static_cast<float>(point.x);
    *y = static_cast<float>(point.y);
    *z = static_cast<float>(point.z);
    ++x;
    ++y;
    ++z;
  }
  return cloud;
}

}  // namespace

class TerrainCostmapNode : public rclcpp::Node
{
public:
  TerrainCostmapNode()
  : Node("terrain_costmap"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    cloud_topic_ = declare_parameter("cloud_topic", "/cloud_registered_body");
    odometry_topic_ = declare_parameter("odometry_topic", "/Odometry");
    terrain_topic_ = declare_parameter("terrain_topic", "/terrain/map");
    support_topic_ =
      declare_parameter("support_topic", "/terrain/support");
    obstacle_topic_ = declare_parameter("obstacle_topic", "/terrain/obstacles");
    global_obstacle_topic_ =
      declare_parameter(
      "global_obstacle_topic", "/terrain/obstacles_global");
    measured_global_obstacle_topic_ =
      declare_parameter(
      "measured_global_obstacle_topic",
      "/terrain/obstacles_measured_global");
    non_traversable_topic_ =
      declare_parameter(
      "non_traversable_topic", "/terrain/non_traversable_global");
    clearing_scan_topic_ =
      declare_parameter("clearing_scan_topic", "/terrain/clear_scan");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    body_frame_ = declare_parameter("body_frame", "body");
    base_frame_ = declare_parameter("base_frame", "base_link");
    history_decay_time_ = declare_parameter("history_decay_time", 2.0);
    measured_obstacle_decay_time_ = declare_parameter("measured_obstacle_decay_time", 0.60);
    if (!std::isfinite(history_decay_time_) || history_decay_time_ <= 0.0 ||
      !std::isfinite(measured_obstacle_decay_time_) || measured_obstacle_decay_time_ <= 0.0 ||
      measured_obstacle_decay_time_ > history_decay_time_)
    {
      throw std::invalid_argument(
              "measured_obstacle_decay_time must be > 0 and <= history_decay_time");
    }
    processing_interval_ = declare_parameter("processing_interval", 0.20);
    maximum_pose_age_ = declare_parameter("maximum_pose_age", 0.20);
    pose_wait_timeout_ = declare_parameter("pose_wait_timeout", 0.10);
    if (!std::isfinite(pose_wait_timeout_) || pose_wait_timeout_ < 0.0 ||
      pose_wait_timeout_ > 0.20)
    {
      throw std::invalid_argument("pose_wait_timeout must be between 0 and 0.20 s");
    }
    pose_history_duration_ =
      declare_parameter("pose_history_duration", 2.0);
    input_minimum_range_ = declare_parameter("input_minimum_range", 0.40);
    input_maximum_range_ = declare_parameter("input_maximum_range", 8.0);
    input_minimum_relative_z_ =
      declare_parameter("input_minimum_relative_z", -1.0);
    input_maximum_relative_z_ =
      declare_parameter("input_maximum_relative_z", 2.0);
    input_voxel_size_ = declare_parameter("input_voxel_size", 0.10);
    scan_angle_increment_ =
      declare_parameter("scan_angle_increment_degrees", 0.50) * M_PI / 180.0;
    scan_range_minimum_ = declare_parameter("scan_range_minimum", 0.40);
    scan_range_maximum_ = declare_parameter("scan_range_maximum", 8.0);
    clearing_endpoint_margin_ = declare_parameter("clearing_endpoint_margin", 0.10);
    if (!std::isfinite(clearing_endpoint_margin_) || clearing_endpoint_margin_ < 0.0) {
      throw std::invalid_argument("clearing_endpoint_margin must be finite and nonnegative");
    }
    publish_terrain_map_ = declare_parameter("publish_terrain_map", true);
    terrain_publish_interval_ =
      declare_parameter("terrain_publish_interval", 0.50);

    AnalyzerParameters analyzer_parameters;
    analyzer_parameters.resolution = declare_parameter("grid_resolution", 0.20);
    analyzer_parameters.radius = declare_parameter("grid_radius", 5.0);
    analyzer_parameters.fit_radius_cells =
      declare_parameter("fit_radius_cells", 2);
    analyzer_parameters.minimum_fit_cells =
      declare_parameter("minimum_fit_cells", 3);
    analyzer_parameters.minimum_support_points =
      declare_parameter("minimum_support_points", 5);
    analyzer_parameters.ground_height_percentile =
      declare_parameter("ground_height_percentile", 0.20);
    analyzer_parameters.positive_outlier_tolerance =
      declare_parameter("positive_outlier_tolerance", 0.10);
    analyzer_parameters.maximum_plane_rmse =
      declare_parameter("maximum_plane_rmse", 0.08);
    analyzer_parameters.maximum_slope =
      std::tan(declare_parameter("maximum_slope_degrees", 30.0) * M_PI / 180.0);
    analyzer_parameters.maximum_step_height =
      declare_parameter("maximum_step_height", 0.18);
    analyzer_parameters.obstacle_minimum_height =
      declare_parameter("obstacle_minimum_height", 0.15);
    analyzer_parameters.obstacle_maximum_height =
      declare_parameter("obstacle_maximum_height", 1.80);
    analyzer_parameters.minimum_obstacle_cluster_points =
      declare_parameter("minimum_obstacle_cluster_points", 3);
    analyzer_parameters.obstacle_cluster_radius_cells =
      declare_parameter("obstacle_cluster_radius_cells", 1);
    analyzer_parameters.synthetic_obstacle_clearance_radius =
      declare_parameter("synthetic_obstacle_clearance_radius", 0.65);
    analyzer_ = std::make_unique<TerrainAnalyzer>(analyzer_parameters);

    NearObstacleParameters near_parameters;
    near_parameters.enabled = declare_parameter("near_obstacle_detection_enabled", false);
    near_parameters.minimum_range = std::max(input_minimum_range_, scan_range_minimum_);
    near_parameters.maximum_range = std::min({
      declare_parameter("near_obstacle_maximum_range", 1.20),
      input_maximum_range_, scan_range_maximum_});
    near_parameters.minimum_relative_z =
      declare_parameter("near_obstacle_minimum_relative_z", -0.35);
    near_parameters.maximum_relative_z =
      declare_parameter("near_obstacle_maximum_relative_z", 0.40);
    const auto self_bounds = declare_parameter<std::vector<double>>(
      "near_obstacle_self_bounds", std::vector<double>{-0.40, 0.40, -0.25, 0.25});
    if (self_bounds.size() != 4) {
      throw std::invalid_argument("near_obstacle_self_bounds requires [min_x, max_x, min_y, max_y]");
    }
    near_parameters.self_minimum_x = self_bounds[0];
    near_parameters.self_maximum_x = self_bounds[1];
    near_parameters.self_minimum_y = self_bounds[2];
    near_parameters.self_maximum_y = self_bounds[3];
    near_parameters.minimum_obstacle_height = analyzer_parameters.obstacle_minimum_height;
    near_parameters.maximum_ground_slope = analyzer_parameters.maximum_slope;
    near_parameters.minimum_cluster_points =
      std::max(3, analyzer_parameters.minimum_obstacle_cluster_points);
    if (near_parameters.enabled) {
      near_obstacle_detector_ = std::make_unique<NearObstacleDetector>(near_parameters);
    }

    HazardStabilizerParameters stabilizer_parameters;
    stabilizer_parameters.voxel_size =
      declare_parameter("global_hazard_voxel_size", 0.20);
    stabilizer_parameters.update_radius = analyzer_parameters.radius;
    stabilizer_parameters.measured_enter_observations =
      declare_parameter("global_measured_enter_observations", 2);
    stabilizer_parameters.synthetic_enter_observations =
      declare_parameter("global_synthetic_enter_observations", 3);
    stabilizer_parameters.synthetic_clearance_radius =
      analyzer_parameters.synthetic_obstacle_clearance_radius;
    stabilizer_parameters.exit_missed_observations =
      declare_parameter("global_exit_missed_observations", 3);
    hazard_stabilizer_ =
      std::make_unique<TerrainHazardStabilizer>(stabilizer_parameters);

    cloud_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    odometry_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = cloud_callback_group_;
    rclcpp::SubscriptionOptions odometry_options;
    odometry_options.callback_group = odometry_callback_group_;

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(2);
    cloud_subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, sensor_qos,
      std::bind(&TerrainCostmapNode::cloudCallback, this, std::placeholders::_1),
      cloud_options);
    odometry_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::QoS(20),
      std::bind(
        &TerrainCostmapNode::odometryCallback, this,
        std::placeholders::_1),
      odometry_options);

    terrain_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      terrain_topic_, rclcpp::SensorDataQoS().keep_last(1));
    support_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      support_topic_, rclcpp::SensorDataQoS().keep_last(1));
    obstacle_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      obstacle_topic_, rclcpp::SensorDataQoS().keep_last(1));
    global_obstacle_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      global_obstacle_topic_, rclcpp::SensorDataQoS().keep_last(1));
    measured_global_obstacle_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      measured_global_obstacle_topic_,
      rclcpp::SensorDataQoS().keep_last(1));
    non_traversable_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      non_traversable_topic_, rclcpp::SensorDataQoS().keep_last(1));
    clearing_scan_publisher_ =
      create_publisher<sensor_msgs::msg::LaserScan>(
      clearing_scan_topic_, rclcpp::SensorDataQoS().keep_last(1));

    RCLCPP_INFO(
      get_logger(),
      "Ramp-aware terrain analysis: %s -> %s and %s",
      cloud_topic_.c_str(), obstacle_topic_.c_str(),
      clearing_scan_topic_.c_str());
  }

private:
  struct Pose
  {
    double stamp{0.0};
    tf2::Transform odom_from_body;
    bool valid{false};
  };

  struct CloudFrame
  {
    double stamp{0.0};
    std::vector<Point> points;
    // Already detected from this scan alone, in odom with the ORIGINAL source
    // time. Retain evidence for the measured window; never refit old near scans.
    std::vector<TerrainPoint> near_obstacles;
  };

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id != odom_frame_ ||
      message->child_frame_id != body_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Expected odometry %s -> %s, received %s -> %s",
        odom_frame_.c_str(), body_frame_.c_str(),
        message->header.frame_id.c_str(), message->child_frame_id.c_str());
    }
    Pose pose;
    tf2::fromMsg(message->pose.pose, pose.odom_from_body);
    pose.stamp = stampSeconds(message->header.stamp);
    pose.valid = true;
    const std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!pose_history_.empty() &&
      pose.stamp + 1e-6 < pose_history_.back().stamp)
    {
      pose_history_.clear();
    }
    pose_history_.push_back(pose);
    while (!pose_history_.empty() &&
      pose.stamp - pose_history_.front().stamp > pose_history_duration_)
    {
      pose_history_.pop_front();
    }
    pose_condition_.notify_all();
  }

  bool poseAt(double stamp, Pose & pose, double & age)
  {
    std::unique_lock<std::mutex> lock(pose_mutex_);
    if (poseAtLocked(stamp, pose, age)) {
      return true;
    }
    // Odometry uses a separate executor callback group. Waiting releases
    // pose_mutex_, so a late matching pose can arrive without blocking it.
    // This is a wall-clock deadline, including when bag playback is paused.
    const bool recovered = pose_condition_.wait_for(
      lock, std::chrono::duration<double>(pose_wait_timeout_),
      [&]() {return poseAtLocked(stamp, pose, age);});
    lock.unlock();
    if (recovered) {
      ++pose_wait_recovered_;
    } else {
      ++pose_wait_expired_;
    }
    return recovered;
  }

  // Called with pose_mutex_ held, including from the waiting predicate.
  bool poseAtLocked(double stamp, Pose & pose, double & age)
  {
    if (pose_history_.empty()) {
      age = -1.0;
      return false;
    }

    const auto next = std::lower_bound(
      pose_history_.begin(), pose_history_.end(), stamp,
      [](const Pose & candidate, double time) {
        return candidate.stamp < time;
      });
    // A pose at the exact cloud timestamp needs no interpolation endpoint.
    // A gap before that pose must not make this exact match look stale.
    if (next != pose_history_.end() && next->stamp == stamp) {
      pose = *next;
      age = 0.0;
      return true;
    }
    if (next == pose_history_.begin()) {
      pose = *next;
      age = std::abs(stamp - pose.stamp);
      return age <= maximum_pose_age_;
    }
    if (next == pose_history_.end()) {
      pose = pose_history_.back();
      age = std::abs(stamp - pose.stamp);
      return age <= maximum_pose_age_;
    }

    const Pose & before = *std::prev(next);
    const Pose & after = *next;
    const double interval = after.stamp - before.stamp;
    if (interval <= 1e-9) {
      pose = after;
      age = std::abs(stamp - after.stamp);
      return age <= maximum_pose_age_;
    }

    const double ratio = std::clamp(
      (stamp - before.stamp) / interval, 0.0, 1.0);
    const tf2::Vector3 origin =
      before.odom_from_body.getOrigin().lerp(
      after.odom_from_body.getOrigin(), ratio);
    tf2::Quaternion rotation =
      before.odom_from_body.getRotation().slerp(
      after.odom_from_body.getRotation(), ratio);
    rotation.normalize();
    pose.stamp = stamp;
    pose.odom_from_body.setOrigin(origin);
    pose.odom_from_body.setRotation(rotation);
    pose.valid = true;
    age = std::max(stamp - before.stamp, after.stamp - stamp);
    return age <= maximum_pose_age_;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const double cloud_stamp = stampSeconds(message->header.stamp);
    const auto cloud_stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (cloud_stamp_ns <= 0) {return;}
    if (last_processed_cloud_time_ >= 0.0 && cloud_stamp < last_processed_cloud_time_) {
      if (last_processed_cloud_time_ - cloud_stamp <= history_decay_time_) {return;}
      // Bag restarts must not mix the previous epoch's future points into the
      // new fit, nor carry confirmations across the timestamp discontinuity.
      history_.clear();
      hazard_stabilizer_->clear();
      last_terrain_publish_time_ = -1.0;
      last_processed_cloud_time_ = -1.0;
    }
    if (last_processed_cloud_time_ >= 0.0 &&
      cloud_stamp >= last_processed_cloud_time_ &&
      cloud_stamp - last_processed_cloud_time_ < processing_interval_)
    {
      return;
    }
    last_processed_cloud_time_ = cloud_stamp;
    Pose pose;
    double pose_age = -1.0;
    if (!poseAt(cloud_stamp, pose, pose_age))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping terrain cloud: no bracketing FAST-LIO pose within %.3f s "
      "(nearest age %.3f s, waited up to %.3f wall s, expired=%zu)",
      maximum_pose_age_, pose_age, pose_wait_timeout_, pose_wait_expired_);
      return;
    }
    if (message->header.frame_id != body_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Expected cloud frame '%s', received '%s'",
        body_frame_.c_str(), message->header.frame_id.c_str());
      return;
    }

    tf2::Transform base_from_body;
    try {
      const geometry_msgs::msg::TransformStamped transform =
        tf_buffer_.lookupTransform(base_frame_, body_frame_, tf2::TimePointZero);
      tf2::fromMsg(transform.transform, base_from_body);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for %s <- %s static transform: %s",
        base_frame_.c_str(), body_frame_.c_str(), exception.what());
      return;
    }

    CloudFrame frame;
    frame.stamp = cloud_stamp;
    // Keep current measured returns before terrain voxel/range/height filtering.
    // Even an unclassified or too-close return must stop a clearing ray.
    // The near detector performs its own bounded, base-frame self filtering.
    std::vector<Point> current_returns_base;
    current_returns_base.reserve(message->width * message->height);
    std::unordered_set<std::uint64_t> occupied_voxels;
    occupied_voxels.reserve(message->width * message->height / 2);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        const tf2::Vector3 point_body(*x, *y, *z);
        const tf2::Vector3 point_base = base_from_body * point_body;
        current_returns_base.push_back(
          {point_base.x(), point_base.y(), point_base.z(), cloud_stamp_ns});
        const double range = std::hypot(point_base.x(), point_base.y());
        if (range < input_minimum_range_ || range > input_maximum_range_ ||
          point_base.z() < input_minimum_relative_z_ ||
          point_base.z() > input_maximum_relative_z_)
        {
          continue;
        }
        const int vx = static_cast<int>(std::floor(*x / input_voxel_size_)) + 512;
        const int vy = static_cast<int>(std::floor(*y / input_voxel_size_)) + 512;
        const int vz = static_cast<int>(std::floor(*z / input_voxel_size_)) + 128;
        if (vx < 0 || vx >= 1024 || vy < 0 || vy >= 1024 ||
          vz < 0 || vz >= 256)
        {
          continue;
        }
        const std::uint64_t key =
          (static_cast<std::uint64_t>(vx) << 18) |
          (static_cast<std::uint64_t>(vy) << 8) |
          static_cast<std::uint64_t>(vz);
        if (!occupied_voxels.insert(key).second) {
          continue;
        }
        const tf2::Vector3 point_odom =
          pose.odom_from_body * point_body;
        frame.points.push_back(
          {point_odom.x(), point_odom.y(), point_odom.z(), cloud_stamp_ns});
      }
    } catch (const std::runtime_error & exception) {
      RCLCPP_ERROR(
        get_logger(), "Invalid PointCloud2 layout: %s", exception.what());
      return;
    }

    const tf2::Transform odom_from_base =
      pose.odom_from_body * base_from_body.inverse();
    const auto near_obstacles = near_obstacle_detector_ ?
      near_obstacle_detector_->detect(current_returns_base) : std::vector<TerrainPoint>{};
    frame.near_obstacles.reserve(near_obstacles.size());
    for (const TerrainPoint & obstacle : near_obstacles) {
      // Global collision checks require physical XYZ, not the relative-height
      // Z used by the local marking cloud. Use this scan's FAST-LIO pose and
      // static base/body transform, not a latest TF or the EKF pose.
      const auto & point = obstacle.point;
      const tf2::Vector3 world = odom_from_base * tf2::Vector3(point.x, point.y, point.z);
      frame.near_obstacles.push_back(
        {{world.x(), world.y(), world.z(), point.observation_stamp_ns},
          obstacle.relative_height, false});
    }

    history_.push_back(std::move(frame));
    while (!history_.empty() &&
      cloud_stamp - history_.front().stamp > history_decay_time_)
    {
      history_.pop_front();
    }

    std::size_t point_count = 0;
    for (const CloudFrame & cloud : history_) {
      point_count += cloud.points.size();
    }
    std::vector<Point> merged;
    merged.reserve(point_count);
    for (const CloudFrame & cloud : history_) {
      merged.insert(merged.end(), cloud.points.begin(), cloud.points.end());
    }

    // Terrain and its near-vehicle confidence region must follow the chassis
    // rotation center, not the offset LiDAR/IMU body origin.
    const tf2::Vector3 base_origin = odom_from_base.getOrigin();
    const auto analysis_start = std::chrono::steady_clock::now();
    const Analysis result =
      analyzer_->analyze(merged, base_origin.x(), base_origin.y());
    const double analysis_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - analysis_start).count();
    std::vector<TerrainPoint> measured_obstacles = result.measured_obstacles;
    for (const CloudFrame & cloud : history_) {
      measured_obstacles.insert(measured_obstacles.end(),
        cloud.near_obstacles.begin(), cloud.near_obstacles.end());
    }
    const auto fresh_measured = freshMeasuredObstacles(
      measured_obstacles, cloud_stamp_ns,
      static_cast<std::int64_t>(std::llround(measured_obstacle_decay_time_ * 1e9)));
    const std::vector<TerrainPoint> stable_global_obstacles =
      hazard_stabilizer_->update(
      fresh_measured, result.synthetic_obstacles,
      base_origin.x(), base_origin.y(), cloud_stamp_ns);

    std_msgs::msg::Header odom_header = message->header;
    odom_header.frame_id = odom_frame_;
    // An expired return must not wait for the slower terrain/support display
    // interval before the offline planner receives its replacement snapshot.
    if (publish_terrain_map_) {
      measured_global_obstacle_publisher_->publish(makeCloud(fresh_measured, odom_header));
    }
    if (publish_terrain_map_ &&
      (last_terrain_publish_time_ < 0.0 ||
      cloud_stamp - last_terrain_publish_time_ >= terrain_publish_interval_))
    {
      terrain_publisher_->publish(makeCloud(result.terrain, odom_header));
      support_publisher_->publish(
        makeCloud(result.support_surface, odom_header));
      global_obstacle_publisher_->publish(
        makeCloud(stable_global_obstacles, odom_header));
      non_traversable_publisher_->publish(
        makeCloud(result.non_traversable, odom_header));
      last_terrain_publish_time_ = cloud_stamp;
    }

    const tf2::Transform body_from_odom =
      pose.odom_from_body.inverse();
    // Local marking still uses only this scan's near detections. Global streams
    // above also receive them, with physical XYZ and bounded original-time age.
    // Neither branch creates support or modifies the fixed navigation map.
    std::vector<Point> obstacles_base;
    obstacles_base.reserve(result.obstacles.size() + near_obstacles.size());
    for (const TerrainPoint & obstacle : result.obstacles) {
      const Point & point = obstacle.point;
      const tf2::Vector3 point_body =
        body_from_odom * tf2::Vector3(point.x, point.y, point.z);
      const tf2::Vector3 point_base = base_from_body * point_body;
      const double range = std::hypot(point_base.x(), point_base.y());
      if (range >= scan_range_minimum_ && range <= scan_range_maximum_)
      {
        obstacles_base.push_back(
          {point_base.x(), point_base.y(), obstacle.relative_height});
      }
    }
    for (const TerrainPoint & obstacle : near_obstacles) {
      obstacles_base.push_back(
        {obstacle.point.x, obstacle.point.y, obstacle.relative_height,
          obstacle.point.observation_stamp_ns});
    }

    std_msgs::msg::Header base_header = message->header;
    base_header.frame_id = base_frame_;
    obstacle_publisher_->publish(makeCloud(obstacles_base, base_header));
    clearing_scan_publisher_->publish(
      makeClearingScan(current_returns_base, obstacles_base, base_header));

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Terrain: frames=%zu points=%zu classified=%zu local_obstacles=%zu "
      "near_obstacles=%zu measured=%zu fresh_measured=%zu measured_window=%.3f synthetic=%zu "
      "stable_global=%zu tracked=%zu "
      "near_synthetic_suppressed=%zu support=%zu analysis_ms=%.3f "
      "cells_observed=%zu cells_unobserved=%zu "
      "reject_fit_neighbors=%zu reject_fit_degenerate=%zu "
      "reject_support=%zu reject_slope=%zu reject_rmse=%zu reject_step=%zu "
      "pose_wait_recovered=%zu pose_wait_expired=%zu",
      history_.size(), merged.size(), result.terrain.size(),
      obstacles_base.size(), near_obstacles.size(), measured_obstacles.size(),
      fresh_measured.size(), measured_obstacle_decay_time_,
      result.synthetic_obstacles.size(), stable_global_obstacles.size(),
      hazard_stabilizer_->trackedVoxelCount(),
      result.suppressed_near_synthetic_obstacles, result.support_surface.size(),
      analysis_ms, result.diagnostics.observed_cells, result.diagnostics.unobserved_cells,
      result.diagnostics.insufficient_fit_cells, result.diagnostics.degenerate_fit_cells,
      result.diagnostics.insufficient_support_cells, result.diagnostics.slope_exceeded_cells,
      result.diagnostics.rmse_exceeded_cells, result.diagnostics.step_exceeded_cells,
      pose_wait_recovered_, pose_wait_expired_);
  }

  sensor_msgs::msg::LaserScan makeClearingScan(
    const std::vector<Point> & current_returns,
    const std::vector<Point> & obstacles,
    const std_msgs::msg::Header & header) const
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header = header;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = scan_angle_increment_;
    scan.range_min = scan_range_minimum_;
    scan.range_max = scan_range_maximum_;
    const std::size_t bin_count = static_cast<std::size_t>(
      std::floor((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1;
    EvidenceClearingParameters parameters;
    parameters.range_min = scan.range_min;
    parameters.range_max = scan.range_max;
    parameters.angle_min = scan.angle_min;
    parameters.angle_increment = scan.angle_increment;
    parameters.bin_count = bin_count;
    parameters.endpoint_margin = clearing_endpoint_margin_;
    scan.ranges = makeEvidenceClearingRanges(current_returns, obstacles, parameters);
    return scan;
  }

  std::string cloud_topic_;
  std::string odometry_topic_;
  std::string terrain_topic_;
  std::string support_topic_;
  std::string obstacle_topic_;
  std::string global_obstacle_topic_;
  std::string measured_global_obstacle_topic_;
  std::string non_traversable_topic_;
  std::string clearing_scan_topic_;
  std::string odom_frame_;
  std::string body_frame_;
  std::string base_frame_;
  double history_decay_time_{2.0};
  double measured_obstacle_decay_time_{0.6};
  double processing_interval_{0.2};
  double maximum_pose_age_{0.2};
  double pose_wait_timeout_{0.1};
  std::size_t pose_wait_recovered_{0};
  std::size_t pose_wait_expired_{0};
  double input_minimum_range_{0.4};
  double input_maximum_range_{8.0};
  double input_minimum_relative_z_{-1.0};
  double input_maximum_relative_z_{2.0};
  double input_voxel_size_{0.1};
  double scan_angle_increment_{0.5 * M_PI / 180.0};
  double scan_range_minimum_{0.4};
  double scan_range_maximum_{8.0};
  double clearing_endpoint_margin_{0.10};
  bool publish_terrain_map_{true};
  double terrain_publish_interval_{0.5};
  double last_terrain_publish_time_{-1.0};
  double last_processed_cloud_time_{-1.0};

  std::mutex pose_mutex_;
  std::condition_variable pose_condition_;
  std::deque<Pose> pose_history_;
  double pose_history_duration_{2.0};
  std::deque<CloudFrame> history_;
  std::unique_ptr<TerrainAnalyzer> analyzer_;
  std::unique_ptr<NearObstacleDetector> near_obstacle_detector_;
  std::unique_ptr<TerrainHazardStabilizer> hazard_stabilizer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odometry_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    terrain_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    support_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    obstacle_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    global_obstacle_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    measured_global_obstacle_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    non_traversable_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
    clearing_scan_publisher_;
};

}  // namespace terrain_costmap

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<terrain_costmap::TerrainCostmapNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
