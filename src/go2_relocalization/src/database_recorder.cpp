#include "go2_relocalization/registration.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iterator>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace go2_relocalization
{
namespace
{
struct StampedPose
{
  int64_t stamp_ns;
  Eigen::Vector3d translation;
  Eigen::Quaterniond orientation;
};

bool readStamp(const builtin_interfaces::msg::Time & stamp, int64_t & result)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {return false;}
  result = static_cast<int64_t>(stamp.sec) * 1000000000ll + stamp.nanosec;
  return result > 0;
}

bool readPose(const geometry_msgs::msg::Pose & pose, StampedPose & result)
{
  result.translation = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.orientation = Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  const double norm = result.orientation.norm();
  if (!result.translation.allFinite() || result.translation.norm() >= 1000000.0 ||
    !result.orientation.coeffs().allFinite() ||
    !std::isfinite(norm) || std::abs(norm - 1.0) > 0.001)
  {
    return false;
  }
  result.orientation.normalize();
  return true;
}

double yaw(const Eigen::Matrix3d & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}
}  // namespace

class DatabaseRecorder : public rclcpp::Node
{
public:
  DatabaseRecorder()
  : Node("relocalization_recorder")
  {
    output_directory_ = declare_parameter<std::string>("output_directory", "");
    const auto cloud_topic = declare_parameter<std::string>(
      "cloud_topic", "/cloud_registered_body");
    const auto odometry_topic = declare_parameter<std::string>("odometry_topic", "/Odometry");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.2);
    max_range_ = declare_parameter<double>("max_range", 20.0);
    min_points_ = declare_parameter<int>("min_points", 100);
    keyframe_distance_ = declare_parameter<double>("keyframe_distance", 1.0);
    keyframe_yaw_ = declare_parameter<double>("keyframe_yaw", 0.2617993877991494);
    max_pose_age_ = declare_parameter<double>("max_pose_age", 0.05);
    max_keyframes_ = declare_parameter<int>("max_keyframes", 5000);
    if (output_directory_.empty() || body_frame_.empty() || odom_frame_.empty() ||
      cloud_topic.empty() || odometry_topic.empty() ||
      !positive(voxel_size_) || !positive(max_range_) || !positive(keyframe_distance_) ||
      !positive(keyframe_yaw_) || keyframe_yaw_ > 3.141592653589793 ||
      voxel_size_ > max_range_ || max_range_ > 100000.0 ||
      !positive(max_pose_age_) || max_pose_age_ > 0.5 || min_points_ < 3 ||
      max_keyframes_ < 1 || max_keyframes_ > 5000)
    {
      throw std::invalid_argument("Invalid recorder parameters; output_directory is required");
    }
    if (std::filesystem::exists(output_directory_)) {
      throw std::invalid_argument("output_directory already exists; recorder refuses overwriting");
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort().durability_volatile();
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {onOdometry(*message);});
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {onCloud(std::move(message));});
    save_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "~/save",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {save(*response);},
      rmw_qos_profile_services_default, save_group_);
    RCLCPP_INFO(
      get_logger(),
      "Recording local body keyframes with timestamp-matched %s poses. "
      "Export with /relocalization_recorder/save to %s. "
      "The saved map coordinates equal this odometry session; no TF or control is published.",
      odom_frame_.c_str(), output_directory_.c_str());
  }

private:
  static bool positive(double value) {return std::isfinite(value) && value > 0.0;}

  void onOdometry(const nav_msgs::msg::Odometry & message)
  {
    StampedPose pose;
    if (message.header.frame_id != odom_frame_ || message.child_frame_id != body_frame_ ||
      !readStamp(message.header.stamp, pose.stamp_ns) || !readPose(message.pose.pose, pose))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected odometry: expected valid nonzero stamp, normalized pose, %s -> %s",
        odom_frame_.c_str(), body_frame_.c_str());
      return;
    }
    if (!poses_.empty() && pose.stamp_ns <= poses_.back().stamp_ns) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected duplicate or backward odometry timestamp");
      return;
    }
    poses_.push_back(pose);
    const int64_t history_ns = static_cast<int64_t>(
      std::max(2.0, 4.0 * max_pose_age_) * 1000000000.0);
    while (poses_.size() > 2000 ||
      (poses_.size() > 2 && poses_.back().stamp_ns - poses_.front().stamp_ns > history_ns))
    {
      poses_.pop_front();
    }
    processPendingClouds();
  }

  bool poseAt(int64_t stamp, Eigen::Isometry3d & transform) const
  {
    if (poses_.empty()) {return false;}
    const int64_t tolerance_ns = static_cast<int64_t>(max_pose_age_ * 1000000000.0);
    auto upper = std::lower_bound(
      poses_.begin(), poses_.end(), stamp,
      [](const StampedPose & pose, int64_t value) {return pose.stamp_ns < value;});
    StampedPose selected;
    if (upper != poses_.end() && upper->stamp_ns == stamp) {
      selected = *upper;
    } else if (upper != poses_.begin() && upper != poses_.end()) {
      const auto & before = *std::prev(upper);
      const auto & after = *upper;
      if (stamp - before.stamp_ns > tolerance_ns || after.stamp_ns - stamp > tolerance_ns) {
        return false;
      }
      const double fraction = static_cast<double>(stamp - before.stamp_ns) /
        static_cast<double>(after.stamp_ns - before.stamp_ns);
      selected.translation = before.translation + fraction * (after.translation - before.translation);
      selected.orientation = before.orientation.slerp(fraction, after.orientation);
    } else {
      selected = upper == poses_.end() ? poses_.back() : poses_.front();
      if (std::abs(selected.stamp_ns - stamp) > tolerance_ns) {return false;}
    }
    transform = Eigen::Isometry3d::Identity();
    transform.linear() = selected.orientation.toRotationMatrix();
    transform.translation() = selected.translation;
    return true;
  }

  void onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    int64_t stamp;
    if (message->header.frame_id != body_frame_ || !readStamp(message->header.stamp, stamp) ||
      stamp <= last_seen_cloud_stamp_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected cloud: expected body frame and increasing nonzero timestamp");
      return;
    }
    last_seen_cloud_stamp_ = stamp;
    pending_clouds_.push_back(std::move(message));
    // Body points may be delivered before the same-stamp odometry publication.
    // Wait for it without deriving a pose from message arrival time.
    while (pending_clouds_.size() > 3) {
      pending_clouds_.pop_front();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Dropped pending cloud: timestamp-matched odometry missing");
    }
    processPendingClouds();
  }

  void processPendingClouds()
  {
    const int64_t tolerance_ns = static_cast<int64_t>(max_pose_age_ * 1000000000.0);
    while (!pending_clouds_.empty()) {
      if (poses_.empty()) {return;}
      int64_t stamp{0};
      readStamp(pending_clouds_.front()->header.stamp, stamp);
      Eigen::Isometry3d pose;
      if (!poseAt(stamp, pose)) {
        if (poses_.back().stamp_ns - stamp <= tolerance_ns) {return;}
        pending_clouds_.pop_front();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Cloud skipped: no odometry pose within %.3f s of its header timestamp", max_pose_age_);
        continue;
      }
      auto message = pending_clouds_.front();
      pending_clouds_.pop_front();
      recordCloud(*message, pose);
    }
  }

  void recordCloud(
    const sensor_msgs::msg::PointCloud2 & message, const Eigen::Isometry3d & pose)
  {
    {
      std::lock_guard<std::mutex> lock(database_mutex_);
      if (database_.keyframes.size() >= static_cast<size_t>(max_keyframes_)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Reached max_keyframes; export before recording another session");
        return;
      }
      if (!database_.keyframes.empty()) {
        const auto & previous = database_.keyframes.back().T_map_body;
        const double yaw_difference = std::remainder(
          yaw(pose.rotation()) - yaw(previous.rotation()), 2.0 * 3.141592653589793);
        if ((pose.translation() - previous.translation()).norm() < keyframe_distance_ &&
          std::abs(yaw_difference) < keyframe_yaw_)
        {
          return;
        }
      }
    }

    bool has_x = false, has_y = false, has_z = false;
    bool malformed_fields = false;
    for (const auto & field : message.fields) {
      const bool numeric = field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        field.count == 1 && static_cast<uint64_t>(field.offset) + 4 <= message.point_step;
      if (field.name == "x") {malformed_fields = malformed_fields || has_x || !numeric; has_x = numeric;}
      if (field.name == "y") {malformed_fields = malformed_fields || has_y || !numeric; has_y = numeric;}
      if (field.name == "z") {malformed_fields = malformed_fields || has_z || !numeric; has_z = numeric;}
    }
    if (message.is_bigendian || malformed_fields || !has_x || !has_y || !has_z || message.point_step == 0 ||
      static_cast<uint64_t>(message.row_step) * message.height > message.data.size() ||
      static_cast<uint64_t>(message.width) * message.point_step > message.row_step)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected malformed XYZ cloud");
      return;
    }
    Cloud raw;
    try {
      pcl::fromROSMsg(message, raw);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Cannot decode point cloud: %s", error.what());
      return;
    }
    Cloud::Ptr finite(new Cloud);
    finite->reserve(raw.size());
    const double range_squared = max_range_ * max_range_;
    for (const auto & point : raw) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
        point.getVector3fMap().squaredNorm() <= range_squared)
      {
        finite->push_back(point);
      }
    }
    Cloud::Ptr reduced(new Cloud);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel.setInputCloud(finite);
    voxel.filter(*reduced);
    if (reduced->size() < static_cast<size_t>(min_points_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Skipped keyframe with only %zu filtered points", reduced->size());
      return;
    }
    Keyframe keyframe;
    keyframe.T_map_body = pose;
    keyframe.cloud = reduced;
    size_t count;
    {
      std::lock_guard<std::mutex> lock(database_mutex_);
      std::ostringstream id;
      id << "k" << std::setw(6) << std::setfill('0') << database_.keyframes.size();
      keyframe.id = id.str();
      database_.keyframes.push_back(std::move(keyframe));
      count = database_.keyframes.size();
    }
    RCLCPP_INFO(get_logger(), "Recorded keyframe %zu with %zu body points", count, reduced->size());
  }

  void save(std_srvs::srv::Trigger::Response & response)
  {
    Database snapshot;
    {
      std::lock_guard<std::mutex> lock(database_mutex_);
      snapshot = database_;
    }
    if (snapshot.keyframes.empty()) {
      response.success = false;
      response.message = "No valid keyframes recorded; nothing exported";
      return;
    }
    // Keep the persistent full map bounded without discarding the actual local
    // scan templates. Building it on the save callback leaves subscriptions free.
    try {
      constexpr size_t max_map_points = 10000000;
      constexpr size_t merge_batch_points = 1000000;
      Cloud::Ptr map(new Cloud);
      const auto reduce = [this](Cloud::Ptr input) {
          Cloud::Ptr output(new Cloud);
          pcl::VoxelGrid<pcl::PointXYZ> voxel;
          voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
          voxel.setInputCloud(input);
          voxel.filter(*output);
          return output;
        };
      for (const auto & keyframe : snapshot.keyframes) {
        Cloud transformed;
        pcl::transformPointCloud(
          *keyframe.cloud, transformed, keyframe.T_map_body.matrix().cast<float>());
        *map += transformed;
        if (map->size() >= merge_batch_points) {map = reduce(map);}
        if (map->size() > max_map_points) {
          throw std::runtime_error("Voxelized map exceeds 10000000 points; split the recording session");
        }
      }
      snapshot.global_cloud = reduce(map);
    } catch (const std::exception & error) {
      response.success = false;
      response.message = std::string("Map export preparation failed: ") + error.what();
      return;
    }
    std::string error;
    response.success = writeDatabase(output_directory_, snapshot, error);
    response.message = response.success ?
      "Saved " + std::to_string(snapshot.keyframes.size()) + " keyframes to " + output_directory_ : error;
    if (response.success) {RCLCPP_INFO(get_logger(), "%s", response.message.c_str());}
    else {RCLCPP_ERROR(get_logger(), "Database export failed: %s", error.c_str());}
  }

  std::string output_directory_, body_frame_, odom_frame_;
  double voxel_size_, max_range_, keyframe_distance_, keyframe_yaw_, max_pose_age_;
  int min_points_, max_keyframes_;
  int64_t last_seen_cloud_stamp_{0};
  std::deque<StampedPose> poses_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_clouds_;
  std::mutex database_mutex_;
  Database database_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::CallbackGroup::SharedPtr save_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
};
}  // namespace go2_relocalization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto recorder = std::make_shared<go2_relocalization::DatabaseRecorder>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(recorder);
    executor.spin();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("relocalization_recorder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
