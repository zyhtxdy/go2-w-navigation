#include "go2_relocalization/registration.hpp"
#include "go2_relocalization/runtime.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace go2_relocalization
{
namespace
{
double seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1e-9 * stamp.nanosec;
}
bool validStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0 && stamp.nanosec < 1000000000 && seconds(stamp) > 0.0;
}
bool readPose(const geometry_msgs::msg::Pose & msg, Eigen::Isometry3d & pose)
{
  Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.0) > 0.01) {return false;}
  pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
  pose.linear() = q.normalized().toRotationMatrix();
  return pose.matrix().allFinite();
}
bool validCloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (cloud.is_bigendian || cloud.width == 0 || cloud.height == 0 || cloud.point_step == 0 ||
    static_cast<uint64_t>(cloud.row_step) < static_cast<uint64_t>(cloud.width) * cloud.point_step ||
    cloud.data.size() < static_cast<uint64_t>(cloud.height) * cloud.row_step)
  {return false;}
  for (const std::string field : {"x", "y", "z"}) {
    const auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(),
      [&field](const auto & f) {return f.name == field;});
    if (it == cloud.fields.end() || it->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      it->count != 1 || it->offset + sizeof(float) > cloud.point_step) {return false;}
  }
  return true;
}
geometry_msgs::msg::Pose poseMessage(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.translation().x();
  result.position.y = pose.translation().y();
  result.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  result.orientation.x = q.x(); result.orientation.y = q.y();
  result.orientation.z = q.z(); result.orientation.w = q.w();
  return result;
}
}  // namespace

class RelocalizationNode : public rclcpp::Node
{
public:
  RelocalizationNode() : Node("relocalization")
  {
    const auto directory = declare_parameter("database_directory", "");
    if (directory.empty()) {throw std::invalid_argument("database_directory is required");}
    map_frame_ = declare_parameter("map_frame", "map");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    body_frame_ = declare_parameter("body_frame", "body");
    if (map_frame_.empty() || odom_frame_.empty() || body_frame_.empty() ||
      map_frame_ == odom_frame_ || odom_frame_ == body_frame_ || map_frame_ == body_frame_)
    {throw std::invalid_argument("map, odom and body frames must be distinct and nonempty");}
    publish_tf_ = declare_parameter("publish_tf", false);
    if (publish_tf_) {
      throw std::invalid_argument(
              "This observation prototype does not publish TF. Keep publish_tf=false; "
              "map->odom integration needs a separate launch and navigation-map changes.");
    }
    processing_interval_ = positive("processing_interval", 0.2);
    global_check_interval_ = positive("global_check_interval", 1.0);
    max_pose_age_ = positive("max_pose_age", 0.05);
    max_input_age_ = positive("max_input_age", 3.0);
    tracking_timeout_ = positive("tracking_timeout", 3.0);
    confirmation_timeout_ = positive("confirmation_timeout", tracking_timeout_);
    max_global_proposal_age_ = positive("max_global_proposal_age", 8.0);
    max_global_verification_age_ = positive("max_global_verification_age", 12.0);
    max_global_result_age_ = positive("max_global_result_age", 6.0);
    max_tracking_translation_ = positive("max_tracking_translation", 0.75);
    max_tracking_rotation_ = positive("max_tracking_rotation", 0.35);
    const int confirmations = declare_parameter("confirmation_frames", 3);
    if (confirmations < 2) {throw std::invalid_argument("confirmation_frames must be at least 2");}
    confirmation_ = std::make_unique<ConfirmationGate>(confirmations,
      positive("confirmation_interval", 0.2), positive("confirmation_translation", 0.30),
      positive("confirmation_rotation", 0.15), confirmation_timeout_);
    RegistrationConfig c;
    c.voxel_size = declare_parameter("registration.voxel_size", c.voxel_size);
    c.max_range = declare_parameter("registration.max_range", c.max_range);
    c.top_k = declare_parameter("registration.top_k", c.top_k);
    c.ring_count = declare_parameter("registration.ring_count", c.ring_count);
    c.sector_count = declare_parameter("registration.sector_count", c.sector_count);
    c.descriptor_height_offset = declare_parameter(
      "registration.descriptor_height_offset", c.descriptor_height_offset);
    c.max_correspondence_distance = declare_parameter(
      "registration.max_correspondence_distance", c.max_correspondence_distance);
    c.max_iterations = declare_parameter("registration.max_iterations", c.max_iterations);
    c.min_points = declare_parameter("registration.min_points", c.min_points);
    c.evaluation_distance = declare_parameter("registration.evaluation_distance", c.evaluation_distance);
    c.min_overlap = declare_parameter("registration.min_overlap", c.min_overlap);
    c.max_rmse = declare_parameter("registration.max_rmse", c.max_rmse);
    c.min_quality_score = declare_parameter("registration.min_quality_score", c.min_quality_score);
    c.ambiguity_distance = declare_parameter("registration.ambiguity_distance", c.ambiguity_distance);
    c.ambiguity_yaw = declare_parameter("registration.ambiguity_yaw", c.ambiguity_yaw);
    c.min_score_gap = declare_parameter("registration.min_score_gap", c.min_score_gap);
    c.min_observability_ratio = declare_parameter(
      "registration.min_observability_ratio", c.min_observability_ratio);
    c.normal_radius = declare_parameter("registration.normal_radius", c.normal_radius);
    c.min_normal_neighbors = declare_parameter("registration.min_normal_neighbors", c.min_normal_neighbors);
    c.max_normal_curvature = declare_parameter("registration.max_normal_curvature", c.max_normal_curvature);
    c.min_normal_samples = declare_parameter("registration.min_normal_samples", c.min_normal_samples);
    c.cache_registration_features = declare_parameter(
      "registration.cache_registration_features", c.cache_registration_features);
    c.candidate_threads = declare_parameter("registration.candidate_threads", c.candidate_threads);
    Database database;
    std::string error;
    if (!validateConfig(c, error) || !loadDatabase(directory, database, error)) {
      throw std::invalid_argument(error);
    }
    registration_config_ = c;
    relocalizer_ = std::make_unique<Relocalizer>(std::move(database), c);
    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 1);
    transform_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>("~/map_to_odom", 1);
    candidate_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
      "~/candidate_map_to_odom", 10);
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/aligned_cloud", rclcpp::SensorDataQoS());
    known_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/known_map", state_qos);
    if (declare_parameter("publish_known_map", false)) {
      sensor_msgs::msg::PointCloud2 map;
      pcl::toROSMsg(*relocalizer_->database().global_cloud, map);
      map.header.frame_id = map_frame_; map.header.stamp = now();
      known_map_pub_->publish(map);
    }
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("~/valid", state_qos);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/status", state_qos);
    const auto cloud_topic = declare_parameter("cloud_topic", "/cloud_registered_body");
    const auto odometry_topic = declare_parameter("odometry_topic", "/Odometry");
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic,
      rclcpp::SensorDataQoS().keep_last(2),
      std::bind(&RelocalizationNode::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odometry_topic,
      rclcpp::SensorDataQoS().keep_last(100),
      std::bind(&RelocalizationNode::odometryCallback, this, std::placeholders::_1));
    reset_service_ = create_service<std_srvs::srv::Trigger>("~/relocalize",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        invalidate("SEARCHING", "manual_global_search");
        response->success = true;
        response->message = "Global search queued; no TF or movement command is published.";
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const double age = now().seconds() - last_verified_stamp_;
        if (valid_ && (age > tracking_timeout_ || age < -0.2)) {
          resetStateLocked("LOST", "tracking_timeout");
        } else if (valid_ && now().seconds() - last_global_verified_stamp_ > max_global_verification_age_) {
          resetStateLocked("LOST", "global_verification_timeout");
        } else if (!valid_ && confirmation_->count() > 0 &&
          confirmationIdleExpired(now().seconds(), last_confirmation_stamp_,
          last_confirmation_completed_, confirmation_timeout_))
        {
          resetStateLocked("SEARCHING", "confirmation_timeout");
        }
        if (valid_ && now().seconds() - last_global_verified_stamp_ >
          0.75 * max_global_verification_age_)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Global verification nearing deadline: age=%.3fs limit=%.3fs "
            "stage=%s job_elapsed_ms=%.1f last_outcome=%s last_status=%s "
            "last_input_age=%.3fs last_job_ms=%.1f rmse=%.4f overlap=%.4f "
            "quality=%.4f score_gap=%.4f normal_information_ratio=%.6f candidates=%zu",
            now().seconds() - last_global_verified_stamp_, max_global_verification_age_,
            global_job_stage_.c_str(), globalJobElapsedMillisecondsLocked(),
            last_global_outcome_.c_str(), last_global_result_.status.c_str(),
            last_global_input_age_, last_global_job_ms_, last_global_result_.rmse,
            last_global_result_.overlap, last_global_result_.quality_score,
            last_global_result_.score_gap, last_global_result_.observability_ratio,
            last_global_result_.candidates.size());
        }
        publishStatusLocked();
      });
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      publishStatusLocked();
    }
    try {
      global_worker_ = std::thread(&RelocalizationNode::globalWork, this);
      worker_ = std::thread(&RelocalizationNode::work, this);
    } catch (...) {
      {std::lock_guard<std::mutex> lock(data_mutex_); stopping_ = true;}
      data_cv_.notify_all();
      if (global_worker_.joinable()) {global_worker_.join();}
      throw;
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu keyframes; observation only, no TF or robot commands",
      relocalizer_->database().keyframes.size());
  }

  ~RelocalizationNode() override
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      stopping_ = true;
    }
    data_cv_.notify_all();
    if (worker_.joinable()) {worker_.join();}
    if (global_worker_.joinable()) {global_worker_.join();}
  }

private:
  double positive(const std::string & name, double value)
  {
    value = declare_parameter(name, value);
    if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument(name + " must be positive");}
    return value;
  }
  bool fresh(double stamp)
  {
    const double age = get_clock()->now().seconds() - stamp;
    return age >= -0.2 && age <= max_input_age_;
  }
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Eigen::Isometry3d pose;
    if (msg->header.frame_id != odom_frame_ || msg->child_frame_id != body_frame_ ||
      !validStamp(msg->header.stamp) || !readPose(msg->pose.pose, pose))
    {invalidate("LOST", "invalid_odometry_frame_stamp_or_pose"); return;}
    const double stamp = seconds(msg->header.stamp);
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto previous = history_.latest();
    if (previous && (stamp < previous->stamp - 0.2 ||
      (stamp > previous->stamp && ((pose.translation() - previous->pose.translation()).norm() > 5.0 ||
      (stamp - previous->stamp < 0.2 && rotationDistance(pose, previous->pose) > 1.0)))))
    {
      history_.clear(); pending_.reset(); global_pending_.reset(); verification_.reset();
      last_processed_stamp_ = last_global_input_stamp_ = -1.0;
      invalidate("SEARCHING", "odometry_discontinuity");
    }
    history_.add(stamp, pose);
    data_cv_.notify_all();
  }
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id != body_frame_ || !validStamp(msg->header.stamp) || !validCloud(*msg)) {
      invalidate("LOST", "invalid_cloud_frame_stamp_or_layout"); return;
    }
    const double stamp = seconds(msg->header.stamp);
    if (!fresh(stamp)) {invalidate("LOST", "cloud_clock_mismatch_or_stale"); return;}
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      global_pending_ = msg;
      if (stamp <= last_processed_stamp_) {return;}
      pending_ = msg;
    }
    data_cv_.notify_all();
  }
  void resetStateLocked(const std::string & state, const std::string & reason)
  {
    if (valid_ || tracking_ || confirmation_->count() > 0) {
      logGlobalContextLocked("Localization invalidated", reason);
    }
    ++generation_; valid_ = false; tracking_ = false; last_global_stamp_ = -1.0;
    last_global_completed_ = -1.0;
    confirmation_->reset(); state_ = state; reason_ = reason;
    qualified_candidate_stamp_ = -1.0;
  }
  double globalJobElapsedMillisecondsLocked() const
  {
    return global_busy_ || verification_busy_ ? std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - global_job_started_wall_).count() : 0.0;
  }
  double measurementAge(double stamp)
  {
    return stamp > 0.0 ? now().seconds() - stamp : -1.0;
  }
  void logGlobalContextLocked(const char * event, const std::string & reason)
  {
    // Keep the evidence in ordinary ROS logs, even when nobody subscribes to
    // diagnostics before a navigation cancellation occurs.
    RCLCPP_WARN(get_logger(),
      "%s: reason=%s tracking_age=%.3fs global_age=%.3fs global_limit=%.3fs "
      "stage=%s job_input_stamp=%.6f job_elapsed_ms=%.1f last_outcome=%s "
      "last_status=%s last_input_stamp=%.6f last_input_age=%.3fs last_job_ms=%.1f "
      "cycle_ms=%.1f rmse=%.4f overlap=%.4f quality=%.4f score_gap=%.4f "
      "normal_information_ratio=%.6f candidates=%zu stale_results=%zu generation_discards=%zu",
      event, reason.c_str(), measurementAge(last_verified_stamp_),
      measurementAge(last_global_verified_stamp_), max_global_verification_age_,
      global_job_stage_.c_str(), global_job_input_stamp_, globalJobElapsedMillisecondsLocked(),
      last_global_outcome_.c_str(), last_global_result_.status.c_str(),
      last_global_result_input_stamp_, last_global_input_age_, last_global_job_ms_,
      last_global_cycle_seconds_ * 1000.0, last_global_result_.rmse,
      last_global_result_.overlap, last_global_result_.quality_score,
      last_global_result_.score_gap, last_global_result_.observability_ratio,
      last_global_result_.candidates.size(), discarded_stale_verifications_,
      discarded_generation_results_);
  }
  void invalidate(const std::string & state, const std::string & reason)
  {
    ++input_epoch_;
    std::lock_guard<std::mutex> lock(state_mutex_);
    resetStateLocked(state, reason);
    publishStatusLocked();
  }
  void publishStatusLocked()
  {
    if (!rclcpp::ok()) {return;}
    std_msgs::msg::Bool validity; validity.data = valid_; valid_pub_->publish(validity);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "relocalization/map_match"; status.hardware_id = "software_prototype";
    status.level = valid_ ? status.OK : status.WARN;
    status.message = state_;
    const auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = value;
        status.values.push_back(item);
      };
    add("valid", valid_ ? "true" : "false"); add("reason", reason_);
    add("confirmation_count", std::to_string(confirmation_->count()));
    add("rmse_m", std::to_string(last_result_.rmse));
    add("overlap", std::to_string(last_result_.overlap));
    add("normal_information_ratio", std::to_string(last_result_.observability_ratio));
    add("last_verified_stamp", std::to_string(last_verified_stamp_));
    add("registration_ms", std::to_string(last_registration_ms_));
    add("registration_candidates", std::to_string(last_candidate_count_));
    add("quality_score", std::to_string(last_result_.quality_score));
    add("quality_score_threshold", std::to_string(registration_config_.min_quality_score));
    add("quality_score_margin", std::to_string(
      last_result_.quality_score - registration_config_.min_quality_score));
    add("best_candidate_id", last_result_.best_candidate_id);
    add("alternative_score", std::to_string(last_result_.alternative_score));
    add("score_gap", std::to_string(last_result_.score_gap));
    add("min_score_gap", std::to_string(registration_config_.min_score_gap));
    add("converged", !last_result_.candidates.empty() &&
      last_result_.candidates.front().converged ? "true" : "false");
    add("min_overlap", std::to_string(registration_config_.min_overlap));
    add("max_rmse_m", std::to_string(registration_config_.max_rmse));
    add("global_search", last_search_global_ ? "true" : "false");
    add("odometry_prior_used", last_prior_used_ ? "true" : "false");
    add("last_global_completed_stamp", std::to_string(last_global_completed_));
    add("global_registration_ms", std::to_string(last_global_registration_ms_));
    add("global_candidates", std::to_string(last_global_candidate_count_));
    add("global_search_busy", global_busy_ ? "true" : "false");
    add("global_verification_busy", verification_busy_ ? "true" : "false");
    add("global_verification_ms", std::to_string(last_verification_ms_));
    add("global_verification_candidates", std::to_string(last_verification_candidate_count_));
    add("discarded_stale_verifications", std::to_string(discarded_stale_verifications_));
    add("tracking_age_s", std::to_string(measurementAge(last_verified_stamp_)));
    add("global_verification_age_s", std::to_string(measurementAge(last_global_verified_stamp_)));
    add("global_verification_limit_s", std::to_string(max_global_verification_age_));
    add("global_result_age_limit_s",std::to_string(std::min(
      max_global_result_age_,max_global_verification_age_)));
    add("foreground_input_age_limit_s",std::to_string(max_input_age_));
    add("global_job_stage", global_job_stage_);
    add("global_job_input_stamp", std::to_string(global_job_input_stamp_));
    add("global_job_elapsed_ms", std::to_string(globalJobElapsedMillisecondsLocked()));
    add("global_result_status", last_global_result_.status);
    add("global_result_outcome", last_global_outcome_);
    add("global_result_input_stamp", std::to_string(last_global_result_input_stamp_));
    add("global_result_input_age_s", std::to_string(last_global_input_age_));
    add("global_result_job_ms", std::to_string(last_global_job_ms_));
    add("global_cycle_ms", std::to_string(last_global_cycle_seconds_ * 1000.0));
    add("global_result_accepted", last_global_result_.accepted ? "true" : "false");
    add("global_result_rmse_m", std::to_string(last_global_result_.rmse));
    add("global_result_overlap", std::to_string(last_global_result_.overlap));
    add("global_result_quality_score", std::to_string(last_global_result_.quality_score));
    add("global_result_score_gap", std::to_string(last_global_result_.score_gap));
    add("global_result_normal_information_ratio", std::to_string(last_global_result_.observability_ratio));
    add("global_result_best_candidate_id", last_global_result_.best_candidate_id);
    add("matching_stage", matching_stage_);
    add("global_proposal_stamp", std::to_string(last_proposal_stamp_));
    add("last_global_verified_stamp", std::to_string(last_global_verified_stamp_));
    add("qualified_candidate_stamp", std::to_string(qualified_candidate_stamp_));
    add("discarded_generation_results", std::to_string(discarded_generation_results_));
    add("tf_published", "false");
    array.status.push_back(status); status_pub_->publish(array);
  }
  void publishMatch(const sensor_msgs::msg::PointCloud2 & input, const Cloud & cloud,
    const MatchResult & match, const Eigen::Isometry3d & map_to_odom)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = input.header; pose.header.frame_id = map_frame_;
    pose.pose = poseMessage(match.pose); pose_pub_->publish(pose);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header; transform.child_frame_id = odom_frame_;
    const auto value = poseMessage(map_to_odom);
    transform.transform.translation.x = value.position.x;
    transform.transform.translation.y = value.position.y;
    transform.transform.translation.z = value.position.z;
    transform.transform.rotation = value.orientation;
    transform_pub_->publish(transform);  // A result topic, not /tf.
    if (aligned_pub_->get_subscription_count() > 0) {
      Cloud aligned;
      // Limit visualization point count independently from registration.
      Cloud display;
      const size_t stride = std::max<size_t>(1, (cloud.size() + 5999) / 6000);
      for (size_t i = 0; i < cloud.size(); i += stride) {display.push_back(cloud[i]);}
      pcl::transformPointCloud(display, aligned, match.pose.matrix().cast<float>());
      sensor_msgs::msg::PointCloud2 output; pcl::toROSMsg(aligned, output);
      output.header = pose.header; aligned_pub_->publish(output);
    }
  }
  struct GlobalProposal
  {
    MatchResult result;
    Eigen::Isometry3d odom_pose;
    double stamp;
    uint64_t epoch, generation;
  };
  struct GlobalVerification
  {
    MatchResult result;
    Eigen::Isometry3d odom_pose;
    sensor_msgs::msg::PointCloud2::SharedPtr input;
    uint64_t epoch, generation;
    double milliseconds, proposal_stamp;
  };

  // Retrieval AND all-hypothesis verification share one background worker.
  // Neither operation can monopolize the lightweight tracking worker.
  void globalWork()
  {
    std::optional<GlobalProposal> active;
    while (rclcpp::ok()) {
      sensor_msgs::msg::PointCloud2::SharedPtr input;
      Eigen::Isometry3d odom_pose, prediction;
      uint64_t epoch, generation;
      bool use_prior, search;
      {
        std::unique_lock<std::mutex> lock(data_mutex_);
        data_cv_.wait_for(lock, std::chrono::milliseconds(50));
        if (stopping_) {return;}
        // Let the foreground consume a completed verification before starting
        // another startup confirmation. Keep both mailboxes bounded.
        if (!global_pending_ || verification_) {continue;}
        epoch = input_epoch_.load();
        {
          std::lock_guard<std::mutex> state_lock(state_mutex_);
          generation = generation_;
          if (active && (active->epoch != epoch || active->generation != generation)) {active.reset();}
          // An active proposal finishes its fresh verification even if startup
          // confirmation has just transitioned to tracking.
          search = !active.has_value();
          if (search && !globalCheckDueBeforeDeadline(tracking_, now().seconds(),
              last_global_completed_, global_check_interval_, last_global_verified_stamp_,
              max_global_verification_age_, std::max(last_global_cycle_seconds_,
              last_global_registration_ms_ * 0.001), processing_interval_ + max_pose_age_))
          {continue;}
          use_prior = tracking_ && fresh(last_verified_stamp_);
          prediction = map_to_odom_;
        }
        input = global_pending_;
        const double stamp = seconds(input->header.stamp);
        if (stamp <= last_global_input_stamp_ || (active && stamp <= active->stamp)) {continue;}
        const auto pose = history_.at(stamp, max_pose_age_);
        if (!pose || !fresh(stamp)) {continue;}
        odom_pose = *pose; last_global_input_stamp_ = stamp;
      }
      try {
        if (active && !globalProposalFresh(now().seconds(), active->stamp, max_global_proposal_age_)) {
          active.reset(); continue;  // Expired seeds never become published poses.
        }
        Cloud cloud; pcl::fromROSMsg(*input, cloud);
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          global_busy_ = search; verification_busy_ = !search;
          global_job_stage_ = search ? "retrieval" : "fresh_verification";
          global_job_input_stamp_ = seconds(input->header.stamp);
          global_job_started_wall_ = std::chrono::steady_clock::now();
          if (search) {global_cycle_started_stamp_ = now().seconds();}
        }
        const auto started = std::chrono::steady_clock::now();
        auto result = search ? relocalizer_->globalMatch(cloud,
          use_prior ? std::optional<Eigen::Isometry3d>(prediction * odom_pose) : std::nullopt) :
          relocalizer_->verifyGlobalProposal(cloud, active->result, active->odom_pose.inverse() * odom_pose);
        const double milliseconds = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
        if (!rclcpp::ok()) {return;}
        bool tracking;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          global_busy_ = verification_busy_ = false;
          global_job_stage_ = "idle";
          last_global_result_ = result;
          last_global_result_input_stamp_ = seconds(input->header.stamp);
          last_global_input_age_ = now().seconds() - last_global_result_input_stamp_;
          last_global_job_ms_ = milliseconds;
          last_global_outcome_ = "completed";
          if (search) {
            last_global_registration_ms_ = milliseconds;
            last_global_candidate_count_ = result.candidates.size();
            last_prior_used_ = use_prior;
          } else {
            last_verification_ms_ = milliseconds;
            last_verification_candidate_count_ = result.candidates.size();
          }
          if (generation != generation_ || epoch != input_epoch_.load()) {
            last_global_outcome_ = "generation_discarded";
            ++discarded_generation_results_; active.reset(); continue;
          }
          tracking = tracking_;
          // Wait between complete retrieval+verification cycles while tracking.
          if (!result.accepted || globalResultFresh(now().seconds(),seconds(input->header.stamp),
              tracking,max_input_age_,max_global_result_age_,max_global_verification_age_)) {
            last_global_completed_ = now().seconds();
            last_global_cycle_seconds_ = std::max(0.0,
              last_global_completed_ - global_cycle_started_stamp_);
          }
        }
        // A complete global registration is already an all-hypothesis check.
        // Reverify a new scan only if retrieval took longer than the original
        // input-age budget; never repeat an expensive check unnecessarily.
        if (result.accepted && !globalResultFresh(now().seconds(),seconds(input->header.stamp),
            tracking,max_input_age_,max_global_result_age_,max_global_verification_age_)) {
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ++discarded_stale_verifications_;
            last_global_outcome_ = "accepted_stale_seed_only";
            logGlobalContextLocked("Background verification kept as seed only", "input_age_exceeded");
          }
          // A slow fresh-verification job must not throw away its entire
          // candidate pool and repeat retrieval plus the normal quiet period.
          // Rescore ALL candidates on the newest real scan instead. No stale
          // measurement is published or used to extend either validity gate.
          active = GlobalProposal{std::move(result), odom_pose, seconds(input->header.stamp), epoch, generation};
          continue;
        }
        if (search) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          last_verification_ms_ = milliseconds;
          last_verification_candidate_count_ = result.candidates.size();
        }
        const double proposal_stamp = active ? active->stamp : seconds(input->header.stamp);
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          if (stopping_) {return;}
          if (epoch != input_epoch_.load()) {active.reset(); continue;}
          verification_ = GlobalVerification{result, odom_pose, input, epoch, generation,
            milliseconds, proposal_stamp};
        }
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          last_global_outcome_ = result.accepted ? "queued_accepted" : "queued_rejected";
        }
        // Startup needs distinct, globally verified frames. Reuse the entire
        // newly scored candidate pool as the next initialization, never scores.
        if (result.accepted && !tracking) {
          active = GlobalProposal{std::move(result), odom_pose, seconds(input->header.stamp), epoch, generation};
        } else {active.reset();}
        data_cv_.notify_all();
      } catch (const std::exception & error) {
        if (!rclcpp::ok()) {return;}
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          global_busy_ = verification_busy_ = false;
          global_job_stage_ = "idle";
          last_global_outcome_ = "exception";
        }
        active.reset();
        invalidate("LOST", std::string("global_matching_exception: ") + error.what());
      }
    }
  }

  void work()
  {
    while (true) {
      sensor_msgs::msg::PointCloud2::SharedPtr input;
      Eigen::Isometry3d odom_pose;
      std::optional<GlobalVerification> verified;
      {
        std::unique_lock<std::mutex> lock(data_mutex_);
        data_cv_.wait(lock, [this]() {return stopping_ || pending_ != nullptr || verification_.has_value();});
        if (stopping_) {return;}
        if (!pending_) {
          bool tracking;
          {std::lock_guard<std::mutex> state_lock(state_mutex_); tracking = tracking_;}
          if (tracking) {
            data_cv_.wait_for(lock, std::chrono::milliseconds(50), [this]() {return stopping_ || pending_;});
            if (stopping_) {return;}
            if (!pending_) {continue;}
          }
        }
        input = pending_ ? std::move(pending_) : verification_->input;
        const double stamp = seconds(input->header.stamp);
        if (!verification_ && stamp - last_processed_stamp_ < processing_interval_) {continue;}
        auto pose = history_.at(stamp, max_pose_age_);
        if (!pose) {
          data_cv_.wait_for(lock, std::chrono::milliseconds(100), [this, stamp]() {
              return stopping_ || history_.at(stamp, max_pose_age_).has_value();
            });
          if (stopping_) {return;}
          pose = history_.at(stamp, max_pose_age_);
        }
        if (!pose) {invalidate("LOST", "no_time_matched_odometry"); continue;}
        odom_pose = *pose; last_processed_stamp_ = stamp;
        if (verification_) {verified = std::move(verification_); verification_.reset();}
      }
      data_cv_.notify_all();
      const uint64_t epoch = input_epoch_.load();
      bool tracking;
      uint64_t generation;
      Eigen::Isometry3d previous_transform;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        tracking = tracking_; generation = generation_; previous_transform = map_to_odom_;
      }
      if (verified && (verified->epoch != epoch || verified->generation != generation)) {verified.reset();}
      if (verified && !globalResultFresh(now().seconds(),seconds(verified->input->header.stamp),
          tracking,max_input_age_,max_global_result_age_,max_global_verification_age_)) {
        // Slow background work is not evidence against a fresh local track.
        // It cannot refresh the global-verification age limit either.
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++discarded_stale_verifications_;
        last_global_outcome_ = "foreground_consumed_stale_result";
        logGlobalContextLocked("Background verification discarded", "foreground_consumed_stale_result");
        verified.reset();
      }
      if (!tracking && !verified) {continue;}
      if (verified && !verified->result.accepted) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (generation == generation_ && epoch == input_epoch_.load()) {
          last_result_ = verified->result;
          resetStateLocked("LOST", verified->result.status); publishStatusLocked();
        }
        continue;
      }
      const bool startup_verification = !tracking && verified.has_value();
      Eigen::Isometry3d prediction = previous_transform;
      if (verified) {prediction = verified->result.pose * verified->odom_pose.inverse();}
      if (startup_verification) {
        // Publish startup confirmation with its ACTUAL globally verified scan,
        // not the timestamp of whichever newer scan woke this worker.
        input = verified->input; odom_pose = verified->odom_pose;
      }
      if (!fresh(seconds(input->header.stamp))) {invalidate("LOST", "input_stale"); continue;}
      try {
        Cloud cloud; pcl::fromROSMsg(*input, cloud);
        const auto started = std::chrono::steady_clock::now();
        auto result = startup_verification ? verified->result :
          relocalizer_->track(cloud, prediction * odom_pose);
        if (!rclcpp::ok()) {return;}
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_registration_ms_ = startup_verification ? verified->milliseconds :
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
        last_candidate_count_ = result.candidates.size();
        last_search_global_ = startup_verification;
        matching_stage_ = startup_verification ? "fresh_global_verification" : "tracking";
        if (generation != generation_ || epoch != input_epoch_.load()) {
          ++discarded_generation_results_; continue;
        }
        last_result_ = result;
        if (!fresh(seconds(input->header.stamp))) {
          resetStateLocked("LOST", "matching_result_stale"); publishStatusLocked(); continue;
        }
        if (!result.accepted) {
          resetStateLocked("LOST", result.status); publishStatusLocked(); continue;
        }
        const Eigen::Isometry3d transform = result.pose * odom_pose.inverse();
        if (tracking && ((transform.translation() - previous_transform.translation()).norm() >
          max_tracking_translation_ || rotationDistance(transform, previous_transform) > max_tracking_rotation_))
        {resetStateLocked("LOST", "tracking_correction_jump"); publishStatusLocked(); continue;}
        if (verified && !globalResultFresh(now().seconds(),seconds(verified->input->header.stamp),
            tracking,max_input_age_,max_global_result_age_,max_global_verification_age_))
        {
          ++discarded_stale_verifications_;
          last_global_outcome_="foreground_completed_after_global_result_budget";
          verified.reset();
        }
        if (verified) {
          last_global_verified_stamp_ = seconds(verified->input->header.stamp);
          last_proposal_stamp_ = verified->proposal_stamp;
          last_global_outcome_ = "applied_fresh_verification";
        }
        qualified_candidate_stamp_ = seconds(input->header.stamp);
        geometry_msgs::msg::TransformStamped candidate;
        candidate.header = input->header;
        candidate.header.frame_id = map_frame_;
        candidate.child_frame_id = odom_frame_;
        const auto candidate_pose = poseMessage(transform);
        candidate.transform.translation.x = candidate_pose.position.x;
        candidate.transform.translation.y = candidate_pose.position.y;
        candidate.transform.translation.z = candidate_pose.position.z;
        candidate.transform.rotation = candidate_pose.orientation;
        candidate_pub_->publish(candidate);
        if (!tracking && !confirmation_->observe(seconds(input->header.stamp), transform)) {
          last_confirmation_stamp_ = seconds(input->header.stamp);
          last_confirmation_completed_ = now().seconds();
          valid_ = false; state_ = "CONFIRMING"; reason_ = "awaiting_distinct_consistent_frames";
          publishStatusLocked(); continue;
        }
        map_to_odom_ = transform; tracking_ = true; valid_ = true;
        last_verified_stamp_ = seconds(input->header.stamp);
        state_ = "TRACKING"; reason_ = result.status;
        publishMatch(*input, cloud, result, transform);
        publishStatusLocked();
      } catch (const std::exception & error) {
        if (!rclcpp::ok()) {return;}
        invalidate("LOST", std::string("matching_exception: ") + error.what());
      }
    }
  }

  std::string map_frame_, odom_frame_, body_frame_;
  bool publish_tf_{false};
  double processing_interval_, global_check_interval_, max_pose_age_, max_input_age_, tracking_timeout_;
  double confirmation_timeout_;
  double max_tracking_translation_, max_tracking_rotation_;
  double max_global_proposal_age_, max_global_verification_age_;
  double max_global_result_age_;
  std::unique_ptr<Relocalizer> relocalizer_;
  RegistrationConfig registration_config_;
  std::unique_ptr<ConfirmationGate> confirmation_;
  std::mutex data_mutex_, state_mutex_;
  std::condition_variable data_cv_;
  bool stopping_{false};
  PoseHistory history_;
  sensor_msgs::msg::PointCloud2::SharedPtr pending_, global_pending_;
  std::optional<GlobalVerification> verification_;
  double last_processed_stamp_{-1.0};
  double last_global_input_stamp_{-1.0};
  std::thread worker_, global_worker_;
  std::atomic<uint64_t> input_epoch_{0};
  uint64_t generation_{0};
  bool tracking_{false}, valid_{false};
  std::string state_{"SEARCHING"}, reason_{"startup_global_search"};
  double last_verified_stamp_{-1.0};
  double qualified_candidate_stamp_{-1.0};
  bool last_search_global_{false}, last_prior_used_{false};
  double last_confirmation_stamp_{-1.0};
  double last_confirmation_completed_{-1.0};
  double last_registration_ms_{0.0};
  std::size_t last_candidate_count_{0};
  std::size_t discarded_generation_results_{0};
  double last_global_stamp_{-1.0};
  double last_global_completed_{-1.0};
  double last_global_registration_ms_{0.0};
  std::size_t last_global_candidate_count_{0};
  std::string global_job_stage_{"idle"}, last_global_outcome_{"not_run"};
  std::chrono::steady_clock::time_point global_job_started_wall_{};
  double global_job_input_stamp_{-1.0};
  double global_cycle_started_stamp_{-1.0}, last_global_cycle_seconds_{0.0};
  double last_global_result_input_stamp_{-1.0}, last_global_input_age_{-1.0}, last_global_job_ms_{0.0};
  MatchResult last_global_result_;
  double last_global_verified_stamp_{-1.0}, last_proposal_stamp_{-1.0};
  bool global_busy_{false}, verification_busy_{false};
  double last_verification_ms_{0.0};
  std::size_t last_verification_candidate_count_{0}, discarded_stale_verifications_{0};
  std::string matching_stage_{"waiting_global"};
  Eigen::Isometry3d map_to_odom_{Eigen::Isometry3d::Identity()};
  MatchResult last_result_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr candidate_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr known_map_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace go2_relocalization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_relocalization::RelocalizationNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("relocalization"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
