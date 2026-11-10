#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class LioBaseOdomNode : public rclcpp::Node
{
public:
  LioBaseOdomNode()
  : Node("lio_base_odom"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto input_topic =
      this->declare_parameter<std::string>("input_topic", "/Odometry");
    const auto output_topic =
      this->declare_parameter<std::string>("output_topic", "/lio/odom_base");
    body_frame_ = this->declare_parameter<std::string>("body_frame", "body");
    base_frame_ =
      this->declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    position_stddev_ =
      this->declare_parameter<double>("position_stddev", 0.05);
    yaw_stddev_ =
      this->declare_parameter<double>("yaw_stddev", 0.02);
    max_initial_distance_ =
      this->declare_parameter<double>("max_initial_distance", 5.0);
    max_planar_speed_ =
      this->declare_parameter<double>("max_planar_speed", 5.0);

    odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>(output_topic, 20);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_topic, 20,
      std::bind(&LioBaseOdomNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Converting %s (%s->%s) to %s (%s->%s)",
      input_topic.c_str(), odom_frame_.c_str(), body_frame_.c_str(),
      output_topic.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id != odom_frame_ ||
      msg->child_frame_id != body_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Expected %s->%s but received %s->%s",
        odom_frame_.c_str(), body_frame_.c_str(),
        msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
      // Relabeling a different producer's coordinates would silently corrupt
      // the EKF. Only the configured odom -> body measurement is meaningful.
      return;
    }

    geometry_msgs::msg::TransformStamped body_to_base_msg;
    try {
      // lookupTransform(target, source) returns T_target_source.
      body_to_base_msg = tf_buffer_.lookupTransform(
        body_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for static %s->%s calibration: %s",
        body_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    tf2::Quaternion q_odom_body;
    tf2::fromMsg(msg->pose.pose.orientation, q_odom_body);
    if (q_odom_body.length2() < 1.0e-12) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring FAST-LIO odometry with invalid orientation");
      return;
    }
    q_odom_body.normalize();

    const tf2::Transform odom_to_body(
      q_odom_body,
      tf2::Vector3(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z));

    tf2::Transform body_to_base;
    tf2::fromMsg(body_to_base_msg.transform, body_to_base);
    const tf2::Transform odom_to_base = odom_to_body * body_to_base;
    const auto & base_position = odom_to_base.getOrigin();

    if (!std::isfinite(base_position.x()) ||
      !std::isfinite(base_position.y()) ||
      !std::isfinite(base_position.z()))
    {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Rejecting non-finite FAST-LIO pose");
      return;
    }

    const double planar_distance =
      std::hypot(base_position.x(), base_position.y());
    if (!have_last_pose_ && planar_distance > max_initial_distance_) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Rejecting divergent initial FAST-LIO pose (x=%.3f, y=%.3f)",
        base_position.x(), base_position.y());
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    if (have_last_pose_) {
      const double dt = (stamp - last_stamp_).seconds();
      if (dt <= 0.0) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Rejecting non-increasing FAST-LIO timestamp");
        return;
      }
      const double step = std::hypot(
        base_position.x() - last_x_,
        base_position.y() - last_y_);
      if (step / dt > max_planar_speed_) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Rejecting FAST-LIO jump: %.3f m in %.3f s (%.3f m/s)",
          step, dt, step / dt);
        return;
      }
    }

    nav_msgs::msg::Odometry output;
    output.header = msg->header;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = odom_to_base.getOrigin().x();
    output.pose.pose.position.y = odom_to_base.getOrigin().y();
    output.pose.pose.position.z = odom_to_base.getOrigin().z();
    output.pose.pose.orientation =
      tf2::toMsg(odom_to_base.getRotation().normalized());

    // FAST-LIO's internal error-state ordering and orientation perturbation do
    // not map directly to nav_msgs/Odometry.  Copying that matrix made
    // robot_localization see yaw variances in the millions after divergence.
    // Publish an explicit measurement model for the planar pose consumed by
    // the navigation EKF.
    output.pose.covariance.fill(0.0);
    output.pose.covariance[0] = position_stddev_ * position_stddev_;
    output.pose.covariance[7] = position_stddev_ * position_stddev_;
    output.pose.covariance[14] = 1.0e6;
    output.pose.covariance[21] = 1.0e6;
    output.pose.covariance[28] = 1.0e6;
    output.pose.covariance[35] = yaw_stddev_ * yaw_stddev_;

    // This adapter supplies LIO pose only, not a velocity measurement.
    output.twist.covariance.fill(0.0);
    for (std::size_t i = 0; i < 6; ++i) {
      output.twist.covariance[i * 6 + i] = 1.0e6;
    }

    odom_pub_->publish(output);
    have_last_pose_ = true;
    last_stamp_ = stamp;
    last_x_ = base_position.x();
    last_y_ = base_position.y();
  }

  std::string body_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  double position_stddev_;
  double yaw_stddev_;
  double max_initial_distance_;
  double max_planar_speed_;
  bool have_last_pose_{false};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  double last_x_{0.0};
  double last_y_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

#ifndef FAST_LIO_NAV2_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioBaseOdomNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
