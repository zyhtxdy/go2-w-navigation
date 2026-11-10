#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "terrain_navigation_interfaces/srv/validate_terrain_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace navigation_bringup
{
using Service = terrain_navigation_interfaces::srv::ValidateTerrainPath;
using Result = Service::Response;

// A response is atomic: its category belongs to this exact path request, not
// to an asynchronously published diagnostic or a previous service call.
class IsTerrainPathValidCondition : public nav2_behavior_tree::BtServiceNode<Service>
{
public:
  IsTerrainPathValidCondition(const std::string & name, const BT::NodeConfiguration & configuration)
  : BtServiceNode(name, configuration, "/terrain/is_path_valid_detailed") {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<nav_msgs::msg::Path>("path", "Terrain path to check"),
      BT::InputPort<int>("required_invalid_checks", 3, "Consecutive geometric failures"),
      BT::OutputPort<bool>("unavailable", "Inputs/RPC unavailable; do not search"),
      BT::OutputPort<std::string>("failure_reason", "Reason for this request")});
  }

  void on_tick() override
  {
    nav_msgs::msg::Path path;
    const auto input = getInput("path", path);
    if (!input) {
      // Nav2 writes the path blackboard value only after the first planning
      // action succeeds. A valid remapping with no value yet is the normal
      // initial-planning event, just like an explicitly empty Path.
      bool initial_path = false;
      const auto mapping = config().input_ports.find("path");
      if (mapping != config().input_ports.end() && config().blackboard) {
        const auto key = getRemappedKey("path", mapping->second);
        if (key) {
          std::lock_guard<std::mutex> lock(config().blackboard->entryMutex());
          const auto value = config().blackboard->getAny(static_cast<std::string>(key.value()));
          initial_path = !value || value->empty();
        }
      }
      // Missing port wiring, literals and populated entries of the wrong
      // type remain configuration errors; do not silently replace them.
      if (!initial_path) {
        throw BT::RuntimeError("IsTerrainPathValid requires path: ", input.error());
      }
    }
    received_response_ = false;
    request_begin_ = std::chrono::steady_clock::now();
    getInput("required_invalid_checks", required_invalid_checks_);
    required_invalid_checks_ = std::max(1, required_invalid_checks_);
    if (path.poses.size() < 2) {
      consecutive_invalid_checks_ = 0;
      setOutput("unavailable", false);
      setOutput("failure_reason", std::string("initial_plan_required"));
      should_send_request_ = false;
      return;
    }
    const auto & first = path.poses.front().pose.position;
    const auto & last = path.poses.back().pose.position;
    const auto stamp = rclcpp::Time(path.header.stamp).nanoseconds();
    if (!path_initialized_ || stamp != path_stamp_ns_ || path_size_ != path.poses.size() ||
      std::hypot(first.x-first_x_, first.y-first_y_) >= 1e-3 ||
      std::hypot(last.x-last_x_, last.y-last_y_) >= 1e-3)
    {consecutive_invalid_checks_ = 0;}
    path_initialized_ = true; path_stamp_ns_ = stamp; path_size_ = path.poses.size();
    first_x_ = first.x; first_y_ = first.y; last_x_ = last.x; last_y_ = last.y;
    request_->path = path;
  }

  BT::NodeStatus unavailable(const std::string & reason)
  {
    consecutive_invalid_checks_ = 0;
    setOutput("unavailable", true);
    setOutput("failure_reason", reason);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "Terrain validation unavailable: %s; stopping tracking without replanning", reason.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus on_completion(std::shared_ptr<Result> result) override
  {
    received_response_ = true;
    if (result->status == Result::VALID && result->is_valid) {
      setOutput("unavailable", false);
      setOutput("failure_reason", result->reason.empty() ? std::string("valid") : result->reason);
      if (result->reason == "local_detour_active" && last_valid_reason_ != result->reason) {
        RCLCPP_INFO(node_->get_logger(), "Keeping reference path while the local controller detours");
      }
      last_valid_reason_ = result->reason;
      consecutive_invalid_checks_ = 0;
      return BT::NodeStatus::SUCCESS;
    }
    const bool geometric = !result->is_valid &&
      (result->status == Result::COLLISION || result->status == Result::TERRAIN_INVALID ||
       result->status == Result::PATH_MISMATCH || result->status == Result::INVALID_REQUEST);
    if (!geometric) {return unavailable(result->reason.empty() ? "unknown_validation_result" : result->reason);}
    setOutput("unavailable", false);
    setOutput("failure_reason", result->reason);
    if (++consecutive_invalid_checks_ < required_invalid_checks_) {
      RCLCPP_WARN(node_->get_logger(), "Terrain geometry invalid (%d/%d): %s",
        consecutive_invalid_checks_, required_invalid_checks_, result->reason.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(node_->get_logger(), "Terrain geometry repeatedly invalid: %s; replanning",
      result->reason.c_str());
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus check_future() override
  {
    // A paused/jumped ROS clock must not postpone a missing RPC indefinitely.
    if (std::chrono::steady_clock::now()-request_begin_ >= server_timeout_) {
      request_sent_ = false;
      service_client_->prune_pending_requests();
      return unavailable("validation_rpc_timeout");
    }
    try {
      const auto status = BtServiceNode<Service>::check_future();
      if (status == BT::NodeStatus::FAILURE && !received_response_) {
        service_client_->prune_pending_requests();
        return unavailable("validation_rpc_timeout");
      }
      return status;
    } catch (const std::exception & error) {
      request_sent_ = false;
      service_client_->prune_pending_requests();
      return unavailable(std::string("validation_rpc_error: ")+error.what());
    }
  }

private:
  std::string last_valid_reason_;
  int required_invalid_checks_{3}, consecutive_invalid_checks_{0};
  bool path_initialized_{false}, received_response_{false};
  std::int64_t path_stamp_ns_{0};
  std::size_t path_size_{0};
  double first_x_{0}, first_y_{0}, last_x_{0}, last_y_{0};
  std::chrono::steady_clock::time_point request_begin_;
};

class TerrainReplanAllowed : public BT::ConditionNode
{
public:
  TerrainReplanAllowed(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts()
  {return {BT::InputPort<bool>("unavailable", true, "Unknown input state blocks search")};}
  BT::NodeStatus tick() override
  {
    bool missing = true;
    getInput("unavailable", missing);
    return missing ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

// Wrap the *whole* tracking pipeline. On unavailable input its failure first
// halts FollowPath (PipelineSequence contract); only then wait/retry validation.
// Returning RUNNING from the service condition itself would let a previously
// running PipelineSequence continue ticking FollowPath, so do not do that.
class TerrainValidationGuard : public BT::DecoratorNode
{
public:
  TerrainValidationGuard(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config) {}
  static BT::PortsList providedPorts()
  {return {BT::InputPort<bool>("unavailable", false, "Validation unavailable"),
      BT::InputPort<unsigned>("wait_timeout_ms", 5000, "Bounded input recovery wait")};}
  BT::NodeStatus tick() override
  {
    if (waiting_ && std::chrono::steady_clock::now() < next_retry_) {
      return BT::NodeStatus::RUNNING;
    }
    const auto status = child_node_->executeTick();
    bool missing = false; getInput("unavailable", missing);
    if (!missing) {waiting_ = false; return status;}
    if (status != BT::NodeStatus::FAILURE) {return status;}
    resetChild();
    next_retry_ = std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
    if (!waiting_) {waiting_ = true; begin_ = std::chrono::steady_clock::now();}
    unsigned timeout = 5000; getInput("wait_timeout_ms", timeout);
    if (std::chrono::steady_clock::now()-begin_ >= std::chrono::milliseconds(timeout)) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  void halt() override {waiting_ = false; BT::DecoratorNode::halt();}
private:
  bool waiting_{false};
  std::chrono::steady_clock::time_point begin_, next_retry_;
};
}  // namespace navigation_bringup

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigation_bringup::IsTerrainPathValidCondition>("IsTerrainPathValid");
  factory.registerNodeType<navigation_bringup::TerrainReplanAllowed>("TerrainReplanAllowed");
  factory.registerNodeType<navigation_bringup::TerrainValidationGuard>("TerrainValidationGuard");
}
