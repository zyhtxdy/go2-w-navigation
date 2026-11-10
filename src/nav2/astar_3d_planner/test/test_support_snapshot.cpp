#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "astar_3d_planner/astar_3d_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace astar_3d_planner
{
namespace
{
using namespace std::chrono_literals;

class SupportSnapshot : public ::testing::Test
{
protected:
  void start(bool fixed)
  {
    context_ = rclcpp::contexts::get_global_default_context();
    rclcpp::InitOptions init;
    init.set_domain_id(149);
    context_->init(0, nullptr, init);
    rclcpp::NodeOptions costmap_options;
    costmap_options.context(context_);
    costmap_options.parameter_overrides({
      {"global_frame", "map"}, {"robot_base_frame", "base_link"},
      {"plugins", std::vector<std::string>{}}, {"filters", std::vector<std::string>{}}});
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_options);
    ASSERT_EQ(costmap_->on_configure(rclcpp_lifecycle::State()),
      nav2_util::CallbackReturn::SUCCESS);
    rclcpp::NodeOptions options;
    options.context(context_);
    options.parameter_overrides({
      {"GridBased.use_static_map", fixed}, {"GridBased.compensate_height_drift", false},
      {"GridBased.support_topic", "/support_snapshot_test/input"},
      {"GridBased.static_obstacle_topic", "/support_snapshot_test/static_obstacles"},
      {"GridBased.obstacle_topic", "/support_snapshot_test/live_obstacles"},
      {"GridBased.footprint_check_enabled", false}});
    parent_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("support_snapshot_parent", options);
    input_ = std::make_shared<rclcpp::Node>("support_snapshot_input",
      rclcpp::NodeOptions().context(context_));
    auto tf = std::make_shared<tf2_ros::Buffer>(parent_->get_clock());
    planner_ = std::make_unique<Astar3DPlanner>();
    planner_->configure(parent_, "GridBased", tf, costmap_);
    auto qos = fixed ? rclcpp::QoS(1).reliable().transient_local() :
      rclcpp::QoS(rclcpp::SensorDataQoS());
    publisher_ = input_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/support_snapshot_test/input", qos);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(parent_->get_node_base_interface());
    executor_->add_node(input_);
    ASSERT_TRUE(wait([this]() {return publisher_->get_subscription_count() == 1;}));
  }

  void TearDown() override
  {
    if (planner_) {planner_->deactivate(); planner_->cleanup();}
    planner_.reset();
    executor_.reset();
    publisher_.reset();
    subscription_.reset();
    input_.reset();
    parent_.reset();
    if (costmap_) {costmap_->on_cleanup(rclcpp_lifecycle::State());}
    costmap_.reset();
    if (context_ && context_->is_valid()) {context_->shutdown("snapshot test finished");}
  }

  void spin(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some();
      std::this_thread::sleep_for(2ms);
    }
  }

  bool wait(const std::function<bool()> & predicate)
  {
    const auto end = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some();
      if (predicate()) {return true;}
      std::this_thread::sleep_for(2ms);
    }
    return predicate();
  }

  void publishSupport()
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header.frame_id = "map";
    message.header.stamp = input_->now();
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(3);
    sensor_msgs::PointCloud2Iterator<float> x(message, "x"), y(message, "y"), z(message, "z");
    for (int i = 0; i < 3; ++i) {
      *x = .05f + i * .2f; *y = .05f; *z = .01f; ++x; ++y; ++z;
    }
    publisher_->publish(message);
    spin(250ms);
  }

  void observe(bool fixed)
  {
    auto qos = fixed ? rclcpp::QoS(1).reliable().transient_local() :
      rclcpp::QoS(rclcpp::SensorDataQoS());
    subscription_ = input_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/planner/support_map", qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        messages_.push_back(*message);
      });
  }

  void checkQos(bool fixed)
  {
    const auto publishers = input_->get_publishers_info_by_topic("/planner/support_map");
    ASSERT_EQ(publishers.size(), 1u);
    const auto qos = publishers.front().qos_profile();
    EXPECT_EQ(qos.reliability(), fixed ? rclcpp::ReliabilityPolicy::Reliable :
      rclcpp::ReliabilityPolicy::BestEffort);
    EXPECT_EQ(qos.durability(), fixed ? rclcpp::DurabilityPolicy::TransientLocal :
      rclcpp::DurabilityPolicy::Volatile);
  }

  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_;
  rclcpp::Node::SharedPtr input_;
  std::unique_ptr<Astar3DPlanner> planner_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::vector<sensor_msgs::msg::PointCloud2> messages_;
};

TEST_F(SupportSnapshot, FixedSupportLoadedBeforeActivationIsPublishedAndRetained)
{
  start(true);
  checkQos(true);
  // The single upstream map publication happens while the plugin is inactive.
  publishSupport();
  observe(true);
  spin(100ms);
  EXPECT_TRUE(messages_.empty());
  planner_->activate();
  ASSERT_TRUE(wait([this]() {return !messages_.empty();}));
  EXPECT_EQ(messages_.back().header.frame_id, "map");
  EXPECT_EQ(messages_.back().width, 3u);
  EXPECT_EQ(messages_.back().height, 1u);
  const auto payload = messages_.back().data;
  subscription_.reset();
  messages_.clear();
  // A later RViz-style subscriber receives the retained snapshot without any
  // second upstream support message or activation.
  observe(true);
  ASSERT_TRUE(wait([this]() {return !messages_.empty();}));
  EXPECT_EQ(messages_.back().data, payload);
  EXPECT_EQ(messages_.back().width, 3u);
}

TEST_F(SupportSnapshot, ReactivationRepublishesFixedSupportWithoutNewInput)
{
  start(true);
  planner_->activate();
  observe(true);
  publishSupport();
  ASSERT_TRUE(wait([this]() {return !messages_.empty();}));
  const auto previous_stamp = rclcpp::Time(messages_.back().header.stamp);
  planner_->deactivate();
  messages_.clear();
  spin(50ms);
  planner_->activate();
  ASSERT_TRUE(wait([this]() {return !messages_.empty();}));
  EXPECT_GT(rclcpp::Time(messages_.back().header.stamp), previous_stamp);
  EXPECT_EQ(messages_.back().width, 3u);
}

TEST_F(SupportSnapshot, OnlineSnapshotRetainsOriginalVolatileSensorQos)
{
  start(false);
  checkQos(false);
  planner_->activate();
  publishSupport();
  observe(false);
  spin(200ms);
  EXPECT_TRUE(messages_.empty());
  // Online publication still follows new support updates and its 1 s limit.
  spin(1s);
  publishSupport();
  ASSERT_TRUE(wait([this]() {return !messages_.empty();}));
  EXPECT_EQ(messages_.back().width, 3u);
}

}  // namespace
}  // namespace astar_3d_planner
