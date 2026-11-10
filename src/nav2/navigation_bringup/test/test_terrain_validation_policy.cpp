#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "terrain_navigation_interfaces/srv/validate_terrain_path.hpp"

using namespace std::chrono_literals;
using Service = terrain_navigation_interfaces::srv::ValidateTerrainPath;
using Result = Service::Response;
using Status = BT::NodeStatus;

class TerrainPolicy : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::InitOptions options; options.set_domain_id(164); 
    rclcpp::init(0, nullptr, options);
    server = std::make_shared<rclcpp::Node>("terrain_policy_server");
    client_node = std::make_shared<rclcpp::Node>("terrain_policy_client");
    service = server->create_service<Service>("/terrain/is_path_valid_detailed",
      [this](Service::Request::SharedPtr, Service::Response::SharedPtr response) {
        ++calls;
        if (delay_ms.load()) {std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms.load()));}
        response->status = code.load(); response->is_valid = code == Result::VALID;
        response->reason = code == Result::VALID ?
          (local_detour.load() ? "local_detour_active" : "valid") :
          "test_reason_"+std::to_string(code.load());
      });
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(server); worker = std::thread([this]() {executor->spin();});
    factory.registerFromPlugin(TERRAIN_BT_PLUGIN_LIBRARY);
    factory.registerFromPlugin("libnav2_pipeline_sequence_bt_node.so");
    bb = BT::Blackboard::create();
    bb->set("node", client_node); bb->set("bt_loop_duration", 10ms);
    bb->set("server_timeout", 60ms); bb->set("wait_for_service_timeout", 2000ms);
    nav_msgs::msg::Path path; path.header.frame_id = "map";
    path.poses.resize(2); path.poses[1].pose.position.x = 1;
    bb->set("path", path);
  }
  void TearDown() override
  {
    if (executor) {executor->cancel();} if (worker.joinable()) {worker.join();}
    if (executor) {executor->remove_node(server); executor.reset();}
    service.reset(); client_node.reset(); server.reset();
    rclcpp::shutdown();
  }
  // Match Nav2's initial blackboard: runtime context exists, but no planner
  // has produced a path value yet. Registering the tree may create an empty
  // typed entry; that is different from an explicitly stored empty Path.
  void freshBlackboardWithoutPath()
  {
    bb = BT::Blackboard::create();
    bb->set("node", client_node); bb->set("bt_loop_duration", 10ms);
    bb->set("server_timeout", 60ms); bb->set("wait_for_service_timeout", 2000ms);
  }

  BT::Tree condition()
  {
    return factory.createTreeFromText(R"(<root main_tree_to_execute="Main"><BehaviorTree ID="Main">
      <IsTerrainPathValid path="{path}" unavailable="{missing}" failure_reason="{why}"
       service_name="/terrain/is_path_valid_detailed" required_invalid_checks="3" server_timeout="60"/>
      </BehaviorTree></root>)", bb);
  }
  Status finish(BT::Tree & tree)
  {
    auto deadline = std::chrono::steady_clock::now()+2s;
    Status result;
    do {result=tree.tickRoot(); if(result!=Status::RUNNING) {return result;}
      std::this_thread::sleep_for(1ms);
    } while(std::chrono::steady_clock::now()<deadline);
    ADD_FAILURE()<<"Service condition did not finish"; return result;
  }
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr bb;
  rclcpp::Node::SharedPtr server, client_node;
  rclcpp::Service<Service>::SharedPtr service;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread worker;
  std::atomic<int> code{Result::VALID}, calls{0}, delay_ms{0};
  std::atomic<bool> local_detour{false};
};

TEST_F(TerrainPolicy, ValidResponseAllowsTracking)
{
  auto tree=condition(); EXPECT_EQ(finish(tree),Status::SUCCESS);
  EXPECT_FALSE(bb->get<bool>("missing"));
}
TEST_F(TerrainPolicy, LocalDetourResponseKeepsTrackingAndPreservesItsReason)
{
  local_detour = true;
  auto tree = condition();
  EXPECT_EQ(finish(tree), Status::SUCCESS);
  EXPECT_FALSE(bb->get<bool>("missing"));
  EXPECT_EQ(bb->get<std::string>("why"), "local_detour_active");
}
TEST_F(TerrainPolicy, InitialEmptyPathRequestsPlanningWithoutRpc)
{
  bb->set("path",nav_msgs::msg::Path{}); auto tree=condition();
  EXPECT_EQ(finish(tree),Status::FAILURE); EXPECT_FALSE(bb->get<bool>("missing"));
  EXPECT_EQ(calls.load(),0);
}
TEST_F(TerrainPolicy, FirstGoalWithoutPathValueRequestsPlanningWithoutRpc)
{
  freshBlackboardWithoutPath();
  ASSERT_EQ(bb->getAny("path"), nullptr);
  auto tree = condition();
  // The factory can register the output port's type without assigning a value.
  const auto * entry = bb->getAny("path");
  ASSERT_TRUE(entry == nullptr || entry->empty());
  Status status = Status::IDLE;
  ASSERT_NO_THROW(status = finish(tree));
  EXPECT_EQ(status, Status::FAILURE);
  EXPECT_FALSE(bb->get<bool>("missing"));
  EXPECT_EQ(bb->get<std::string>("why"), "initial_plan_required");
  EXPECT_EQ(calls.load(), 0);
}

TEST_F(TerrainPolicy, FirstGoalPlansThenValidatesAndAcceptsAnotherGoal)
{
  freshBlackboardWithoutPath();
  // A previously latched unavailable flag must not suppress initial planning.
  bb->set("missing", true);
  int plans = 0;
  bool goal_changed = true;
  factory.registerSimpleCondition("GoalChanged", [&](BT::TreeNode &) {
      return goal_changed ? Status::SUCCESS : Status::FAILURE;
    });
  factory.registerSimpleAction("PlanInitialPath", [&](BT::TreeNode &) {
      ++plans;
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      path.header.stamp.sec = plans;
      path.poses.resize(2);
      path.poses.back().pose.position.x = plans;
      bb->set("path", path);
      goal_changed = false;
      return Status::SUCCESS;
    });
  auto tree = factory.createTreeFromText(R"(
    <root main_tree_to_execute="Main"><BehaviorTree ID="Main">
      <TerrainValidationGuard unavailable="{missing}" wait_timeout_ms="1000">
        <Fallback>
          <ReactiveSequence>
            <IsTerrainPathValid path="{path}" unavailable="{missing}"
              failure_reason="{why}" server_timeout="60"/>
            <Inverter><GoalChanged/></Inverter>
          </ReactiveSequence>
          <Sequence>
            <TerrainReplanAllowed unavailable="{missing}"/>
            <PlanInitialPath/>
          </Sequence>
        </Fallback>
      </TerrainValidationGuard>
    </BehaviorTree></root>)", bb);
  Status status = Status::IDLE;
  ASSERT_NO_THROW(status = finish(tree));
  EXPECT_EQ(status, Status::SUCCESS);
  EXPECT_EQ(plans, 1);
  EXPECT_EQ(calls.load(), 0);
  EXPECT_FALSE(bb->get<bool>("missing"));
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("path").poses.size(), 2u);

  // Once the planner has populated path, the same tree must use the service.
  ASSERT_NO_THROW(status = finish(tree));
  EXPECT_EQ(status, Status::SUCCESS);
  EXPECT_EQ(calls.load(), 1);
  EXPECT_EQ(plans, 1);
  EXPECT_EQ(bb->get<std::string>("why"), "valid");

  goal_changed = true;
  ASSERT_NO_THROW(status = finish(tree));
  EXPECT_EQ(status, Status::SUCCESS);
  EXPECT_EQ(plans, 2);
  EXPECT_EQ(calls.load(), 2);
  EXPECT_DOUBLE_EQ(bb->get<nav_msgs::msg::Path>("path").poses.back().pose.position.x, 2.0);
  ASSERT_NO_THROW(status = finish(tree));
  EXPECT_EQ(status, Status::SUCCESS);
  EXPECT_EQ(plans, 2);
  EXPECT_EQ(calls.load(), 3);
}

TEST_F(TerrainPolicy, MissingPathPortMappingRemainsAnError)
{
  EXPECT_THROW({
    auto tree = factory.createTreeFromText(R"(
      <root main_tree_to_execute="Main"><BehaviorTree ID="Main">
        <IsTerrainPathValid unavailable="{missing}" failure_reason="{why}"
          server_timeout="60"/>
      </BehaviorTree></root>)", bb);
    tree.tickRoot();
  }, std::exception);
  EXPECT_EQ(calls.load(), 0);
}

TEST_F(TerrainPolicy, WrongPathBlackboardTypeRemainsAnError)
{
  freshBlackboardWithoutPath();
  bb->set("path", 42);
  // Some BT.CPP versions reject the conflict while building; others surface
  // it during getInput. Both must remain configuration errors, not bootstrap.
  EXPECT_THROW({auto tree = condition(); tree.tickRoot();}, std::exception);
  EXPECT_EQ(calls.load(), 0);
}

TEST_F(TerrainPolicy, GeometryFailuresRetainThreeCheckHysteresis)
{
  for (auto value : {Result::COLLISION,Result::TERRAIN_INVALID,Result::PATH_MISMATCH}) {
    code=value; auto tree=condition();
    EXPECT_EQ(finish(tree),Status::SUCCESS); EXPECT_EQ(finish(tree),Status::SUCCESS);
    EXPECT_EQ(finish(tree),Status::FAILURE); EXPECT_FALSE(bb->get<bool>("missing"));
  }
}
TEST_F(TerrainPolicy, MissingInputsFailImmediatelyAndDoNotCountAsGeometryFailures)
{
  auto tree=condition();
  for (auto value : {Result::LOCALIZATION_UNAVAILABLE,Result::OBSTACLES_UNAVAILABLE,
      Result::TF_UNAVAILABLE,Result::MAP_UNAVAILABLE,Result::INTERNAL_ERROR}) {
    code=Result::COLLISION; EXPECT_EQ(finish(tree),Status::SUCCESS);
    code=value; EXPECT_EQ(finish(tree),Status::FAILURE); EXPECT_TRUE(bb->get<bool>("missing"));
    code=Result::COLLISION; EXPECT_EQ(finish(tree),Status::SUCCESS);
    code=Result::VALID; EXPECT_EQ(finish(tree),Status::SUCCESS);
  }
}
TEST_F(TerrainPolicy, RpcTimeoutBlocksReplanningAndCanRecover)
{
  auto tree=condition(); delay_ms=160;
  EXPECT_EQ(finish(tree),Status::FAILURE); EXPECT_TRUE(bb->get<bool>("missing"));
  EXPECT_EQ(bb->get<std::string>("why"),"validation_rpc_timeout");
  delay_ms=0; std::this_thread::sleep_for(180ms);
  EXPECT_EQ(finish(tree),Status::SUCCESS); EXPECT_FALSE(bb->get<bool>("missing"));
}

TEST_F(TerrainPolicy, GuardHaltsRunningControllerAndDoesNotReplanUntilInputsReturn)
{
  int plans=0, control_ticks=0, halts=0;
  factory.registerSimpleAction("Plan",[&](BT::TreeNode &) {++plans;return Status::SUCCESS;});
  class Controller : public BT::ActionNodeBase {
  public:
    Controller(const std::string & n,const BT::NodeConfiguration & c,int & t,int & h)
    :BT::ActionNodeBase(n,c),ticks(t),halts(h) {}
    Status tick() override {++ticks;return Status::RUNNING;}
    void halt() override {++halts;setStatus(Status::IDLE);}
    int & ticks; int & halts;
  };
  factory.registerBuilder<Controller>("Controller",[&](const auto & n,const auto & c) {
      return std::make_unique<Controller>(n,c,control_ticks,halts);});
  auto tree=factory.createTreeFromText(R"(
    <root main_tree_to_execute="Main"><BehaviorTree ID="Main">
      <TerrainValidationGuard unavailable="{missing}" wait_timeout_ms="1000">
       <PipelineSequence>
        <Fallback>
          <IsTerrainPathValid path="{path}" unavailable="{missing}" failure_reason="{why}" server_timeout="60"/>
          <Sequence><TerrainReplanAllowed unavailable="{missing}"/><Plan/></Sequence>
        </Fallback>
        <Controller/>
       </PipelineSequence>
      </TerrainValidationGuard>
    </BehaviorTree></root>)",bb);
  auto until=[&](const auto & ready) {
      auto end=std::chrono::steady_clock::now()+2s;
      while(!ready() && std::chrono::steady_clock::now()<end) {
        tree.tickRoot(); std::this_thread::sleep_for(2ms);
      }
      EXPECT_TRUE(ready());
    };
  until([&]() {return control_ticks>0;});
  code=Result::TF_UNAVAILABLE;
  until([&]() {return halts>0;});
  EXPECT_EQ(plans,0);
  const int stopped=control_ticks;
  for(int i=0;i<25;++i) {EXPECT_EQ(tree.tickRoot(),Status::RUNNING);std::this_thread::sleep_for(2ms);}
  EXPECT_EQ(control_ticks,stopped); EXPECT_EQ(plans,0);
  code=Result::VALID;
  until([&]() {return control_ticks>stopped;});
  EXPECT_EQ(plans,0); tree.haltTree();
}
TEST_F(TerrainPolicy, GuardWaitIsBoundedAndDoesNotRunPlanner)
{
  bb->set("missing",true);int plans=0;
  factory.registerSimpleAction("Plan",[&](BT::TreeNode &) {++plans;return Status::SUCCESS;});
  auto tree=factory.createTreeFromText(R"(<root main_tree_to_execute="Main"><BehaviorTree ID="Main">
    <TerrainValidationGuard unavailable="{missing}" wait_timeout_ms="0">
     <Sequence><TerrainReplanAllowed unavailable="{missing}"/><Plan/></Sequence>
    </TerrainValidationGuard></BehaviorTree></root>)",bb);
  EXPECT_EQ(tree.tickRoot(),Status::FAILURE); EXPECT_EQ(plans,0);
}
