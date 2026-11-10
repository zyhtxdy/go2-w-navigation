#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "astar_3d_planner/astar_3d.hpp"
#include "astar_3d_planner/swept_footprint.hpp"

namespace astar_3d_planner
{
namespace
{
OccupyMap::Config mapConfig()
{
  OccupyMap::Config config;
  config.resolution = .05;
  config.inflate_radius = .24;
  config.support_radius = .15;
  return config;
}

Astar3D::Config searchConfig()
{
  Astar3D::Config config;
  config.surface_connection_radius = .3;
  config.max_slope_deg = 10;
  config.max_step_height = .08;
  config.robot_height = .75;
  config.max_search_nodes = 100000;
  // Keep the first terrain-only choice determined by route length.
  config.obstacle_cost_weight = 0;
  return config;
}

void populateMap(OccupyMap & map)
{
  std::vector<OccupyMap::Point> support;
  for (int x = 0; x <= 12; ++x) {
    support.push_back({.025 + .1 * x, .025, .01});
  }
  for (int y = -6; y < 0; ++y) {
    support.push_back({.025, .025 + .1 * y, .01});
  }
  for (int y = 1; y <= 3; ++y) {
    support.push_back({.825, .025 + .1 * y, .01});
  }
  map.updateSupport(support);
  // The branch endpoint fits at yaw 0 and pi/2. Its northeast obstacle
  // intersects the rectangle only during the complete intervening turn.
  map.setStaticObstacles({{1.125, .625, .21}});
}

std::vector<OccupyMap::Point> groundPath(
  const OccupyMap & map, const Astar3D & astar, const std::vector<VoxelKey> & keys)
{
  std::vector<OccupyMap::Point> points;
  for (const auto & key : keys) {
    VoxelKey ground;
    double height = 0;
    if (!map.findSurfaceGroundUnlocked(key, astar.preferredGroundGapCells(),
        astar.groundGapToleranceCells(), ground) ||
      !map.supportHeightUnlocked(ground, height))
    {
      ADD_FAILURE() << "Route key has no observed ground";
      return {};
    }
    auto point = map.keyToPos(key);
    point[2] = height;
    points.push_back(point);
  }
  return points;
}

Astar3D::MotionConstraints motionConstraints(
  const OccupyMap & map, const Astar3D & astar, const SweptFootprint & checker)
{
  Astar3D::MotionConstraints motion;
  motion.maximum_seconds = 3;
  motion.transition = [&checker](const auto & from, const auto & to, double in, double out) {
      return checker.validatePath({from, to}, in, out);
    };
  motion.terminal = [&map, &astar, &checker](const auto & key, double arrival_yaw) {
      return checker.validatePath(groundPath(map, astar, {key}), arrival_yaw, 0);
    };
  return motion;
}
}  // namespace

TEST(GoalManeuvers, SearchesOtherToleratedGoalsWhenPreferredGoalCannotCompleteManeuver)
{
  OccupyMap map(mapConfig());
  populateMap(map);
  const auto lock = map.lockForReading();
  Astar3D astar(map, searchConfig());
  SweptFootprint checker(map, SweptFootprint::Config{});
  VoxelKey start, bad_goal, good_goal, reached;
  ASSERT_TRUE(astar.projectToSurface(.025, -.575, .01, .01, .1, start));
  ASSERT_TRUE(astar.projectToSurface(.825, .325, .01, .01, .1, bad_goal));
  ASSERT_TRUE(astar.projectToSurface(1.225, .025, .01, .01, .1, good_goal));

  // The goal disk intersects only the two branch endpoints. Earlier support
  // on both branches lies outside tolerance, so these are all real options.
  const auto goals = astar.surfaceCandidates(1.265, .495, .01, .48, .1);
  ASSERT_EQ(goals.size(), 2u);
  ASSERT_NE(std::find(goals.begin(), goals.end(), bad_goal), goals.end());
  ASSERT_NE(std::find(goals.begin(), goals.end(), good_goal), goals.end());

  const auto bad_point = groundPath(map, astar, {bad_goal});
  const auto good_point = groundPath(map, astar, {good_goal});
  ASSERT_EQ(bad_point.size(), 1u);
  ASSERT_EQ(good_point.size(), 1u);
  const double north = std::acos(-1.0) / 2;
  EXPECT_TRUE(checker.poseCollisionFree(bad_point.front(), 0));
  EXPECT_TRUE(checker.poseCollisionFree(bad_point.front(), north));
  EXPECT_FALSE(checker.validatePath(bad_point, north, 0));

  // The initial L-shaped support also prevents a direct-goal fast path from
  // hiding the ordinary-search/fallback decision in an integration fixture.
  std::vector<OccupyMap::Point> direct;
  const auto start_point = groundPath(map, astar, {start});
  ASSERT_EQ(start_point.size(), 1u);
  EXPECT_FALSE(astar.tryDirectGroundPath(start_point.front(), bad_point.front(), 0, direct));
  EXPECT_FALSE(astar.tryDirectGroundPath(start_point.front(), good_point.front(), 0, direct));

  std::vector<VoxelKey> route;
  ASSERT_TRUE(astar.searchToAny(start, goals, route, reached));
  ASSERT_EQ(reached, bad_goal);
  EXPECT_FALSE(checker.validatePath(groundPath(map, astar, route), 0, 0));

  const auto motion = motionConstraints(map, astar, checker);
  Astar3D::SearchDiagnostics diagnostic;
  EXPECT_FALSE(astar.searchToAny(start, {bad_goal}, route, reached,
    nullptr, &diagnostic, &motion));
  EXPECT_TRUE(route.empty());
  EXPECT_STREQ(diagnostic.reason, "ground_disconnected");
  EXPECT_GT(diagnostic.terminal_rejected, 0u);

  ASSERT_TRUE(astar.searchToAny(start, goals, route, reached,
    nullptr, &diagnostic, &motion));
  EXPECT_EQ(reached, good_goal);
  EXPECT_TRUE(checker.validatePath(groundPath(map, astar, route), 0, 0));
}

TEST(GoalManeuvers, RejectsEveryGoalWhenAllTerminalTurnsAreBlocked)
{
  OccupyMap map(mapConfig());
  populateMap(map);
  const auto lock = map.lockForReading();
  Astar3D astar(map, searchConfig());
  SweptFootprint checker(map, SweptFootprint::Config{});
  VoxelKey start, reached;
  ASSERT_TRUE(astar.projectToSurface(.025, -.575, .01, .01, .1, start));

  const auto goals = astar.surfaceCandidates(.825, .45, .01, .25, .1);
  ASSERT_EQ(goals.size(), 2u);
  const double north = std::acos(-1.0) / 2;
  for (const auto & goal : goals) {
    const auto point = groundPath(map, astar, {goal});
    ASSERT_EQ(point.size(), 1u);
    EXPECT_TRUE(checker.poseCollisionFree(point.front(), 0));
    EXPECT_TRUE(checker.poseCollisionFree(point.front(), north));
    EXPECT_FALSE(checker.validatePath(point, north, 0));
  }

  std::vector<VoxelKey> route;
  ASSERT_TRUE(astar.searchToAny(start, goals, route, reached));
  const auto motion = motionConstraints(map, astar, checker);
  Astar3D::SearchDiagnostics diagnostic;
  EXPECT_FALSE(astar.searchToAny(start, goals, route, reached,
    nullptr, &diagnostic, &motion));
  EXPECT_TRUE(route.empty());
  EXPECT_STREQ(diagnostic.reason, "ground_disconnected");
  EXPECT_GT(diagnostic.terminal_rejected, 0u);
}
TEST(GoalManeuvers, CorridorSearchMergesArrivalHeadingsButKeepsCompleteBodyEdges)
{
  auto mc=mapConfig(); mc.resolution=.1;
  OccupyMap map(mc);
  std::vector<OccupyMap::Point> floor;
  for(int x=-5;x<=35;++x) {for(int y=-10;y<=10;++y) {
    floor.push_back({x*.1+.05,y*.1+.05,.01});
  }}
  map.updateSupport(floor);
  map.setStaticObstacles({{1.05,.05,.21}});
  auto body=SweptFootprint::Config{};body.check_in_place_turn_sweeps=false;
  SweptFootprint checker(map,body);
  const auto read=map.lockForReading();
  Astar3D astar(map,searchConfig());
  VoxelKey start,goal,reached;
  ASSERT_TRUE(astar.projectToSurface(.05,.05,.01,.01,.1,start));
  ASSERT_TRUE(astar.projectToSurface(2.05,.05,.01,.01,.1,goal));
  Astar3D::MotionConstraints motion;
  motion.include_heading_in_state=false;motion.turn_cost_weight=0;
  motion.maximum_seconds=3;
  motion.transition=[&](const auto & a,const auto & b,double,double out) {
      return checker.validatePath({a,b},out,out);
    };
  motion.terminal=[&](const auto & key,double) {
      auto p=groundPath(map,astar,{key});
      return checker.poseCollisionFree(p.front(),0);
    };
  std::vector<VoxelKey> route;
  Astar3D::SearchDiagnostics diagnostics;
  ASSERT_TRUE(astar.searchToAny(start,{goal},route,reached,nullptr,&diagnostics,&motion));
  const auto points=groundPath(map,astar,route);
  EXPECT_TRUE(checker.validatePath(points,0,0));
  EXPECT_TRUE(std::any_of(points.begin(),points.end(),[](const auto & p) {
    return std::abs(p[1]-.05)>.25;
  }));
  EXPECT_GT(diagnostics.motion_rejected,0u);
}

TEST(GoalManeuvers, CorridorStateCannotCarryAnArrivalDependentTurnCost)
{
  OccupyMap map(mapConfig());populateMap(map);
  const auto read=map.lockForReading();
  Astar3D astar(map,searchConfig());
  Astar3D::MotionConstraints motion;
  motion.include_heading_in_state=false;
  motion.transition=[](const auto &,const auto &,double,double) {return true;};
  std::vector<VoxelKey> route;VoxelKey reached;
  Astar3D::SearchDiagnostics diagnostic;
  EXPECT_FALSE(astar.searchToAny({0,0,0},{{1,0,0}},route,reached,nullptr,&diagnostic,&motion));
  EXPECT_STREQ(diagnostic.reason,"invalid_motion_constraints");
}
}  // namespace astar_3d_planner
