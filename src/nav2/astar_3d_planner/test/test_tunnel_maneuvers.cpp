#include <gtest/gtest.h>
#include <cmath>
#include "astar_3d_planner/astar_3d.hpp"
#include "astar_3d_planner/path_refiner.hpp"
#include "astar_3d_planner/swept_footprint.hpp"

namespace astar_3d_planner
{
namespace
{
OccupyMap::Config mapConfig()
{
  OccupyMap::Config c;
  c.resolution = .1; c.inflate_radius = .24; c.cost_inflate_cells = 2;
  c.support_radius = .15;
  return c;
}
Astar3D::Config searchConfig()
{
  Astar3D::Config c;
  c.max_step_height = .08; c.max_slope_deg = 10; c.robot_height = .75;
  c.surface_connection_radius = .3; c.max_search_nodes = 100000;
  return c;
}
std::vector<OccupyMap::Point> floorGrid()
{
  std::vector<OccupyMap::Point> p;
  for (int x = -10; x <= 20; ++x) {
    for (int y = -8; y <= 8; ++y) {p.push_back({.01 + x*.2, .01 + y*.2, .01});}
  }
  return p;
}
std::vector<OccupyMap::Point> groundPath(const OccupyMap & map,
  const Astar3D & astar, const std::vector<VoxelKey> & keys)
{
  std::vector<OccupyMap::Point> p;
  for (const auto & key : keys) {
    VoxelKey ground; double z = 0;
    EXPECT_TRUE(map.findSurfaceGroundUnlocked(key, astar.preferredGroundGapCells(),
      astar.groundGapToleranceCells(), ground));
    EXPECT_TRUE(map.supportHeightUnlocked(ground, z));
    auto q = map.keyToPos(key);q[2] = z;p.push_back(q);
  }
  return p;
}
double maximumTurn(const std::vector<OccupyMap::Point> & path)
{
  double max_turn = 0, previous = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double next = std::atan2(path[i][1]-path[i-1][1],path[i][0]-path[i-1][0]);
    if (i > 1) {max_turn=std::max(max_turn,std::abs(std::remainder(next-previous,2*std::acos(-1.0))));}
    previous=next;
  }
  return max_turn;
}
}

TEST(TunnelManeuvers, RetainsDifferentArrivalHeadingsUntilTerminalManeuverIsChecked)
{
  OccupyMap map(mapConfig());map.updateSupport(floorGrid());
  const auto lock = map.lockForReading();Astar3D astar(map,searchConfig());
  VoxelKey start, goal, reached;
  ASSERT_TRUE(astar.projectToSurface(.01,.01,.01,.1,.5,start));
  ASSERT_TRUE(astar.projectToSurface(1.01,1.01,.01,.1,.5,goal));
  Astar3D::MotionConstraints constraints;
  constraints.start_yaw = std::acos(-1.0)/2;
  constraints.transition=[](const auto &,const auto &,double,double){return true;};
  constraints.terminal=[](const auto &,double yaw){return std::abs(yaw)<.01;};
  std::vector<VoxelKey> route;Astar3D::SearchDiagnostics diagnostic;
  ASSERT_TRUE(astar.searchToAny(start,{goal},route,reached,nullptr,&diagnostic,&constraints));
  ASSERT_GE(route.size(),2u);
  EXPECT_EQ(route[route.size()-2].y,goal.y);
  EXPECT_LT(route[route.size()-2].x,goal.x);
  EXPECT_GT(diagnostic.terminal_rejected,0u);
  EXPECT_GT(diagnostic.heuristic_expanded,0u);
  double informed_cost=0, reference_cost=0;
  constraints.maximum_seconds=2.0;
  ASSERT_TRUE(astar.searchToAny(start,{goal},route,reached,&informed_cost,nullptr,&constraints));
  auto reference_config=searchConfig();reference_config.heuristic_weight=0.0;
  ASSERT_TRUE(Astar3D(map,reference_config).searchToAny(start,{goal},route,reached,
    &reference_cost,nullptr,&constraints));
  EXPECT_NEAR(informed_cost,reference_cost,1e-8);
}

TEST(TunnelManeuvers, CrossesOnePointOneMetreGapWithCompleteRectangularSweeps)
{
  OccupyMap map(mapConfig());map.updateSupport(floorGrid());
  std::vector<OccupyMap::Point> boxes;
  // Raw occupied voxel faces are y=-0.5 and y=+0.6: 1.1 m clear gap.
  for (int x=0;x<=15;++x) {
    for (int y=0;y<=5;++y) {
      for (int z=1;z<=7;++z) {
        boxes.push_back({.01+x*.1,.61+y*.1,.01+z*.1});
        boxes.push_back({.01+x*.1,-.51-y*.1,.01+z*.1});
      }
    }
  }
  map.setStaticObstacles(boxes);
  const auto lock=map.lockForReading();Astar3D astar(map,searchConfig());
  SweptFootprint::Config body;body.min_x=-.41;body.max_x=.41;body.min_y=-.26;body.max_y=.26;
  SweptFootprint checker(map,body);
  VoxelKey start,goal,reached;
  ASSERT_TRUE(astar.projectToSurface(-1.39,.41,.01,.1,.5,start));
  ASSERT_TRUE(astar.projectToSurface(3.01,-.39,.01,.1,.5,goal));
  Astar3D::MotionConstraints motion;
  motion.maximum_seconds=3.0;  // Geometry correctness, independent of host CPU load.
  motion.transition=[&](const auto & a,const auto & b,double in,double out){
      return checker.validatePath({a,b},in,out);};
  motion.terminal=[&](const auto & key,double yaw){
      const auto p=groundPath(map,astar,{key});return checker.validatePath(p,yaw,0);};
  std::vector<VoxelKey> keys;Astar3D::SearchDiagnostics diagnostic;
  ASSERT_TRUE(astar.searchToAny(start,{goal},keys,reached,nullptr,&diagnostic,&motion));
  auto points=groundPath(map,astar,keys);
  ASSERT_TRUE(checker.validatePath(points,0,0));
  PathRefiner::Config refinement;refinement.maximum_seconds=2.0;
  const auto refined=PathRefiner(refinement).refine(points,0,0,false,[&](auto & candidate){
      return astar.refreshAndValidateTerrainPath(start,candidate,nullptr,true) &&
             checker.validatePath(candidate,0,0);});
  EXPECT_TRUE(checker.validatePath(refined,0,0));
  EXPECT_LE(PathRefiner::objective(refined,0,0,.15),PathRefiner::objective(points,0,0,.15)+1e-8);
  EXPECT_LT(refined.size(),points.size());
  RecordProperty("raw_vertices",static_cast<int>(points.size()));
  RecordProperty("refined_vertices",static_cast<int>(refined.size()));
  RecordProperty("raw_maximum_turn_deg",maximumTurn(points)*180/std::acos(-1.0));
  RecordProperty("refined_maximum_turn_deg",maximumTurn(refined)*180/std::acos(-1.0));
  RecordProperty("heading_search_expanded",static_cast<int>(diagnostic.expanded));
  RecordProperty("motion_rejected",static_cast<int>(diagnostic.motion_rejected));
}

TEST(TunnelManeuvers, RejectsSolidBarrierInsteadOfSmoothingThroughIt)
{
  OccupyMap map(mapConfig());map.updateSupport(floorGrid());
  std::vector<OccupyMap::Point> wall;
  for(int y=-20;y<=20;++y) {for(int z=1;z<=7;++z){wall.push_back({.61,.01+y*.1,.01+z*.1});}}
  map.setStaticObstacles(wall);const auto lock=map.lockForReading();
  Astar3D astar(map,searchConfig());SweptFootprint checker(map,SweptFootprint::Config{});
  VoxelKey start,goal,reached;
  ASSERT_TRUE(astar.projectToSurface(-.39,.01,.01,.1,.5,start));
  ASSERT_TRUE(astar.projectToSurface(1.81,.01,.01,.1,.5,goal));
  Astar3D::MotionConstraints motion;
  motion.transition=[&](const auto & a,const auto & b,double in,double out){return checker.validatePath({a,b},in,out);};
  std::vector<VoxelKey> route;
  EXPECT_FALSE(astar.searchToAny(start,{goal},route,reached,nullptr,nullptr,&motion));
  EXPECT_TRUE(route.empty());
}

TEST(TunnelManeuvers, CornerRefinementReducesHeadingJumpAndKeepsEndpoints)
{
  const PathRefiner::Points points{{0,0,0},{1,0,0},{1,1,0},{2,1,0}};
  PathRefiner::Config config;config.maximum_seconds=1;
  // Keep the route around a corner: reject any segment cutting y=x near centre.
  auto validate=[](auto & candidate) {
      for(std::size_t i=1;i<candidate.size();++i){
        if(std::hypot(candidate[i][0]-candidate[i-1][0],candidate[i][1]-candidate[i-1][1])>1.05){return false;}}
      return true;};
  PathRefiner::Diagnostics diagnostic;
  auto result=PathRefiner(config).refine(points,0,0,false,validate,&diagnostic);
  EXPECT_EQ(result.front(),points.front());EXPECT_EQ(result.back(),points.back());
  EXPECT_GT(diagnostic.bevels,0u);
  EXPECT_LT(maximumTurn(result),maximumTurn(points));
}

TEST(TunnelManeuvers, RefinerCannotBypassTerrainFailureOrMoveSelectedBlindConnector)
{
  const PathRefiner::Points points{{0,0,0},{.2,.2,0},{.4,.2,0},{1,.2,0}};
  PathRefiner::Config config;config.maximum_seconds=1;config.maximum_checks=3;
  PathRefiner::Diagnostics diagnostic;
  auto result=PathRefiner(config).refine(points,0,0,true,[&](auto & candidate){
      return candidate.size()>2 && candidate[1]==points[1];},&diagnostic);
  ASSERT_GT(result.size(),2u);EXPECT_EQ(result[1],points[1]);
  EXPECT_LE(diagnostic.checks,config.maximum_checks);
  auto rejected=PathRefiner(config).refine(points,0,0,false,[](auto &){return false;});
  EXPECT_EQ(rejected,points);
}

TEST(TunnelManeuvers, ClearanceCostPreservesSafeDetourInsteadOfCuttingTowardWall)
{
  OccupyMap map(mapConfig());map.updateSupport(floorGrid());
  map.setStaticObstacles({{1.01,.01,.31}});
  const auto read=map.lockForReading();Astar3D astar(map,searchConfig());
  SweptFootprint checker(map,SweptFootprint::Config{});
  const PathRefiner::Points original{{.01,.45,.01},{1.01,.65,.01},{2.01,.45,.01}};
  VoxelKey start;
  ASSERT_TRUE(astar.projectToSurface(.01,.45,.01,.2,.5,start));
  const auto valid=[&](auto & candidate){
      return astar.refreshAndValidateTerrainPath(start,candidate,nullptr,true) &&
             checker.validatePath(candidate,0,0);
    };
  const auto cost=[&](const auto & p){
      auto key=map.posToKey(p[0],p[1],p[2]);++key.z;
      return map.obstacleCostUnlocked(key,astar.robotHeightCells()-1);
    };
  PathRefiner::Config config;config.maximum_seconds=1;PathRefiner refiner(config);
  auto geometric=refiner.refine(original,0,0,false,valid);
  ASSERT_EQ(geometric.size(),2u);
  double before=0,after=0;
  refiner.score(geometric,0,0,cost,&before);
  PathRefiner::Diagnostics diagnostic;
  auto result=refiner.refine(original,0,0,false,valid,&diagnostic,cost);
  refiner.score(result,0,0,cost,&after);
  EXPECT_GT(result.size(),2u);EXPECT_LT(after,before);
  EXPECT_GT(diagnostic.clearance_rejected,0u);
  EXPECT_TRUE(checker.validatePath(result,0,0));
  EXPECT_EQ(result.front(),original.front());EXPECT_EQ(result.back(),original.back());
}

TEST(TunnelManeuvers, IntegratedVoxelCostIsIndependentOfPathDensification)
{
  PathRefiner::Config config;config.turn_weight=0;config.clearance_weight=1;
  PathRefiner refiner(config);
  const auto cost=[](const auto & p){return std::floor(p[0]/.1)==5 && p[2]<1.0 ? 1.0 : 0.0;};
  const PathRefiner::Points sparse{{.01,0,.01},{1.01,0,.01}};
  const PathRefiner::Points dense{{.01,0,.01},{.243,0,.01},{.531,0,.01},{.777,0,.01},{1.01,0,.01}};
  EXPECT_NEAR(refiner.score(sparse,0,0,cost),1.1,1e-10);
  EXPECT_NEAR(refiner.score(dense,0,0,cost),1.1,1e-10);
  EXPECT_NEAR(refiner.score({{1.01,0,.01},{.01,0,.01}},0,0,cost),1.1,1e-10);
  EXPECT_NEAR(refiner.score({{.01,0,2.01},{1.01,0,2.01}},0,0,cost),1.0,1e-10);
}

TEST(TunnelManeuvers, InvalidOrUnboundedClearanceScoreKeepsValidatedInput)
{
  const PathRefiner::Points original{{0,0,0},{.5,.2,0},{1,0,0}};
  PathRefiner::Config config;config.maximum_cost_samples=1;
  PathRefiner::Diagnostics diagnostic;
  auto result=PathRefiner(config).refine(original,0,0,false,[](auto &){return true;},
    &diagnostic,[](const auto &){return 1.0;});
  EXPECT_EQ(result,original);EXPECT_TRUE(diagnostic.clearance_unavailable);
  config.maximum_cost_samples=4096;
  result=PathRefiner(config).refine(original,0,0,false,[](auto &){return true;},
    &diagnostic,[](const auto &){return -1.0;});
  EXPECT_EQ(result,original);EXPECT_TRUE(diagnostic.clearance_unavailable);
}
}  // namespace astar_3d_planner
