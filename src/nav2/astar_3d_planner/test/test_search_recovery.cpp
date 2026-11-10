#include <gtest/gtest.h>

#include "astar_3d_planner/astar_3d.hpp"

namespace astar_3d_planner
{
namespace
{
std::vector<OccupyMap::Point> disconnectedGround()
{
  return {{.01,.01,.01},{.21,.01,.01},{.41,.01,.01},{2.01,.01,.01}};
}
}

TEST(SearchRecovery, ExhaustedOrdinarySearchReportsOnlyItsReachableComponent)
{
  OccupyMap map(OccupyMap::Config{}); map.updateSupport(disconnectedGround());
  const auto lock=map.lockForReading(); Astar3D astar(map,Astar3D::Config{});
  VoxelKey start,goal,reached;
  ASSERT_TRUE(astar.projectToSurface(.01,.01,.01,.01,.1,start));
  ASSERT_TRUE(astar.projectToSurface(2.01,.01,.01,.01,.1,goal));
  std::vector<VoxelKey> path; Astar3D::SearchDiagnostics diagnostic;
  EXPECT_FALSE(astar.searchToAny(start,{goal},path,reached,nullptr,&diagnostic));
  EXPECT_STREQ(diagnostic.reason,"ground_disconnected");
  ASSERT_EQ(diagnostic.exhausted_component.size(),3u);
  for(const auto & key:diagnostic.exhausted_component) {EXPECT_LT(key.x,5);}
}

TEST(SearchRecovery, TimeoutAndNodeLimitDoNotProveAComponentDisconnected)
{
  OccupyMap map(OccupyMap::Config{}); map.updateSupport(disconnectedGround());
  const auto lock=map.lockForReading();
  for(bool timeout:{false,true}) {
    Astar3D::Config config;
    if(timeout) {config.maximum_search_seconds=1e-12;} else {config.max_search_nodes=1;}
    Astar3D astar(map,config);
    VoxelKey start,goal,reached;
    ASSERT_TRUE(astar.projectToSurface(.01,.01,.01,.01,.1,start));
    ASSERT_TRUE(astar.projectToSurface(2.01,.01,.01,.01,.1,goal));
    std::vector<VoxelKey> path; Astar3D::SearchDiagnostics diagnostic;
    EXPECT_FALSE(astar.searchToAny(start,{goal},path,reached,nullptr,&diagnostic));
    EXPECT_STREQ(diagnostic.reason,timeout ? "search_time_limit" : "search_node_limit");
    EXPECT_TRUE(diagnostic.exhausted_component.empty());
  }
}

TEST(SearchRecovery, HeadingFailureCannotExcludeOrdinarySupportStates)
{
  OccupyMap map(OccupyMap::Config{}); map.updateSupport(disconnectedGround());
  const auto lock=map.lockForReading(); Astar3D astar(map,Astar3D::Config{});
  VoxelKey start,goal,reached;
  ASSERT_TRUE(astar.projectToSurface(.01,.01,.01,.01,.1,start));
  ASSERT_TRUE(astar.projectToSurface(.41,.01,.01,.01,.1,goal));
  Astar3D::MotionConstraints motion;
  motion.transition=[](const auto &,const auto &,double,double) {return false;};
  std::vector<VoxelKey> path; Astar3D::SearchDiagnostics diagnostic;
  EXPECT_FALSE(astar.searchToAny(start,{goal},path,reached,nullptr,&diagnostic,&motion));
  EXPECT_TRUE(diagnostic.exhausted_component.empty());
  EXPECT_TRUE(astar.search(start,goal,path));
}

TEST(SearchRecovery, NegativeTimeBudgetIsRejectedAndZeroKeepsOrdinarySearch)
{
  OccupyMap map(OccupyMap::Config{}); map.updateSupport(disconnectedGround());
  const auto lock=map.lockForReading();
  for(double budget:{-1.0,0.0}) {
    Astar3D::Config config; config.maximum_search_seconds=budget;
    Astar3D astar(map,config); VoxelKey start,goal,reached;
    ASSERT_TRUE(astar.projectToSurface(.01,.01,.01,.01,.1,start));
    ASSERT_TRUE(astar.projectToSurface(.41,.01,.01,.01,.1,goal));
    std::vector<VoxelKey> path; Astar3D::SearchDiagnostics diagnostic;
    EXPECT_EQ(astar.searchToAny(start,{goal},path,reached,nullptr,&diagnostic),budget==0);
    if(budget<0) {EXPECT_STREQ(diagnostic.reason,"invalid_search_budget");}
  }
}
}  // namespace astar_3d_planner
