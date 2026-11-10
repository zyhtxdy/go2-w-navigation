#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include "astar_3d_planner/goal_connections.hpp"

namespace astar_3d_planner
{
namespace
{
std::vector<OccupyMap::Point> recorded(const char * name)
{
  std::ifstream input(std::string(TEST_TERMINAL_MAP_DIRECTORY)+"/"+name);
  std::vector<OccupyMap::Point> result;
  std::string line; std::getline(input,line);
  while(std::getline(input,line)) {
    std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);
    OccupyMap::Point p;
    if(row>>p[0]>>p[1]>>p[2]) {
      for(auto & v:p) {v=static_cast<float>(v);}
      result.push_back(p);
    }
  }
  return result;
}
OccupyMap::Config configuration()
{
  OccupyMap::Config c;c.resolution=.1;c.inflate_radius=.24;
  c.footprint_inscribed_radius=.26;c.support_inference_enabled=true;c.support_inference_radius=.4;
  c.synthetic_obstacle_clearance_radius=.65;return c;
}
Astar3D::Config plannerConfig()
{
  Astar3D::Config c;c.robot_height=.75;c.max_step_height=.08;c.max_slope_deg=10;
  c.surface_connection_radius=.3;return c;
}
SweptFootprint::Config bodyConfig()
{
  SweptFootprint::Config c;c.min_x=-.41;c.max_x=.41;c.min_y=-.26;c.max_y=.26;return c;
}
}

TEST(GoalConnections, RecordedBlockedCentresHaveFullyCheckedSubcellApproaches)
{
  OccupyMap map(configuration());map.updateSupport(recorded("support.csv"));
  map.setStaticObstacles(recorded("obstacles.csv"));
  Astar3D astar(map,plannerConfig());SweptFootprint checker(map,bodyConfig());
  const auto lock=map.lockForReading();
  for(double x:{1.5,2.5}) {
    for(double yaw:{0.0,std::acos(-1.0)}) {
      GoalConnectionDiagnostics diagnostics;
      std::vector<GoalPose> poses;
      const auto paths=subcellGoalConnections(map,astar,checker,x,.4,-.38,yaw,.2,1.0,1.5,&diagnostics,&poses);
      ASSERT_FALSE(poses.empty())<<x<<','<<yaw;
      if(x==1.5 && yaw==0) {ASSERT_FALSE(paths.empty());}
      EXPECT_LE(paths.size(),64u);EXPECT_LE(diagnostics.connectors_checked,512u);
      bool bypasses_blocked_terminal_centre=false;
      for(const auto & pose:poses) {
        const auto & p=pose.point;
        EXPECT_LE(std::hypot(p[0]-x,p[1]-.4),.2+1e-9);
        EXPECT_TRUE(checker.poseCollisionFree(p,yaw));
        auto centre=map.keyToPos(pose.key);centre[2]=p[2];
        bypasses_blocked_terminal_centre |= !checker.poseCollisionFree(centre,yaw);
      }
      for(const auto & path:paths) {
        ASSERT_GE(path.points.size(),2u);
        const auto & p=path.points.back();
        EXPECT_LE(std::hypot(p[0]-x,p[1]-.4),.2+1e-9);
        EXPECT_TRUE(checker.poseCollisionFree(p,yaw));
        auto centre=map.keyToPos(map.posToKey(p[0],p[1],p[2]));centre[2]=p[2];
        bypasses_blocked_terminal_centre |= !checker.poseCollisionFree(centre,yaw);
        const auto & from=path.points.front();
        const double approach=std::atan2(p[1]-from[1],p[0]-from[0]);
        EXPECT_TRUE(checker.validatePath(path.points,approach,yaw));
        auto refreshed=path.points;
        EXPECT_TRUE(astar.refreshAndValidateTerrainPath(path.anchor,refreshed,nullptr,true));
      }
      EXPECT_TRUE(bypasses_blocked_terminal_centre);
    }
  }
}

TEST(GoalConnections, RecordedBlockedGroundAndUnobservedTargetsAreNotInvented)
{
  OccupyMap map(configuration());map.updateSupport(recorded("support.csv"));
  map.setStaticObstacles(recorded("obstacles.csv"));
  Astar3D astar(map,plannerConfig());SweptFootprint checker(map,bodyConfig());
  const auto lock=map.lockForReading();
  EXPECT_TRUE(subcellGoalConnections(map,astar,checker,3.5,.4,-.38,0,.2,1.0,1.5).empty());
  EXPECT_TRUE(subcellGoalConnections(map,astar,checker,-50,-50,-.38,0,.2,1.0,1.5).empty());
  EXPECT_TRUE(subcellGoalConnections(map,astar,checker,1.5,.4,-.38,0,0,1.0,1.5).empty());
}

TEST(GoalConnections, RealNewObstacleStillVetoesPreviouslyValidTerminalConnection)
{
  OccupyMap map(configuration());map.updateSupport(recorded("support.csv"));
  map.setStaticObstacles(recorded("obstacles.csv"));
  Astar3D astar(map,plannerConfig());SweptFootprint checker(map,bodyConfig());
  std::vector<GoalConnection> paths;
  {const auto lock=map.lockForReading();
    paths=subcellGoalConnections(map,astar,checker,1.5,.4,-.38,0,.2,1.0,1.5);}
  ASSERT_FALSE(paths.empty());
  auto obstruction=paths.front().points.back();obstruction[2]+=.3;
  ASSERT_TRUE(map.updateObstacles({obstruction},1.5,.4,1000000000LL,{OccupyMap::kMeasured},true));
  const auto lock=map.lockForReading();
  EXPECT_FALSE(checker.validatePath(paths.front().points,0,0));
}

TEST(GoalConnections, GroundClearanceAndDirectPathAgreeOnFractionalTopHeight)
{
  OccupyMap::Config c;c.resolution=.1;c.inflate_radius=0;c.support_radius=0;
  OccupyMap map(c);
  std::vector<OccupyMap::Point> floor;
  for(int x=-3;x<=13;++x) {for(int y=-3;y<=3;++y) {floor.push_back({x*.1+.05,y*.1+.05,-.377});}}
  map.updateSupport(floor);map.setStaticObstacles({{.55,.05,.45}});
  Astar3D astar(map,plannerConfig());std::vector<OccupyMap::Point> path;
  {const auto lock=map.lockForReading();
    EXPECT_DOUBLE_EQ(astar.groundObstacleCost({.65,.05,-.377}),0.0);
    EXPECT_TRUE(astar.tryDirectGroundPath({.05,.05,-.377},{1.05,.05,-.377},0,path));
    VoxelKey start,goal;
    ASSERT_TRUE(astar.projectToSurface(.05,.05,-.377,0,.1,start));
    ASSERT_TRUE(astar.projectToSurface(1.05,.05,-.377,0,.1,goal));
    EXPECT_TRUE(astar.segmentTraversable(start,goal));}
  map.setStaticObstacles({{.55,.05,.35}});
  const auto lock=map.lockForReading();
  EXPECT_FALSE(astar.tryDirectGroundPath({.05,.05,-.377},{1.05,.05,-.377},0,path));
  EXPECT_GT(astar.groundObstacleCost({.65,.05,-.377}),0.0);
}
}
