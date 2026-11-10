#include <gtest/gtest.h>

#include <cmath>
#include <algorithm>
#include <random>
#include <thread>
#include <vector>

#include "astar_3d_planner/astar_3d.hpp"

namespace astar_3d_planner
{
namespace
{
OccupyMap::Config config()
{
  OccupyMap::Config c;
  c.inflate_radius = 0;
  c.support_merge_height = .02;
  c.support_inference_enabled = true;
  return c;
}

// The unobserved centre lies strictly inside four real ground observations.
std::vector<OccupyMap::Point> ring(double height = .01)
{
  return {{-.15, .05, height}, {.25, .05, height},
    {.05, -.15, height}, {.05, .25, height}};
}

bool groundAt(OccupyMap & map, double x, double y, double height, VoxelKey & ground)
{
  return map.findSupportByHeightUnlocked(x, y, height, .02, ground);
}
}

TEST(GroundInference, EnclosedHoleEntersSearchAndEveryTerrainCheck)
{
  OccupyMap map(config());
  map.updateSupport(ring());
  const auto lock = map.lockForReading();
  VoxelKey ground;
  ASSERT_TRUE(groundAt(map, .05, .05, .01, ground));
  EXPECT_TRUE(map.inferredSupportUnlocked(ground));
  double height;
  ASSERT_TRUE(map.supportHeightUnlocked(ground, height));
  EXPECT_NEAR(height, .01, 1e-9);
  // Provenance does not alter the measured map snapshot or its identity.
  EXPECT_EQ(map.supportSnapshotUnlocked().size(), 4u);

  Astar3D::Config c;
  c.start_projection_xy_radius = .01;
  c.goal_projection_xy_radius = .01;
  c.max_slope_deg = 10;
  Astar3D astar(map, c);
  VoxelKey start, goal;
  ASSERT_TRUE(astar.projectToSurface(-.15, .05, .01, .01, .1, start));
  ASSERT_TRUE(astar.projectToSurface(.25, .05, .01, .01, .1, goal));
  std::vector<VoxelKey> path;
  ASSERT_TRUE(astar.search(start, goal, path));
  bool uses_inferred = false;
  for (const auto & key : path) {
    VoxelKey layer;
    ASSERT_TRUE(map.findSurfaceGroundUnlocked(key, astar.preferredGroundGapCells(),
        astar.groundGapToleranceCells(), layer));
    uses_inferred |= map.inferredSupportUnlocked(layer);
  }
  EXPECT_TRUE(uses_inferred);
  EXPECT_TRUE(astar.segmentTraversable(start, goal));
  std::vector<OccupyMap::Point> direct;
  ASSERT_TRUE(astar.tryDirectGroundPath({-.15,.05,.01}, {.25,.05,.01}, 0, direct));
  auto dense = astar.densifyPath(direct, .025);
  EXPECT_TRUE(astar.refreshAndValidateTerrainPath(start, dense));
  EXPECT_EQ(astar.pathPointTraversability({.05,.05,.01}),
    OccupyMap::Traversability::kTraversable);
}

TEST(GroundInference, NeverExtendsSingleSidedGroundOrCollinearObservations)
{
  for (const auto & points : std::vector<std::vector<OccupyMap::Point>>{
      {{-.15,.05,.01}, {-.05,-.05,.01}, {-.05,.15,.01}},
      {{-.15,.05,.01}, {-.05,.05,.01}, {.15,.05,.01}, {.25,.05,.01}}})
  {
    auto c = config(); c.support_radius = 0.0;  // Isolate interpolation from legacy borrowing.
    OccupyMap map(c); map.updateSupport(points);
    const auto lock = map.lockForReading();
    VoxelKey ground;
    EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));
    EXPECT_FALSE(map.findGroundUnlocked({0,0,2},6,ground));
  }
}

TEST(GroundInference, DoesNotCascadeAcrossUnobservedArea)
{
  OccupyMap map(config()); map.updateSupport(ring());
  const auto lock = map.lockForReading();
  VoxelKey ground;
  ASSERT_TRUE(groundAt(map,.05,.05,.01,ground));
  EXPECT_FALSE(groundAt(map,.35,.05,.01,ground));
  EXPECT_FALSE(groundAt(map,.45,.05,.01,ground));
}

TEST(GroundInference, LargeHoleRemainsUnknown)
{
  auto points = ring();
  for (auto & p : points) {p[0] = .05 + 2*(p[0]-.05); p[1] = .05 + 2*(p[1]-.05);}
  OccupyMap map(config()); map.updateSupport(points);
  const auto lock = map.lockForReading();
  VoxelKey ground;
  EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));
}

TEST(GroundInference, RejectsDiscontinuousAndNoisyHeights)
{
  for (double change : {.08, .05}) {
    auto points = ring(); points.back()[2] += change;
    OccupyMap map(config()); map.updateSupport(points);
    const auto lock = map.lockForReading();
    VoxelKey ground;
    EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));
  }
}

TEST(GroundInference, AcceptsBoundedPlaneButRejectsExcessiveSlope)
{
  for (double slope : {.08, .12}) {
    auto c = config(); c.support_inference_max_slope_deg = 5.0;
    auto points = ring(.10);
    for (auto & p : points) {p[2] += slope * (p[0]-.05);}
    OccupyMap map(c); map.updateSupport(points);
    const auto lock = map.lockForReading();
    VoxelKey ground;
    EXPECT_EQ(groundAt(map,.05,.05,.10,ground), slope < .1);
  }
}

TEST(GroundInference, SeparatesLayersWithoutCreatingIntermediateFloor)
{
  auto points = ring(.01), upper = ring(3.01);
  points.insert(points.end(),upper.begin(),upper.end());
  OccupyMap map(config()); map.updateSupport(points);
  const auto lock = map.lockForReading();
  VoxelKey lower_key, upper_key, missing;
  ASSERT_TRUE(groundAt(map,.05,.05,.01,lower_key));
  ASSERT_TRUE(groundAt(map,.05,.05,3.01,upper_key));
  EXPECT_NE(lower_key.z,upper_key.z);
  EXPECT_FALSE(groundAt(map,.05,.05,1.5,missing));
}

TEST(GroundInference, NeverInventsLayerInsideAlreadyMeasuredColumn)
{
  auto points=ring(); points.push_back({.05,.05,.5});
  OccupyMap map(config()); map.updateSupport(points);
  const auto lock = map.lockForReading();
  VoxelKey ground;
  EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));
  ASSERT_TRUE(groundAt(map,.05,.05,.5,ground));
  EXPECT_FALSE(map.inferredSupportUnlocked(ground));
}

TEST(GroundInference, LiveObstaclesStillBlockPreviouslyInferredGround)
{
  OccupyMap map(config()); map.updateSupport(ring());
  {
    const auto lock = map.lockForReading(); VoxelKey ground;
    ASSERT_TRUE(groundAt(map,.05,.05,.01,ground));
  }
  ASSERT_TRUE(map.updateObstacles({{.05,.05,.11}},0,0));
  const auto lock = map.lockForReading();
  Astar3D astar(map,Astar3D::Config{});
  EXPECT_EQ(astar.pathPointTraversability({.05,.05,.01}),
    OccupyMap::Traversability::kOccupied);
  std::vector<OccupyMap::Point> direct;
  EXPECT_FALSE(astar.tryDirectGroundPath({-.15,.05,.01}, {.25,.05,.01}, 0, direct));
}

TEST(GroundInference, SupportUpdateInvalidatesCachedInferenceAndMeasurementsWin)
{
  OccupyMap map(config()); map.updateSupport(ring(),1);
  {const auto lock=map.lockForReading(); VoxelKey ground;
    ASSERT_TRUE(groundAt(map,.05,.05,.01,ground));}
  map.updateSupport({{.05,.05,.15}},2);
  {const auto lock=map.lockForReading(); VoxelKey ground;
    EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));
    ASSERT_TRUE(groundAt(map,.05,.05,.15,ground));
    EXPECT_FALSE(map.inferredSupportUnlocked(ground));}
  map.reset();
  {const auto lock=map.lockForReading(); VoxelKey ground;
    EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));}
}

TEST(GroundInference, BoundsCacheAndSupportsConcurrentReaders)
{
  auto c=config(); c.support_inference_max_columns=32;
  OccupyMap map(c); map.updateSupport(ring());
  std::vector<std::thread> readers;
  for (int t=0;t<4;++t) {readers.emplace_back([&]() {
      const auto lock=map.lockForReading();
      for(int i=0;i<100;++i) {VoxelKey ground; groundAt(map,.05,.05,.01,ground);}
    });}
  for(auto & reader:readers) {reader.join();}
  const auto lock=map.lockForReading();
  for(int i=1;i<100;++i) {VoxelKey ground; groundAt(map,i,.05,.01,ground);}
  EXPECT_LE(map.groundQueriesUnlocked(100).size(),32u);
  EXPECT_EQ(map.groundQueriesUnlocked(1).size(),1u);
}

TEST(GroundInference, DisabledPreservesMeasuredOnlyCandidates)
{
  auto c=config(); c.support_inference_enabled=false;
  OccupyMap map(c); map.updateSupport(ring());
  const auto lock=map.lockForReading(); VoxelKey ground;
  EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));
  EXPECT_TRUE(map.groundQueriesUnlocked().empty());
}

TEST(GroundInference, EnablingInterpolationPreservesMeasuredNeighbourFallback)
{
  for (bool enabled : {false,true}) {
    auto c=config(); c.support_inference_enabled=enabled;
    OccupyMap map(c); map.updateSupport({{.15,.05,.01}});
    const auto read=map.lockForReading(); VoxelKey ground;
    EXPECT_TRUE(map.findGroundUnlocked({0,0,2},6,ground));
    EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));
  }
}

TEST(GroundInference, ActualRasterCoordinatesRecoverLoggedInteriorHole)
{
  auto c=config(); c.support_inference_radius=.3;
  OccupyMap map(c);
  std::vector<OccupyMap::Point> points{{1.8,-.2,-.36898},{2.,-.6,-.37253},
    {2.,-.2,-.37157},{2.2,-.4,-.37496}};
  for (auto & p:points) {for (auto & coordinate:p) {coordinate=static_cast<float>(coordinate);}}
  map.updateSupport(points);
  const auto read=map.lockForReading(); VoxelKey ground;
  ASSERT_TRUE(groundAt(map,1.955454545,-.35,-.38,ground));
  EXPECT_TRUE(map.inferredSupportUnlocked(ground));
  EXPECT_EQ(map.supportSnapshotUnlocked().size(),4u);
  EXPECT_TRUE(groundAt(map,1.99999,-.30001,-.38,ground));
}

TEST(GroundInference, CachedCentreSupportDoesNotAuthorizeUnenclosedCellCorners)
{
  auto c=config(); c.support_radius=0.0;
  OccupyMap map(c);
  map.updateSupport({{-.01,0.,.01},{.11,0.,.01},{.05,.11,.01}});
  const auto read=map.lockForReading(); VoxelKey ground;
  EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));
  EXPECT_FALSE(groundAt(map,.001,.099,.01,ground));
  Astar3D planner(map,Astar3D::Config{});
  EXPECT_EQ(planner.pathPointTraversability({.001,.099,.01}),
    OccupyMap::Traversability::kNoGroundSupport);
}

TEST(GroundInference, InterpolationNeverChangesAnExistingMeasuredGroundSelection)
{
  auto a=config(), b=config(); a.support_inference_enabled=false;
  b.support_inference_radius=.3;
  OccupyMap legacy(a), additive(b);
  const std::vector<OccupyMap::Point> points{{.01,.02,.01},{.21,.02,.02},
    {.01,.22,.03},{.21,.22,.01},{.01,.02,2.01}};
  legacy.updateSupport(points); additive.updateSupport(points);
  const auto first=legacy.lockForReading(); const auto second=additive.lockForReading();
  for (int x=-3;x<6;++x) {for(int y=-3;y<6;++y) {for(int z=-2;z<30;++z) {
    VoxelKey old_ground,new_ground;
    if (legacy.findGroundUnlocked({x,y,z},6,old_ground)) {
      ASSERT_TRUE(additive.findGroundUnlocked({x,y,z},6,new_ground));
      EXPECT_TRUE(new_ground==old_ground);
    }
  }}}
}

TEST(GroundInference, MeasuredCoordinatesFollowHeightRekeyAndReset)
{
  auto c=config(); c.support_merge_height=.15;
  OccupyMap map(c);
  map.updateSupport({{.001,.002,.01}},1);
  map.updateSupport({{.012,.013,.12}},2);
  {
    const auto read=map.lockForReading(); VoxelKey ground;
    ASSERT_TRUE(groundAt(map,.015,.015,.12,ground));
    std::array<double,2> xy;
    ASSERT_TRUE(map.measuredSupportXYUnlocked(ground,xy));
    EXPECT_NEAR(xy[0],.012,1e-9); EXPECT_NEAR(xy[1],.013,1e-9);
    EXPECT_FALSE(map.measuredSupportXYUnlocked({0,0,0},xy));
  }
  map.reset();
  const auto read=map.lockForReading(); std::array<double,2> xy;
  EXPECT_FALSE(map.measuredSupportXYUnlocked({0,0,1},xy));
}

TEST(GroundInference, InterpolationKeepsWorldGeometryAcrossRasterOriginShifts)
{
  for (double shift : {-.05,0.,.05}) {
    auto c=config(); c.support_inference_radius=.3;
    auto points=ring();
    for(auto & p:points) {p[0]+=shift; p[1]+=shift;}
    OccupyMap map(c); map.updateSupport(points);
    const auto read=map.lockForReading(); VoxelKey ground;
    EXPECT_TRUE(groundAt(map,.05+shift,.05+shift,.01,ground));
  }
}

TEST(GroundInference, ProductionRadiusStillRejectsLargeUnobservedHoleAndNearStep)
{
  auto c=config(); c.support_inference_radius=.4;
  {
    auto points=ring();
    for(auto & p:points) {p[0]=.05+3*(p[0]-.05); p[1]=.05+3*(p[1]-.05);}
    OccupyMap map(c); map.updateSupport(points);
    const auto read=map.lockForReading(); VoxelKey ground;
    EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));
  }
  {
    auto points=ring(); points.back()[2]+=.08;
    OccupyMap map(c); map.updateSupport(points);
    const auto read=map.lockForReading(); VoxelKey ground;
    EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));
  }
}

TEST(GroundInference, InputOrderAndRepeatedSamplesDoNotChangeInferredHeight)
{
  auto c=config(); c.support_inference_radius=.4;
  auto points=ring();
  for(auto & p:points) {p[2]+=.03*(p[0]-.05)-.02*(p[1]-.05);}
  const auto original=points;
  points.insert(points.end(),original.begin(),original.end());
  std::mt19937 random(1949);
  for(int trial=0;trial<20;++trial) {
    std::shuffle(points.begin(),points.end(),random);
    OccupyMap map(c); map.updateSupport(points);
    const auto read=map.lockForReading(); VoxelKey ground;
    ASSERT_TRUE(groundAt(map,.05,.05,.01,ground));
    double height;
    ASSERT_TRUE(map.supportHeightUnlocked(ground,height));
    EXPECT_NEAR(height,.01,1e-10);
    EXPECT_EQ(map.supportSnapshotUnlocked().size(),4u);
  }
}

TEST(GroundInference, SignedWorldOffsetsSurviveFloat32Transport)
{
  auto c=config(); c.support_inference_radius=.4;
  for(const auto & offset:std::vector<std::array<double,2>>{
      {0,0},{-10,20},{50,-50},{1000,-1000}})
  {
    auto points=ring();
    for(auto & p:points) {
      p[0]=static_cast<float>(offset[0]+.05+1.5*(p[0]-.05));
      p[1]=static_cast<float>(offset[1]+.05+1.5*(p[1]-.05));
    }
    OccupyMap map(c); map.updateSupport(points);
    const auto read=map.lockForReading(); VoxelKey ground;
    EXPECT_TRUE(groundAt(map,offset[0]+.05,offset[1]+.05,.01,ground));
  }
}

TEST(GroundInference, DenseAndSparseChecksAgreeOnJitteredGround)
{
  auto c=config(); c.support_inference_radius=.4;
  std::mt19937 random(20261028);
  std::uniform_real_distribution<double> jitter(-.008,.008);
  std::vector<OccupyMap::Point> points;
  for(int x=-7;x<=14;++x) {for(int y=-7;y<=7;++y) {
    points.push_back({static_cast<float>(x*.2+.01+jitter(random)),
      static_cast<float>(y*.2+.01+jitter(random)),.01});
  }}
  OccupyMap map(c); map.updateSupport(points);
  const auto read=map.lockForReading();
  Astar3D::Config search; search.max_step_height=.08; search.max_slope_deg=10;
  Astar3D planner(map,search);
  VoxelKey start;
  ASSERT_TRUE(planner.projectToSurface(-.31,-.27,.01,.1,.1,start));
  std::vector<OccupyMap::Point> direct;
  ASSERT_TRUE(planner.tryDirectGroundPath({-.31,-.27,.01},{1.71,.39,.01},0,direct));
  for(double spacing:{.2,.1,.05,.013,.007}) {
    auto dense=planner.densifyPath(direct,spacing);
    EXPECT_TRUE(planner.refreshAndValidateTerrainPath(start,dense)) << spacing;
    for(const auto & p:dense) {
      EXPECT_EQ(planner.pathPointTraversability(p),OccupyMap::Traversability::kTraversable);
    }
  }
}

TEST(GroundInference, ResidualBoundaryRejectsJustOverLimit)
{
  for(double change:{.039,.041}) {
    auto c=config(); c.support_inference_radius=.4;
    auto points=ring(); points.back()[2]+=change;
    OccupyMap map(c); map.updateSupport(points);
    const auto read=map.lockForReading(); VoxelKey ground;
    EXPECT_EQ(groundAt(map,.05,.05,.02,ground),change<.04);
  }
}

TEST(GroundInference, StaleSupportCannotReplaceWarmInferenceButNewEvidenceCan)
{
  auto c=config(); c.support_inference_radius=.4;
  OccupyMap map(c); map.updateSupport(ring(),10);
  {const auto read=map.lockForReading(); VoxelKey ground;
    ASSERT_TRUE(groundAt(map,.05,.05,.01,ground));}
  map.updateSupport({{.05,.05,.5}},9);
  {const auto read=map.lockForReading(); VoxelKey ground;
    EXPECT_TRUE(groundAt(map,.05,.05,.01,ground));
    EXPECT_EQ(map.supportSnapshotUnlocked().size(),4u);}
  map.updateSupport({{.05,.05,.5}},11);
  {const auto read=map.lockForReading(); VoxelKey ground;
    EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));
    EXPECT_TRUE(groundAt(map,.05,.05,.5,ground));}
}

TEST(GroundInference, RepeatedHeightRekeysInvalidateAllOldInferredLayers)
{
  auto c=config(); c.support_inference_radius=.4; c.support_merge_height=.15;
  OccupyMap map(c);
  for(int frame=0;frame<4;++frame) {
    const double height=.01+frame*.1;
    map.updateSupport(ring(height),frame+1);
    const auto read=map.lockForReading(); VoxelKey ground;
    EXPECT_TRUE(groundAt(map,.05,.05,height,ground));
    EXPECT_EQ(map.supportSnapshotUnlocked().size(),4u);
    if(frame) {EXPECT_FALSE(groundAt(map,.05,.05,height-.1,ground));}
  }
}

TEST(GroundInference, MovingSamplesInsideUnchangedBinsInvalidatesCoverage)
{
  auto c=config(); c.support_inference_radius=.4;
  OccupyMap map(c);
  map.updateSupport({{-.01,-.01,.01},{.21,-.01,.01},{-.01,.21,.01}},1);
  {const auto read=map.lockForReading(); VoxelKey ground;
    ASSERT_TRUE(groundAt(map,.05,.05,.01,ground));}
  // Same XY/Z voxel keys and heights, different measured triangle coverage.
  map.updateSupport({{-.09,-.09,.01},{.201,-.09,.01},{-.09,.201,.01}},2);
  const auto read=map.lockForReading(); VoxelKey ground;
  EXPECT_FALSE(groundAt(map,.05,.05,.01,ground));
  EXPECT_EQ(map.supportSnapshotUnlocked().size(),3u);
}

TEST(GroundInference, DynamicClearPreservesFixedHazardsAndGroundInference)
{
  auto c=config(); c.support_inference_radius=.4;
  OccupyMap map(c); map.updateSupport(ring(),1);
  map.setStaticObstacles({{.25,.05,.11}});
  ASSERT_TRUE(map.updateObstacles({{.05,.05,.11}},0,0,10));
  {const auto read=map.lockForReading(); Astar3D planner(map,Astar3D::Config{});
    EXPECT_EQ(planner.pathPointTraversability({.05,.05,.01}),OccupyMap::Traversability::kOccupied);}
  EXPECT_FALSE(map.updateObstacles({},0,0,9,{},true));
  ASSERT_TRUE(map.updateObstacles({},0,0,11,{},true));
  const auto read=map.lockForReading(); Astar3D planner(map,Astar3D::Config{});
  EXPECT_EQ(planner.pathPointTraversability({.05,.05,.01}),OccupyMap::Traversability::kTraversable);
  EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(map.posToKey(.25,.05,.11)));
  VoxelKey ground;
  ASSERT_TRUE(groundAt(map,.05,.05,.01,ground));
  EXPECT_TRUE(map.inferredSupportUnlocked(ground));
}

}  // namespace astar_3d_planner
