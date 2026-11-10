#include <gtest/gtest.h>
#include <atomic>
#include <future>
#include <limits>
#include <thread>
#include "astar_3d_planner/occupy_map.hpp"
#include "astar_3d_planner/swept_footprint.hpp"

using astar_3d_planner::OccupyMap;

TEST(StaticMap, ProvenanceIsMergedPerVoxelAndMissingMetadataCannotRefine)
{
  OccupyMap::Config config;
  config.resolution = .1;
  OccupyMap map(config);
  const OccupyMap::Point first{.205, .03, .25}, last{.29, .09, .25};
  const auto key = map.posToKey(.25, .05, .25);
  map.setStaticObstacles({first, last}, 1,
    {OccupyMap::kMeasured, OccupyMap::kMeasured});
  {const auto read = map.lockForReading();
    OccupyMap::ObstacleBounds bounds;
    ASSERT_TRUE(map.staticObstacleBoundsUnlocked(key, bounds));
    EXPECT_EQ(bounds.minimum, first);
    EXPECT_EQ(bounds.maximum, last);
    EXPECT_EQ(bounds.sources, OccupyMap::kMeasured);
    OccupyMap::ObstacleEvidence evidence;
    ASSERT_TRUE(map.obstacleEvidenceUnlocked(key, evidence));
    EXPECT_EQ(evidence.sources, OccupyMap::kStaticMap);}
  map.setStaticObstacles({first, last}, 2,
    {OccupyMap::kMeasured, OccupyMap::kSynthetic});
  {const auto read = map.lockForReading();
    OccupyMap::ObstacleBounds bounds;
    EXPECT_FALSE(map.staticObstacleBoundsUnlocked(key, bounds));
    EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(key));}
  map.setStaticObstacles({last}, 3);
  {const auto read = map.lockForReading();
    OccupyMap::ObstacleBounds bounds;
    EXPECT_FALSE(map.staticObstacleBoundsUnlocked(key, bounds));
    EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(key));}
  EXPECT_THROW(map.setStaticObstacles({first, last}, 4,
      {OccupyMap::kMeasured}), std::invalid_argument);
}

TEST(StaticMap, PlanningSnapshotDoesNotBlockLiveWriterAndRemainsConsistent)
{
  OccupyMap map(OccupyMap::Config{});
  map.updateSupport({{0, 0, -.4}, {1, 0, -.4}}, 1000000000LL);
  map.setStaticObstacles({{3, 0, .2}}, 1000000000LL);
  ASSERT_TRUE(map.updateObstacles({{2, 0, .2}}, 0, 0, 1000000000LL,
    {OccupyMap::kMeasured}, true, {}, {1000000000LL}));
  auto snapshot = map.planningSnapshotAfterExpiry(1100000000LL, 600000000LL);
  // Simulate a long search owning its immutable view. A worker must be able
  // to commit without releasing that view, rather than waiting for freshness
  // to expire. Release the test lock before joining even on failure.
  auto search_read = snapshot->lockForReading();
  auto writer = std::async(std::launch::async, [&]() {
      return map.updateObstacles({{1, 0, .2}}, 0, 0, 1200000000LL,
        {OccupyMap::kMeasured}, true, {}, {1200000000LL});
    });
  EXPECT_EQ(writer.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  search_read.unlock();
  ASSERT_TRUE(writer.get());
  EXPECT_TRUE(snapshot->rawObstacleOccupiedUnlocked(snapshot->posToKey(2, 0, .2)));
  EXPECT_FALSE(snapshot->rawObstacleOccupiedUnlocked(snapshot->posToKey(1, 0, .2)));
  const auto live_read = map.lockForReading();
  EXPECT_EQ(snapshot->obstacleStampUnlocked(), 1000000000LL);
  EXPECT_EQ(map.obstacleStampUnlocked(), 1200000000LL);
  EXPECT_TRUE(map.planningStateMatchesUnlocked(*snapshot));
  EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(map.posToKey(1, 0, .2)));
  EXPECT_FALSE(map.rawObstacleOccupiedUnlocked(map.posToKey(2, 0, .2)));
  EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(map.posToKey(3, 0, .2)));
}

TEST(StaticMap, LatestMapSweepVetoesNewObstacleAbsentFromPlanningSnapshot)
{
  OccupyMap map(OccupyMap::Config{});
  map.updateSupport({{0, 0, -.4}, {1, 0, -.4}}, 1000000000LL);
  auto snapshot = map.planningSnapshotAfterExpiry(1100000000LL, 600000000LL);
  const astar_3d_planner::SweptFootprint::Config body;
  const std::vector<OccupyMap::Point> path{{0, 0, -.4}, {1, 0, -.4}};
  EXPECT_TRUE(astar_3d_planner::SweptFootprint(*snapshot, body).validatePath(path, 0, 0));
  ASSERT_TRUE(map.updateObstacles({{.7, 0, .1}}, 0, 0, 1200000000LL,
    {OccupyMap::kMeasured}, true, {}, {1200000000LL}));
  const auto read = map.lockForReading();
  EXPECT_FALSE(astar_3d_planner::SweptFootprint(map, body).validatePath(path, 0, 0));
  EXPECT_TRUE(astar_3d_planner::SweptFootprint(*snapshot, body).validatePath(path, 0, 0));
}

TEST(StaticMap, ReferenceSnapshotOnlyRemovesNearbyPureMeasuredLiveVoxels)
{
  OccupyMap::Config config;
  config.inflate_radius = .2;
  OccupyMap map(config);
  const OccupyMap::Point near{.45, .05, .25};
  const OccupyMap::Point synthetic{.85, .15, .25};
  const OccupyMap::Point unknown{.15, .55, .25};
  const OccupyMap::Point far{1.35, .05, .25};
  map.updateSupport({{0, 0, -.4}}, 1000000000LL);
  ASSERT_TRUE(map.updateObstacles({near, synthetic, unknown, far}, 0, 0,
    1100000000LL, {OccupyMap::kMeasured, OccupyMap::kSynthetic,
      OccupyMap::kUnknown, OccupyMap::kMeasured}, true));
  const auto read = map.lockForReading();
  const auto original_count = map.rawVoxelCountUnlocked();
  const auto near_cost_key = map.posToKey(.45, -.25, .25);
  EXPECT_GT(map.obstacleCostUnlocked(near_cost_key, 0), 0);
  std::size_t ignored = 999;
  auto snapshot = map.referenceSnapshotUnlocked(0, 0, 1.0, &ignored);
  EXPECT_EQ(ignored, 1u);
  EXPECT_EQ(snapshot->rawVoxelCountUnlocked(), original_count - 1);
  EXPECT_FALSE(snapshot->rawObstacleOccupiedUnlocked(map.posToKey(near[0], near[1], near[2])));
  EXPECT_FALSE(snapshot->occupiedUnlocked(map.posToKey(near[0], near[1], near[2])));
  EXPECT_DOUBLE_EQ(snapshot->obstacleCostUnlocked(near_cost_key, 0), 0);
  for (const auto & point : {synthetic, unknown, far}) {
    EXPECT_TRUE(snapshot->rawObstacleOccupiedUnlocked(
        map.posToKey(point[0], point[1], point[2])));
  }
  // Filtering the route's view must not change physical collision checks or
  // the input generation, and must leave the complete support map intact.
  EXPECT_EQ(map.rawVoxelCountUnlocked(), original_count);
  EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(map.posToKey(near[0], near[1], near[2])));
  EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(near[0], near[1], near[2])));
  EXPECT_GT(map.obstacleCostUnlocked(near_cost_key, 0), 0);
  EXPECT_TRUE(map.planningStateMatchesUnlocked(*snapshot));
  EXPECT_EQ(snapshot->supportStampUnlocked(), map.supportStampUnlocked());
  EXPECT_EQ(snapshot->obstacleStampUnlocked(), map.obstacleStampUnlocked());
  EXPECT_EQ(snapshot->supportSnapshotUnlocked(), map.supportSnapshotUnlocked());
}

TEST(StaticMap, ReferenceSnapshotPreservesCoincidentStaticAndMixedLiveOwnership)
{
  OccupyMap::Config config;
  config.inflate_radius = 0;
  OccupyMap map(config);
  const OccupyMap::Point fixed{.45, .05, .25};
  const OccupyMap::Point mixed{.35, .35, .25};
  const OccupyMap::Point removable{.15, -.45, .25};
  map.setStaticObstacles({fixed}, 1000000000LL, {OccupyMap::kMeasured});
  ASSERT_TRUE(map.updateObstacles({fixed, mixed, mixed, removable}, 0, 0,
    1100000000LL, {OccupyMap::kMeasured, OccupyMap::kMeasured,
      OccupyMap::kSynthetic, OccupyMap::kMeasured}, true));
  const auto read = map.lockForReading();
  std::size_t ignored = 0;
  auto snapshot = map.referenceSnapshotUnlocked(0, 0, 1, &ignored);
  EXPECT_EQ(ignored, 1u);
  for (const auto & point : {fixed, mixed}) {
    const auto key = map.posToKey(point[0], point[1], point[2]);
    EXPECT_TRUE(snapshot->rawObstacleOccupiedUnlocked(key));
  }
  EXPECT_FALSE(snapshot->rawObstacleOccupiedUnlocked(
      map.posToKey(removable[0], removable[1], removable[2])));
  OccupyMap::ObstacleBounds bounds;
  ASSERT_TRUE(snapshot->measuredObstacleBoundsUnlocked(map.posToKey(.45, .05, .25), bounds));
  EXPECT_NE(bounds.sources & OccupyMap::kStaticMap, 0);
  EXPECT_FALSE(snapshot->measuredObstacleBoundsUnlocked(map.posToKey(.35, .35, .25), bounds));
  snapshot->clearDynamicObstacles();
  EXPECT_TRUE(snapshot->rawObstacleOccupiedUnlocked(map.posToKey(.45, .05, .25)));
  EXPECT_TRUE(snapshot->staticObstacleBoundsUnlocked(map.posToKey(.45, .05, .25), bounds));
}

TEST(StaticMap, ReferenceSnapshotRetainsCellsCrossingLocalRadiusBoundary)
{
  OccupyMap::Config config;
  config.inflate_radius = 0;
  OccupyMap map(config);
  const OccupyMap::Point inside{.45, .05, .25};
  // This point and its cell centre are inside the circle, but its farthest
  // voxel corner is outside. It must remain in the global reference view.
  const OccupyMap::Point crossing{.45, .25, .25};
  ASSERT_TRUE(map.updateObstacles({inside, crossing}, .05, .05, 1,
    {OccupyMap::kMeasured, OccupyMap::kMeasured}, true));
  const auto read = map.lockForReading();
  std::size_t ignored = 0;
  auto snapshot = map.referenceSnapshotUnlocked(.05, .05, .5, &ignored);
  EXPECT_EQ(ignored, 1u);
  EXPECT_FALSE(snapshot->rawObstacleOccupiedUnlocked(map.posToKey(.45, .05, .25)));
  EXPECT_TRUE(snapshot->rawObstacleOccupiedUnlocked(map.posToKey(.45, .25, .25)));
  auto strict = map.referenceSnapshotUnlocked(.05, .05, 0, &ignored);
  EXPECT_EQ(ignored, 0u);
  EXPECT_EQ(strict->rawVoxelCountUnlocked(), map.rawVoxelCountUnlocked());
}

TEST(StaticMap, ReferenceSnapshotRejectsInvalidRegion)
{
  OccupyMap map(OccupyMap::Config{});
  const auto read = map.lockForReading();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(map.referenceSnapshotUnlocked(nan, 0, 1), std::invalid_argument);
  EXPECT_THROW(map.referenceSnapshotUnlocked(0, nan, 1), std::invalid_argument);
  EXPECT_THROW(map.referenceSnapshotUnlocked(0, 0, nan), std::invalid_argument);
  EXPECT_THROW(map.referenceSnapshotUnlocked(0, 0, -1), std::invalid_argument);
}

TEST(StaticMap, SnapshotExpiresOnlyOldMeasuredReturnsAndPreservesProvenance)
{
  OccupyMap map(OccupyMap::Config{});
  map.updateSupport({{0, 0, -.4}}, 1000000000LL);
  map.setStaticObstacles({{1, 0, .1}}, 1000000000LL);
  ASSERT_TRUE(map.updateObstacles({{2, 0, .1}, {3, 0, .1}, {4, 0, .1}}, 0, 0,
    1200000000LL, {1, 1, 2}, true, {}, {100000000LL, 1100000000LL, 100000000LL}));
  auto snapshot = map.planningSnapshotAfterExpiry(1200000000LL, 600000000LL);
  EXPECT_TRUE(snapshot->readyUnlocked());
  EXPECT_TRUE(snapshot->rawObstacleOccupiedUnlocked(snapshot->posToKey(1, 0, .1)));
  EXPECT_FALSE(snapshot->rawObstacleOccupiedUnlocked(snapshot->posToKey(2, 0, .1)));
  EXPECT_TRUE(snapshot->rawObstacleOccupiedUnlocked(snapshot->posToKey(3, 0, .1)));
  EXPECT_TRUE(snapshot->rawObstacleOccupiedUnlocked(snapshot->posToKey(4, 0, .1)));
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(snapshot->obstacleEvidenceUnlocked(snapshot->posToKey(3, 0, .1), evidence));
  EXPECT_EQ(evidence.sources, OccupyMap::kMeasured);
  EXPECT_EQ(evidence.observed_stamp_ns, 1100000000LL);
  EXPECT_EQ(evidence.point, (OccupyMap::Point{3, 0, .1}));
  double height;
  ASSERT_TRUE(snapshot->supportHeightUnlocked(snapshot->posToKey(0, 0, -.4), height));
  EXPECT_DOUBLE_EQ(height, -.4);
}

TEST(StaticMap, TerrainChangeOrRestartInvalidatesPlanningGeneration)
{
  OccupyMap map(OccupyMap::Config{});
  map.updateSupport({{0, 0, -.4}}, 1000000000LL);
  auto snapshot = map.planningSnapshotAfterExpiry(1100000000LL, 600000000LL);
  map.clearDynamicObstacles(true);
  {const auto read = map.lockForReading();
    EXPECT_FALSE(map.planningStateMatchesUnlocked(*snapshot));}
  snapshot = map.planningSnapshotAfterExpiry(1200000000LL, 600000000LL);
  map.updateSupport({{1, 0, -.4}}, 1300000000LL);
  {const auto read = map.lockForReading();
    EXPECT_FALSE(map.planningStateMatchesUnlocked(*snapshot));}
}

TEST(StaticMap, QueryTimeExpiryUsesOriginalMeasuredTimeAndKeepsUncertainEvidence)
{
  OccupyMap::Config config;config.inflate_radius=.2;
  OccupyMap map(config);
  ASSERT_TRUE(map.updateObstacles({{1,0,.5},{2,0,.5},{3,0,.5},{4,0,.5},{5,0,.5},{6,0,.5}},
    0,0,1000000000LL,{1,1,1,4,2,1},true,{},
    {500000000LL,700000000LL,0,500000000LL,500000000LL,2000000000LL}));
  {auto read=map.lockForReadingAfterExpiry(1100000000LL,600000000LL);
    EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(map.posToKey(1,0,.5)));}
  std::size_t expired=0;
  {auto read=map.lockForReadingAfterExpiry(1101000000LL,600000000LL,&expired);
    EXPECT_EQ(expired,1u);
    EXPECT_FALSE(map.rawObstacleOccupiedUnlocked(map.posToKey(1,0,.5)));
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(1.1,0,.5)));
    for(int x=2;x<=6;++x) {EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(map.posToKey(x,0,.5)));}
    EXPECT_EQ(map.obstacleStampUnlocked(),1000000000LL);}
}

TEST(StaticMap, ExpiredDynamicObservationCannotEraseCoincidentFixedObstacle)
{
  OccupyMap map(OccupyMap::Config{});
  map.setStaticObstacles({{1,0,.5}});
  ASSERT_TRUE(map.updateObstacles({{1,0,.5}},0,0,1000000000LL,{1},true,{}, {500000000LL}));
  const auto read=map.lockForReadingAfterExpiry(1200000000LL,600000000LL);
  EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(map.posToKey(1,0,.5)));
  OccupyMap::ObstacleEvidence evidence;
  ASSERT_TRUE(map.obstacleEvidenceUnlocked(map.posToKey(1,0,.5),evidence));
  EXPECT_EQ(evidence.sources,OccupyMap::kStaticMap);
}

TEST(StaticMap, FreshestReturnInVoxelSurvivesAndExpiredDataCannotRefreshWatermark)
{
  OccupyMap map(OccupyMap::Config{});
  ASSERT_TRUE(map.updateObstacles({{1.01,0,.5},{1.02,0,.5}},0,0,1000000000LL,
    {1,1},true,{}, {500000000LL,950000000LL}));
  {auto read=map.lockForReadingAfterExpiry(1200000000LL,600000000LL);
    EXPECT_TRUE(map.rawObstacleOccupiedUnlocked(map.posToKey(1.01,0,.5)));}
  {auto read=map.lockForReadingAfterExpiry(1600000000LL,600000000LL);
    EXPECT_FALSE(map.rawObstacleOccupiedUnlocked(map.posToKey(1.01,0,.5)));}
  EXPECT_FALSE(map.updateObstacles({{1.01,0,.5}},0,0,1000000000LL,{1},true,{}, {950000000LL}));
  ASSERT_TRUE(map.updateObstacles({{1.01,0,.5}},0,0,1100000000LL,{1},true,{}, {950000000LL}));
  {auto read=map.lockForReadingAfterExpiry(1600000000LL,600000000LL);
    EXPECT_FALSE(map.rawObstacleOccupiedUnlocked(map.posToKey(1.01,0,.5)));}
}

TEST(StaticMap, ConcurrentNewSnapshotCannotReinsertExpiredEvidenceIntoPlanningRead)
{
  OccupyMap::Config config;config.inflate_radius=0;
  OccupyMap map(config);std::atomic<bool> started{false};
  std::thread writer([&](){
      started.store(true);
      for(int i=1;i<=500;++i) {
        if(i%50==0) {map.reset();}
        map.updateObstacles({{1,0,.5}},0,0,1000000000LL+i,{1},true,{}, {100000000LL});
      }
    });
  while(!started.load()) {std::this_thread::yield();}
  for(int i=0;i<500;++i) {
    auto read=map.lockForReadingAfterExpiry(1200000000LL,600000000LL);
    EXPECT_FALSE(map.rawObstacleOccupiedUnlocked(map.posToKey(1,0,.5)));
  }
  writer.join();
}

TEST(StaticMap, DoesNotBecomeReadyUntilBothMapPartsArrive)
{
  OccupyMap::Config config;
  config.require_static_obstacles = true;
  OccupyMap map(config);
  map.updateSupport({{0, 0, 0}});
  {auto read = map.lockForReading(); EXPECT_FALSE(map.readyUnlocked());}
  map.setStaticObstacles({});
  {auto read = map.lockForReading(); EXPECT_TRUE(map.readyUnlocked());}
  map.reset();
  {auto read = map.lockForReading(); EXPECT_FALSE(map.readyUnlocked());}
}

TEST(StaticMap, LiveClearingAndReplacementNeverEraseFixedObstacles)
{
  OccupyMap::Config config;
  config.inflate_radius = 0.2;
  OccupyMap map(config);
  map.setStaticObstacles({{0, 0, 0.5}});
  map.updateObstacles({{0, 0, 0.5}, {1, 0, 0.5}}, 0, 0, 1);
  for (int i = 2; i < 10; ++i) {map.updateObstacles({}, 0, 0, i);}
  {auto read = map.lockForReading();
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(0, 0, .5)));
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(.1, 0, .5)));
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(1, 0, .5)));}
  map.updateObstacles({{2, 0, .5}}, 0, 0, 10, {}, true);
  map.updateObstacles({{3, 0, .5}}, 0, 0, 11, {}, true);
  {auto read = map.lockForReading();
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(0, 0, .5)));
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(2, 0, .5)));
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(3, 0, .5)));}
  map.clearDynamicObstacles();
  {auto read = map.lockForReading();
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(0, 0, .5)));
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(3, 0, .5)));}
}

TEST(StaticMap, ReplacingFixedLayerPreservesOverlappingLiveOwnership)
{
  OccupyMap::Config config;
  config.inflate_radius = 0;
  OccupyMap map(config);
  map.setStaticObstacles({{0, 0, .5}, {1, 0, .5}});
  map.updateObstacles({{0, 0, .5}}, 0, 0, 1);
  map.setStaticObstacles({{2, 0, .5}});
  {auto read = map.lockForReading();
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(0, 0, .5)));
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(1, 0, .5)));
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(2, 0, .5)));}
  map.clearDynamicObstacles();
  {auto read = map.lockForReading();
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(0, 0, .5)));
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(2, 0, .5)));}
}

TEST(StaticMap, RejectsOnlineHeightCompensation)
{
  OccupyMap::Config config;
  config.compensate_height_drift = true;
  OccupyMap map(config);
  EXPECT_THROW(map.setStaticObstacles({}), std::invalid_argument);
}

TEST(StaticMap, RestartedLiveStreamAcceptsEarlierStampsAndKeepsFixedOwnership)
{
  OccupyMap::Config config;
  config.require_static_obstacles = true;
  config.inflate_radius = 0;
  OccupyMap map(config);
  map.updateSupport({{0, 0, 0}}, 100);
  map.setStaticObstacles({{0, 0, .5}}, 100);
  map.updateObstacles({{0, 0, .5}, {1, 0, .5}}, 0, 0, 100, {}, true);
  const auto previous_epoch = map.dynamicObstacleEpoch();

  map.clearDynamicObstacles(true);
  const auto current_epoch = map.dynamicObstacleEpoch();
  ASSERT_NE(current_epoch, previous_epoch);
  {auto read = map.lockForReading();
    EXPECT_TRUE(map.readyUnlocked());
    EXPECT_EQ(map.supportStampUnlocked(), 100);
    EXPECT_EQ(map.obstacleStampUnlocked(), 0);
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(0, 0, .5)));
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(1, 0, .5)));
    OccupyMap::ObstacleEvidence evidence;
    ASSERT_TRUE(map.obstacleEvidenceUnlocked(map.posToKey(0, 0, .5), evidence));
    EXPECT_EQ(evidence.sources, OccupyMap::kStaticMap);}

  // A restarted replay is earlier than the retained immutable support stamp.
  map.updateObstacles({{0, 0, .5}, {2, 0, .5}}, 0, 0, 10, {}, true, current_epoch);
  {auto read = map.lockForReading();
    EXPECT_EQ(map.obstacleStampUnlocked(), 10);
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(2, 0, .5)));}
  map.clearDynamicObstacles();
  {auto read = map.lockForReading();
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(0, 0, .5)));
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(2, 0, .5)));
    OccupyMap::ObstacleEvidence evidence;
    ASSERT_TRUE(map.obstacleEvidenceUnlocked(map.posToKey(0, 0, .5), evidence));
    EXPECT_EQ(evidence.sources, OccupyMap::kStaticMap);}
}

TEST(StaticMap, InFlightCloudFromOldEpochCannotReplaceRecoveredLiveWindow)
{
  OccupyMap::Config config;
  config.inflate_radius = 0;
  OccupyMap map(config);
  map.setStaticObstacles({{0, 0, .5}});
  const auto in_flight_epoch = map.dynamicObstacleEpoch();
  map.updateObstacles({{1, 0, .5}}, 0, 0, 100, {}, true, in_flight_epoch);
  map.clearDynamicObstacles(true);
  const auto recovered_epoch = map.dynamicObstacleEpoch();
  map.updateObstacles({{2, 0, .5}}, 0, 0, 10, {}, true, recovered_epoch);

  // Simulate a slow TF/point-cloud conversion completing after recovery.
  map.updateObstacles({{3, 0, .5}}, 0, 0, 101, {}, true, in_flight_epoch);
  // Older out-of-order frames within the new epoch remain rejected too.
  map.updateObstacles({{4, 0, .5}}, 0, 0, 9, {}, true, recovered_epoch);
  {auto read = map.lockForReading();
    EXPECT_EQ(map.obstacleStampUnlocked(), 10);
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(0, 0, .5)));
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(1, 0, .5)));
    EXPECT_TRUE(map.occupiedUnlocked(map.posToKey(2, 0, .5)));
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(3, 0, .5)));
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(4, 0, .5)));}
}

TEST(StaticMap, OrdinaryClearKeepsOrderingAndFullResetInvalidatesInFlightWork)
{
  OccupyMap::Config config;
  config.inflate_radius = 0;
  OccupyMap map(config);
  const auto original_epoch = map.dynamicObstacleEpoch();
  map.updateObstacles({{1, 0, .5}}, 0, 0, 100);
  map.clearDynamicObstacles();
  EXPECT_EQ(map.dynamicObstacleEpoch(), original_epoch);
  map.updateObstacles({{2, 0, .5}}, 0, 0, 10);
  {auto read = map.lockForReading();
    EXPECT_EQ(map.obstacleStampUnlocked(), 100);
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(2, 0, .5)));}
  map.reset();
  EXPECT_NE(map.dynamicObstacleEpoch(), original_epoch);
  map.updateObstacles({{3, 0, .5}}, 0, 0, 101, {}, true, original_epoch);
  {auto read = map.lockForReading();
    EXPECT_EQ(map.obstacleStampUnlocked(), 0);
    EXPECT_FALSE(map.occupiedUnlocked(map.posToKey(3, 0, .5)));}
}

TEST(StaticMap, FloatTransportDoesNotBreakRegularSupportGrid)
{
  OccupyMap::Config config;
  config.require_static_obstacles = true;
  OccupyMap fixed(config);
  const double x = static_cast<float>(2.8);
  ASSERT_LT(x, 2.8);
  fixed.updateSupport({{x, 0, -.5}, {3.0, 0, -.5}});
  fixed.setStaticObstacles({});
  {auto read = fixed.lockForReading();
    double height;
    EXPECT_TRUE(fixed.supportHeightUnlocked({28, 0, -5}, height));
    EXPECT_TRUE(fixed.supportHeightUnlocked({30, 0, -5}, height));}
  config.require_static_obstacles = false;
  OccupyMap online(config);
  online.updateSupport({{x, 0, -.5}});
  {auto read = online.lockForReading();
    double height;
    EXPECT_TRUE(online.supportHeightUnlocked({28, 0, -5}, height));}
}
