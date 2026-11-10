#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "astar_3d_planner/astar_3d_planner.hpp"
#include "astar_3d_planner/swept_footprint.hpp"

namespace astar_3d_planner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

OccupyMap::Config mapConfig()
{
  OccupyMap::Config config;
  config.resolution = .05;
  config.inflate_radius = .3;
  return config;
}

SweptFootprint::Config paddedFootprint()
{
  SweptFootprint::Config config;
  config.min_x = -.41;
  config.max_x = .41;
  config.min_y = -.26;
  config.max_y = .26;
  return config;
}

std::vector<OccupyMap::Point> corridorWalls()
{
  std::vector<OccupyMap::Point> walls;
  for (int i = -20; i <= 80; ++i) {
    walls.push_back({i * .05 + .025, .375, .225});
    walls.push_back({i * .05 + .025, -.375, .225});
  }
  return walls;
}

nav_msgs::msg::Path pathFromPoints(
  const std::vector<OccupyMap::Point> & points, double goal_yaw = 0)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    pose.pose.position.z = point[2];
    pose.pose.orientation.w = 1;
    path.poses.push_back(pose);
  }
  if (!path.poses.empty()) {
    path.poses.back().pose.orientation.z = std::sin(goal_yaw / 2);
    path.poses.back().pose.orientation.w = std::cos(goal_yaw / 2);
  }
  return path;
}

std::vector<OccupyMap::Point> pointsFromPath(const nav_msgs::msg::Path & path)
{
  std::vector<OccupyMap::Point> points;
  for (const auto & pose : path.poses) {
    const auto & point = pose.pose.position;
    points.push_back({point.x, point.y, point.z});
  }
  return points;
}

nav_msgs::msg::Path straightPath()
{
  std::vector<OccupyMap::Point> points;
  for (int i = 0; i <= 40; ++i) {points.push_back({i * .05, 0, 0});}
  return pathFromPoints(points);
}

bool containsPoint(const nav_msgs::msg::Path & path, double x, double y)
{
  for (const auto & pose : path.poses) {
    if (std::hypot(pose.pose.position.x - x, pose.pose.position.y - y) < 1e-9) {
      return true;
    }
  }
  return false;
}
}  // namespace

TEST(PathReconnectionGeometry, ForwardConnectionAvoidsFalseLateralTurnNearWalls)
{
  OccupyMap map(mapConfig());
  map.setStaticObstacles(corridorWalls());
  SweptFootprint checker(map, paddedFootprint());
  const auto lock = map.lockForReading();
  const OccupyMap::Point current{0, .02, 0};

  ASSERT_TRUE(checker.poseCollisionFree(current, 0));
  ASSERT_TRUE(checker.validatePath({{0, 0, 0}, {2, 0, 0}}, 0, 0));

  // A nearest-point stitch invents a 90-degree lateral maneuver for only
  // two centimetres of cross-track error. The real body cannot make that turn.
  SweptFootprint::Failure lateral_failure;
  EXPECT_FALSE(checker.validatePath(
      {current, {0, 0, 0}, {2, 0, 0}}, 0, 0, &lateral_failure));
  EXPECT_STREQ(lateral_failure.reason, "footprint_collision");
  EXPECT_EQ(lateral_failure.index, 0u);

  // Retain the complete rectangular turn and translation checks for the
  // shallow forward connection instead of weakening the collision checker.
  EXPECT_TRUE(checker.validatePath(
      {current, {.4, 0, 0}, {2, 0, 0}}, 0, 0));
}

TEST(PathReconnectionGeometry, ForwardConnectionCannotHideCurrentBodyCollision)
{
  OccupyMap map(mapConfig());
  map.setStaticObstacles(corridorWalls());
  SweptFootprint checker(map, paddedFootprint());
  const auto lock = map.lockForReading();
  const OccupyMap::Point current{0, .10, 0};

  ASSERT_TRUE(checker.validatePath({{0, 0, 0}, {2, 0, 0}}, 0, 0));
  ASSERT_FALSE(checker.poseCollisionFree(current, 0));
  SweptFootprint::Failure failure;
  EXPECT_FALSE(checker.validatePath(
      {current, {.8, 0, 0}, {2, 0, 0}}, 0, 0, &failure));
  EXPECT_STREQ(failure.reason, "footprint_collision");
  EXPECT_EQ(failure.index, 0u);
  EXPECT_DOUBLE_EQ(failure.position[0], current[0]);
  EXPECT_DOUBLE_EQ(failure.position[1], current[1]);
}

TEST(PathReconnectionGeometry, ForwardLookaheadCannotCutObstacleInsideCorner)
{
  OccupyMap map(mapConfig());
  map.setStaticObstacles({{1.625, .525, .225}});
  SweptFootprint checker(map, paddedFootprint());
  const auto lock = map.lockForReading();
  const OccupyMap::Point current{1.5, 0, 0};

  // The original corner has room for its complete rectangular rotation.
  ASSERT_TRUE(checker.validatePath(
      {current, {2, 0, 0}, {2, 2, 0}}, 0, kPi / 2));
  // A point farther along the route is not automatically a safe connector:
  // the chord to it cuts through an obstacle beside the original route.
  SweptFootprint::Failure failure;
  EXPECT_FALSE(checker.validatePath(
      {current, {2, .5, 0}, {2, 2, 0}}, 0, kPi / 2, &failure));
  EXPECT_STREQ(failure.reason, "footprint_collision");
  EXPECT_TRUE(failure.has_collision);
  EXPECT_EQ(failure.index, 1u);
}

TEST(PathReconnection, NearWallsUsesSafeForwardConnectionWithFullRectangle)
{
  OccupyMap map(mapConfig());
  map.setStaticObstacles(corridorWalls());
  SweptFootprint checker(map, paddedFootprint());
  const auto lock = map.lockForReading();
  const auto cached = straightPath();
  nav_msgs::msg::Path projected;
  geometry_msgs::msg::PoseStamped projection;
  double progress = 0;
  ASSERT_TRUE(selectStablePathSuffix(
      cached, .4, .02, .75, .3, progress, projected, &projection));
  ASSERT_FALSE(checker.validatePath(pointsFromPath(projected), 0, 0));

  PathReconnectDiagnostics diagnostics;
  nav_msgs::msg::Path output;
  ASSERT_TRUE(reconnectPathSuffix(projected, projection, .6,
      [&](nav_msgs::msg::Path & candidate) {
        return checker.validatePath(pointsFromPath(candidate), 0, 0);
      }, output, &diagnostics));

  ASSERT_GE(output.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.position.x, .4);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.position.y, .02);
  EXPECT_GT(output.poses[1].pose.position.x, projection.pose.position.x + .1);
  EXPECT_NEAR(output.poses[1].pose.position.y, 0, 1e-9);
  EXPECT_TRUE(checker.validatePath(pointsFromPath(output), 0, 0));
  EXPECT_TRUE(diagnostics.used_forward);
  EXPECT_GT(diagnostics.forward_distance, .1);
  EXPECT_LE(diagnostics.forward_distance, .6 + 1e-9);
  EXPECT_LE(diagnostics.attempts, 5u);
  EXPECT_EQ(output.header, cached.header);
  EXPECT_EQ(output.poses.back(), cached.poses.back());
  EXPECT_NEAR(progress, .4, 1e-9);
}

TEST(PathReconnection, ActualBodyCollisionRejectsAllForwardCandidates)
{
  OccupyMap map(mapConfig());
  map.setStaticObstacles(corridorWalls());
  SweptFootprint checker(map, paddedFootprint());
  const auto lock = map.lockForReading();
  const auto cached = straightPath();
  ASSERT_TRUE(checker.validatePath(pointsFromPath(cached), 0, 0));
  nav_msgs::msg::Path projected;
  geometry_msgs::msg::PoseStamped projection;
  double progress = 0;
  ASSERT_TRUE(selectStablePathSuffix(
      cached, .4, .10, .75, .3, progress, projected, &projection));

  PathReconnectDiagnostics diagnostics;
  nav_msgs::msg::Path output;
  EXPECT_FALSE(reconnectPathSuffix(projected, projection, .6,
      [&](nav_msgs::msg::Path & candidate) {
        if (candidate.poses.empty()) {
          ADD_FAILURE() << "Reconnect validation received an empty path";
          return false;
        }
        EXPECT_DOUBLE_EQ(candidate.poses.front().pose.position.x, .4);
        EXPECT_DOUBLE_EQ(candidate.poses.front().pose.position.y, .10);
        return checker.validatePath(pointsFromPath(candidate), 0, 0);
      }, output, &diagnostics));
  EXPECT_GE(diagnostics.attempts, 1u);
  EXPECT_LE(diagnostics.attempts, 5u);
}

TEST(PathReconnection, ForwardConnectionStillValidatesObstacleInRetainedTail)
{
  OccupyMap map(mapConfig());
  auto obstacles = corridorWalls();
  obstacles.push_back({1.425, .025, .225});
  map.setStaticObstacles(obstacles);
  SweptFootprint checker(map, paddedFootprint());
  const auto lock = map.lockForReading();
  ASSERT_TRUE(checker.poseCollisionFree({.4, .02, 0}, 0));
  nav_msgs::msg::Path projected;
  geometry_msgs::msg::PoseStamped projection;
  double progress = 0;
  ASSERT_TRUE(selectStablePathSuffix(
      straightPath(), .4, .02, .75, .3, progress, projected, &projection));

  nav_msgs::msg::Path output;
  EXPECT_FALSE(reconnectPathSuffix(projected, projection, .6,
      [&](nav_msgs::msg::Path & candidate) {
        if (candidate.poses.empty()) {
          ADD_FAILURE() << "Reconnect validation received an empty path";
          return false;
        }
        EXPECT_DOUBLE_EQ(candidate.poses.back().pose.position.x, 2);
        return checker.validatePath(pointsFromPath(candidate), 0, 0);
      }, output));
}

TEST(PathReconnection, LookaheadPreservesFirstCornerAndGoalOrientation)
{
  const auto cached = pathFromPoints({{0, 0, 0}, {2, 0, 0}, {2, 2, 0}}, kPi / 2);
  nav_msgs::msg::Path projected;
  geometry_msgs::msg::PoseStamped projection;
  double progress = 0;
  ASSERT_TRUE(selectStablePathSuffix(
      cached, 1.5, .02, .75, .3, progress, projected, &projection));

  PathReconnectDiagnostics diagnostics;
  nav_msgs::msg::Path output;
  ASSERT_TRUE(reconnectPathSuffix(projected, projection, .6,
      [](nav_msgs::msg::Path &) {return true;}, output, &diagnostics));
  ASSERT_GE(output.poses.size(), 3u);
  EXPECT_TRUE(diagnostics.used_forward);
  EXPECT_LE(diagnostics.forward_distance, .5 + 1e-9);
  EXPECT_LE(output.poses[1].pose.position.x, 2 + 1e-9);
  EXPECT_NEAR(output.poses[1].pose.position.y, 0, 1e-9);
  EXPECT_TRUE(containsPoint(output, 2, 0));
  EXPECT_EQ(output.poses.back(), cached.poses.back());
}

TEST(PathReconnection, LookaheadAtCrossingRetainsCurrentSegmentAndLaterRoute)
{
  const auto cached = pathFromPoints({
      {0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {1, 2, 0}, {1, -1, 0}});
  nav_msgs::msg::Path projected;
  geometry_msgs::msg::PoseStamped projection;
  double progress = 0;
  ASSERT_TRUE(selectStablePathSuffix(
      cached, .98, .01, .75, .3, progress, projected, &projection));
  ASSERT_NEAR(progress, .98, 1e-9);

  PathReconnectDiagnostics diagnostics;
  nav_msgs::msg::Path output;
  ASSERT_TRUE(reconnectPathSuffix(projected, projection, .6,
      [](nav_msgs::msg::Path &) {return true;}, output, &diagnostics));
  ASSERT_GE(output.poses.size(), 5u);
  EXPECT_TRUE(diagnostics.used_forward);
  EXPECT_GT(output.poses[1].pose.position.x, 1);
  EXPECT_NEAR(output.poses[1].pose.position.y, 0, 1e-9);
  EXPECT_TRUE(containsPoint(output, 2, 0));
  EXPECT_TRUE(containsPoint(output, 2, 2));
  EXPECT_TRUE(containsPoint(output, 1, 2));
  EXPECT_EQ(output.poses.back(), cached.poses.back());
  EXPECT_NEAR(progress, .98, 1e-9);
}

TEST(PathReconnection, RepeatedLookaheadDoesNotAdvanceObservedProgress)
{
  const auto cached = straightPath();
  double progress = 0;
  for (double robot_x : {.4, .4, .42, .40}) {
    nav_msgs::msg::Path projected;
    geometry_msgs::msg::PoseStamped projection;
    const double expected_progress = std::max(progress, robot_x);
    ASSERT_TRUE(selectStablePathSuffix(
        cached, robot_x, .02, .75, .3, progress, projected, &projection));
    ASSERT_NEAR(progress, expected_progress, 1e-9);
    nav_msgs::msg::Path output;
    PathReconnectDiagnostics diagnostics;
    ASSERT_TRUE(reconnectPathSuffix(projected, projection, .6,
        [](nav_msgs::msg::Path &) {return true;}, output, &diagnostics));
    EXPECT_TRUE(diagnostics.used_forward);
    EXPECT_NEAR(progress, expected_progress, 1e-9);
    ASSERT_GE(output.poses.size(), 2u);
    EXPECT_GT(output.poses[1].pose.position.x, progress + .1);
  }
}

TEST(PathReconnection, DisabledAndNegligibleOffsetRetainProjectedSuffix)
{
  const auto cached = straightPath();
  for (const auto & parameters : std::vector<std::array<double, 2>>{
      {.02, 0}, {.0005, .6}})
  {
    nav_msgs::msg::Path projected;
    geometry_msgs::msg::PoseStamped projection;
    double progress = 0;
    ASSERT_TRUE(selectStablePathSuffix(
        cached, .4, parameters[0], .75, .3, progress, projected, &projection));
    nav_msgs::msg::Path output;
    PathReconnectDiagnostics diagnostics;
    std::size_t calls = 0;
    ASSERT_TRUE(reconnectPathSuffix(projected, projection, parameters[1],
        [&](nav_msgs::msg::Path &) {++calls; return true;}, output, &diagnostics));
    EXPECT_FALSE(diagnostics.used_forward);
    EXPECT_EQ(output, projected);
    EXPECT_EQ(calls, 1u);
    EXPECT_EQ(diagnostics.attempts, calls);
  }
}

TEST(PublishedPathStart, OnlyLiveRobotRequestsMayBeAdvancedAfterSearch)
{
  auto request = straightPath().poses.front();
  auto robot = request;
  robot.pose.position.x = .04;
  robot.pose.position.y = -.02;
  EXPECT_TRUE(planningStartMatchesRobot(request, robot));
  robot.pose.position.x = .4;
  EXPECT_FALSE(planningStartMatchesRobot(request, robot));
  robot = request;
  robot.pose.position.z = .4;
  EXPECT_FALSE(planningStartMatchesRobot(request, robot));
  robot = request;
  robot.pose.orientation.z = 1;
  robot.pose.orientation.w = 0;
  EXPECT_FALSE(planningStartMatchesRobot(request, robot));
  robot = request;
  robot.header.frame_id = "odom";
  EXPECT_FALSE(planningStartMatchesRobot(request, robot));
}

TEST(PublishedPathStart, MotionDuringSearchPrunesRearPrefixBeforeShimSampling)
{
  const auto planned = straightPath();
  auto latest = planned.poses.front();
  latest.pose.position.x = .43;
  nav_msgs::msg::Path output;
  ASSERT_TRUE(reanchorPublishedPath(planned, latest, .75, .6,
      [](nav_msgs::msg::Path &) {return true;}, output));
  ASSERT_GT(output.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.position.x, latest.pose.position.x);
  EXPECT_EQ(output.poses.back(), planned.poses.back());
  // Reproduce Shim's 15cm sample selection: the old .15m sample is behind
  // the .43m robot, while the returned path's sample must be ahead.
  auto sample = std::find_if(output.poses.begin()+1, output.poses.end(), [&](const auto & p) {
      return std::hypot(p.pose.position.x-output.poses.front().pose.position.x,
        p.pose.position.y-output.poses.front().pose.position.y) >= .15;
    });
  ASSERT_NE(sample, output.poses.end());
  EXPECT_GT(sample->pose.position.x, latest.pose.position.x);
  EXPECT_GT(sample->pose.position.x, .58);
  EXPECT_FALSE(containsPoint(output, 0, 0));
}

TEST(PublishedPathStart, LateralMotionUsesValidatedForwardConnectorAndKeepsTail)
{
  OccupyMap map(mapConfig());
  map.setStaticObstacles(corridorWalls());
  SweptFootprint checker(map, paddedFootprint());
  const auto lock = map.lockForReading();
  const auto planned = straightPath();
  auto latest = planned.poses.front();
  latest.pose.position.x = .4;
  latest.pose.position.y = .02;
  nav_msgs::msg::Path output;
  std::size_t checks = 0;
  ASSERT_TRUE(reanchorPublishedPath(planned, latest, .75, .6,
      [&](nav_msgs::msg::Path & candidate) {
        ++checks;
        return checker.validatePath(pointsFromPath(candidate), 0, 0);
      }, output));
  EXPECT_GT(checks, 0u);
  ASSERT_GE(output.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.position.x, .4);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.position.y, .02);
  EXPECT_GT(output.poses[1].pose.position.x, .5);
  EXPECT_TRUE(checker.validatePath(pointsFromPath(output), 0, 0));
  EXPECT_EQ(output.poses.back(), planned.poses.back());
}

TEST(PublishedPathStart, RefreshCannotPublishAConnectorWithCurrentBodyCollision)
{
  OccupyMap map(mapConfig());
  map.setStaticObstacles(corridorWalls());
  SweptFootprint checker(map, paddedFootprint());
  const auto lock = map.lockForReading();
  const auto planned = straightPath();
  ASSERT_TRUE(checker.validatePath(pointsFromPath(planned), 0, 0));
  auto latest = planned.poses.front();
  latest.pose.position.x = .4;
  latest.pose.position.y = .10;
  nav_msgs::msg::Path output;
  EXPECT_FALSE(reanchorPublishedPath(planned, latest, .75, .6,
      [&](nav_msgs::msg::Path & candidate) {
        return checker.validatePath(pointsFromPath(candidate), 0, 0);
      }, output));
  EXPECT_TRUE(output.poses.empty());
}

TEST(PublishedPathStart, RequiredRearGoalRetainsItsDirectionAndFinalHeading)
{
  std::vector<OccupyMap::Point> points;
  for (int i = 0; i <= 40; ++i) {points.push_back({-.05*i, 0, 0});}
  const auto planned = pathFromPoints(points, kPi/2);
  auto latest = planned.poses.front();
  latest.pose.position.x = .20;
  nav_msgs::msg::Path output;
  ASSERT_TRUE(reanchorPublishedPath(planned, latest, .75, .6,
      [](nav_msgs::msg::Path &) {return true;}, output));
  ASSERT_GE(output.poses.size(), 2u);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.position.x, .20);
  EXPECT_LT(output.poses[1].pose.position.x, latest.pose.position.x);
  EXPECT_EQ(output.poses.back(), planned.poses.back());
}

TEST(PublishedPathStart, ForwardConnectionRetainsNecessaryCorner)
{
  const auto planned = pathFromPoints({{0,0,0},{2,0,0},{2,2,0}}, kPi/2);
  auto latest = planned.poses.front();
  latest.pose.position.x = 1.5;
  latest.pose.position.y = .02;
  nav_msgs::msg::Path output;
  ASSERT_TRUE(reanchorPublishedPath(planned, latest, .75, .6,
      [](nav_msgs::msg::Path &) {return true;}, output));
  EXPECT_DOUBLE_EQ(output.poses.front().pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(output.poses.front().pose.position.y, .02);
  EXPECT_TRUE(containsPoint(output, 2, 0));
  EXPECT_EQ(output.poses.back(), planned.poses.back());
}
}  // namespace astar_3d_planner
