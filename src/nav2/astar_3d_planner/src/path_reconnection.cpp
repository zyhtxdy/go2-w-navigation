#include "astar_3d_planner/astar_3d_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace astar_3d_planner
{
namespace
{
double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{return std::hypot(a.x-b.x, a.y-b.y);}

void directions(nav_msgs::msg::Path & path)
{
  for (std::size_t i=1; i<path.poses.size(); ++i) {
    auto & pose=path.poses[i-1];
    const auto & next=path.poses[i].pose.position;
    const double dx=next.x-pose.pose.position.x, dy=next.y-pose.pose.position.y;
    if (std::hypot(dx,dy)<=1e-9) {continue;}
    const double yaw=std::atan2(dy,dx);
    pose.pose.orientation.x=pose.pose.orientation.y=0;
    pose.pose.orientation.z=std::sin(yaw/2); pose.pose.orientation.w=std::cos(yaw/2);
  }
}
}

bool planningStartMatchesRobot(
  const geometry_msgs::msg::PoseStamped & requested,
  const geometry_msgs::msg::PoseStamped & robot,
  double xy_tolerance, double yaw_tolerance)
{
  if (requested.header.frame_id != robot.header.frame_id ||
    !std::isfinite(xy_tolerance) || xy_tolerance < 0 ||
    !std::isfinite(yaw_tolerance) || yaw_tolerance < 0 ||
    !std::isfinite(requested.pose.position.x) || !std::isfinite(requested.pose.position.y) ||
    !std::isfinite(requested.pose.position.z) || !std::isfinite(robot.pose.position.x) ||
    !std::isfinite(robot.pose.position.y) || !std::isfinite(robot.pose.position.z) ||
    distance(requested.pose.position, robot.pose.position) > xy_tolerance ||
    std::abs(requested.pose.position.z-robot.pose.position.z) > xy_tolerance)
  {return false;}
  const auto yaw = [](const auto & q, double & result) {
      const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
      if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {return false;}
      result = std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
      return std::isfinite(result);
    };
  double requested_yaw, robot_yaw;
  return yaw(requested.pose.orientation, requested_yaw) && yaw(robot.pose.orientation, robot_yaw) &&
    std::abs(std::atan2(std::sin(requested_yaw-robot_yaw),
      std::cos(requested_yaw-robot_yaw))) <= yaw_tolerance;
}

bool reanchorPublishedPath(
  const nav_msgs::msg::Path & planned_path,
  const geometry_msgs::msg::PoseStamped & robot,
  double maximum_distance, double maximum_forward_distance,
  const std::function<bool(nav_msgs::msg::Path &)> & validate,
  nav_msgs::msg::Path & output)
{
  output = nav_msgs::msg::Path{};
  if (planned_path.poses.empty() || planned_path.header.frame_id != robot.header.frame_id ||
    !std::isfinite(robot.pose.position.x) || !std::isfinite(robot.pose.position.y) ||
    !std::isfinite(maximum_distance) || maximum_distance <= 0)
  {return false;}
  nav_msgs::msg::Path suffix;
  geometry_msgs::msg::PoseStamped projection;
  double progress = 0;
  if (!selectStablePathSuffix(planned_path, robot.pose.position.x, robot.pose.position.y,
      maximum_distance, 0.0, progress, suffix, &projection))
  {return false;}
  return reconnectPathSuffix(suffix, projection, maximum_forward_distance, validate, output);
}

bool reconnectPathSuffix(
  const nav_msgs::msg::Path & suffix,
  const geometry_msgs::msg::PoseStamped & projection,
  double maximum_forward_distance,
  const std::function<bool(nav_msgs::msg::Path &)> & validate,
  nav_msgs::msg::Path & output,
  PathReconnectDiagnostics * diagnostics)
{
  if (diagnostics) {*diagnostics=PathReconnectDiagnostics{};}
  output=nav_msgs::msg::Path{};
  if (!validate || suffix.poses.empty() || !std::isfinite(maximum_forward_distance) ||
    maximum_forward_distance<0) {return false;}
  const auto finite=[](const auto & p) {
      return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    };
  if (!finite(projection.pose.position) ||
    !std::all_of(suffix.poses.begin(),suffix.poses.end(),
      [&](const auto & p) {return finite(p.pose.position);})) {return false;}
  const auto attempt=[&](nav_msgs::msg::Path candidate, double forward) {
      if (diagnostics) {++diagnostics->attempts;}
      if (!validate(candidate)) {return false;}
      output=std::move(candidate);
      if (diagnostics) {
        diagnostics->used_forward=forward>0;
        diagnostics->forward_distance=forward;
      }
      return true;
    };

  // The original suffix has actual pose, then its projection only when they
  // differ. A supported/on-path start or an endpoint needs no new connector.
  const bool can_reconnect=maximum_forward_distance>0 && suffix.poses.size()>2 &&
    distance(suffix.poses.front().pose.position,projection.pose.position)>1e-3 &&
    distance(suffix.poses[1].pose.position,projection.pose.position)<=1e-6;
  if (can_reconnect) {
    std::vector<double> accumulated(suffix.poses.size(),0);
    double available=0, direction_x=0, direction_y=0;
    bool have_direction=false;
    // Stay on the local straight leg. Compare to its original tangent, not
    // just adjacent segments, so small turns cannot accumulate past a bend.
    for (std::size_t i=2;i<suffix.poses.size();++i) {
      const auto & a=suffix.poses[i-1].pose.position;
      const auto & b=suffix.poses[i].pose.position;
      const double length=distance(a,b);
      if (length<=1e-9) {accumulated[i]=available;continue;}
      const double x=(b.x-a.x)/length, y=(b.y-a.y)/length;
      if (!have_direction) {direction_x=x;direction_y=y;have_direction=true;}
      else if (x*direction_x+y*direction_y<std::cos(.05)) {break;}
      available+=length; accumulated[i]=available;
      if (available>=maximum_forward_distance) {break;}
    }
    available=std::min(available,maximum_forward_distance);
    const auto begin=std::chrono::steady_clock::now();
    // Four local candidates, furthest first; then the unchanged projection
    // suffix as fallback. Full validation, not heading alone, selects a path.
    for (int trial=4;trial>0 && available>1e-6;--trial) {
      if (std::chrono::steady_clock::now()-begin>std::chrono::milliseconds(50)) {break;}
      const double forward=available*trial/4;
      for (std::size_t i=2;i<suffix.poses.size();++i) {
        if (accumulated[i]<=0 || accumulated[i]+1e-10<forward) {continue;}
        const double previous=accumulated[i-1];
        const double length=accumulated[i]-previous;
        if (length<=1e-9) {continue;}
        const double ratio=std::clamp((forward-previous)/length,0.0,1.0);
        const auto & a=suffix.poses[i-1].pose.position;
        const auto & b=suffix.poses[i].pose.position;
        auto join=suffix.poses[i];
        join.pose.position.x=a.x+ratio*(b.x-a.x);
        join.pose.position.y=a.y+ratio*(b.y-a.y);
        join.pose.position.z=a.z+ratio*(b.z-a.z);
        nav_msgs::msg::Path candidate;candidate.header=suffix.header;
        candidate.poses.push_back(suffix.poses.front());
        candidate.poses.push_back(join);
        for (std::size_t j=i;j<suffix.poses.size();++j) {
          const auto & next=suffix.poses[j];
          if (distance(candidate.poses.back().pose.position,next.pose.position)>1e-9 ||
            std::abs(candidate.poses.back().pose.position.z-next.pose.position.z)>1e-9)
          {candidate.poses.push_back(next);}
        }
        candidate.poses.back().pose.orientation=suffix.poses.back().pose.orientation;
        directions(candidate);
        if (attempt(std::move(candidate),forward)) {return true;}
        break;
      }
    }
  }
  return attempt(suffix,0.0);
}
}  // namespace astar_3d_planner
