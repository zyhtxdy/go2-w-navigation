#include "astar_3d_planner/swept_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace astar_3d_planner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
bool finite(const OccupyMap::Point & p)
{return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);}
}

SweptFootprint::SweptFootprint(const OccupyMap & map, const Config & config)
: map_(map), config_(config)
{
  if (!std::isfinite(config.min_x) || !std::isfinite(config.max_x) ||
    !std::isfinite(config.min_y) || !std::isfinite(config.max_y) ||
    !std::isfinite(config.robot_height) || config.min_x >= config.max_x ||
    config.min_y >= config.max_y || config.robot_height <= 0 ||
    config.robot_height > 10 || config.max_samples == 0 ||
    config.max_x - config.min_x > 10 || config.max_y - config.min_y > 10 ||
    std::max({std::abs(config.min_x), std::abs(config.max_x),
      std::abs(config.min_y), std::abs(config.max_y)}) > 10 ||
    !std::isfinite(config.near_start_static_refinement_radius) ||
    config.near_start_static_refinement_radius < 0 ||
    config.near_start_static_refinement_radius > 1.0 ||
    !std::isfinite(config.near_start_static_obstacle_margin) ||
    config.near_start_static_obstacle_margin < 0.01 ||
    config.near_start_static_obstacle_margin > 0.10 ||
    !std::isfinite(config.static_obstacle_extent_margin) ||
    config.static_obstacle_extent_margin < 0.01 ||
    config.static_obstacle_extent_margin > 0.10 ||
    !std::isfinite(config.near_start_measured_refinement_radius) ||
    config.near_start_measured_refinement_radius < 0 ||
    config.near_start_measured_refinement_radius > 1.0 ||
    !std::isfinite(config.near_start_measured_obstacle_margin) ||
    config.near_start_measured_obstacle_margin < 0.02 ||
    config.near_start_measured_obstacle_margin > 0.10 ||
    (config.near_start_center &&
      (!std::isfinite((*config.near_start_center)[0]) ||
      !std::isfinite((*config.near_start_center)[1]))))
  {throw std::invalid_argument("Invalid rectangular footprint envelope");}
  corner_radius_ = std::hypot(std::max(std::abs(config.min_x), std::abs(config.max_x)),
      std::max(std::abs(config.min_y), std::abs(config.max_y)));
}

bool SweptFootprint::poseCollisionFree(const OccupyMap::Point & p, double yaw,
  VoxelKey * collision, double sweep_padding, double vertical_padding) const
{
  const double resolution = map_.resolution();
  if (!finite(p) || !std::isfinite(yaw) || !std::isfinite(sweep_padding) ||
    sweep_padding < 0 || sweep_padding > 10 || !std::isfinite(vertical_padding) ||
    vertical_padding < 0 || vertical_padding > 10 ||
    std::max({std::abs(p[0]), std::abs(p[1]), std::abs(p[2])}) / resolution >
    std::numeric_limits<int>::max() / 4.0)
  {return false;}
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double ac = std::abs(c), as = std::abs(s);
  const double ox = (config_.min_x + config_.max_x) / 2;
  const double oy = (config_.min_y + config_.max_y) / 2;
  const double cx = p[0] + c * ox - s * oy;
  const double cy = p[1] + s * ox + c * oy;
  const double hx = (config_.max_x - config_.min_x) / 2 + sweep_padding;
  const double hy = (config_.max_y - config_.min_y) / 2 + sweep_padding;
  const double ex = ac * hx + as * hy, ey = as * hx + ac * hy;
  const auto lower = map_.posToKey(cx - ex, cy - ey, p[2]);
  const auto upper = map_.posToKey(cx + ex, cy + ey, p[2]);
  // Quantize the actual top, not ceil(height) added to the ground-cell index.
  // The latter includes a disjoint extra upper layer on fractional ground.
  const int body_bottom = map_.posToKey(0, 0, p[2] - vertical_padding).z + 1;
  const int body_top = map_.posToKey(0, 0, p[2] + config_.robot_height + vertical_padding).z;
  const double voxel_half = resolution / 2;
  // Refine only inside the request's original start neighbourhood. Do not
  // reset that neighbourhood for each edge explored by heading-aware search.
  const bool refine_near_static = config_.near_start_center &&
    config_.near_start_static_refinement_radius > 0 &&
    std::hypot(p[0] - (*config_.near_start_center)[0],
      p[1] - (*config_.near_start_center)[1]) + sweep_padding <=
    config_.near_start_static_refinement_radius;
  const bool refine_measured = config_.near_start_center &&
    config_.near_start_measured_refinement_radius > 0 &&
    std::hypot(p[0] - (*config_.near_start_center)[0],
      p[1] - (*config_.near_start_center)[1]) + sweep_padding <=
    config_.near_start_measured_refinement_radius;
  for (int x = lower.x - 1; x <= upper.x; ++x) {
    for (int y = lower.y - 1; y <= upper.y; ++y) {
      const auto center = map_.keyToPos({x, y, 0});
      const double dx = center[0] - cx, dy = center[1] - cy;
      // Separating-axis test for an oriented rectangle against the complete
      // obstacle voxel square. A voxel centre alone can miss a touching corner.
      if (std::abs(dx) > ex + voxel_half + 1e-10 ||
        std::abs(dy) > ey + voxel_half + 1e-10 ||
        std::abs(c * dx + s * dy) > hx + voxel_half * (ac + as) + 1e-10 ||
        std::abs(-s * dx + c * dy) > hy + voxel_half * (ac + as) + 1e-10)
      {continue;}
      // Match the terrain planner's convention: inspect body cells above the
      // observed ground cell; support is maintained separately from obstacles.
      for (int z = body_bottom; z <= body_top; ++z) {
        const VoxelKey key{x, y, z};
        if (map_.rawObstacleOccupiedUnlocked(key)) {
          OccupyMap::ObstacleBounds bounds;
          double obstacle_margin = config_.static_obstacle_extent_margin;
          bool refine = false;
          if (refine_measured && map_.measuredObstacleBoundsUnlocked(key, bounds)) {
            refine = true;
            obstacle_margin = std::max(config_.near_start_measured_obstacle_margin,
              config_.static_obstacle_extent_margin);
          } else if ((config_.refine_static_obstacle_extents || refine_near_static) &&
            map_.staticObstacleBoundsUnlocked(key, bounds))
          {
            refine = true;
            if (!config_.refine_static_obstacle_extents) {
              obstacle_margin = config_.near_start_static_obstacle_margin;
            }
          }
          if (refine) {
            // Preserve every saved point in the cell and an explicit XY
            // uncertainty margin. Keep the existing vertical voxel policy.
            // This only removes empty quantization slivers, never enlarges
            // the original occupied voxel or ignores a real point in it.
            const double m = obstacle_margin;
            const double x0 = std::max(center[0] - voxel_half, bounds.minimum[0] - m);
            const double x1 = std::min(center[0] + voxel_half, bounds.maximum[0] + m);
            const double y0 = std::max(center[1] - voxel_half, bounds.minimum[1] - m);
            const double y1 = std::min(center[1] + voxel_half, bounds.maximum[1] + m);
            const double bx = (x0 + x1) / 2 - cx, by = (y0 + y1) / 2 - cy;
            const double bhx = (x1 - x0) / 2, bhy = (y1 - y0) / 2;
            if (std::abs(bx) > ex + bhx + 1e-10 ||
              std::abs(by) > ey + bhy + 1e-10 ||
              std::abs(c * bx + s * by) > hx + ac * bhx + as * bhy + 1e-10 ||
              std::abs(-s * bx + c * by) > hy + as * bhx + ac * bhy + 1e-10)
            {continue;}
          }
          if (collision) {*collision = key;}
          return false;
        }
      }
    }
  }
  return true;
}

bool SweptFootprint::validatePath(const std::vector<OccupyMap::Point> & path,
  double start_yaw, double goal_yaw, Failure * failure) const
{
  Failure result;
  const auto finish = [&](bool valid) {
      if (failure) {*failure = result;} return valid;
    };
  const auto valid_position = [&](const OccupyMap::Point & p) {
      return finite(p) &&
             std::max({std::abs(p[0]), std::abs(p[1]), std::abs(p[2])}) /
             map_.resolution() <= std::numeric_limits<int>::max() / 4.0;
    };
  if (path.empty() || !std::isfinite(start_yaw) || !std::isfinite(goal_yaw) ||
    !std::all_of(path.begin(), path.end(), valid_position))
  {result.reason = "footprint_invalid_pose"; return finish(false);}
  const auto motion = [&](const OccupyMap::Point & a, const OccupyMap::Point & b,
      double yaw_a, double delta, std::size_t index) {
      // Translation includes Z so a rising centre cannot skip a body-height cell.
      const double translation = std::sqrt(
        std::pow(b[0] - a[0], 2) + std::pow(b[1] - a[1], 2) + std::pow(b[2] - a[2], 2));
      const double movement = translation + corner_radius_ * std::abs(delta);
      const double count = std::max(1.0, std::ceil(movement / (map_.resolution() * .25)));
      if (!std::isfinite(count) || count + result.samples + 1 > config_.max_samples) {
        result.reason = "footprint_sampling_budget_exceeded"; result.index = index;
        return false;
      }
      const auto steps = static_cast<std::size_t>(count);
      // At any time the nearest sample is at most this far away, including
      // angular corner travel. Expanding both body axes encloses that disk.
      const double margin = movement / (2 * steps);
      const double vertical_margin = std::abs(b[2] - a[2]) / (2 * steps);
      for (std::size_t i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        result.position = {a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]),
          a[2] + t * (b[2] - a[2])};
        result.yaw = yaw_a + t * delta; result.index = index; ++result.samples;
        if (!poseCollisionFree(result.position, result.yaw, &result.collision, margin, vertical_margin)) {
          result.reason = "footprint_collision"; result.has_collision = true; return false;
        }
      }
      return true;
    };
  const auto turn = [&](const OccupyMap::Point & p, double from, double to, std::size_t i) {
      const double delta = std::remainder(to - from, 2 * kPi);
      if (!motion(p, p, from, delta, i)) {return false;}
      // The controller's half-turn sign can differ at exactly +/-pi. Require
      // both arcs there, including for an asymmetric base-relative envelope.
      return std::abs(std::abs(delta) - kPi) > 1e-8 || motion(p, p, from, -delta, i);
    };
  double yaw = start_yaw;
  if (!config_.check_in_place_turn_sweeps &&
    !motion(path.front(), path.front(), start_yaw, 0, 0)) {return finish(false);}
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1]; const auto & b = path[i];
    const double dx = b[0] - a[0], dy = b[1] - a[1];
    const double heading = std::hypot(dx, dy) > 1e-9 ? std::atan2(dy, dx) : yaw;
    if ((config_.check_in_place_turn_sweeps && !turn(a, yaw, heading, i - 1)) ||
      !motion(a, b, heading, 0, i)) {
      return finish(false);
    }
    yaw = heading;
  }
  if (config_.check_in_place_turn_sweeps) {
    if (!turn(path.back(), yaw, goal_yaw, path.size() - 1)) {return finish(false);}
  } else if (!motion(path.back(), path.back(), goal_yaw, 0, path.size() - 1)) {
    return finish(false);
  }
  return finish(true);
}
}  // namespace astar_3d_planner
