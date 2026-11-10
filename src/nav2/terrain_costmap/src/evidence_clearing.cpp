#include "terrain_costmap/evidence_clearing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace terrain_costmap
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

bool validGeometry(const EvidenceClearingParameters & p)
{
  return p.bin_count > 0 && std::isfinite(p.range_min) && p.range_min >= 0.0 &&
         std::isfinite(p.range_max) && p.range_max > p.range_min &&
         p.range_max <= std::numeric_limits<float>::max() &&
         std::isfinite(p.angle_min) && std::isfinite(p.angle_increment) &&
         p.angle_increment > 0.0 && p.angle_increment <= kTwoPi &&
         (p.bin_count - 1) * p.angle_increment <= kTwoPi + 1e-6 &&
         std::isfinite(p.endpoint_margin) && p.endpoint_margin >= 0.0;
}

bool pointBin(
  const Point & point, const EvidenceClearingParameters & p,
  std::size_t & index, double & range)
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
    return false;
  }
  range = std::hypot(point.x, point.y);
  if (!std::isfinite(range)) {
    return false;
  }
  double angle = std::fmod(
    std::atan2(point.y, point.x) - std::fmod(p.angle_min, kTwoPi), kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  // Scan geometry commonly arrives as float fields. Snap only their rounding
  // error at bin edges, including the equivalent -pi/+pi seam.
  const double tolerance = std::min(
    p.angle_increment * 1e-3, 8.0 * std::numeric_limits<float>::epsilon() * kTwoPi);
  if (kTwoPi - angle <= tolerance) {
    angle = 0.0;
  }
  double position = angle / p.angle_increment;
  const double edge = std::round(position);
  if (std::abs(position - edge) * p.angle_increment <= tolerance) {
    position = edge;
  }
  if (position >= static_cast<double>(p.bin_count)) {
    return false;
  }
  index = static_cast<std::size_t>(std::floor(position));
  return true;
}

}  // namespace

std::vector<float> makeEvidenceClearingRanges(
  const std::vector<Point> & current_returns,
  const std::vector<Point> & classified_obstacles,
  const EvidenceClearingParameters & p)
{
  std::vector<float> ranges(p.bin_count, std::numeric_limits<float>::quiet_NaN());
  if (!validGeometry(p)) {
    return ranges;
  }
  std::vector<double> endpoints(p.bin_count, std::numeric_limits<double>::infinity());
  for (const Point & point : current_returns) {
    std::size_t index;
    double range;
    if (pointBin(point, p, index, range)) {
      endpoints[index] = std::min(endpoints[index], range);
    }
  }
  for (const Point & point : classified_obstacles) {
    std::size_t index;
    double range;
    if (pointBin(point, p, index, range) && std::isfinite(endpoints[index])) {
      endpoints[index] = std::min(endpoints[index], range);
    }
  }
  for (std::size_t index = 0; index < p.bin_count; ++index) {
    if (!std::isfinite(endpoints[index])) {
      continue;
    }
    const double endpoint = std::min(endpoints[index], p.range_max) - p.endpoint_margin;
    if (endpoint < p.range_min) {
      continue;
    }
    float range = static_cast<float>(endpoint);
    if (static_cast<double>(range) > endpoint) {
      range = std::nextafter(range, 0.0f);
    }
    if (static_cast<double>(range) >= p.range_min) {
      ranges[index] = range;
    }
  }
  return ranges;
}

}  // namespace terrain_costmap
