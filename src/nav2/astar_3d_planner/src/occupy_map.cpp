#include "astar_3d_planner/occupy_map.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <stdexcept>

namespace astar_3d_planner
{

namespace
{

std::size_t hashCombine(std::size_t seed, std::size_t value) noexcept
{
  return seed ^ (value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
}

}  // namespace

std::size_t VoxelKeyHash::operator()(const VoxelKey & key) const noexcept
{
  std::size_t seed = std::hash<int>{}(key.x);
  seed = hashCombine(seed, std::hash<int>{}(key.y));
  return hashCombine(seed, std::hash<int>{}(key.z));
}

std::size_t ColumnKeyHash::operator()(const ColumnKey & key) const noexcept
{
  return hashCombine(std::hash<int>{}(key.x), std::hash<int>{}(key.y));
}

OccupyMap::OccupyMap(const Config & config)
: config_(config),
  inv_resolution_(1.0 / config.resolution),
  inflate_cells_(std::max(0, static_cast<int>(std::ceil(
      config.inflate_radius / config.resolution)))),
  support_radius_cells_(std::max(0, static_cast<int>(std::ceil(
      config.support_radius / config.resolution)))),
  obstacle_update_radius_cells_(std::max(0, static_cast<int>(std::floor(
      config.obstacle_update_radius / config.resolution))))
{
  if (config_.footprint_inscribed_radius &&
    (!std::isfinite(*config_.footprint_inscribed_radius) || *config_.footprint_inscribed_radius < 0))
  {
    throw std::invalid_argument("invalid footprint inscribed radius");
  }
  if (!std::isfinite(config_.obstacle_extent_prefilter_margin) ||
    config_.obstacle_extent_prefilter_margin < 0.0)
  {
    throw std::invalid_argument("invalid obstacle extent prefilter margin");
  }
  hard_prefilter_radius_ = inflate_cells_ * config_.resolution;
  if (config_.footprint_inscribed_radius) {
    hard_prefilter_radius_ = std::min(
      hard_prefilter_radius_, *config_.footprint_inscribed_radius);
  }
  const auto hard_offset = [&](int distance_sq) {
      if (!config_.footprint_inscribed_radius) {
        return distance_sq <= inflate_cells_ * inflate_cells_;
      }
      // The furthest point in a query voxel from an obstacle voxel's AABB is
      // exactly their centre separation (same grid and voxel size). Therefore
      // reject only when EVERY subcell body origin's inscribed disk collides.
      // Never ceil the disk to a whole-cell radius: exact XY can be off-centre.
      return *config_.footprint_inscribed_radius > 0.0 &&
             distance_sq * config_.resolution * config_.resolution <=
             hard_prefilter_radius_ * hard_prefilter_radius_;
    };
  if (!std::isfinite(config_.support_inference_radius) ||
    config_.support_inference_radius <= 0.0 || config_.support_inference_radius > 0.5 ||
    !std::isfinite(config_.support_inference_max_height_span) ||
    config_.support_inference_max_height_span <= 0.0 ||
    !std::isfinite(config_.support_inference_max_residual) ||
    config_.support_inference_max_residual <= 0.0 ||
    !std::isfinite(config_.support_inference_max_slope_deg) ||
    config_.support_inference_max_slope_deg < 0.0 ||
    config_.support_inference_max_slope_deg >= 90.0 ||
    config_.support_inference_max_columns <= 0)
  {
    throw std::invalid_argument("invalid bounded support inference parameters");
  }
  if (!std::isfinite(config_.synthetic_obstacle_clearance_radius) ||
    config_.synthetic_obstacle_clearance_radius < 0.0)
  {
    throw std::invalid_argument("invalid synthetic obstacle clearance radius");
  }
  if (config_.compensate_height_drift &&
    (config_.height_drift_min_overlap < 3 ||
    !std::isfinite(config_.height_drift_max_update) || config_.height_drift_max_update <= 0.0 ||
    config_.height_drift_max_update > 0.02 ||
    !std::isfinite(config_.height_drift_inlier_tolerance) || config_.height_drift_inlier_tolerance <= 0.0 ||
    !std::isfinite(config_.height_drift_min_consensus) || config_.height_drift_min_consensus <= 0.5 ||
    config_.height_drift_min_consensus > 1.0 ||
    !std::isfinite(config_.height_drift_min_xy_span) || config_.height_drift_min_xy_span <= 0.0 ||
    !std::isfinite(config_.height_drift_min_change) || config_.height_drift_min_change < 0.0 ||
    config_.height_drift_min_change >= config_.height_drift_max_update))
  {
    throw std::invalid_argument("invalid planning height-drift compensation parameters");
  }
  for (int dx = -inflate_cells_; dx <= inflate_cells_; ++dx) {
    for (int dy = -inflate_cells_; dy <= inflate_cells_; ++dy) {
      if (hard_offset(dx * dx + dy * dy)) {
        inflate_offsets_.push_back({dx, dy});
      }
    }
  }

  const int cost_radius = inflate_cells_ + std::max(0, config.cost_inflate_cells);
  for (int dx = -cost_radius; dx <= cost_radius; ++dx) {
    for (int dy = -cost_radius; dy <= cost_radius; ++dy) {
      const int distance_sq = dx * dx + dy * dy;
      if (!hard_offset(distance_sq) && distance_sq <= cost_radius * cost_radius)
      {
        // Cells released from the old hard mask still carry clearance cost.
        // A zero-radius/off-origin envelope also needs a finite source cost.
        cost_offsets_.push_back({dx, dy, std::max(1, distance_sq)});
      }
    }
  }
}

void OccupyMap::reconcileHeightDatumUnlocked(const std::vector<Point> & points)
{
  if (!config_.compensate_height_drift || support_heights_.empty()) {return;}
  std::unordered_map<ColumnKey, std::vector<double>, ColumnKeyHash> incoming_columns;
  for (const auto & point : points) {
    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
      continue;
    }
    const auto key = posToSupportKey(point[0], point[1], point[2]);
    incoming_columns[{key.x, key.y}].push_back(point[2]);
  }
  struct Match {ColumnKey column; double delta;};
  std::vector<Match> matches;
  std::size_t overlaps = 0;
  for (const auto & incoming : incoming_columns) {
    const auto column = support_columns_.find(incoming.first);
    if (column == support_columns_.end()) {continue;}
    ++overlaps;
    double best_error = std::numeric_limits<double>::infinity();
    double best_delta = 0.0;
    bool found = false;
    for (double measured : incoming.second) {
      int close_layers = 0;
      double delta = 0.0;
      for (int z : column->second) {
        double previous;
        supportHeightUnlocked({incoming.first.x, incoming.first.y, z}, previous);
        if (std::abs(measured - previous) <= config_.height_drift_max_update + 1e-9) {
          ++close_layers;
          delta = measured - previous;
        }
      }
      // A step, another storey, or an ambiguous pair of close layers must not
      // drive a common translation. Each XY column supplies at most one vote.
      if (close_layers == 1 && std::abs(delta) < best_error) {
        best_error = std::abs(delta); best_delta = delta; found = true;
      }
    }
    if (found) {matches.push_back({incoming.first, best_delta});}
  }
  if (matches.size() < static_cast<std::size_t>(config_.height_drift_min_overlap)) {return;}
  std::vector<double> deltas;
  for (const auto & match : matches) {deltas.push_back(match.delta);}
  const auto middle = deltas.begin() + deltas.size() / 2;
  std::nth_element(deltas.begin(), middle, deltas.end());
  const double shift = *middle;
  if (std::abs(shift) < config_.height_drift_min_change ||
    std::abs(shift) > config_.height_drift_max_update) {return;}
  std::size_t inliers = 0;
  int min_x = std::numeric_limits<int>::max(), min_y = min_x;
  int max_x = std::numeric_limits<int>::lowest(), max_y = max_x;
  for (const auto & match : matches) {
    if (std::abs(match.delta - shift) > config_.height_drift_inlier_tolerance) {continue;}
    ++inliers;
    min_x = std::min(min_x, match.column.x); max_x = std::max(max_x, match.column.x);
    min_y = std::min(min_y, match.column.y); max_y = std::max(max_y, match.column.y);
  }
  if (inliers < static_cast<std::size_t>(config_.height_drift_min_overlap) ||
    static_cast<double>(inliers) < config_.height_drift_min_consensus * overlaps ||
    (max_x - min_x) * config_.resolution < config_.height_drift_min_xy_span ||
    (max_y - min_y) * config_.resolution < config_.height_drift_min_xy_span)
  {
    return;
  }
  // Keep canonical keys/heights and all obstacle inflation references stable.
  // All old geometry follows the same measured coordinate-datum translation;
  // real slopes, step heights and relative storey separations remain unchanged.
  height_datum_offset_ += shift;
}

std::vector<OccupyMap::Point> OccupyMap::supportSnapshotUnlocked() const
{
  std::vector<Point> result;
  result.reserve(support_heights_.size());
  for (const auto & entry : support_heights_) {
    const auto position = keyToPos(entry.first);
    result.push_back({position[0], position[1], entry.second + height_datum_offset_});
  }
  return result;
}

void OccupyMap::updateSupport(const std::vector<Point> & points, std::int64_t stamp_ns)
{
  if (points.empty()) {
    return;
  }

  std::unique_lock<std::shared_mutex> lock(mutex_);
  if (stamp_ns > 0 && stamp_ns <= last_support_stamp_ns_) {return;}
  // Derived terrain is valid for exactly one measured-support generation.
  {std::lock_guard<std::mutex> cache_lock(ground_cache_mutex_); ground_cache_.clear();}
  reconcileHeightDatumUnlocked(points);
  if (stamp_ns > 0) {last_support_stamp_ns_ = stamp_ns;}

  struct HeightSamples
  {
    double sum{0.0};
    double x_sum{0.0}, y_sum{0.0};
    std::size_t count{0};
  };
  std::unordered_map<VoxelKey, HeightSamples, VoxelKeyHash> incoming;
  incoming.reserve(points.size());
  for (const auto & point : points) {
    if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
      const auto key = posToSupportKey(point[0], point[1], point[2]);
      auto & samples = incoming[key];
      samples.sum += point[2] - height_datum_offset_;
      samples.x_sum += point[0]; samples.y_sum += point[1];
      ++samples.count;
    }
  }
  std::vector<std::pair<VoxelKey, double>> observations;
  observations.reserve(incoming.size());
  for (const auto & entry : incoming) {
    observations.emplace_back(
      entry.first, entry.second.sum / static_cast<double>(entry.second.count));
  }
  std::sort(
    observations.begin(), observations.end(),
    [](const auto & left, const auto & right) {
      if (left.first.x != right.first.x) {
        return left.first.x < right.first.x;
      }
      if (left.first.y != right.first.y) {
        return left.first.y < right.first.y;
      }
      return left.first.z < right.first.z;
    });

  support_occupancy_.reserve(support_occupancy_.size() + incoming.size());
  support_heights_.reserve(support_heights_.size() + incoming.size());
  std::unordered_map<VoxelKey, double, VoxelKeyHash> observed_heights;
  std::unordered_map<VoxelKey, std::array<double, 2>, VoxelKeyHash> observed_positions;

  for (const auto & observation : observations) {
    const auto & support_key = observation.first;
    const double height = observation.second;
    const auto & samples = incoming.at(support_key);
    const std::array<double, 2> position{
      samples.x_sum / samples.count, samples.y_sum / samples.count};
    auto & column =
      support_columns_[ColumnKey{support_key.x, support_key.y}];
    // Match against continuous measured heights, not the Z index of an older
    // observation. Otherwise the metre-valued merge threshold changes at bin
    // boundaries and a corrected surface can become stranded on its old key.
    const auto matching_layer = std::min_element(
      column.begin(), column.end(),
      [&](int left, int right) {
        const double left_error = std::abs(
          support_heights_.at({support_key.x, support_key.y, left}) - height);
        const double right_error = std::abs(
          support_heights_.at({support_key.x, support_key.y, right}) - height);
        return left_error != right_error ? left_error < right_error : left < right;
      });
    if (matching_layer != column.end() &&
      std::abs(support_heights_.at(
          {support_key.x, support_key.y, *matching_layer}) - height) <=
      config_.support_merge_height + 1e-9)
    {
      const VoxelKey layer{support_key.x, support_key.y, *matching_layer};
      const double previous_height = support_heights_.at(layer);
      const auto selected = observed_heights.find(layer);
      if (selected == observed_heights.end() ||
        std::abs(height - previous_height) <
        std::abs(selected->second - previous_height))
      {
        observed_heights[layer] = height;
        observed_positions[layer] = position;
      }
      continue;
    }
    const auto inserted = support_occupancy_.insert(support_key);
    if (!inserted.second) {
      continue;
    }
    column.push_back(support_key.z);
    std::sort(column.begin(), column.end());
    support_heights_[support_key] = height;
    support_positions_[support_key] = position;
    observed_heights[support_key] = height;
    observed_positions[support_key] = position;
    updateBounds(support_key);
  }
  // Re-key only touched columns after matching the whole frame. The graph's
  // Z index must follow the current measured height even after repeated small
  // corrections cross several voxels. Obstacle keys/ownership are independent.
  std::unordered_set<ColumnKey, ColumnKeyHash> changed_columns;
  for (const auto & entry : observed_heights) {
    changed_columns.insert({entry.first.x, entry.first.y});
  }
  for (const auto & xy : changed_columns) {
    auto & column = support_columns_.at(xy);
    const bool needs_rekey = std::any_of(column.begin(), column.end(), [&](int z) {
        const auto observation = observed_heights.find({xy.x, xy.y, z});
        return observation != observed_heights.end() &&
          posToKey(0, 0, observation->second + height_datum_offset_).z != z;
      });
    if (!needs_rekey) {
      for (const int z : column) {
        const VoxelKey key{xy.x, xy.y, z};
        const auto observation = observed_heights.find(key);
        if (observation != observed_heights.end()) {
          support_heights_[key] = observation->second;
          support_positions_[key] = observed_positions.at(key);
        }
      }
      continue;
    }
    struct Layer {double height; bool observed; std::array<double, 2> xy;};
    std::map<int, Layer> updated;
    for (const int z : column) {
      const VoxelKey previous_key{xy.x, xy.y, z};
      const auto observation = observed_heights.find(previous_key);
      const bool observed = observation != observed_heights.end();
      const double height = observed ? observation->second : support_heights_.at(previous_key);
      const auto position = observed ? observed_positions.at(previous_key) :
        support_positions_.at(previous_key);
      const int current_z = posToKey(0, 0, height + height_datum_offset_).z;
      const auto existing = updated.find(current_z);
      // The voxel representation has one layer per Z bin. If corrected layers
      // converge into the same bin, retain an actual latest observation rather
      // than inventing an average height. Prefer it over an unobserved old layer.
      if (existing == updated.end() ||
        (observed && !existing->second.observed) ||
        (observed == existing->second.observed && height < existing->second.height))
      {
        updated[current_z] = {height, observed, position};
      }
      support_occupancy_.erase(previous_key);
      support_heights_.erase(previous_key);
      support_positions_.erase(previous_key);
    }
    column.clear();
    for (const auto & entry : updated) {
      const VoxelKey key{xy.x, xy.y, entry.first};
      column.push_back(entry.first);
      support_occupancy_.insert(key);
      support_heights_[key] = entry.second.height;
      support_positions_[key] = entry.second.xy;
      updateBounds(key);
    }
  }
}

bool OccupyMap::updateObstacles(
  const std::vector<Point> & points, double observer_x, double observer_y,
  std::int64_t stamp_ns, const std::vector<std::uint8_t> & sources,
  bool replace_dynamic, std::optional<std::uint64_t> expected_epoch,
  const std::vector<std::int64_t> & observed_stamps_ns)
{
  if (!sources.empty() && sources.size() != points.size()) {
    throw std::invalid_argument("obstacle provenance size differs from points");
  }
  if (!observed_stamps_ns.empty() && observed_stamps_ns.size() != points.size()) {
    throw std::invalid_argument("obstacle observation timestamps size differs from points");
  }
  std::unique_lock<std::shared_mutex> lock(mutex_);
  if (expected_epoch && *expected_epoch != dynamic_obstacle_epoch_) {return false;}
  if (stamp_ns > 0 && (stamp_ns <= last_obstacle_stamp_ns_ ||
    (config_.compensate_height_drift && stamp_ns < last_support_stamp_ns_))) {return false;}
  if (replace_dynamic) {clearDynamicObstaclesUnlocked();}
  if (stamp_ns > 0) {last_obstacle_stamp_ns_ = stamp_ns;}
  std::unordered_set<VoxelKey, VoxelKeyHash> incoming;
  incoming.reserve(points.size());
  std::unordered_map<VoxelKey, ObstacleEvidence, VoxelKeyHash> observations;
  std::unordered_map<VoxelKey, ObstacleBounds, VoxelKeyHash> observation_bounds;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto & point = points[i];
    if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2])) {
      const auto key = posToKey(point[0], point[1], point[2]);
      const std::uint8_t source = !sources.empty() &&
        (sources[i] == kMeasured || sources[i] == kSynthetic) ? sources[i] :
        static_cast<std::uint8_t>(kUnknown);
      if (source == kSynthetic && config_.synthetic_obstacle_clearance_radius > 0.0 &&
        std::hypot(point[0] - observer_x, point[1] - observer_y) <=
        config_.synthetic_obstacle_clearance_radius) {continue;}
      incoming.insert(key);
      const auto observed_stamp = observed_stamps_ns.empty() ? 0 :
        std::max<std::int64_t>(0, observed_stamps_ns[i]);
      const Point canonical{point[0], point[1], point[2] - height_datum_offset_};
      auto bounds = observation_bounds.emplace(key, ObstacleBounds{canonical, canonical, source});
      bounds.first->second.sources |= source;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        bounds.first->second.minimum[axis] = std::min(bounds.first->second.minimum[axis], canonical[axis]);
        bounds.first->second.maximum[axis] = std::max(bounds.first->second.maximum[axis], canonical[axis]);
      }
      const auto inserted = observations.emplace(
        key, ObstacleEvidence{key,
          Point{point[0], point[1], point[2] - height_datum_offset_},
          point[2], stamp_ns, source, observed_stamp,
          Point{point[0], point[1], point[2] - height_datum_offset_}});
      if (!inserted.second) {
        auto & evidence = inserted.first->second;
        evidence.sources |= source;
        const Point canonical{point[0], point[1], point[2] - height_datum_offset_};
        if (canonical < evidence.clearance_point) {evidence.clearance_point = canonical;}
        if (observed_stamp > evidence.observed_stamp_ns ||
          (observed_stamp == evidence.observed_stamp_ns && canonical < evidence.point))
        {
          evidence.point = canonical; evidence.observed_z = point[2];
          evidence.observed_stamp_ns = observed_stamp;
        }
      }
    }
  }

  ++obstacle_update_sequence_;
  obstacle_occupancy_.reserve(obstacle_occupancy_.size() + incoming.size());
  obstacle_last_seen_.reserve(obstacle_last_seen_.size() + incoming.size());

  const std::uint64_t clear_after = static_cast<std::uint64_t>(
    std::max(1, config_.obstacle_clear_after_updates));
  for (const auto & obstacle_key : incoming) {
    dynamic_obstacle_history_[obstacle_key].push_back(
      {obstacle_update_sequence_, observation_bounds.at(obstacle_key)});
  }
  for (auto history = dynamic_obstacle_history_.begin();
    history != dynamic_obstacle_history_.end();)
  {
    auto & observations_for_cell = history->second;
    while (!observations_for_cell.empty() &&
      obstacle_update_sequence_ - observations_for_cell.front().sequence >= clear_after)
    {
      observations_for_cell.pop_front();
    }
    if (observations_for_cell.empty()) {
      // The existing observable-radius rule can keep distant occupancy alive.
      // With no recent bounds its geometry falls back to a complete voxel.
      dynamic_obstacle_bounds_.erase(history->first);
      history = dynamic_obstacle_history_.erase(history);
      continue;
    }
    auto bounds = observations_for_cell.front().bounds;
    for (const auto & observation : observations_for_cell) {
      bounds.sources |= observation.bounds.sources;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        bounds.minimum[axis] = std::min(bounds.minimum[axis], observation.bounds.minimum[axis]);
        bounds.maximum[axis] = std::max(bounds.maximum[axis], observation.bounds.maximum[axis]);
      }
    }
    dynamic_obstacle_bounds_[history->first] = bounds;
    const auto active_evidence = obstacle_evidence_.find(history->first);
    if (obstacle_last_seen_.count(history->first) &&
      active_evidence != obstacle_evidence_.end())
    {
      active_evidence->second.sources = bounds.sources;
    }
    ++history;
  }

  for (const auto & obstacle_key : incoming) {
    obstacle_last_seen_[obstacle_key] = obstacle_update_sequence_;
    auto evidence = observations.at(obstacle_key);
    // Keep unknown/synthetic flags for all still-valid observation frames,
    // but let them expire instead of latching them for the cell's lifetime.
    evidence.sources = dynamic_obstacle_bounds_.at(obstacle_key).sources;
    obstacle_evidence_[obstacle_key] = evidence;
    if (obstacle_occupancy_.find(obstacle_key) == obstacle_occupancy_.end()) {
      addObstacleUnlocked(obstacle_key);
    }
  }

  const VoxelKey observer = posToKey(observer_x, observer_y, 0.0);
  std::vector<VoxelKey> expired;
  expired.reserve(obstacle_last_seen_.size() / 16 + 1);
  for (const auto & entry : obstacle_last_seen_) {
    const VoxelKey & key = entry.first;
    const auto evidence = obstacle_evidence_.find(key);
    if (config_.synthetic_obstacle_clearance_radius > 0.0 &&
      evidence != obstacle_evidence_.end() && evidence->second.sources == kSynthetic &&
      std::hypot(evidence->second.clearance_point[0] - observer_x,
      evidence->second.clearance_point[1] - observer_y) <=
      config_.synthetic_obstacle_clearance_radius)
    {
      expired.push_back(key); continue;
    }
    const long long dx = static_cast<long long>(key.x) - observer.x;
    const long long dy = static_cast<long long>(key.y) - observer.y;
    if (dx * dx + dy * dy >
      static_cast<long long>(obstacle_update_radius_cells_) *
      obstacle_update_radius_cells_)
    {
      continue;
    }
    if (obstacle_update_sequence_ - entry.second >= clear_after) {
      expired.push_back(key);
    }
  }
  for (const auto & key : expired) {
    const auto fixed = static_obstacles_.find(key);
    if (fixed == static_obstacles_.end()) {removeObstacleUnlocked(key);}
    else {obstacle_evidence_[key] = fixed->second;}
    obstacle_last_seen_.erase(key);
    dynamic_obstacle_bounds_.erase(key);
    dynamic_obstacle_history_.erase(key);
  }
  return true;
}

void OccupyMap::clearDynamicObstaclesUnlocked()
{
  for (const auto & entry : obstacle_last_seen_) {
    const auto fixed = static_obstacles_.find(entry.first);
    if (fixed == static_obstacles_.end()) {removeObstacleUnlocked(entry.first);}
    else {obstacle_evidence_[entry.first] = fixed->second;}
  }
  obstacle_last_seen_.clear();
  dynamic_obstacle_bounds_.clear();
  dynamic_obstacle_history_.clear();
}

std::unique_ptr<OccupyMap> OccupyMap::planningSnapshotAfterExpiry(
  std::int64_t now_ns, std::int64_t maximum_age_ns)
{
  const auto read = lockForReadingAfterExpiry(now_ns, maximum_age_ns);
  return snapshotUnlocked();
}

std::unique_ptr<OccupyMap> OccupyMap::referenceSnapshotUnlocked(
  double observer_x, double observer_y, double radius, std::size_t * ignored_count) const
{
  if (!std::isfinite(observer_x) || !std::isfinite(observer_y) ||
    !std::isfinite(radius) || radius < 0.0)
  {throw std::invalid_argument("Finite reference observer and nonnegative radius required");}
  if (ignored_count) {*ignored_count = 0;}
  auto result = snapshotUnlocked();
  if (radius == 0.0) {return result;}

  std::vector<VoxelKey> ignored;
  const double half = config_.resolution / 2;
  for (const auto & entry : result->obstacle_last_seen_) {
    const auto & key = entry.first;
    // Diagnostic evidence can be replaced by a live return even while the
    // fixed layer owns the same voxel. Check ownership independently.
    if (result->static_obstacles_.count(key)) {continue;}
    const auto evidence = result->obstacle_evidence_.find(key);
    const auto bounds = result->dynamic_obstacle_bounds_.find(key);
    if (evidence == result->obstacle_evidence_.end() ||
      bounds == result->dynamic_obstacle_bounds_.end() ||
      evidence->second.sources != kMeasured || bounds->second.sources != kMeasured)
    {continue;}
    const auto center = result->keyToPos(key);
    const double dx = std::abs(center[0] - observer_x) + half;
    const double dy = std::abs(center[1] - observer_y) + half;
    // Keep cells crossing the boundary, even when their representative point
    // is nearby: the discarded geometry must lie wholly in the local region.
    if (std::hypot(dx, dy) <= radius) {ignored.push_back(key);}
  }
  for (const auto & key : ignored) {
    result->removeObstacleUnlocked(key);
    result->obstacle_last_seen_.erase(key);
  }
  if (ignored_count) {*ignored_count = ignored.size();}
  return result;
}

std::unique_ptr<OccupyMap> OccupyMap::snapshotUnlocked() const
{
  auto result = std::make_unique<OccupyMap>(config_);
  result->obstacle_update_sequence_ = obstacle_update_sequence_;
  result->dynamic_obstacle_epoch_ = dynamic_obstacle_epoch_;
  result->height_datum_offset_ = height_datum_offset_;
  result->last_support_stamp_ns_ = last_support_stamp_ns_;
  result->last_obstacle_stamp_ns_ = last_obstacle_stamp_ns_;
  result->support_occupancy_ = support_occupancy_;
  result->support_heights_ = support_heights_;
  result->support_positions_ = support_positions_;
  result->support_columns_ = support_columns_;
  result->obstacle_occupancy_ = obstacle_occupancy_;
  result->inflated_ref_count_ = inflated_ref_count_;
  result->obstacle_cost_ = obstacle_cost_;
  result->obstacle_last_seen_ = obstacle_last_seen_;
  result->obstacle_evidence_ = obstacle_evidence_;
  result->static_obstacles_ = static_obstacles_;
  result->static_obstacle_bounds_ = static_obstacle_bounds_;
  result->dynamic_obstacle_bounds_ = dynamic_obstacle_bounds_;
  result->dynamic_obstacle_history_ = dynamic_obstacle_history_;
  result->static_obstacles_loaded_ = static_obstacles_loaded_;
  result->min_key_ = min_key_;
  result->max_key_ = max_key_;
  {
    std::lock_guard<std::mutex> cache_lock(ground_cache_mutex_);
    result->ground_cache_ = ground_cache_;
  }
  return result;
}

bool OccupyMap::planningStateMatchesUnlocked(const OccupyMap & snapshot) const
{
  return dynamic_obstacle_epoch_ == snapshot.dynamic_obstacle_epoch_ &&
         last_support_stamp_ns_ == snapshot.last_support_stamp_ns_ &&
         height_datum_offset_ == snapshot.height_datum_offset_ &&
         static_obstacles_loaded_ == snapshot.static_obstacles_loaded_;
}

OccupyMap::ReadLock OccupyMap::lockForReadingAfterExpiry(
  std::int64_t now_ns,std::int64_t maximum_age_ns,std::size_t * expired_count)
{
  if(now_ns<=0 || maximum_age_ns<=0) {
    throw std::invalid_argument("Positive planning time and observation age required");
  }
  std::size_t count=0;
  for(;;) {
    std::uint64_t generation,epoch;
    {
      std::unique_lock<std::shared_mutex> write(mutex_);
      std::vector<VoxelKey> expired;
      for(const auto & entry:obstacle_last_seen_) {
        const auto evidence=obstacle_evidence_.find(entry.first);
        if(evidence==obstacle_evidence_.end()) {continue;}
        const auto & value=evidence->second;
        if(value.sources==kMeasured && value.observed_stamp_ns>0 &&
          now_ns>=value.observed_stamp_ns &&
          now_ns-value.observed_stamp_ns>maximum_age_ns) {expired.push_back(entry.first);}
      }
      for(const auto & key:expired) {
        const auto fixed=static_obstacles_.find(key);
        if(fixed==static_obstacles_.end()) {removeObstacleUnlocked(key);}
        else {obstacle_evidence_[key]=fixed->second;}
        obstacle_last_seen_.erase(key);
        dynamic_obstacle_bounds_.erase(key);
        dynamic_obstacle_history_.erase(key);
      }
      count+=expired.size();generation=obstacle_update_sequence_;epoch=dynamic_obstacle_epoch_;
    }
    ReadLock read(mutex_);
    // A newer DDS snapshot can arrive while downgrading the write lock.
    // Recheck it rather than exposing newly reinserted expired points.
    if(generation==obstacle_update_sequence_ && epoch==dynamic_obstacle_epoch_) {
      if(expired_count) {*expired_count=count;}
      return read;
    }
  }
}

void OccupyMap::clearDynamicObstacles(bool restart_stream)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  clearDynamicObstaclesUnlocked();
  if (restart_stream) {
    last_obstacle_stamp_ns_ = 0;
    ++dynamic_obstacle_epoch_;
  }
}

std::uint64_t OccupyMap::dynamicObstacleEpoch() const
{
  const auto read = lockForReading();
  return dynamic_obstacle_epoch_;
}

void OccupyMap::setStaticObstacles(
  const std::vector<Point> & points, std::int64_t stamp_ns,
  const std::vector<std::uint8_t> & sources)
{
  if (!sources.empty() && sources.size() != points.size()) {
    throw std::invalid_argument("static obstacle provenance size differs from points");
  }
  std::unique_lock<std::shared_mutex> lock(mutex_);
  if (config_.compensate_height_drift) {
    throw std::invalid_argument("Static maps cannot use online height-drift compensation");
  }
  for (const auto & entry : static_obstacles_) {
    if (obstacle_last_seen_.find(entry.first) == obstacle_last_seen_.end()) {
      removeObstacleUnlocked(entry.first);
    }
  }
  static_obstacles_.clear();
  static_obstacle_bounds_.clear();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto & point = points[i];
    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
      continue;
    }
    const auto key = posToKey(point[0], point[1], point[2]);
    const std::uint8_t source = !sources.empty() &&
      (sources[i] == kMeasured || sources[i] == kSynthetic) ? sources[i] :
      static_cast<std::uint8_t>(kUnknown);
    auto bounds = static_obstacle_bounds_.emplace(key, ObstacleBounds{point, point, source});
    bounds.first->second.sources |= source;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      bounds.first->second.minimum[axis] = std::min(bounds.first->second.minimum[axis], point[axis]);
      bounds.first->second.maximum[axis] = std::max(bounds.first->second.maximum[axis], point[axis]);
    }
    const ObstacleEvidence evidence{key, point, point[2], stamp_ns, kStaticMap};
    static_obstacles_[key] = evidence;
    if (!obstacle_occupancy_.count(key)) {addObstacleUnlocked(key);}
    if (!obstacle_last_seen_.count(key)) {obstacle_evidence_[key] = evidence;}
  }
  static_obstacles_loaded_ = true;  // An explicitly empty obstacle map is valid.
}

void OccupyMap::reset()
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  support_occupancy_.clear();
  support_heights_.clear();
  support_positions_.clear();
  obstacle_occupancy_.clear();
  inflated_ref_count_.clear();
  obstacle_cost_.clear();
  obstacle_last_seen_.clear();
  obstacle_evidence_.clear();
  static_obstacles_.clear();
  static_obstacle_bounds_.clear();
  dynamic_obstacle_bounds_.clear();
  dynamic_obstacle_history_.clear();
  static_obstacles_loaded_ = false;
  support_columns_.clear();
  {std::lock_guard<std::mutex> cache_lock(ground_cache_mutex_); ground_cache_.clear();}
  obstacle_update_sequence_ = 0;
  ++dynamic_obstacle_epoch_;
  height_datum_offset_ = 0.0;
  last_support_stamp_ns_ = 0;
  last_obstacle_stamp_ns_ = 0;
  min_key_ = {
    std::numeric_limits<int>::max(), std::numeric_limits<int>::max(),
    std::numeric_limits<int>::max()};
  max_key_ = {
    std::numeric_limits<int>::lowest(), std::numeric_limits<int>::lowest(),
    std::numeric_limits<int>::lowest()};
}

OccupyMap::ReadLock OccupyMap::lockForReading() const
{
  return ReadLock(mutex_);
}

bool OccupyMap::readyUnlocked() const
{
  return !support_occupancy_.empty() &&
         (!config_.require_static_obstacles || static_obstacles_loaded_);
}

std::size_t OccupyMap::rawVoxelCountUnlocked() const
{
  return support_occupancy_.size() + obstacle_occupancy_.size();
}

VoxelKey OccupyMap::posToKey(double x, double y, double z) const
{
  return VoxelKey{
    static_cast<int>(std::floor(x * inv_resolution_)),
    static_cast<int>(std::floor(y * inv_resolution_)),
    static_cast<int>(std::floor((z - height_datum_offset_) * inv_resolution_))};
}

VoxelKey OccupyMap::posToSupportKey(double x, double y, double z) const
{
  auto key = posToKey(x, y, z);
  const double coordinates[] = {x, y};
  int * axes[] = {&key.x, &key.y};
  for (int axis = 0; axis < 2; ++axis) {
    const double scaled = coordinates[axis] * inv_resolution_;
    const double nearest = std::round(scaled);
    // FLOAT32 transport of world raster coordinates can put a boundary just
    // below its intended bin. Cap correction at 0.001 voxel so this cannot
    // become arbitrary rounding at large map coordinates. Obstacles retain
    // the original floor-based quantization and inflation footprint.
    const double tolerance = std::min(1e-3, std::max(1e-6,
        std::abs(scaled) * std::numeric_limits<float>::epsilon() * 2.0));
    if (std::abs(scaled - nearest) <= tolerance) {
      *axes[axis] = static_cast<int>(nearest);
    }
  }
  return key;
}

OccupyMap::Point OccupyMap::keyToPos(const VoxelKey & key) const
{
  return Point{
    (static_cast<double>(key.x) + 0.5) * config_.resolution,
    (static_cast<double>(key.y) + 0.5) * config_.resolution,
    (static_cast<double>(key.z) + 0.5) * config_.resolution + height_datum_offset_};
}

bool OccupyMap::occupiedUnlocked(const VoxelKey & key) const
{
  if (inflated_ref_count_.find(key) == inflated_ref_count_.end()) {return false;}
  if (!config_.footprint_inscribed_radius ||
    (!config_.refine_static_obstacle_extents && !config_.refine_measured_obstacle_extents))
  {
    return true;
  }
  const double query_min_x = key.x * config_.resolution;
  const double query_min_y = key.y * config_.resolution;
  const double radius_sq = hard_prefilter_radius_ * hard_prefilter_radius_;
  for (const auto & offset : inflate_offsets_) {
    const VoxelKey source{key.x - offset[0], key.y - offset[1], key.z};
    if (!obstacle_occupancy_.count(source)) {continue;}
    ObstacleBounds bounds;
    const bool refined =
      (config_.refine_measured_obstacle_extents &&
      measuredObstacleBoundsUnlocked(source, bounds)) ||
      (config_.refine_static_obstacle_extents &&
      staticObstacleBoundsUnlocked(source, bounds));
    if (!refined) {return true;}
    // This is a necessary collision only when EVERY possible continuous XY
    // origin in this query cell has its inscribed disk intersect these bounds.
    // The convex point-to-AABB squared distance reaches its cell maximum at a
    // corner. Margins use the minimum enabled refinement policy so this box is
    // contained by every box the final scoped rectangle checker may use.
    const double source_min_x = source.x * config_.resolution;
    const double source_min_y = source.y * config_.resolution;
    const double margin = config_.obstacle_extent_prefilter_margin;
    const double min_x = std::max(source_min_x, bounds.minimum[0] - margin);
    const double min_y = std::max(source_min_y, bounds.minimum[1] - margin);
    const double max_x = std::min(source_min_x + config_.resolution, bounds.maximum[0] + margin);
    const double max_y = std::min(source_min_y + config_.resolution, bounds.maximum[1] + margin);
    double maximum_distance_sq = 0.0;
    for (int ix = 0; ix < 2; ++ix) {
      for (int iy = 0; iy < 2; ++iy) {
        const double x = query_min_x + ix * config_.resolution;
        const double y = query_min_y + iy * config_.resolution;
        const double dx = std::max({min_x - x, 0.0, x - max_x});
        const double dy = std::max({min_y - y, 0.0, y - max_y});
        maximum_distance_sq = std::max(maximum_distance_sq, dx * dx + dy * dy);
      }
    }
    if (maximum_distance_sq <= radius_sq) {return true;}
  }
  return false;
}

bool OccupyMap::staticObstacleBoundsUnlocked(
  const VoxelKey & cell, ObstacleBounds & bounds) const
{
  if (obstacle_last_seen_.count(cell)) {return false;}
  const auto found = static_obstacle_bounds_.find(cell);
  if (found == static_obstacle_bounds_.end() || found->second.sources != kMeasured) {
    return false;
  }
  bounds = found->second;
  return true;
}

bool OccupyMap::measuredObstacleBoundsUnlocked(
  const VoxelKey & cell, ObstacleBounds & bounds) const
{
  const auto found = dynamic_obstacle_bounds_.find(cell);
  if (!obstacle_last_seen_.count(cell) || found == dynamic_obstacle_bounds_.end() ||
    found->second.sources != kMeasured) {return false;}
  bounds = found->second;
  bounds.minimum[2] += height_datum_offset_;
  bounds.maximum[2] += height_datum_offset_;
  const auto fixed = static_obstacle_bounds_.find(cell);
  if (fixed != static_obstacle_bounds_.end()) {
    if (fixed->second.sources != kMeasured) {return false;}
    bounds.sources |= kStaticMap;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      bounds.minimum[axis] = std::min(bounds.minimum[axis], fixed->second.minimum[axis]);
      bounds.maximum[axis] = std::max(bounds.maximum[axis], fixed->second.maximum[axis]);
    }
  }
  return true;
}

bool OccupyMap::obstacleEvidenceUnlocked(
  const VoxelKey & cell, ObstacleEvidence & evidence) const
{
  const auto source = obstacle_evidence_.find(cell);
  if (source != obstacle_evidence_.end()) {
    evidence = source->second; evidence.point[2] += height_datum_offset_; return true;
  }
  bool found = false;
  int best_distance = std::numeric_limits<int>::max();
  // Use exactly the offsets used to build occupancy, rather than guessing
  // from a newer cloud or a continuous-radius approximation.
  for (const auto & offset : inflate_offsets_) {
    const VoxelKey key{cell.x - offset[0], cell.y - offset[1], cell.z};
    const auto entry = obstacle_evidence_.find(key);
    if (entry == obstacle_evidence_.end()) {continue;}
    const int distance = offset[0] * offset[0] + offset[1] * offset[1];
    if (!found || distance < best_distance) {
      evidence = entry->second; best_distance = distance; found = true;
    }
  }
  if (found) {evidence.point[2] += height_datum_offset_;}
  return found;
}

std::vector<OccupyMap::Point> OccupyMap::inflationSnapshotUnlocked(
  const VoxelKey & source) const
{
  std::vector<Point> result;
  if (obstacle_occupancy_.find(source) == obstacle_occupancy_.end()) {return result;}
  result.reserve(inflate_offsets_.size());
  for (const auto & offset : inflate_offsets_) {
    result.push_back(keyToPos(VoxelKey{source.x + offset[0], source.y + offset[1], source.z}));
  }
  return result;
}

bool OccupyMap::supportHeightUnlocked(const VoxelKey & ground, double & height) const
{
  const auto observation = support_heights_.find(ground);
  if (observation == support_heights_.end()) {
    if (support_columns_.count({ground.x, ground.y})) {return false;}
    for (const auto & layer : inferColumnUnlocked(
        {ground.x, ground.y}, keyToPos(ground)[2]).layers)
    {
      if (layer.z == ground.z) {height = layer.height + height_datum_offset_; return true;}
    }
    return false;
  }
  height = observation->second + height_datum_offset_;
  return true;
}

bool OccupyMap::measuredSupportXYUnlocked(
  const VoxelKey & ground, std::array<double, 2> & xy) const
{
  const auto found = support_positions_.find(ground);
  if (found == support_positions_.end()) {return false;}
  xy = found->second;
  return true;
}

bool OccupyMap::findSupportByHeightUnlocked(
  double x, double y, double observed_height, double tolerance, VoxelKey & ground) const
{
  const auto key = posToSupportKey(x, y, observed_height);
  const auto layers = groundLayersUnlocked({key.x, key.y}, observed_height);
  double closest_error = std::numeric_limits<double>::infinity();
  bool found = false;
  for (const int layer_z : layers) {
    const VoxelKey layer{key.x, key.y, layer_z};
    double height;
    if (!supportHeightUnlocked(layer, height)) {
      continue;
    }
    const double error = std::abs(height - observed_height);
    if (error <= tolerance + 1e-9 && error < closest_error) {
      ground = layer;
      closest_error = error;
      found = true;
    }
  }
  return found;
}

bool OccupyMap::findGroundUnlocked(
  const VoxelKey & candidate, int support_depth_cells, VoxelKey & ground) const
{
  // Preserve the measured-neighbour contract before trying interpolation.
  // Enabling interpolation must not remove an existing measured connection.
  if (findMeasuredGroundUnlocked(candidate,support_depth_cells,ground)) {return true;}
  if (config_.support_inference_enabled) {
    const auto layers = groundLayersUnlocked(
      {candidate.x, candidate.y}, keyToPos(candidate)[2]);
    int best_gap = support_depth_cells + 1;
    for (int z : layers) {
      const int gap = candidate.z - z;
      if (gap > 0 && gap <= support_depth_cells && gap < best_gap) {
        ground = {candidate.x, candidate.y, z}; best_gap = gap;
      }
    }
    return best_gap <= support_depth_cells;
  }
  return false;
}

bool OccupyMap::findMeasuredGroundUnlocked(
  const VoxelKey & candidate, int support_depth_cells, VoxelKey & ground) const
{
  for (int dz = 1; dz <= support_depth_cells; ++dz) {
    const int ground_z = candidate.z - dz;
    for (int dx = -support_radius_cells_; dx <= support_radius_cells_; ++dx) {
      for (int dy = -support_radius_cells_; dy <= support_radius_cells_; ++dy) {
        if (dx * dx + dy * dy > support_radius_cells_ * support_radius_cells_) {
          continue;
        }
        const VoxelKey key{candidate.x + dx, candidate.y + dy, ground_z};
        if (support_occupancy_.find(key) != support_occupancy_.end()) {
          ground = key;
          return true;
        }
      }
    }
  }
  return false;
}

bool OccupyMap::findSurfaceGroundUnlocked(
  const VoxelKey & candidate, int preferred_ground_gap_cells,
  int ground_gap_tolerance_cells, VoxelKey & ground) const
{
  const auto layers = groundLayersUnlocked({candidate.x, candidate.y},
    keyToPos({candidate.x, candidate.y, candidate.z - preferred_ground_gap_cells})[2]);

  int best_error = ground_gap_tolerance_cells + 1;
  bool found = false;
  for (const int ground_z : layers) {
    const int gap = candidate.z - ground_z;
    const int error = std::abs(gap - preferred_ground_gap_cells);
    if (gap <= 0 || error > ground_gap_tolerance_cells ||
      error >= best_error)
    {
      continue;
    }
    best_error = error;
    ground = VoxelKey{candidate.x, candidate.y, ground_z};
    found = true;
  }
  return found;
}

bool OccupyMap::traversableOnSurfaceUnlocked(
  const VoxelKey & candidate, const VoxelKey & ground,
  int robot_height_cells, int preferred_ground_gap_cells,
  int ground_gap_tolerance_cells, VoxelKey * collision, double robot_height_metres) const
{
  if (candidate.x != ground.x || candidate.y != ground.y)
  {
    return false;
  }
  const int ground_gap = candidate.z - ground.z;
  if (ground_gap <= 0 ||
    std::abs(ground_gap - preferred_ground_gap_cells) >
    ground_gap_tolerance_cells)
  {
    return false;
  }
  double ground_height;
  if (!supportHeightUnlocked(ground, ground_height)) {
    return false;
  }
  const int observed_ground_z = posToKey(0.0, 0.0, ground_height).z;
  const int body_top_z = robot_height_metres > 0.0 ?
    posToKey(0, 0, ground_height + robot_height_metres).z :
    std::max(ground.z, observed_ground_z) + robot_height_cells;
  for (int z = std::min(ground.z, observed_ground_z) + 1; z <= body_top_z; ++z) {
    if (occupiedUnlocked(VoxelKey{candidate.x, candidate.y, z})) {
      if (collision) {*collision = VoxelKey{candidate.x, candidate.y, z};}
      return false;
    }
  }
  return true;
}

bool OccupyMap::traversableUnlocked(
  const VoxelKey & candidate, int support_depth_cells, int robot_height_cells,
  int preferred_ground_gap_cells, int ground_gap_tolerance_cells,
  VoxelKey * ground, double robot_height_metres) const
{
  return traversabilityUnlocked(
    candidate, support_depth_cells, robot_height_cells,
    preferred_ground_gap_cells, ground_gap_tolerance_cells, ground, robot_height_metres) ==
         Traversability::kTraversable;
}

OccupyMap::Traversability OccupyMap::traversabilityUnlocked(
  const VoxelKey & candidate, int support_depth_cells, int robot_height_cells,
  int preferred_ground_gap_cells, int ground_gap_tolerance_cells,
  VoxelKey * ground, double robot_height_metres) const
{
  VoxelKey detected_ground;
  if (!findGroundUnlocked(candidate, support_depth_cells, detected_ground)) {
    return Traversability::kNoGroundSupport;
  }

  const int ground_gap = candidate.z - detected_ground.z;
  if (std::abs(ground_gap - preferred_ground_gap_cells) > ground_gap_tolerance_cells) {
    return Traversability::kGroundGapMismatch;
  }

  double ground_height;
  if (!supportHeightUnlocked(detected_ground, ground_height)) {
    return Traversability::kNoGroundSupport;
  }
  const int observed_ground_z = posToKey(0.0, 0.0, ground_height).z;
  const int body_top_z = robot_height_metres > 0.0 ?
    posToKey(0, 0, ground_height + robot_height_metres).z :
    std::max(detected_ground.z, observed_ground_z) + robot_height_cells;
  // Support and obstacle voxels are now separate.  Check clearance from the
  // first voxel above the fitted ground so low curbs and unsafe-cell markers
  // cannot hide below the nominal planning-height node.
  for (int z = std::min(detected_ground.z, observed_ground_z) + 1; z <= body_top_z; ++z) {
    if (occupiedUnlocked(VoxelKey{candidate.x, candidate.y, z})) {
      return Traversability::kOccupied;
    }
  }

  if (ground != nullptr) {
    *ground = detected_ground;
  }
  return Traversability::kTraversable;
}

double OccupyMap::obstacleCostUnlocked(
  const VoxelKey & candidate, int robot_height_cells) const
{
  double maximum_cost = 0.0;
  for (int dz = 0; dz <= robot_height_cells; ++dz) {
    const auto iterator =
      obstacle_cost_.find(VoxelKey{candidate.x, candidate.y, candidate.z + dz});
    if (iterator != obstacle_cost_.end()) {
      maximum_cost = std::max(maximum_cost, static_cast<double>(iterator->second));
    }
  }
  return maximum_cost;
}

std::vector<VoxelKey> OccupyMap::surfaceCandidatesUnlocked(
  double x, double y, double desired_ground_z, double horizontal_radius,
  double vertical_radius, int support_depth_cells, int robot_height_cells,
  int preferred_ground_gap_cells, int ground_gap_tolerance_cells,
  SurfaceDiagnostics * diagnostics, double robot_height_metres) const
{
  if (diagnostics) {*diagnostics = SurfaceDiagnostics{};}
  const auto record = [&](const VoxelKey & ground, const char * reason,
      const VoxelKey * collision = nullptr) {
      if (!diagnostics) {return;}
      if (diagnostics->candidates.size() >= 256) {++diagnostics->omitted_candidates; return;}
      SurfaceDiagnostics::Candidate entry;
      entry.ground = ground; entry.reason = reason;
      if (collision) {entry.has_collision = true; entry.collision = *collision;}
      diagnostics->candidates.push_back(entry);
    };
  (void)support_depth_cells;
  const VoxelKey center = posToSupportKey(x, y, desired_ground_z);
  const int xy_cells =
    std::max(0, static_cast<int>(std::ceil(horizontal_radius * inv_resolution_)));
  const int z_cells =
    std::max(0, static_cast<int>(std::ceil(vertical_radius * inv_resolution_)));

  struct ScoredCandidate
  {
    double vertical_score;
    double horizontal_score;
    VoxelKey key;
  };
  std::vector<ScoredCandidate> scored;
  std::unordered_set<VoxelKey, VoxelKeyHash> unique;

  for (int dx = -xy_cells; dx <= xy_cells; ++dx) {
    for (int dy = -xy_cells; dy <= xy_cells; ++dy) {
      // The query can be anywhere inside its cell. A circle centred on its
      // integer key can omit a metrically close support column (and include
      // one outside the requested radius). Use world distances instead.
      // Always retain the query's own column: a zero-radius lookup means
      // "support in this column", even when its centre is offset from XY.
      const auto position = keyToPos({center.x + dx, center.y + dy, 0});
      const double distance = std::hypot(position[0] - x, position[1] - y);
      if ((dx != 0 || dy != 0) && distance > horizontal_radius + 1e-9) {
        continue;
      }
      const auto layers = groundLayersUnlocked({center.x + dx, center.y + dy}, desired_ground_z);
      if (layers.empty()) {
        continue;
      }
      if (diagnostics) {++diagnostics->columns;}
      for (const int ground_z : layers) {
        if (diagnostics) {++diagnostics->layers;}
        if (std::abs(ground_z - center.z) > z_cells) {
          if (diagnostics) {++diagnostics->outside_height;}
          record({center.x + dx, center.y + dy, ground_z}, "outside_height_window");
          continue;
        }
        const VoxelKey candidate{
          center.x + dx, center.y + dy, ground_z + preferred_ground_gap_cells};
        if (!unique.insert(candidate).second) {
          continue;
        }
        const VoxelKey ground{
          center.x + dx, center.y + dy, ground_z};
        VoxelKey collision;
        if (!traversableOnSurfaceUnlocked(
            candidate, ground, robot_height_cells,
            preferred_ground_gap_cells, ground_gap_tolerance_cells,
            diagnostics ? &collision : nullptr, robot_height_metres))
        {
          // With a real support layer and its preferred gap, rejection is
          // collision clearance. Retain the closest tested collision column.
          if (diagnostics) {
            ++diagnostics->occupied;
            record(ground, "body_clearance_blocked", &collision);
            const auto distance = [&](const VoxelKey & cell) {
                const auto p = keyToPos(cell); return std::hypot(p[0] - x, p[1] - y);
              };
            if (!diagnostics->has_collision ||
              distance(collision) < distance(diagnostics->collision))
            {
              diagnostics->collision = collision; diagnostics->has_collision = true;
            }
          }
          continue;
        }
        if (diagnostics) {++diagnostics->accepted;}
        record(ground, "accepted");
        const double xy_score = distance * distance;
        const double z_score = static_cast<double>(
          (ground_z - center.z) * (ground_z - center.z));
        scored.push_back(ScoredCandidate{z_score, xy_score, candidate});
      }
    }
  }

  std::sort(
    scored.begin(), scored.end(),
    [](const ScoredCandidate & left, const ScoredCandidate & right) {
      // A ground vehicle must remain on the requested terrain level. A
      // horizontally closer table top must never win over the expected floor.
      if (left.vertical_score != right.vertical_score) {
        return left.vertical_score < right.vertical_score;
      }
      if (left.horizontal_score != right.horizontal_score) {
        return left.horizontal_score < right.horizontal_score;
      }
      if (left.key.x != right.key.x) {
        return left.key.x < right.key.x;
      }
      if (left.key.y != right.key.y) {
        return left.key.y < right.key.y;
      }
      return left.key.z < right.key.z;
    });

  std::vector<VoxelKey> result;
  result.reserve(scored.size());
  for (const auto & item : scored) {
    result.push_back(item.key);
  }
  return result;
}

std::size_t OccupyMap::combineHash(std::size_t seed, std::size_t value) noexcept
{
  return hashCombine(seed, value);
}

void OccupyMap::addObstacleUnlocked(const VoxelKey & obstacle_key)
{
  if (!obstacle_occupancy_.insert(obstacle_key).second) {
    return;
  }
  updateBounds(obstacle_key);
  for (const auto & offset : inflate_offsets_) {
    const VoxelKey key{
      obstacle_key.x + offset[0],
      obstacle_key.y + offset[1],
      obstacle_key.z};
    ++inflated_ref_count_[key];
  }
  for (const auto & offset : cost_offsets_) {
    const VoxelKey key{
      obstacle_key.x + offset[0],
      obstacle_key.y + offset[1],
      obstacle_key.z};
    const float cost = 1.0F / std::sqrt(static_cast<float>(offset[2]));
    auto iterator = obstacle_cost_.find(key);
    if (iterator == obstacle_cost_.end() || iterator->second < cost) {
      obstacle_cost_[key] = cost;
    }
  }
}

void OccupyMap::removeObstacleUnlocked(const VoxelKey & obstacle_key)
{
  if (obstacle_occupancy_.erase(obstacle_key) == 0) {
    return;
  }
  obstacle_evidence_.erase(obstacle_key);
  dynamic_obstacle_bounds_.erase(obstacle_key);
  dynamic_obstacle_history_.erase(obstacle_key);
  for (const auto & offset : inflate_offsets_) {
    const VoxelKey key{
      obstacle_key.x + offset[0],
      obstacle_key.y + offset[1],
      obstacle_key.z};
    auto iterator = inflated_ref_count_.find(key);
    if (iterator == inflated_ref_count_.end()) {
      continue;
    }
    if (--iterator->second <= 0) {
      inflated_ref_count_.erase(iterator);
    }
  }
  for (const auto & offset : cost_offsets_) {
    recomputeObstacleCostUnlocked(
      VoxelKey{
        obstacle_key.x + offset[0],
        obstacle_key.y + offset[1],
        obstacle_key.z});
  }
}

void OccupyMap::recomputeObstacleCostUnlocked(const VoxelKey & key)
{
  float maximum_cost = 0.0F;
  for (const auto & offset : cost_offsets_) {
    const VoxelKey source{
      key.x - offset[0], key.y - offset[1], key.z};
    if (obstacle_occupancy_.find(source) == obstacle_occupancy_.end()) {
      continue;
    }
    maximum_cost = std::max(
      maximum_cost, 1.0F / std::sqrt(static_cast<float>(offset[2])));
  }
  if (maximum_cost > 0.0F) {
    obstacle_cost_[key] = maximum_cost;
  } else {
    obstacle_cost_.erase(key);
  }
}

void OccupyMap::updateBounds(const VoxelKey & key)
{
  min_key_.x = std::min(min_key_.x, key.x);
  min_key_.y = std::min(min_key_.y, key.y);
  min_key_.z = std::min(min_key_.z, key.z);
  max_key_.x = std::max(max_key_.x, key.x);
  max_key_.y = std::max(max_key_.y, key.y);
  max_key_.z = std::max(max_key_.z, key.z);
}

}  // namespace astar_3d_planner
