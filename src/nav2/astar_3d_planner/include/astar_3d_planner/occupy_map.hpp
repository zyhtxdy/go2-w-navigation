#ifndef ASTAR_3D_PLANNER__OCCUPY_MAP_HPP_
#define ASTAR_3D_PLANNER__OCCUPY_MAP_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace astar_3d_planner
{

struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept;
};

struct ColumnKey
{
  int x{0};
  int y{0};

  bool operator==(const ColumnKey & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct ColumnKeyHash
{
  std::size_t operator()(const ColumnKey & key) const noexcept;
};

class OccupyMap
{
public:
  enum class Traversability
  {
    kTraversable,
    kNoGroundSupport,
    kGroundGapMismatch,
    kOccupied
  };

  struct Config
  {
    double resolution{0.1};
    double inflate_radius{0.3};
    // Only set when complete rectangular sweeps guard every returned path.
    // nullopt keeps the legacy circle; 0 disables this preliminary rejection.
    std::optional<double> footprint_inscribed_radius;
    // A necessary-collision prefilter must use an inner bound of every
    // obstacle extent policy that a later rectangular check can enable.
    // Near-start policies deliberately apply globally only to this prefilter;
    // the complete footprint checker still enforces their actual scope.
    bool refine_static_obstacle_extents{false};
    bool refine_measured_obstacle_extents{false};
    double obstacle_extent_prefilter_margin{0.0};
    int cost_inflate_cells{2};
    double support_radius{0.15};
    double support_merge_height{0.15};
    // Additive bounded interpolation; never replaces measured-neighbour support.
    bool support_inference_enabled{false};
    double support_inference_radius{0.25};
    double support_inference_max_height_span{0.06};
    double support_inference_max_residual{0.01};
    double support_inference_max_slope_deg{10.0};
    int support_inference_max_columns{32768};
    double obstacle_update_radius{4.5};
    int obstacle_clear_after_updates{3};
    double synthetic_obstacle_clearance_radius{0.0};
    bool compensate_height_drift{false};
    bool require_static_obstacles{false};
    int height_drift_min_overlap{30};
    double height_drift_max_update{0.02};
    double height_drift_inlier_tolerance{0.008};
    double height_drift_min_consensus{0.7};
    double height_drift_min_xy_span{0.6};
    double height_drift_min_change{0.001};
  };

  using Point = std::array<double, 3>;
  using ReadLock = std::shared_lock<std::shared_mutex>;

  // Provenance cannot clear occupancy. Only purely measured bounds can
  // refine a full voxel; synthetic, unknown and mixed bounds remain whole.
  enum ObstacleSource : std::uint8_t {kMeasured = 1, kSynthetic = 2, kUnknown = 4, kStaticMap = 8};
  struct ObstacleEvidence
  {
    VoxelKey key{};
    Point point{};  // canonical height; exposed in the current height datum
    double observed_z{0.0};
    std::int64_t stamp_ns{0};
    std::uint8_t sources{kUnknown};
    // Actual sensor observation represented by point; zero means unavailable.
    // stamp_ns above remains the input cloud's analysis/publication stamp.
    std::int64_t observed_stamp_ns{0};
    // Preserve the pre-provenance representative used by synthetic clearance;
    // selecting a fresher diagnostic point must not change that policy.
    Point clearance_point{};
  };
  struct ObstacleBounds
  {
    Point minimum{}, maximum{};
    std::uint8_t sources{kUnknown};
  };
  struct SurfaceDiagnostics
  {
    struct Candidate
    {
      VoxelKey ground{};
      const char * reason{"accepted"};
      bool has_collision{false};
      VoxelKey collision{};
    };
    std::vector<Candidate> candidates;
    std::size_t omitted_candidates{0};
    std::size_t columns{0};
    std::size_t layers{0};
    std::size_t outside_height{0};
    std::size_t occupied{0};
    std::size_t accepted{0};
    bool has_collision{false};
    VoxelKey collision{};
  };
  struct GroundQuery
  {
    Point point{};
    bool inferred{false};
    const char * reason{"unknown"};
  };

  explicit OccupyMap(const Config & config);

  void updateSupport(const std::vector<Point> & points, std::int64_t stamp_ns = 0);
  bool updateObstacles(
    const std::vector<Point> & points, double observer_x, double observer_y,
    std::int64_t stamp_ns = 0,
    const std::vector<std::uint8_t> & sources = {}, bool replace_dynamic = false,
    std::optional<std::uint64_t> expected_epoch = std::nullopt,
    const std::vector<std::int64_t> & observed_stamps_ns = {});
  // Static ownership is independent of expiring live observations. Missing or
  // invalid provenance is unknown. Same-voxel sources are ORed and only a
  // purely measured voxel may expose a refined extent. Diagnostic evidence
  // keeps kStaticMap as its ownership marker independently of this semantic.
  void setStaticObstacles(
    const std::vector<Point> & points, std::int64_t stamp_ns = 0,
    const std::vector<std::uint8_t> & sources = {});
  // A restarted live stream may have an earlier clock origin. Invalidating its
  // epoch also prevents already-transforming clouds from repopulating the map.
  void clearDynamicObstacles(bool restart_stream = false);
  std::uint64_t dynamicObstacleEpoch() const;
  void reset();

  ReadLock lockForReading() const;
  bool readyUnlocked() const;
  std::size_t rawVoxelCountUnlocked() const;
  bool heightCompensationEnabled() const {return config_.compensate_height_drift;}
  double heightDatumOffsetUnlocked() const {return height_datum_offset_;}
  std::vector<Point> supportSnapshotUnlocked() const;
  // Only queried missing columns, capped for visualization. Measured support
  // snapshots and fixed-map identity are never augmented with inferred data.
  std::vector<GroundQuery> groundQueriesUnlocked(std::size_t limit = 2048) const;
  bool inferredSupportUnlocked(const VoxelKey & ground) const;
  std::int64_t supportStampUnlocked() const {return last_support_stamp_ns_;}
  std::int64_t obstacleStampUnlocked() const {return last_obstacle_stamp_ns_;}
  double resolution() const {return config_.resolution;}
  // Expire only timestamped, purely measured dynamic evidence, then acquire
  // a shared snapshot of that same update generation. Static/unknown evidence
  // and the stream's health/monotonic timestamp are not altered.
  ReadLock lockForReadingAfterExpiry(std::int64_t now_ns, std::int64_t maximum_age_ns,
    std::size_t * expired_count = nullptr);
  // Copy one consistent generation, then release the live map before search.
  std::unique_ptr<OccupyMap> planningSnapshotAfterExpiry(
    std::int64_t now_ns, std::int64_t maximum_age_ns);
  // Caller holds this map's read lock and has checked input freshness. This
  // private view is only for reference-route checks while the local controller
  // handles nearby measured returns; never use it to plan or authorize motion.
  // A whole source voxel must lie inside the XY radius. Static ownership and
  // uncertain/mixed evidence are retained, including coincident fixed/live data.
  std::unique_ptr<OccupyMap> referenceSnapshotUnlocked(
    double observer_x, double observer_y, double radius,
    std::size_t * ignored_count = nullptr) const;
  // Caller holds this map's read lock; the snapshot is private to the request.
  bool planningStateMatchesUnlocked(const OccupyMap & snapshot) const;
  int inflationCells() const {return inflate_cells_;}
  bool rectangularPrefilter() const {return config_.footprint_inscribed_radius.has_value();}
  double hardPrefilterRadius() const {return hard_prefilter_radius_;}
  double syntheticClearanceRadius() const {return config_.synthetic_obstacle_clearance_radius;}
  // Find an actual active source voxel contributing to this occupied cell.
  bool obstacleEvidenceUnlocked(const VoxelKey & cell, ObstacleEvidence & evidence) const;
  // Complete, purely measured saved-point bounds, never just a diagnostic
  // representative. A live observation delegates to the dynamic accessor.
  bool staticObstacleBoundsUnlocked(const VoxelKey & cell, ObstacleBounds & bounds) const;
  // Only purely measured dynamic evidence is refinable. Include fixed-map
  // bounds in the same voxel so a live return cannot hide a fixed obstacle.
  bool measuredObstacleBoundsUnlocked(const VoxelKey & cell, ObstacleBounds & bounds) const;
  std::vector<Point> inflationSnapshotUnlocked(const VoxelKey & source) const;

  VoxelKey posToKey(double x, double y, double z) const;
  Point keyToPos(const VoxelKey & key) const;

  bool occupiedUnlocked(const VoxelKey & key) const;
  // Full-body checks must use source voxels, not inflate the footprint again.
  bool rawObstacleOccupiedUnlocked(const VoxelKey & key) const
  {return obstacle_occupancy_.find(key) != obstacle_occupancy_.end();}
  // The voxel key identifies a support layer; its centre is not the measured
  // height. Callers must hold the map read lock when accessing this value.
  bool supportHeightUnlocked(const VoxelKey & ground, double & height) const;
  bool measuredSupportXYUnlocked(const VoxelKey & ground, std::array<double, 2> & xy) const;
  bool findSupportByHeightUnlocked(
    double x, double y, double observed_height, double tolerance,
    VoxelKey & ground) const;
  bool findGroundUnlocked(
    const VoxelKey & candidate, int support_depth_cells, VoxelKey & ground) const;
  bool findMeasuredGroundUnlocked(
    const VoxelKey & candidate, int support_depth_cells, VoxelKey & ground) const;
  bool findSurfaceGroundUnlocked(
    const VoxelKey & candidate, int preferred_ground_gap_cells,
    int ground_gap_tolerance_cells, VoxelKey & ground) const;
  bool traversableOnSurfaceUnlocked(
    const VoxelKey & candidate, const VoxelKey & ground,
    int robot_height_cells, int preferred_ground_gap_cells,
    int ground_gap_tolerance_cells, VoxelKey * collision = nullptr,
    double robot_height_metres = 0.0) const;
  bool traversableUnlocked(
    const VoxelKey & candidate, int support_depth_cells, int robot_height_cells,
    int preferred_ground_gap_cells, int ground_gap_tolerance_cells,
    VoxelKey * ground = nullptr, double robot_height_metres = 0.0) const;
  Traversability traversabilityUnlocked(
    const VoxelKey & candidate, int support_depth_cells, int robot_height_cells,
    int preferred_ground_gap_cells, int ground_gap_tolerance_cells,
    VoxelKey * ground = nullptr, double robot_height_metres = 0.0) const;
  double obstacleCostUnlocked(
    const VoxelKey & candidate, int robot_height_cells) const;

  std::vector<VoxelKey> surfaceCandidatesUnlocked(
    double x, double y, double desired_ground_z, double horizontal_radius,
    double vertical_radius, int support_depth_cells, int robot_height_cells,
    int preferred_ground_gap_cells, int ground_gap_tolerance_cells,
    SurfaceDiagnostics * diagnostics = nullptr, double robot_height_metres = 0.0) const;

private:
  static std::size_t combineHash(std::size_t seed, std::size_t value) noexcept;
  void updateBounds(const VoxelKey & key);
  void addObstacleUnlocked(const VoxelKey & key);
  void removeObstacleUnlocked(const VoxelKey & key);
  void recomputeObstacleCostUnlocked(const VoxelKey & key);
  void reconcileHeightDatumUnlocked(const std::vector<Point> & points);
  VoxelKey posToSupportKey(double x, double y, double z) const;
  struct InferredLayer {int z; double height;};
  struct InferredColumn
  {
    std::vector<InferredLayer> layers;
    double query_height{0.0};
    const char * reason{"insufficient_measured_support"};
  };
  InferredColumn inferColumnUnlocked(const ColumnKey & column, double query_height) const;
  std::vector<int> groundLayersUnlocked(const ColumnKey & column, double query_height) const;
  void clearDynamicObstaclesUnlocked();
  std::unique_ptr<OccupyMap> snapshotUnlocked() const;

  Config config_;
  double inv_resolution_{10.0};
  int inflate_cells_{3};
  double hard_prefilter_radius_{0.0};
  int support_radius_cells_{2};
  int obstacle_update_radius_cells_{45};
  std::uint64_t obstacle_update_sequence_{0};
  std::uint64_t dynamic_obstacle_epoch_{0};
  double height_datum_offset_{0.0};
  std::int64_t last_support_stamp_ns_{0};
  std::int64_t last_obstacle_stamp_ns_{0};
  std::vector<std::array<int, 2>> inflate_offsets_;
  std::vector<std::array<int, 3>> cost_offsets_;

  mutable std::shared_mutex mutex_;
  std::unordered_set<VoxelKey, VoxelKeyHash> support_occupancy_;
  std::unordered_map<VoxelKey, double, VoxelKeyHash> support_heights_;
  std::unordered_set<VoxelKey, VoxelKeyHash> obstacle_occupancy_;
  std::unordered_map<VoxelKey, int, VoxelKeyHash> inflated_ref_count_;
  std::unordered_map<VoxelKey, float, VoxelKeyHash> obstacle_cost_;
  std::unordered_map<VoxelKey, std::uint64_t, VoxelKeyHash> obstacle_last_seen_;
  std::unordered_map<VoxelKey, ObstacleEvidence, VoxelKeyHash> obstacle_evidence_;
  std::unordered_map<VoxelKey, ObstacleEvidence, VoxelKeyHash> static_obstacles_;
  std::unordered_map<VoxelKey, ObstacleBounds, VoxelKeyHash> static_obstacle_bounds_;
  std::unordered_map<VoxelKey, ObstacleBounds, VoxelKeyHash> dynamic_obstacle_bounds_;
  struct DynamicBoundsObservation
  {
    std::uint64_t sequence;
    ObstacleBounds bounds;
  };
  // Same lifetime in accepted updates as obstacle_clear_after_updates.
  // Repeated returns in one voxel must not retain an old extremum forever.
  std::unordered_map<VoxelKey, std::deque<DynamicBoundsObservation>, VoxelKeyHash>
    dynamic_obstacle_history_;
  bool static_obstacles_loaded_{false};
  std::unordered_map<ColumnKey, std::vector<int>, ColumnKeyHash> support_columns_;
  std::unordered_map<VoxelKey, std::array<double, 2>, VoxelKeyHash> support_positions_;
  // All callers also hold the map read lock. This mutex only protects the
  // lazy per-support-generation cache against concurrent planner queries.
  mutable std::mutex ground_cache_mutex_;
  mutable std::unordered_map<ColumnKey, InferredColumn, ColumnKeyHash> ground_cache_;

  VoxelKey min_key_{
    std::numeric_limits<int>::max(), std::numeric_limits<int>::max(),
    std::numeric_limits<int>::max()};
  VoxelKey max_key_{
    std::numeric_limits<int>::lowest(), std::numeric_limits<int>::lowest(),
    std::numeric_limits<int>::lowest()};
};

}  // namespace astar_3d_planner

#endif  // ASTAR_3D_PLANNER__OCCUPY_MAP_HPP_
