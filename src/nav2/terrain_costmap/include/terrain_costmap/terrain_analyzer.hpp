#ifndef TERRAIN_COSTMAP__TERRAIN_ANALYZER_HPP_
#define TERRAIN_COSTMAP__TERRAIN_ANALYZER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace terrain_costmap
{

struct Point
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  // Original sensor-frame time, never the time of a later terrain analysis.
  // Zero denotes unavailable provenance (e.g. an offline map sample).
  std::int64_t observation_stamp_ns{0};
};

struct TerrainPoint
{
  Point point;
  double relative_height{0.0};
  // True only for a collision marker synthesized from an unsafe terrain
  // classification. Measured returns above a fitted ground plane are false.
  bool synthetic{false};
};

struct AnalyzerParameters
{
  double resolution{0.20};
  double radius{5.0};
  int fit_radius_cells{2};
  int minimum_fit_cells{3};
  // Actual returns close to the selected ground layer, not all obstacle
  // returns in nearby cells. Independent occupied cells are checked above.
  int minimum_support_points{5};
  // Use a lower percentile instead of the absolute minimum return as the
  // cell surface height. This prevents a single multipath/desk-edge outlier
  // from tilting the fitted ground plane.
  double ground_height_percentile{0.20};
  // Half of this height tolerance selects two-sided consensus layers;
  // the full value is retained for recovering a floor under dense objects.
  double positive_outlier_tolerance{0.10};
  double maximum_plane_rmse{0.08};
  double maximum_slope{0.60};
  double maximum_step_height{0.18};
  double obstacle_minimum_height{0.15};
  double obstacle_maximum_height{1.80};
  // Measured obstacle returns must have this many neighbors in the local
  // grid before entering the global obstacle stream.
  int minimum_obstacle_cluster_points{3};
  int obstacle_cluster_radius_cells{1};
  // Terrain fits whose kernels overlap the LiDAR blind zone are unreliable
  // beside the vehicle. Suppress only synthesized markers in this radius;
  // measured above-ground obstacles remain active.
  double synthetic_obstacle_clearance_radius{0.0};
  // Opt-in offline fallback: fit a consensus of actual observed layers when
  // sparse-cell percentiles make the initial fit fail slope/RMSE gates.
  bool recover_sparse_ground_layers{false};
};

struct AnalysisDiagnostics
{
  std::size_t observed_cells{0};
  std::size_t unobserved_cells{0};
  // Each observed cell has exactly one final outcome below. When several
  // limits fail, report the first gate: fit, support, slope, RMSE, then step.
  std::size_t insufficient_fit_cells{0};
  std::size_t degenerate_fit_cells{0};
  std::size_t insufficient_support_cells{0};
  std::size_t slope_exceeded_cells{0};
  std::size_t rmse_exceeded_cells{0};
  std::size_t step_exceeded_cells{0};
  std::size_t traversable_cells{0};
};

struct Analysis
{
  // Optional export evidence for each observed cell, not a traversability override.
  struct CellDiagnostic
  {
    Point center;
    double minimum_z{0.0};
    double maximum_z{0.0};
    double plane_z{0.0};
    double slope{0.0};
    double rmse{0.0};
    int support_returns{0};
    std::size_t returns{0};
    bool fitted{false};
    bool observed_ground{false};
    bool synthetic_hazard{false};
    bool recovered_layer{false};
    const char * outcome{"unobserved"};
  };
  std::vector<CellDiagnostic> cell_diagnostics;
  std::vector<TerrainPoint> terrain;
  // Kept for local-costmap compatibility: measured and synthesized hazards.
  std::vector<TerrainPoint> obstacles;
  std::vector<TerrainPoint> measured_obstacles;
  std::vector<TerrainPoint> synthetic_obstacles;
  // One fitted ground-height sample for every observed, traversable cell.
  // Unlike terrain, this contains no returns above the ground plane and is
  // therefore safe to use as the support source of a 2.5D/3D planner.
  std::vector<Point> support_surface;
  std::vector<Point> non_traversable;
  std::vector<unsigned char> supported_cells;
  std::size_t grid_width{0};
  std::size_t suppressed_near_synthetic_obstacles{0};
  AnalysisDiagnostics diagnostics;
};

class TerrainAnalyzer
{
public:
  explicit TerrainAnalyzer(const AnalyzerParameters & parameters);

  Analysis analyze(
    const std::vector<Point> & points, double center_x, double center_y,
    bool collect_diagnostics = false) const;

private:
  AnalyzerParameters parameters_;
};

}  // namespace terrain_costmap

#endif  // TERRAIN_COSTMAP__TERRAIN_ANALYZER_HPP_
