#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <utility>
#include <vector>

#include "terrain_costmap/terrain_analyzer.hpp"
#include "terrain_costmap/terrain_hazard_stabilizer.hpp"

using terrain_costmap::AnalyzerParameters;
using terrain_costmap::Point;
using terrain_costmap::TerrainAnalyzer;
using terrain_costmap::TerrainHazardStabilizer;
using terrain_costmap::TerrainPoint;
using terrain_costmap::HazardStabilizerParameters;

namespace
{

void expectConsistentDiagnostics(const terrain_costmap::Analysis & analysis)
{
  const auto & diagnostics = analysis.diagnostics;
  EXPECT_EQ(
    diagnostics.observed_cells,
    diagnostics.insufficient_fit_cells + diagnostics.degenerate_fit_cells +
    diagnostics.insufficient_support_cells + diagnostics.slope_exceeded_cells +
    diagnostics.rmse_exceeded_cells + diagnostics.step_exceeded_cells +
    diagnostics.traversable_cells);
  EXPECT_EQ(
    diagnostics.observed_cells + diagnostics.unobserved_cells,
    analysis.grid_width * analysis.grid_width);
  EXPECT_EQ(diagnostics.traversable_cells, analysis.support_surface.size());
  EXPECT_EQ(
    diagnostics.slope_exceeded_cells + diagnostics.rmse_exceeded_cells +
    diagnostics.step_exceeded_cells, analysis.non_traversable.size());
}

}  // namespace

TEST(TerrainAnalyzerDiagnostics, EmptySceneIsUnobservedRatherThanUnsafe)
{
  const auto result = TerrainAnalyzer(AnalyzerParameters{}).analyze({}, 0.0, 0.0);
  expectConsistentDiagnostics(result);
  EXPECT_EQ(result.diagnostics.observed_cells, 0u);
  EXPECT_GT(result.diagnostics.unobserved_cells, 0u);
  EXPECT_TRUE(result.non_traversable.empty());
  EXPECT_TRUE(result.obstacles.empty());
}

TEST(TerrainAnalyzerDiagnostics, IsolatedReturnReportsMissingFitNeighbors)
{
  const auto result = TerrainAnalyzer(AnalyzerParameters{}).analyze(
    {{0.0, 0.0, 0.0}}, 0.0, 0.0);
  expectConsistentDiagnostics(result);
  EXPECT_EQ(result.diagnostics.insufficient_fit_cells, 1u);
  EXPECT_TRUE(result.support_surface.empty());
  EXPECT_TRUE(result.non_traversable.empty());
}

TEST(TerrainAnalyzerDiagnostics, CollinearReturnsReportDegenerateFit)
{
  const auto result = TerrainAnalyzer(AnalyzerParameters{}).analyze(
    {{-0.2, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.2, 0.0, 0.0}}, 0.0, 0.0);
  expectConsistentDiagnostics(result);
  EXPECT_EQ(result.diagnostics.degenerate_fit_cells, 3u);
  EXPECT_TRUE(result.support_surface.empty());
  EXPECT_TRUE(result.non_traversable.empty());
}

TEST(TerrainAnalyzerDiagnostics, TooFewReturnsTakePriorityOverUnsafeSlope)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 4;
  parameters.maximum_slope = 0.1;
  const auto result = TerrainAnalyzer(parameters).analyze(
    {{0.0, 0.0, 0.0}, {0.2, 0.0, 0.2}, {0.0, 0.2, 0.2}}, 0.0, 0.0);
  expectConsistentDiagnostics(result);
  EXPECT_EQ(result.diagnostics.insufficient_support_cells, 3u);
  EXPECT_EQ(result.diagnostics.slope_exceeded_cells, 0u);
  EXPECT_TRUE(result.support_surface.empty());
  // Insufficient evidence remains unknown; diagnostics cannot turn it into
  // a synthesized obstacle or change the existing classification.
  EXPECT_TRUE(result.non_traversable.empty());
}

TEST(TerrainAnalyzerDiagnostics, SteepPlaneReportsSlopeRejection)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  parameters.maximum_slope = 0.1;
  std::vector<Point> points;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      points.push_back({ix * 0.2, iy * 0.2, ix * 0.2 * 0.5});
    }
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0.0, 0.0);
  expectConsistentDiagnostics(result);
  EXPECT_EQ(result.diagnostics.slope_exceeded_cells, points.size());
  EXPECT_EQ(result.diagnostics.rmse_exceeded_cells, 0u);
  EXPECT_TRUE(result.support_surface.empty());
}

TEST(TerrainAnalyzerDiagnostics, RoughSurfaceReportsPlaneError)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  parameters.maximum_slope = 10.0;
  parameters.maximum_plane_rmse = 0.001;
  std::vector<Point> points;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      points.push_back({ix * 0.2, iy * 0.2, (ix + iy) % 2 == 0 ? 0.04 : -0.04});
    }
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0.0, 0.0);
  expectConsistentDiagnostics(result);
  EXPECT_GT(result.diagnostics.rmse_exceeded_cells, 0u);
  EXPECT_EQ(result.diagnostics.slope_exceeded_cells, 0u);
  EXPECT_TRUE(result.support_surface.empty());
}

TEST(TerrainAnalyzerDiagnostics, AllowedLocalFitsStillReportExcessiveStep)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  parameters.maximum_slope = 10.0;
  parameters.maximum_plane_rmse = 1.0;
  parameters.positive_outlier_tolerance = 1.0;
  parameters.maximum_step_height = 0.08;
  std::vector<Point> points;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      points.push_back({ix * 0.2, iy * 0.2, ix < 0 ? 0.0 : 0.12});
    }
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0.0, 0.0);
  expectConsistentDiagnostics(result);
  EXPECT_EQ(result.diagnostics.step_exceeded_cells, 14u);
  EXPECT_EQ(result.diagnostics.slope_exceeded_cells, 0u);
  EXPECT_EQ(result.diagnostics.rmse_exceeded_cells, 0u);
  EXPECT_GT(result.diagnostics.traversable_cells, 0u);
}

TEST(TerrainAnalyzerDiagnostics, FlatObservedCellsMatchPublishedSupport)
{
  std::vector<Point> points;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      points.push_back({ix * 0.2, iy * 0.2, -0.45});
    }
  }
  const auto result = TerrainAnalyzer(AnalyzerParameters{}).analyze(points, 0.0, 0.0);
  expectConsistentDiagnostics(result);
  EXPECT_EQ(result.diagnostics.observed_cells, points.size());
  EXPECT_EQ(result.diagnostics.traversable_cells, points.size());
  EXPECT_TRUE(result.obstacles.empty());
  for (const Point & support : result.support_surface) {
    EXPECT_NEAR(support.z, -0.45, 1.0e-9);
  }
}

TEST(TerrainAnalyzer, RaisedOcclusionDoesNotTurnConfirmedAdjacentFloorIntoAStep)
{
  for (double height : {.081, .12, .25, .70}) {
    SCOPED_TRACE(height);
    AnalyzerParameters parameters;
    parameters.maximum_step_height = .08;
    parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
    std::vector<Point> points;
    for (int x = -4; x <= 4; ++x) {
      for (int y = -4; y <= 4; ++y) {
        if (x != 0 || y != 0) {points.push_back({1.0 + x * .2, y * .2, -.35});}
      }
    }
    // Only the raised surface is visible in this cell, no floor under it.
    points.push_back({1.0, 0, -.35 + height});
    const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0, true);
    expectConsistentDiagnostics(result);
    bool saw_occlusion = false;
    for (const auto & cell : result.cell_diagnostics) {
      if (std::hypot(cell.center.x - 1.0, cell.center.y) < .01) {
        saw_occlusion = true;
        EXPECT_FALSE(cell.observed_ground);
        EXPECT_TRUE(cell.synthetic_hazard);  // Low raised steps also stay blocked.
      }
      if (std::abs(std::hypot(cell.center.x - 1.0, cell.center.y) - .2) < .01) {
        EXPECT_TRUE(cell.observed_ground);
        EXPECT_FALSE(cell.synthetic_hazard);
        EXPECT_STREQ(cell.outcome, "traversable");
      }
    }
    EXPECT_TRUE(saw_occlusion);
    for (const auto & support : result.support_surface) {
      EXPECT_GT(std::hypot(support.x - 1.0, support.y), .01);  // No invented floor under the object.
    }
    bool blocked_center = false;
    for (const auto & hazard : result.synthetic_obstacles) {
      if (std::hypot(hazard.point.x - 1.0, hazard.point.y) < .01) {blocked_center = true;}
    }
    EXPECT_TRUE(blocked_center);
  }
}

TEST(TerrainAnalyzer, UnconfirmedDownwardDiscontinuityKeepsTheObservedFloorEdgeBlocked)
{
  AnalyzerParameters parameters;
  parameters.maximum_step_height = .08;
  std::vector<Point> points;
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {
      points.push_back({1.0 + x * .2, y * .2, (x == 0 && y == 0) ? -.47 : -.35});
    }
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0, true);
  expectConsistentDiagnostics(result);
  for (const auto & cell : result.cell_diagnostics) {
    if (std::abs(std::hypot(cell.center.x - 1.0, cell.center.y) - .2) < .01) {
      EXPECT_TRUE(cell.synthetic_hazard);  // The upward-occlusion fix must not erase drop-edge protection.
    }
  }
}

TEST(TerrainAnalyzer, RampIsGround)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  TerrainAnalyzer analyzer(parameters);
  std::vector<Point> points;
  for (int ix = -20; ix <= 20; ++ix) {
    for (int iy = -20; iy <= 20; ++iy) {
      const double x = ix * 0.2;
      const double y = iy * 0.2;
      points.push_back({x, y, 0.20 * x});
    }
  }
  const auto result = analyzer.analyze(points, 0.0, 0.0);
  EXPECT_GT(result.terrain.size(), 1000u);
  EXPECT_GT(result.support_surface.size(), 1000u);
  EXPECT_TRUE(result.obstacles.empty());
  for (const Point & support : result.support_surface) {
    EXPECT_NEAR(support.z, 0.20 * support.x, 1.0e-6);
  }
}

TEST(TerrainAnalyzer, BoxAboveRampIsObstacle)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  TerrainAnalyzer analyzer(parameters);
  std::vector<Point> points;
  for (int ix = -20; ix <= 20; ++ix) {
    for (int iy = -20; iy <= 20; ++iy) {
      const double x = ix * 0.2;
      const double y = iy * 0.2;
      points.push_back({x, y, 0.15 * x});
    }
  }
  for (int i = 0; i < 20; ++i) {
    points.push_back({2.0 + 0.01 * (i % 3), 0.02 * (i % 5), 0.8});
  }
  const auto result = analyzer.analyze(points, 0.0, 0.0);
  EXPECT_GT(result.obstacles.size(), 10u);
  EXPECT_GT(result.measured_obstacles.size(), 10u);
  EXPECT_TRUE(result.synthetic_obstacles.empty());
  ASSERT_FALSE(result.support_surface.empty());
  for (const Point & support : result.support_surface) {
    // The box top must never leak into the planner support surface.
    EXPECT_LT(support.z - 0.15 * support.x, 0.1);
  }
}

TEST(TerrainAnalyzer, ExcessiveSlopeHasNoSupport)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  parameters.maximum_slope = 0.4;
  TerrainAnalyzer analyzer(parameters);
  std::vector<Point> points;
  for (int ix = -10; ix <= 10; ++ix) {
    for (int iy = -10; iy <= 10; ++iy) {
      const double x = ix * 0.2;
      const double y = iy * 0.2;
      points.push_back({x, y, 0.8 * x});
    }
  }
  const auto result = analyzer.analyze(points, 0.0, 0.0);
  EXPECT_TRUE(result.terrain.empty());
  EXPECT_TRUE(result.support_surface.empty());
  EXPECT_FALSE(result.non_traversable.empty());
  EXPECT_FALSE(result.obstacles.empty());
  EXPECT_FALSE(result.synthetic_obstacles.empty());
}

TEST(TerrainAnalyzer, HeightDiscontinuityIsNotTraversable)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  TerrainAnalyzer analyzer(parameters);
  std::vector<Point> points;
  for (int ix = -20; ix <= 20; ++ix) {
    for (int iy = -20; iy <= 20; ++iy) {
      const double x = ix * 0.2;
      const double y = iy * 0.2;
      points.push_back({x, y, x < 0.0 ? 0.0 : 0.30});
    }
  }
  const auto result = analyzer.analyze(points, 0.0, 0.0);
  EXPECT_FALSE(result.non_traversable.empty());
  EXPECT_FALSE(result.obstacles.empty());
}

TEST(TerrainAnalyzer, NearVehicleSyntheticHazardsAreSuppressed)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  parameters.maximum_slope = 0.4;
  parameters.synthetic_obstacle_clearance_radius = 0.65;
  TerrainAnalyzer analyzer(parameters);
  std::vector<Point> points;
  for (int ix = -20; ix <= 20; ++ix) {
    for (int iy = -20; iy <= 20; ++iy) {
      const double x = ix * 0.2;
      const double y = iy * 0.2;
      points.push_back({x, y, 0.8 * x});
    }
  }

  const auto result = analyzer.analyze(points, 0.0, 0.0);
  EXPECT_GT(result.suppressed_near_synthetic_obstacles, 0u);
  EXPECT_FALSE(result.obstacles.empty());
  for (const auto & obstacle : result.obstacles) {
    if (obstacle.synthetic) {
      EXPECT_GT(
        std::hypot(obstacle.point.x, obstacle.point.y),
        parameters.synthetic_obstacle_clearance_radius);
    }
  }
}

TEST(TerrainAnalyzer, NearVehicleMeasuredObstacleIsNeverSuppressed)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  parameters.synthetic_obstacle_clearance_radius = 0.65;
  TerrainAnalyzer analyzer(parameters);
  std::vector<Point> points;
  for (int ix = -20; ix <= 20; ++ix) {
    for (int iy = -20; iy <= 20; ++iy) {
      points.push_back({ix * 0.2, iy * 0.2, 0.0});
    }
  }
  for (int i = 0; i < 20; ++i) {
    points.push_back({0.50 + 0.01 * (i % 3), 0.01 * (i % 5), 0.60});
  }

  const auto result = analyzer.analyze(points, 0.0, 0.0);
  bool found_near_measured_obstacle = false;
  for (const auto & obstacle : result.obstacles) {
    const double range = std::hypot(obstacle.point.x, obstacle.point.y);
    if (!obstacle.synthetic && range <=
      parameters.synthetic_obstacle_clearance_radius)
    {
      found_near_measured_obstacle = true;
      EXPECT_GT(obstacle.relative_height, 0.5);
    }
  }
  EXPECT_TRUE(found_near_measured_obstacle);
}

TEST(TerrainAnalyzer, LowerHeightOutlierDoesNotTiltGround)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  parameters.ground_height_percentile = 0.20;
  TerrainAnalyzer analyzer(parameters);
  std::vector<Point> points;
  for (int ix = -10; ix <= 10; ++ix) {
    for (int iy = -10; iy <= 10; ++iy) {
      const double x = ix * 0.2;
      const double y = iy * 0.2;
      points.push_back({x, y, -0.8});
      for (int sample = 0; sample < 5; ++sample) {
        points.push_back({x, y, 0.001 * sample});
      }
    }
  }

  const auto result = analyzer.analyze(points, 0.0, 0.0);
  ASSERT_GT(result.support_surface.size(), 300u);
  for (const Point & support : result.support_surface) {
    EXPECT_NEAR(support.z, 0.0, 0.01);
  }
}

TEST(TerrainAnalyzer, SparseObservedLayerRecoveryKeepsAmbiguousCentersRejected)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.maximum_step_height = .08;
  std::vector<Point> points;
  for (int x = -2; x <= 2; ++x) {
    for (int y = -2; y <= 2; ++y) {
      const double height = .03 * x * .2 + .02 * y * .2;
      points.push_back({x * .2, y * .2, height});
      if ((x + y) % 2 == 0) {points.push_back({x * .2, y * .2, height - .8});}
    }
  }
  const auto baseline = TerrainAnalyzer(parameters).analyze(points, 0, 0, true);
  parameters.recover_sparse_ground_layers = true;
  const auto recovered = TerrainAnalyzer(parameters).analyze(points, 0, 0, true);
  expectConsistentDiagnostics(recovered);
  ASSERT_GT(recovered.support_surface.size(), baseline.support_surface.size());
  // Neighboring low returns may be outliers, but a center with its own lower
  // return cannot safely promote the upper layer to ground.
  ASSERT_EQ(recovered.support_surface.size(), 12u);
  EXPECT_TRUE(recovered.synthetic_obstacles.empty());
  bool used_recovery = false;
  for (const auto & cell : recovered.cell_diagnostics) {
    used_recovery |= cell.recovered_layer;
    if (cell.returns > 1) {
      EXPECT_FALSE(cell.recovered_layer);
      EXPECT_STRNE(cell.outcome, "traversable");
    }
  }
  EXPECT_TRUE(used_recovery);
  for (const auto & support : recovered.support_surface) {
    EXPECT_NEAR(support.z, .03 * support.x + .02 * support.y, 1e-8);
    const auto x = std::lround(support.x / .2), y = std::lround(support.y / .2);
    EXPECT_NE((x + y) % 2, 0);
  }
  // Entirely unobserved cells around the patch still have no support.
  EXPECT_EQ(recovered.cell_diagnostics.size(), 25u);
}

TEST(TerrainAnalyzer, SparseRecoveryPreservesRealStepsAndSteepGround)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.maximum_step_height = .08;
  parameters.recover_sparse_ground_layers = true;
  std::vector<Point> step, slope;
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {
      step.push_back({x * .2, y * .2, x < 0 ? -.35 : 0.0});
      slope.push_back({x * .2, y * .2, .4 * x * .2});
    }
  }
  const auto step_result = TerrainAnalyzer(parameters).analyze(step, 0, 0, true);
  const auto slope_result = TerrainAnalyzer(parameters).analyze(slope, 0, 0, true);
  expectConsistentDiagnostics(step_result);
  expectConsistentDiagnostics(slope_result);
  ASSERT_FALSE(step_result.synthetic_obstacles.empty());
  ASSERT_FALSE(slope_result.synthetic_obstacles.empty());
  EXPECT_TRUE(slope_result.support_surface.empty());
  for (const auto & support : step_result.support_surface) {
    EXPECT_NEAR(support.z, support.x < 0 ? -.35 : 0.0, 1e-8);
  }
}

TEST(TerrainAnalyzer, SparseRecoveryDoesNotReplaceSteepGroundWithUpperLayer)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.maximum_step_height = .08;
  std::vector<Point> points;
  for (int x = -2; x <= 2; ++x) {
    for (int y = -2; y <= 2; ++y) {
      points.push_back({x * .2, y * .2, .4 * x * .2});
      points.push_back({x * .2, y * .2, .6});
    }
  }
  const auto baseline = TerrainAnalyzer(parameters).analyze(points, 0, 0, true);
  parameters.recover_sparse_ground_layers = true;
  const auto recovered = TerrainAnalyzer(parameters).analyze(points, 0, 0, true);
  expectConsistentDiagnostics(recovered);
  EXPECT_TRUE(recovered.support_surface.empty());
  EXPECT_EQ(recovered.diagnostics.slope_exceeded_cells, 25u);
  EXPECT_EQ(recovered.non_traversable.size(), baseline.non_traversable.size());
  ASSERT_FALSE(recovered.synthetic_obstacles.empty());
  EXPECT_EQ(recovered.synthetic_obstacles.size(), baseline.synthetic_obstacles.size());
  for (const auto & cell : recovered.cell_diagnostics) {EXPECT_FALSE(cell.recovered_layer);}
}

TEST(TerrainAnalyzer, SparseRecoveryKeepsRealStepBelowUpperLayer)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.maximum_step_height = .08;
  parameters.recover_sparse_ground_layers = true;
  std::vector<Point> points;
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {
      points.push_back({x * .2, y * .2, x < 0 ? -.35 : 0.0});
      points.push_back({x * .2, y * .2, .6});
    }
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0, true);
  expectConsistentDiagnostics(result);
  ASSERT_FALSE(result.synthetic_obstacles.empty());
  ASSERT_FALSE(result.support_surface.empty());
  for (const auto & support : result.support_surface) {
    EXPECT_NEAR(support.z, support.x < 0 ? -.35 : 0.0, 1e-8);
  }
  // Upper returns remain real obstacles even where a mixed step kernel needs
  // recovery; selecting that upper layer as ground would discard them.
  ASSERT_EQ(result.measured_obstacles.size(), 81u);
  for (const auto & obstacle : result.measured_obstacles) {
    EXPECT_DOUBLE_EQ(obstacle.point.z, .6);
  }
}

TEST(TerrainAnalyzer, SparseRecoveryKeepsOcclusionAndGroundConfidenceGates)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.maximum_step_height = .08;
  parameters.recover_sparse_ground_layers = true;
  std::vector<Point> points;
  for (int x = -2; x <= 2; ++x) {
    for (int y = -2; y <= 2; ++y) {
      if (x == 0 && y == 0) {continue;}
      points.push_back({x * .2, y * .2, 0.0});
      if ((x + y) % 2 == 0) {points.push_back({x * .2, y * .2, -.8});}
    }
  }
  for (int i = 0; i < 5; ++i) {points.push_back({0, 0, .6});}
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0, true);
  expectConsistentDiagnostics(result);
  for (const auto & point : result.support_surface) {
    EXPECT_GT(std::hypot(point.x, point.y), 1e-9);
  }
  EXPECT_EQ(result.measured_obstacles.size(), 5u);
  std::vector<Point> sparse{{0, 0, 0}, {.2, 0, 0}, {0, .2, 0}, {.2, .2, 0}};
  for (int i = 0; i < 20; ++i) {sparse.push_back({0, 0, .8});}
  EXPECT_TRUE(TerrainAnalyzer(parameters).analyze(sparse, 0, 0).support_surface.empty());
}

TEST(TerrainAnalyzerDiagnostics, PerCellExportDoesNotChangeTraversability)
{
  AnalyzerParameters parameters;
  std::vector<Point> points;
  for (int x = -3; x <= 3; ++x) {
    for (int y = -3; y <= 3; ++y) {points.push_back({x * .2, y * .2, 0.0});}
  }
  for (int i = 0; i < 5; ++i) {points.push_back({0, 0, .5});}
  const auto plain = TerrainAnalyzer(parameters).analyze(points, 0, 0);
  const auto detailed = TerrainAnalyzer(parameters).analyze(points, 0, 0, true);
  EXPECT_TRUE(plain.cell_diagnostics.empty());
  EXPECT_EQ(detailed.cell_diagnostics.size(), detailed.diagnostics.observed_cells);
  EXPECT_EQ(plain.support_surface.size(), detailed.support_surface.size());
  EXPECT_EQ(plain.obstacles.size(), detailed.obstacles.size());
  for (const auto & cell : detailed.cell_diagnostics) {
    EXPECT_GT(cell.returns, 0u);
    EXPECT_TRUE(std::isfinite(cell.center.z));
    EXPECT_LE(cell.minimum_z, cell.maximum_z);
    EXPECT_STREQ(cell.outcome, "traversable");
  }
}

TEST(TerrainAnalyzer, SmallNegativeOutliersDoNotReplaceConsistentFloorPercentile)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.maximum_step_height = 0.08;
  std::vector<Point> points;
  for (int ix = 0; ix < 2; ++ix) {
    for (int iy = 0; iy < 2; ++iy) {
      // The 20th percentile is zero in every cell. A single return 8 cm
      // below the floor must not be recovered as an apparent 40% slope.
      points.push_back({ix * 0.2, iy * 0.2, ix == 0 ? 0.0 : -0.08});
      for (int sample = 0; sample < 5; ++sample) {
        points.push_back({ix * 0.2, iy * 0.2, 0.0});
      }
    }
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0.0, 0.0);
  expectConsistentDiagnostics(result);
  ASSERT_EQ(result.support_surface.size(), 4u);
  EXPECT_EQ(result.diagnostics.slope_exceeded_cells, 0u);
  EXPECT_TRUE(result.non_traversable.empty());
  EXPECT_TRUE(result.obstacles.empty());
  for (const Point & support : result.support_surface) {
    EXPECT_NEAR(support.z, 0.0, 1.0e-9);
  }
}

TEST(TerrainAnalyzer, DenseObstacleTopStillRecoversObservedFloorBelowIt)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.maximum_step_height = 0.08;
  std::vector<Point> points;
  for (int ix = -2; ix <= 2; ++ix) {
    for (int iy = -2; iy <= 2; ++iy) {
      points.push_back({ix * 0.2, iy * 0.2, 0.0});
    }
  }
  // The occupied cell's percentile is the box top, but its independently
  // observed floor return agrees with its eight surrounding floor cells.
  for (int sample = 0; sample < 20; ++sample) {
    points.push_back({0.0, 0.0, 0.60});
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0.0, 0.0);
  expectConsistentDiagnostics(result);
  ASSERT_EQ(result.support_surface.size(), 25u);
  ASSERT_EQ(result.measured_obstacles.size(), 20u);
  EXPECT_TRUE(result.synthetic_obstacles.empty());
  for (const Point & support : result.support_surface) {
    EXPECT_NEAR(support.z, 0.0, 1.0e-9);
  }
  for (const TerrainPoint & obstacle : result.measured_obstacles) {
    EXPECT_NEAR(obstacle.relative_height, 0.60, 1.0e-9);
  }
}

TEST(TerrainAnalyzer, WorldSupportGridDoesNotMoveWithRollingWindowCenter)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.maximum_step_height = 0.08;
  TerrainAnalyzer analyzer(parameters);
  std::vector<Point> points;
  for (int ix = -5; ix <= 5; ++ix) {
    for (int iy = -5; iy <= 5; ++iy) {
      // Include both signs and points offset within their fixed world cells.
      const double x = ix * 0.2 + 0.03;
      const double y = iy * 0.2 - 0.02;
      points.push_back({x, y, -0.45 + 0.03 * x + 0.02 * y});
    }
  }
  const auto reference = analyzer.analyze(points, 0.0, 0.0);
  ASSERT_EQ(reference.support_surface.size(), points.size());
  std::map<std::pair<double, double>, double> reference_heights;
  for (const Point & support : reference.support_surface) {
    reference_heights[{support.x, support.y}] = support.z;
  }
  for (const auto & center : std::vector<std::pair<double, double>>{
      {0.07, 0.07}, {-0.07, -0.07}, {0.27, -0.33}, {-0.31, 0.23}})
  {
    SCOPED_TRACE(::testing::Message() << center.first << ", " << center.second);
    const auto result = analyzer.analyze(points, center.first, center.second);
    expectConsistentDiagnostics(result);
    ASSERT_EQ(result.support_surface.size(), reference_heights.size());
    for (const Point & support : result.support_surface) {
      const auto found = reference_heights.find({support.x, support.y});
      ASSERT_NE(found, reference_heights.end());
      EXPECT_NEAR(support.z, found->second, 1.0e-9);
    }
  }
}

TEST(TerrainAnalyzer, FixedWorldGridUsesActualRobotCenterForSyntheticClearance)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.synthetic_obstacle_clearance_radius = 0.25;
  std::vector<Point> points;
  for (int ix = -4; ix <= 4; ++ix) {
    for (int iy = -4; iy <= 4; ++iy) {
      points.push_back({ix * 0.2, iy * 0.2, ix * 0.2 * 0.4});
    }
  }
  constexpr double robot_x = 0.07;
  constexpr double robot_y = 0.07;
  const auto result = TerrainAnalyzer(parameters).analyze(points, robot_x, robot_y);
  expectConsistentDiagnostics(result);
  std::size_t expected_suppressed = 0;
  bool found_off_grid_near_cell = false;
  for (const Point & unsafe : result.non_traversable) {
    if (std::hypot(unsafe.x - robot_x, unsafe.y - robot_y) <= 0.25) {
      ++expected_suppressed;
    }
    if (std::abs(unsafe.x - 0.2) < 1.0e-9 &&
      std::abs(unsafe.y - 0.2) < 1.0e-9)
    {
      // This cell is outside clearance around the snapped origin (0,0),
      // but inside clearance around the actual robot position.
      found_off_grid_near_cell = true;
    }
  }
  ASSERT_TRUE(found_off_grid_near_cell);
  EXPECT_GT(expected_suppressed, 0u);
  EXPECT_EQ(result.suppressed_near_synthetic_obstacles, expected_suppressed);
  for (const TerrainPoint & obstacle : result.synthetic_obstacles) {
    EXPECT_GT(std::hypot(obstacle.point.x - robot_x, obstacle.point.y - robot_y), 0.25);
  }
}

TEST(TerrainAnalyzer, IsolatedAboveGroundReturnIsRejected)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  parameters.minimum_obstacle_cluster_points = 3;
  TerrainAnalyzer analyzer(parameters);
  std::vector<Point> points;
  for (int ix = -10; ix <= 10; ++ix) {
    for (int iy = -10; iy <= 10; ++iy) {
      points.push_back({ix * 0.2, iy * 0.2, 0.0});
    }
  }
  points.push_back({2.0, 0.0, 0.7});

  const auto result = analyzer.analyze(points, 0.0, 0.0);
  EXPECT_TRUE(result.measured_obstacles.empty());
}

TEST(TerrainHazardStabilizer, MeasuredHazardUsesEntryAndExitHysteresis)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 2;
  parameters.synthetic_enter_observations = 3;
  parameters.exit_missed_observations = 3;
  TerrainHazardStabilizer stabilizer(parameters);
  const std::vector<TerrainPoint> measured{
    {{1.0, 0.0, 0.5}, 0.5, false}};

  EXPECT_TRUE(stabilizer.update(measured, {}, 0.0, 0.0).empty());
  EXPECT_EQ(stabilizer.update(measured, {}, 0.0, 0.0).size(), 1u);
  EXPECT_EQ(stabilizer.update({}, {}, 0.0, 0.0).size(), 1u);
  EXPECT_EQ(stabilizer.update({}, {}, 0.0, 0.0).size(), 1u);
  EXPECT_TRUE(stabilizer.update({}, {}, 0.0, 0.0).empty());
}

TEST(TerrainHazardStabilizer, SyntheticHazardRequiresMoreEvidence)
{
  HazardStabilizerParameters parameters;
  parameters.measured_enter_observations = 2;
  parameters.synthetic_enter_observations = 3;
  TerrainHazardStabilizer stabilizer(parameters);
  const std::vector<TerrainPoint> synthetic{
    {{1.0, 0.0, 0.15}, 0.15, true}};

  EXPECT_TRUE(stabilizer.update({}, synthetic, 0.0, 0.0).empty());
  EXPECT_TRUE(stabilizer.update({}, synthetic, 0.0, 0.0).empty());
  const auto stable = stabilizer.update({}, synthetic, 0.0, 0.0);
  ASSERT_EQ(stable.size(), 1u);
  EXPECT_TRUE(stable.front().synthetic);
}

TEST(TerrainHazardStabilizer, MovingNearRetainedSyntheticClearsItWithoutClearingMeasured)
{
  HazardStabilizerParameters parameters;
  parameters.synthetic_clearance_radius = .65;
  parameters.synthetic_enter_observations = 1;
  parameters.measured_enter_observations = 1;
  TerrainHazardStabilizer stabilizer(parameters);
  const std::vector<TerrainPoint> synthetic{{{1, 0, .15}, .15, true}};
  const std::vector<TerrainPoint> measured{{{1, .2, .5}, .5, false}};
  ASSERT_EQ(stabilizer.update(measured, synthetic, 0, 0).size(), 2u);
  const auto remaining = stabilizer.update(measured, {}, .5, 0);
  ASSERT_EQ(remaining.size(), 1u);
  EXPECT_FALSE(remaining.front().synthetic);
  const auto repeated = stabilizer.update(measured, synthetic, .5, 0);
  ASSERT_EQ(repeated.size(), 1u);
  EXPECT_FALSE(repeated.front().synthetic);
}

TEST(TerrainAnalyzerRobustFit, SparseRecordedLowerLayerIsNotTiltedByHighCells)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.maximum_step_height = 0.08;
  // Independent recorded cells from ground_fit_trace.json. Elevated cells
  // used to pull OLS to 18.78 degrees and reject the observed origin floor.
  const std::vector<Point> points{
    {-.4, -.4, -.3493969738}, {-.4, 0, -.3442369998},
    {-.2, -.4, -.3457682729}, {-.2, -.2, -.3452078700},
    {-.2, .2, -.3445441723}, {-.2, .4, -.3481375873},
    {0, -.4, -.3273055255}, {0, -.2, -.3494176865},
    {0, 0, -.3469210565}, {.2, -.4, -.0741908252},
    {.2, -.2, -.3360432386}, {.2, 0, -.3254466057},
    {.2, .2, -.3440775275}, {.4, -.4, -.0936529562},
    {.4, -.2, .3690049052}, {.4, .4, -.3425042331}};
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0);
  expectConsistentDiagnostics(result);
  bool found_origin = false;
  for (const Point & point : result.support_surface) {
    if (std::hypot(point.x, point.y) < 1.0e-9) {
      found_origin = true;
      EXPECT_NEAR(point.z, -.344, .015);
    }
  }
  EXPECT_TRUE(found_origin);
}

TEST(TerrainAnalyzerRobustFit, SixDegreeRampSurvivesSparseHighCellContamination)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.maximum_step_height = .08;
  const double slope = std::tan(6.0 * std::acos(-1.0) / 180.0);
  std::vector<Point> points;
  for (int ix = -8; ix <= 8; ++ix) {
    for (int iy = -8; iy <= 8; ++iy) {
      // Missing floor in occasional occupied cells represents occlusion,
      // rather than inventing floor samples underneath the raised surface.
      const bool high = (ix + 20) % 5 == 0 && (iy + 20) % 3 == 0;
      points.push_back({ix * .2, iy * .2, slope * ix * .2 + (high ? .5 : 0.0)});
    }
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0);
  expectConsistentDiagnostics(result);
  // The occupied cells and their immediate step-border neighbors stay blocked.
  ASSERT_GT(result.support_surface.size(), 190u);
  for (const Point & point : result.support_surface) {
    EXPECT_NEAR(point.z, slope * point.x, .015);
  }
}

TEST(TerrainAnalyzerRobustFit, RealSteepRampIsNotFlattenedByConsensus)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  const double slope = std::tan(15.0 * std::acos(-1.0) / 180.0);
  std::vector<Point> points;
  for (int ix = -6; ix <= 6; ++ix) {
    for (int iy = -6; iy <= 6; ++iy) {
      points.push_back({ix * .2, iy * .2, slope * ix * .2});
    }
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0);
  expectConsistentDiagnostics(result);
  EXPECT_TRUE(result.support_surface.empty());
  EXPECT_EQ(result.diagnostics.slope_exceeded_cells, points.size());
  EXPECT_FALSE(result.synthetic_obstacles.empty());
}

TEST(TerrainAnalyzerRobustFit, HighReturnsCannotSupplyMissingGroundConfidence)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 5;
  std::vector<Point> points{{0, 0, 0}, {.2, 0, 0}, {0, .2, 0}, {.2, .2, 0}};
  for (int i = 0; i < 20; ++i) {
    points.push_back({0, 0, .8});
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0);
  expectConsistentDiagnostics(result);
  EXPECT_TRUE(result.support_surface.empty());
  EXPECT_EQ(result.diagnostics.insufficient_support_cells, 4u);
}

TEST(TerrainAnalyzerRobustFit, OccludedCenterHasNoInventedSupport)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  std::vector<Point> points;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      if (ix != 0 || iy != 0) {
        points.push_back({ix * .2, iy * .2, 0});
      }
    }
  }
  for (int i = 0; i < 4; ++i) {
    points.push_back({0, 0, .6});
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0);
  expectConsistentDiagnostics(result);
  for (const Point & point : result.support_surface) {
    EXPECT_GT(std::hypot(point.x, point.y), 1.0e-9);
  }
  ASSERT_EQ(result.measured_obstacles.size(), 4u);
  EXPECT_GT(result.diagnostics.step_exceeded_cells, 0u);
}

TEST(TerrainAnalyzerRobustFit, UnobservedStripAndRealStepAreNotFilled)
{
  AnalyzerParameters parameters;
  parameters.maximum_step_height = .08;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.minimum_support_points = 3;
  std::vector<Point> points;
  for (int ix = -8; ix <= 8; ++ix) {
    for (int iy = -8; iy <= 8; ++iy) {
      if (iy == 4 || iy == 5) {
        continue;
      }
      points.push_back({ix * .2, iy * .2, ix >= 0 ? .12 : 0});
    }
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0);
  expectConsistentDiagnostics(result);
  EXPECT_FALSE(result.non_traversable.empty());
  for (const Point & point : result.support_surface) {
    EXPECT_FALSE(std::abs(point.y - .8) < 1.0e-9 || std::abs(point.y - 1.0) < 1.0e-9);
    EXPECT_FALSE(std::abs(point.x) < 1.0e-9 || std::abs(point.x + .2) < 1.0e-9);
  }
}

TEST(TerrainAnalyzerRobustFit, EightCentimeterLimitStillRejectsPositiveAndNegativeSteps)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  parameters.maximum_step_height = .08;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  for (double step : {.081, -.081, .30, -.30}) {
    SCOPED_TRACE(step);
    std::vector<Point> points;
    for (int ix = -6; ix <= 6; ++ix) {
      for (int iy = -6; iy <= 6; ++iy) {
        points.push_back({ix * .2, iy * .2, ix < 0 ? 0 : step});
      }
    }
    const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0);
    expectConsistentDiagnostics(result);
    EXPECT_FALSE(result.non_traversable.empty());
    for (int iy = -6; iy <= 6; ++iy) {
      bool lower_side = false;
      bool upper_side = false;
      for (const Point & point : result.support_surface) {
        if (std::abs(point.y - iy * .2) < 1.0e-9) {
          lower_side = lower_side || std::abs(point.x + .2) < 1.0e-9;
          upper_side = upper_side || std::abs(point.x) < 1.0e-9;
        }
      }
      // A valid surface on one side may remain; no adjacent support pair
      // can cross the observed discontinuity in either direction.
      EXPECT_FALSE(lower_side && upper_side);
    }
  }
}

TEST(TerrainAnalyzerRobustFit, LowerCenterReturnIsNotReplacedByUnobservedFloor)
{
  AnalyzerParameters parameters;
  parameters.minimum_support_points = 3;
  parameters.maximum_step_height = .08;
  std::vector<Point> points;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      points.push_back({ix * .2, iy * .2, ix == 0 && iy == 0 ? -.30 : 0});
    }
  }
  const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0);
  expectConsistentDiagnostics(result);
  EXPECT_FALSE(result.non_traversable.empty());
  for (const Point & point : result.support_surface) {
    EXPECT_GT(std::hypot(point.x, point.y), 1.0e-9);
  }
}

TEST(TerrainAnalyzerRobustFit, HighSpatialExtremesCannotHideMajorityGroundConsensus)
{
  AnalyzerParameters parameters;
  parameters.maximum_slope = std::tan(10.0 * std::acos(-1.0) / 180.0);
  parameters.maximum_step_height = .08;
  const std::vector<std::pair<int, int>> high_cells{
    {-2, -2}, {-2, 2}, {2, -2}, {2, 2}, {-2, 0}, {0, -2}, {0, 2}};
  for (double slope : {0.0, std::tan(6.0 * std::acos(-1.0) / 180.0)}) {
    SCOPED_TRACE(slope);
    std::vector<Point> points;
    for (int ix = -2; ix <= 2; ++ix) {
      for (int iy = -2; iy <= 2; ++iy) {
        bool high = false;
        for (const auto & cell : high_cells) {
          high = high || (cell.first == ix && cell.second == iy);
        }
        points.push_back({ix * .2, iy * .2, slope * ix * .2 + (high ? .5 : 0.0)});
      }
    }
    const auto result = TerrainAnalyzer(parameters).analyze(points, 0, 0);
    expectConsistentDiagnostics(result);
    bool found_origin = false;
    for (const Point & point : result.support_surface) {
      if (std::hypot(point.x, point.y) < 1.0e-9) {
        found_origin = true;
        EXPECT_NEAR(point.z, 0.0, 1.0e-8);
      }
    }
    EXPECT_TRUE(found_origin);
  }
}
