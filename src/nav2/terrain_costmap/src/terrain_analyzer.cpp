#include "terrain_costmap/terrain_analyzer.hpp"

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace terrain_costmap
{
namespace
{

enum class FitResult {Valid, InsufficientCells, Degenerate};
enum class Rejection {None, InsufficientFit, DegenerateFit, InsufficientSupport, Slope, Rmse};

struct Cell
{
  double surface_z{std::numeric_limits<double>::infinity()};
  double observed_minimum_z{std::numeric_limits<double>::infinity()};
  std::vector<double> height_samples;
  bool plane_valid{false};
  bool observed_non_traversable{false};
  Eigen::Vector3d plane{Eigen::Vector3d::Zero()};
  bool observed_ground{false};
  double observed_ground_z{std::numeric_limits<double>::infinity()};
  Rejection rejection{Rejection::None};
  bool fitted{false};
  int fit_support{0};
  double fit_rmse{0.0};
  double fit_slope{0.0};
  bool recovered_layer{false};
};

// A small deterministic consensus fit. Each occupied grid cell contributes one
// height, so a dense wall or person cannot win by contributing more returns.
// The fast path handles ordinary flat/sloping kernels without any hypotheses.
FitResult solvePlane(
  const std::vector<Cell> & cells, int width, int center_x, int center_y,
  int radius, double resolution, double positive_outlier_tolerance,
  int minimum_cells, Eigen::Vector3d & plane, int & support, double & rmse)
{
  struct Sample {Eigen::Vector3d xy1; const Cell * cell;};
  std::vector<Sample> samples;
  samples.reserve(static_cast<std::size_t>((2 * radius + 1) * (2 * radius + 1)));
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      const int x = center_x + dx;
      const int y = center_y + dy;
      if (x < 0 || x >= width || y < 0 || y >= width) {
        continue;
      }
      const Cell & cell = cells[static_cast<std::size_t>(x * width + y)];
      if (std::isfinite(cell.surface_z)) {
        samples.push_back({Eigen::Vector3d(dx * resolution, dy * resolution, 1.0), &cell});
      }
    }
  }
  if (samples.size() < static_cast<std::size_t>(minimum_cells)) {
    return FitResult::InsufficientCells;
  }
  // This tolerance selects a coherent layer. The existing RMSE/slope/step
  // limits still determine traversability afterwards; no slope is assumed.
  const double tolerance = std::max(1.0e-6, 0.5 * positive_outlier_tolerance);
  const double tolerance_squared = tolerance * tolerance;
  auto fit = [&](const Eigen::Vector3d * reference, Eigen::Vector3d & result) {
      Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      int count = 0;
      for (const Sample & sample : samples) {
        if (reference != nullptr &&
          std::abs(sample.cell->surface_z - reference->dot(sample.xy1)) > tolerance)
        {
          continue;
        }
        normal.noalias() += sample.xy1 * sample.xy1.transpose();
        rhs.noalias() += sample.xy1 * sample.cell->surface_z;
        ++count;
      }
      if (count < minimum_cells || std::abs(normal.determinant()) < 1.0e-9) {
        return false;
      }
      result = normal.ldlt().solve(rhs);
      return result.allFinite();
    };
  if (!fit(nullptr, plane)) {
    return FitResult::Degenerate;
  }
  bool coherent = true;
  for (const Sample & sample : samples) {
    coherent = coherent &&
      std::abs(sample.cell->surface_z - plane.dot(sample.xy1)) <= tolerance;
  }
  bool use_consensus = false;
  if (!coherent) {
    const std::size_t required = std::max(
      static_cast<std::size_t>(minimum_cells), (samples.size() * 3 + 4) / 5);
    double best_cost = std::numeric_limits<double>::infinity();
    std::size_t best_count = 0;
    Eigen::Vector3d best_plane = plane;
    auto consider = [&](const Eigen::Vector3d & candidate) {
        double cost = 0.0;
        std::size_t count = 0;
        std::size_t below = 0;
        for (const Sample & sample : samples) {
          const double residual = sample.cell->surface_z - candidate.dot(sample.xy1);
          cost += std::min(residual * residual, tolerance_squared);
          count += std::abs(residual) <= tolerance;
          below += residual < -tolerance;
        }
        // Do not promote an upper surface over a substantial observed lower
        // layer, even when the upper surface contains more returns.
        if (count < required || below > std::max<std::size_t>(1, samples.size() / 5)) {
          return;
        }
        if (cost < best_cost - 1.0e-12 ||
          (std::abs(cost - best_cost) <= 1.0e-12 && count > best_count))
        {
          best_cost = cost;
          best_count = count;
          best_plane = candidate;
        }
      };
    consider(plane);
    // Restrict anchor selection to the lower residual layer of the initial
    // plane. Pure geometric extremes can all lie on raised obstacle tops,
    // hiding an otherwise dominant floor layer from every sampled triple.
    // Sorting residuals rather than absolute Z retains genuine sloping ground.
    std::vector<std::pair<double, std::size_t>> residual_order;
    residual_order.reserve(samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
      residual_order.emplace_back(
        samples[i].cell->surface_z - plane.dot(samples[i].xy1), i);
    }
    std::sort(residual_order.begin(), residual_order.end());
    // Farthest-point selection spreads at most eight anchors over this layer.
    // 56 triples remains a fixed per-kernel hypothesis budget.
    std::vector<std::size_t> anchors;
    std::size_t closest = residual_order.front().second;
    for (std::size_t index = 1; index < required; ++index) {
      const std::size_t i = residual_order[index].second;
      if (samples[i].xy1.head<2>().squaredNorm() <
        samples[closest].xy1.head<2>().squaredNorm())
      {
        closest = i;
      }
    }
    anchors.push_back(closest);
    std::vector<double> distance(samples.size(), std::numeric_limits<double>::infinity());
    while (anchors.size() < std::min<std::size_t>(8, required)) {
      const auto & last = samples[anchors.back()].xy1;
      std::size_t farthest = residual_order.front().second;
      for (std::size_t index = 0; index < required; ++index) {
        const std::size_t i = residual_order[index].second;
        distance[i] = std::min(distance[i], (samples[i].xy1 - last).squaredNorm());
        if (distance[i] > distance[farthest]) {
          farthest = i;
        }
      }
      anchors.push_back(farthest);
    }
    for (std::size_t i = 0; i < anchors.size(); ++i) {
      for (std::size_t j = i + 1; j < anchors.size(); ++j) {
        for (std::size_t k = j + 1; k < anchors.size(); ++k) {
          const Sample & p = samples[anchors[i]];
          const Sample & q = samples[anchors[j]];
          const Sample & r = samples[anchors[k]];
          const double dx1 = q.xy1.x() - p.xy1.x();
          const double dy1 = q.xy1.y() - p.xy1.y();
          const double dx2 = r.xy1.x() - p.xy1.x();
          const double dy2 = r.xy1.y() - p.xy1.y();
          const double area = dx1 * dy2 - dy1 * dx2;
          if (std::abs(area) < 1.0e-9) {
            continue;
          }
          const double dz1 = q.cell->surface_z - p.cell->surface_z;
          const double dz2 = r.cell->surface_z - p.cell->surface_z;
          Eigen::Vector3d candidate;
          candidate.x() = (dz1 * dy2 - dy1 * dz2) / area;
          candidate.y() = (dx1 * dz2 - dz1 * dx2) / area;
          candidate.z() = p.cell->surface_z - candidate.head<2>().dot(p.xy1.head<2>());
          consider(candidate);
        }
      }
    }
    if (std::isfinite(best_cost)) {
      plane = best_plane;
      for (int iteration = 0; iteration < 2; ++iteration) {
        Eigen::Vector3d refitted;
        if (!fit(&plane, refitted)) {
          return FitResult::Degenerate;
        }
        plane = refitted;
      }
      std::size_t final_count = 0;
      std::size_t final_below = 0;
      for (const Sample & sample : samples) {
        const double residual = sample.cell->surface_z - plane.dot(sample.xy1);
        final_count += std::abs(residual) <= tolerance;
        final_below += residual < -tolerance;
      }
      use_consensus = final_count >= required &&
        final_below <= std::max<std::size_t>(1, samples.size() / 5);
      if (!use_consensus && !fit(nullptr, plane)) {
        return FitResult::Degenerate;
      }
    }
  }
  // Count actual returns near the selected plane, rather than every high
  // return in a cell whose percentile happened to agree with the floor.
  support = 0;
  double squared_error = 0.0;
  int residual_count = 0;
  for (const Sample & sample : samples) {
    const double height = plane.dot(sample.xy1);
    const double residual = sample.cell->surface_z - height;
    if (!use_consensus || std::abs(residual) <= tolerance) {
      squared_error += residual * residual;
      ++residual_count;
    }
    for (double observed_height : sample.cell->height_samples) {
      support += std::abs(observed_height - height) <= tolerance;
    }
  }
  rmse = residual_count > 0 ? std::sqrt(squared_error / residual_count) :
    std::numeric_limits<double>::infinity();
  return FitResult::Valid;
}

// Sparse cells can select a lone below-floor return as their 20th percentile.
// Recover only a plane supported by actual returns in >=60% of independent
// cells, without promoting it above the center cell's lowest observation.
// No observations are inserted, and slope/RMSE/step gates still apply.
bool recoverObservedLayer(
  const std::vector<Cell> & cells, int width, int center_x, int center_y,
  const AnalyzerParameters & parameters, Eigen::Vector3d & plane, int & support,
  double & rmse)
{
  struct Sample {Eigen::Vector3d xy1; const Cell * cell;};
  std::vector<Sample> samples;
  std::vector<double> seeds;
  const int radius = parameters.fit_radius_cells;
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      const int x = center_x + dx, y = center_y + dy;
      if (x < 0 || x >= width || y < 0 || y >= width) {continue;}
      const auto & cell = cells[static_cast<std::size_t>(x * width + y)];
      if (cell.height_samples.empty()) {continue;}
      samples.push_back({{dx * parameters.resolution, dy * parameters.resolution, 1.0}, &cell});
      seeds.insert(seeds.end(), cell.height_samples.begin(), cell.height_samples.end());
    }
  }
  if (samples.size() < static_cast<std::size_t>(parameters.minimum_fit_cells)) {return false;}
  std::sort(seeds.begin(), seeds.end());
  const double tolerance = std::max(1.0e-6, .5 * parameters.positive_outlier_tolerance);
  const double center_minimum_z =
    cells[static_cast<std::size_t>(center_x * width + center_y)].observed_minimum_z;
  const std::size_t required = std::max(static_cast<std::size_t>(parameters.minimum_fit_cells),
    (samples.size() * 3 + 4) / 5);
  std::size_t best_count = 0;
  double best_cost = std::numeric_limits<double>::infinity();
  Eigen::Vector3d best = Eigen::Vector3d::Zero();
  const auto nearest = [](const Cell & cell, double predicted) {
      return *std::min_element(cell.height_samples.begin(), cell.height_samples.end(),
        [predicted](double a, double b) {return std::abs(a - predicted) < std::abs(b - predicted);});
    };
  const std::size_t hypotheses = std::min<std::size_t>(32, seeds.size());
  for (std::size_t index = 0; index < hypotheses; ++index) {
    const auto seed_index = hypotheses == 1 ? 0 : index * (seeds.size() - 1) / (hypotheses - 1);
    Eigen::Vector3d candidate(0, 0, seeds[seed_index]);
    bool valid = true;
    for (int iteration = 0; iteration < 4; ++iteration) {
      Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      int count = 0;
      for (const auto & sample : samples) {
        const double height = nearest(*sample.cell, candidate.dot(sample.xy1));
        if (std::abs(height - candidate.dot(sample.xy1)) > tolerance) {continue;}
        normal.noalias() += sample.xy1 * sample.xy1.transpose();
        rhs.noalias() += sample.xy1 * height;
        ++count;
      }
      if (count < parameters.minimum_fit_cells || std::abs(normal.determinant()) < 1.0e-9) {
        valid = false; break;
      }
      candidate = normal.ldlt().solve(rhs);
      if (!candidate.allFinite()) {valid = false; break;}
    }
    if (!valid || std::hypot(candidate.x(), candidate.y()) > parameters.maximum_slope) {continue;}
    // A neighboring upper layer cannot erase this cell's actual lower return.
    // Mixed layers remain ambiguous even when their upper plane has more votes.
    if (candidate.z() - center_minimum_z > tolerance) {continue;}
    std::size_t count = 0;
    double cost = 0.0;
    for (const auto & sample : samples) {
      const double residual = nearest(*sample.cell, candidate.dot(sample.xy1)) - candidate.dot(sample.xy1);
      if (std::abs(residual) <= tolerance) {++count; cost += residual * residual;}
    }
    if (count < required || std::sqrt(cost / count) > parameters.maximum_plane_rmse) {continue;}
    if (count > best_count || (count == best_count && (cost < best_cost - 1e-12 ||
        (std::abs(cost - best_cost) <= 1e-12 && candidate.z() < best.z()))))
    {best_count = count; best_cost = cost; best = candidate;}
  }
  if (best_count < required) {return false;}
  int recovered_support = 0;
  for (const auto & sample : samples) {
    const double height = best.dot(sample.xy1);
    for (double observed : sample.cell->height_samples) {
      recovered_support += std::abs(observed - height) <= tolerance;
    }
  }
  if (recovered_support < parameters.minimum_support_points) {return false;}
  plane = best; rmse = std::sqrt(best_cost / best_count); support = recovered_support;
  return true;
}

}  // namespace

TerrainAnalyzer::TerrainAnalyzer(const AnalyzerParameters & parameters)
: parameters_(parameters)
{
  if (parameters_.resolution <= 0.0 || parameters_.radius <= parameters_.resolution ||
    parameters_.fit_radius_cells < 1 || parameters_.minimum_fit_cells < 3 ||
    parameters_.ground_height_percentile < 0.0 ||
    parameters_.ground_height_percentile > 0.5 ||
    parameters_.minimum_obstacle_cluster_points < 1 ||
    parameters_.obstacle_cluster_radius_cells < 0 ||
    parameters_.synthetic_obstacle_clearance_radius < 0.0)
  {
    throw std::invalid_argument("invalid terrain analyzer parameters");
  }
}

Analysis TerrainAnalyzer::analyze(
  const std::vector<Point> & points, double center_x, double center_y,
  bool collect_diagnostics) const
{
  const int half_width =
    static_cast<int>(std::ceil(parameters_.radius / parameters_.resolution));
  const int width = 2 * half_width + 1;
  std::vector<Cell> cells(static_cast<std::size_t>(width * width));

  // Move the window over one fixed world grid. A continuously moving origin
  // changes cell membership and publishes interleaved old/new support centres
  // into the planner even when the observed world surface has not changed.
  const int center_cell_x = static_cast<int>(
    std::floor(center_x / parameters_.resolution + 0.5));
  const int center_cell_y = static_cast<int>(
    std::floor(center_y / parameters_.resolution + 0.5));
  auto cellIndex = [&](const Point & point, int & x, int & y) {
      x = static_cast<int>(
        std::floor(point.x / parameters_.resolution + 0.5)) -
        center_cell_x + half_width;
      y = static_cast<int>(
        std::floor(point.y / parameters_.resolution + 0.5)) -
        center_cell_y + half_width;
      return x >= 0 && x < width && y >= 0 && y < width;
    };
  auto worldCellCenter = [&](int x, int y) {
      return Point{
        (center_cell_x + x - half_width) * parameters_.resolution,
        (center_cell_y + y - half_width) * parameters_.resolution, 0.0};
    };

  for (const Point & point : points) {
    int x = 0;
    int y = 0;
    if (!cellIndex(point, x, y)) {
      continue;
    }
    Cell & cell = cells[static_cast<std::size_t>(x * width + y)];
    cell.height_samples.push_back(point.z);
    cell.observed_minimum_z = std::min(cell.observed_minimum_z, point.z);
  }

  for (Cell & cell : cells) {
    if (cell.height_samples.empty()) {
      continue;
    }
    const std::size_t percentile_index = std::min(
      cell.height_samples.size() - 1,
      static_cast<std::size_t>(
        std::floor(parameters_.ground_height_percentile *
        static_cast<double>(cell.height_samples.size()))));
    std::nth_element(
      cell.height_samples.begin(),
      cell.height_samples.begin() + static_cast<std::ptrdiff_t>(percentile_index),
      cell.height_samples.end());
    cell.surface_z = cell.height_samples[percentile_index];
  }

  // Recover a floor return below a dense obstacle top only when the percentile
  // height is above the surrounding floor. If the percentile already agrees
  // with its neighbors, recovering a lower outlier would undo its rejection.
  std::vector<double> percentile_heights;
  percentile_heights.reserve(cells.size());
  for (const Cell & cell : cells) {
    percentile_heights.push_back(cell.surface_z);
  }
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < width; ++y) {
      Cell & cell = cells[static_cast<std::size_t>(x * width + y)];
      if (!std::isfinite(cell.observed_minimum_z)) {
        continue;
      }
      std::vector<double> neighbor_heights;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= width) {
            continue;
          }
          const double height =
            percentile_heights[static_cast<std::size_t>(nx * width + ny)];
          if (std::isfinite(height)) {
            neighbor_heights.push_back(height);
          }
        }
      }
      if (neighbor_heights.size() < 3) {
        continue;
      }
      const auto middle =
        neighbor_heights.begin() +
        static_cast<std::ptrdiff_t>(neighbor_heights.size() / 2);
      std::nth_element(
        neighbor_heights.begin(), middle, neighbor_heights.end());
      if (cell.surface_z - *middle > parameters_.positive_outlier_tolerance &&
        std::abs(cell.observed_minimum_z - *middle) <=
        parameters_.positive_outlier_tolerance)
      {
        cell.surface_z = cell.observed_minimum_z;
      }
    }
  }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < width; ++y) {
      Cell & cell = cells[static_cast<std::size_t>(x * width + y)];
      if (!std::isfinite(cell.surface_z)) {
        continue;
      }
      Eigen::Vector3d plane;
      int support = 0;
      double rmse = 0.0;
      const FitResult fit_result = solvePlane(
          cells, width, x, y, parameters_.fit_radius_cells,
          parameters_.resolution, parameters_.positive_outlier_tolerance,
          parameters_.minimum_fit_cells, plane, support, rmse);
      if (fit_result != FitResult::Valid) {
        cell.rejection = fit_result == FitResult::InsufficientCells ?
          Rejection::InsufficientFit : Rejection::DegenerateFit;
        continue;
      }

      if (parameters_.recover_sparse_ground_layers &&
        (std::hypot(plane.x(), plane.y()) > parameters_.maximum_slope ||
        rmse > parameters_.maximum_plane_rmse))
      {
        cell.recovered_layer = recoverObservedLayer(
          cells, width, x, y, parameters_, plane, support, rmse);
      }
      const double slope = std::hypot(plane.x(), plane.y());
      cell.fitted = true;
      cell.plane = plane;
      cell.fit_support = support;
      cell.fit_rmse = rmse;
      cell.fit_slope = slope;
      const double ground_tolerance = std::max(1.0e-6, 0.5 * parameters_.positive_outlier_tolerance);
      double closest_ground_residual = std::numeric_limits<double>::infinity();
      for (const double height : cell.height_samples) {
        const double residual = std::abs(height - plane.z());
        if (residual <= ground_tolerance &&
          (residual < closest_ground_residual ||
          (residual == closest_ground_residual && height < cell.observed_ground_z)))
        {
          cell.observed_ground = true;
          cell.observed_ground_z = height;
          closest_ground_residual = residual;
        }
      }
      if (support >= parameters_.minimum_support_points &&
        slope <= parameters_.maximum_slope &&
        rmse <= parameters_.maximum_plane_rmse)
      {
        cell.plane = plane;
        cell.plane_valid = true;
        if (!cell.observed_ground) {
          // A nearby plane is useful to classify real object returns, but
          // cannot invent a floor observation underneath an occluding object.
          cell.rejection = Rejection::InsufficientSupport;
        }
      } else if (support >= parameters_.minimum_support_points) {
        cell.observed_non_traversable = true;
        cell.rejection = slope > parameters_.maximum_slope ?
          Rejection::Slope : Rejection::Rmse;
      } else {
        cell.rejection = Rejection::InsufficientSupport;
      }
    }
  }

  // A locally valid plane can still be separated from its neighbor by a
  // step. Mark both confirmed ground sides of a discontinuity as unsafe.
  // A high return in an occluded cell is not a second observed ground layer:
  // keep that raised cell unsafe without turning adjacent measured floor into
  // an artificial obstacle. Unknown downward drops remain conservative.
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < width; ++y) {
      Cell & cell = cells[static_cast<std::size_t>(x * width + y)];
      if (!cell.plane_valid) {
        continue;
      }
      for (const auto & offset :
        {std::pair<int, int>{1, 0}, std::pair<int, int>{0, 1}})
      {
        const int nx = x + offset.first;
        const int ny = y + offset.second;
        if (nx >= width || ny >= width) {
          continue;
        }
        Cell & neighbor = cells[static_cast<std::size_t>(nx * width + ny)];
        const double observed_height = cell.observed_ground ?
          cell.observed_ground_z : cell.surface_z;
        const double neighbor_height = neighbor.observed_ground ?
          neighbor.observed_ground_z : neighbor.surface_z;
        if (neighbor.plane_valid &&
          std::abs(observed_height - neighbor_height) >
          parameters_.maximum_step_height)
        {
          const bool raised_occluded_neighbor = cell.observed_ground &&
            !neighbor.observed_ground && neighbor_height > observed_height;
          const bool raised_occluded_cell = neighbor.observed_ground &&
            !cell.observed_ground && observed_height > neighbor_height;
          if (!raised_occluded_neighbor) {
            cell.observed_non_traversable = true;
            cell.rejection = Rejection::None;
          }
          if (!raised_occluded_cell) {
            neighbor.observed_non_traversable = true;
            neighbor.rejection = Rejection::None;
          }
        }
      }
    }
  }

  Analysis analysis;
  analysis.grid_width = static_cast<std::size_t>(width);
  analysis.supported_cells.resize(cells.size(), 0);
  for (std::size_t i = 0; i < cells.size(); ++i) {
    const bool supported =
      cells[i].plane_valid && cells[i].observed_ground && !cells[i].observed_non_traversable;
    analysis.supported_cells[i] = supported ? 1 : 0;
    if (!std::isfinite(cells[i].surface_z)) {
      ++analysis.diagnostics.unobserved_cells;
    } else {
      ++analysis.diagnostics.observed_cells;
      switch (cells[i].rejection) {
        case Rejection::InsufficientFit:
          ++analysis.diagnostics.insufficient_fit_cells;
          break;
        case Rejection::DegenerateFit:
          ++analysis.diagnostics.degenerate_fit_cells;
          break;
        case Rejection::InsufficientSupport:
          ++analysis.diagnostics.insufficient_support_cells;
          break;
        case Rejection::Slope:
          ++analysis.diagnostics.slope_exceeded_cells;
          break;
        case Rejection::Rmse:
          ++analysis.diagnostics.rmse_exceeded_cells;
          break;
        case Rejection::None:
          if (supported) {
            ++analysis.diagnostics.traversable_cells;
          } else {
            ++analysis.diagnostics.step_exceeded_cells;
          }
          break;
      }
    }
    const int x = static_cast<int>(i) / width;
    const int y = static_cast<int>(i) % width;
    const Point cell_center = worldCellCenter(x, y);
    if (collect_diagnostics && std::isfinite(cells[i].surface_z)) {
      const char * outcome = supported ? "traversable" : "step_exceeded";
      switch (cells[i].rejection) {
        case Rejection::InsufficientFit: outcome = "insufficient_fit"; break;
        case Rejection::DegenerateFit: outcome = "degenerate_fit"; break;
        case Rejection::InsufficientSupport: outcome = "insufficient_support"; break;
        case Rejection::Slope: outcome = "slope_exceeded"; break;
        case Rejection::Rmse: outcome = "rmse_exceeded"; break;
        case Rejection::None: break;
      }
      Analysis::CellDiagnostic diagnostic;
      diagnostic.center = {cell_center.x, cell_center.y, cells[i].surface_z};
      diagnostic.minimum_z = cells[i].observed_minimum_z;
      diagnostic.maximum_z = *std::max_element(
        cells[i].height_samples.begin(), cells[i].height_samples.end());
      diagnostic.plane_z = cells[i].plane.z();
      diagnostic.slope = cells[i].fit_slope;
      diagnostic.rmse = cells[i].fit_rmse;
      diagnostic.support_returns = cells[i].fit_support;
      diagnostic.returns = cells[i].height_samples.size();
      diagnostic.fitted = cells[i].fitted;
      diagnostic.observed_ground = cells[i].observed_ground;
      diagnostic.synthetic_hazard = cells[i].observed_non_traversable &&
        std::hypot(cell_center.x - center_x, cell_center.y - center_y) >
        parameters_.synthetic_obstacle_clearance_radius;
      diagnostic.recovered_layer = cells[i].recovered_layer;
      diagnostic.outcome = outcome;
      analysis.cell_diagnostics.push_back(diagnostic);
    }
    if (supported) {
      // At the center of this cell the local plane sample is [0, 0, 1],
      // hence plane.z() is the fitted support height.
      analysis.support_surface.push_back(
        {
          cell_center.x,
          cell_center.y,
          cells[i].plane.z()
        });
    }
    if (cells[i].observed_non_traversable &&
      std::isfinite(cells[i].surface_z))
    {
      analysis.non_traversable.push_back(
        {
          cell_center.x,
          cell_center.y,
          cells[i].surface_z
        });
    }
  }

  analysis.terrain.reserve(points.size());
  std::vector<TerrainPoint> measured_candidates;
  std::vector<int> obstacle_counts(cells.size(), 0);
  for (const Point & point : points) {
    int x = 0;
    int y = 0;
    if (!cellIndex(point, x, y)) {
      continue;
    }
    const Cell & cell = cells[static_cast<std::size_t>(x * width + y)];
    if (!cell.plane_valid) {
      continue;
    }
    const Point cell_center = worldCellCenter(x, y);
    const Eigen::Vector3d sample(
      point.x - cell_center.x, point.y - cell_center.y, 1.0);
    const double relative_height = point.z - cell.plane.dot(sample);
    // A point sampled from the fitted ground can be a few ulps below the
    // plane, and ARM/x86 produce slightly different signs for that residual.
    // Treat negative residuals inside the accepted plane RMSE as ground; only
    // a materially lower outlier is rejected.
    if (relative_height < -parameters_.maximum_plane_rmse ||
      relative_height > parameters_.obstacle_maximum_height)
    {
      continue;
    }
    analysis.terrain.push_back({point, std::max(0.0, relative_height)});
    if (relative_height >= parameters_.obstacle_minimum_height) {
      measured_candidates.push_back({point, relative_height, false});
      ++obstacle_counts[static_cast<std::size_t>(x * width + y)];
    }
  }

  for (const TerrainPoint & candidate : measured_candidates) {
    int x = 0;
    int y = 0;
    if (!cellIndex(candidate.point, x, y)) {
      continue;
    }
    int cluster_points = 0;
    for (int dx = -parameters_.obstacle_cluster_radius_cells;
      dx <= parameters_.obstacle_cluster_radius_cells; ++dx)
    {
      for (int dy = -parameters_.obstacle_cluster_radius_cells;
        dy <= parameters_.obstacle_cluster_radius_cells; ++dy)
      {
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx < 0 || nx >= width || ny < 0 || ny >= width) {
          continue;
        }
        cluster_points +=
          obstacle_counts[static_cast<std::size_t>(nx * width + ny)];
      }
    }
    if (cluster_points >= parameters_.minimum_obstacle_cluster_points) {
      analysis.measured_obstacles.push_back(candidate);
      analysis.obstacles.push_back(candidate);
    }
  }

  for (const Point & point : analysis.non_traversable) {
    Point obstacle_point = point;
    // Put synthetic unsafe-cell returns above the ground.  A planner that
    // checks body clearance must not mistake this marker for support.
    obstacle_point.z += parameters_.obstacle_minimum_height;
    const double distance_to_vehicle =
      std::hypot(point.x - center_x, point.y - center_y);
    if (distance_to_vehicle <=
      parameters_.synthetic_obstacle_clearance_radius)
    {
      ++analysis.suppressed_near_synthetic_obstacles;
      continue;
    }
    const TerrainPoint obstacle{
      obstacle_point, parameters_.obstacle_minimum_height, true};
    analysis.synthetic_obstacles.push_back(obstacle);
    analysis.obstacles.push_back(obstacle);
  }
  return analysis;
}

}  // namespace terrain_costmap
