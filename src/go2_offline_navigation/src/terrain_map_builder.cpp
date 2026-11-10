#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <terrain_costmap/terrain_analyzer.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

// Offline-only executable: fits the same terrain model on overlapping tiles.
// Only observed cells enter the support map. No interpolation fills blind areas.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("terrain_costmap");
    const auto input = node->declare_parameter<std::string>("input_map", "");
    const auto output = node->declare_parameter<std::string>("output_directory", "");
    terrain_costmap::AnalyzerParameters p;
    p.resolution = node->declare_parameter("grid_resolution", 0.2);
    p.radius = node->declare_parameter("grid_radius", 5.0);
    p.fit_radius_cells = node->declare_parameter("fit_radius_cells", 2);
    p.minimum_fit_cells = node->declare_parameter("minimum_fit_cells", 3);
    p.minimum_support_points = node->declare_parameter("minimum_support_points", 5);
    p.ground_height_percentile = node->declare_parameter("ground_height_percentile", 0.2);
    p.positive_outlier_tolerance = node->declare_parameter("positive_outlier_tolerance", 0.1);
    p.maximum_plane_rmse = node->declare_parameter("maximum_plane_rmse", 0.08);
    p.maximum_slope = std::tan(node->declare_parameter("maximum_slope_degrees", 10.0) *
      3.141592653589793 / 180.0);
    p.maximum_step_height = node->declare_parameter("maximum_step_height", 0.08);
    p.obstacle_minimum_height = node->declare_parameter("obstacle_minimum_height", 0.15);
    p.obstacle_maximum_height = node->declare_parameter("obstacle_maximum_height", 1.8);
    p.minimum_obstacle_cluster_points = node->declare_parameter("minimum_obstacle_cluster_points", 3);
    p.obstacle_cluster_radius_cells = node->declare_parameter("obstacle_cluster_radius_cells", 1);
    p.recover_sparse_ground_layers = node->declare_parameter("recover_sparse_ground_layers", false);
    // Tile centers are not robot positions: never erase hazards around them.
    p.synthetic_obstacle_clearance_radius = 0.0;
    if (input.empty() || output.empty() || !std::filesystem::is_directory(output)) {
      throw std::runtime_error("input_map and an existing staging output_directory are required");
    }
    if (!std::isfinite(p.resolution) || !std::isfinite(p.radius) || p.resolution < 0.02 ||
      p.radius > 20.0 || p.radius < (p.fit_radius_cells + 3) * p.resolution) {
      throw std::runtime_error("Invalid or unbounded tile geometry");
    }
    terrain_costmap::TerrainAnalyzer analyzer(p);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(input, cloud) < 0 || cloud.empty() || cloud.size() > 10000000) {
      throw std::runtime_error("Cannot load nonempty map PCD (limit 10000000 points)");
    }
    const int half = static_cast<int>(std::ceil(p.radius / p.resolution));
    if (half > 250) {throw std::runtime_error("Tile grid exceeds the bounded processing size");}
    const int margin = p.fit_radius_cells + p.obstacle_cluster_radius_cells + 2;
    const int inner = 2 * (half - margin);
    if (inner < 2) {throw std::runtime_error("Tile halo leaves no interior");}
    using Key = std::pair<int, int>;
    std::map<Key, std::vector<terrain_costmap::Point>> buckets;
    const auto owner = [inner](double coordinate, double resolution) {
      const auto cell = static_cast<int>(std::floor(coordinate / resolution + 0.5));
      return static_cast<int>(std::floor(static_cast<double>(cell) / inner));
    };
    for (const auto &point : cloud) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        std::abs(point.x) > 100000.0 || std::abs(point.y) > 100000.0 || std::abs(point.z) > 100000.0) {
        throw std::runtime_error("Map contains non-finite or unreasonable coordinates");
      }
      buckets[{owner(point.x, p.resolution), owner(point.y, p.resolution)}].push_back(
        {point.x, point.y, point.z});
    }
    std::ofstream support(std::filesystem::path(output) / "support.csv");
    std::ofstream obstacles(std::filesystem::path(output) / "obstacles.csv");
    std::ofstream sources(std::filesystem::path(output) / "hazard_sources.csv");
    std::ofstream diagnostics(std::filesystem::path(output) / "terrain_tiles.csv");
    std::ofstream cells(std::filesystem::path(output) / "terrain_cells.csv");
    support << "x,y,z\n" << std::setprecision(12);
    obstacles << "x,y,z\n" << std::setprecision(12);
    sources << "x,y,z,source\n" << std::setprecision(12);
    cells << "x,y,surface_z,minimum_z,maximum_z,plane_z,slope,rmse,support_returns,"
      "returns,fitted,observed_ground,synthetic_hazard,recovered_layer,outcome\n" << std::setprecision(12);
    diagnostics << "tile_x,tile_y,observed_cells,unobserved_cells,insufficient_fit_cells,"
      "degenerate_fit_cells,insufficient_support_cells,slope_exceeded_cells,"
      "rmse_exceeded_cells,step_exceeded_cells,traversable_cells,"
      "owned_support_points,owned_measured_hazards,owned_synthetic_hazards\n";
    size_t support_count = 0, obstacle_count = 0;
    const auto write = [](std::ofstream &file, const terrain_costmap::Point &point) {
      file << point.x << ',' << point.y << ',' << point.z << '\n';
    };
    for (const auto &bucket : buckets) {
      const int tx = bucket.first.first, ty = bucket.first.second;
      const double cx = (tx * inner + inner / 2) * p.resolution;
      const double cy = (ty * inner + inner / 2) * p.resolution;
      std::vector<terrain_costmap::Point> tile;
      // Include neighboring buckets as a halo; fitting is independent of tile borders.
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          auto neighbor = buckets.find({tx + dx, ty + dy});
          if (neighbor == buckets.end()) {continue;}
          for (const auto &point : neighbor->second) {
            if (std::abs(point.x - cx) <= p.radius + p.resolution &&
              std::abs(point.y - cy) <= p.radius + p.resolution) {tile.push_back(point);}
          }
        }
      }
      const auto result = analyzer.analyze(tile, cx, cy, true);
      const auto owned = [&](const terrain_costmap::Point &point) {
        return owner(point.x, p.resolution) == tx && owner(point.y, p.resolution) == ty;
      };
      size_t owned_support = 0, owned_measured = 0, owned_synthetic = 0;
      for (const auto & cell : result.cell_diagnostics) {
        if (!owned(cell.center)) {continue;}
        cells << cell.center.x << ',' << cell.center.y << ',' << cell.center.z << ',' <<
          cell.minimum_z << ',' << cell.maximum_z << ',' << cell.plane_z << ',' <<
          cell.slope << ',' << cell.rmse << ',' << cell.support_returns << ',' <<
          cell.returns << ',' << cell.fitted << ',' << cell.observed_ground << ',' <<
          cell.synthetic_hazard << ',' << cell.recovered_layer << ',' << cell.outcome << '\n';
      }
      for (const auto &point : result.support_surface) {
        if (owned(point)) {write(support, point); ++support_count; ++owned_support;}
      }
      for (const auto &point : result.obstacles) {
        if (owned(point.point)) {
          write(obstacles, point.point); ++obstacle_count;
          sources << point.point.x << ',' << point.point.y << ',' << point.point.z << ',' <<
            (point.synthetic ? "synthetic" : "measured") << '\n';
          if (point.synthetic) {++owned_synthetic;} else {++owned_measured;}
        }
      }
      const auto &d = result.diagnostics;
      // Analyzer cell counts include this tile's fitting halo; do not sum
      // them as unique global cells. Only owned_* export counts are disjoint.
      diagnostics << tx << ',' << ty << ',' << d.observed_cells << ',' << d.unobserved_cells <<
        ',' << d.insufficient_fit_cells << ',' << d.degenerate_fit_cells <<
        ',' << d.insufficient_support_cells << ',' << d.slope_exceeded_cells <<
        ',' << d.rmse_exceeded_cells << ',' << d.step_exceeded_cells <<
        ',' << d.traversable_cells << ',' << owned_support << ',' << owned_measured <<
        ',' << owned_synthetic << '\n';
    }
    support.close(); obstacles.close(); sources.close(); diagnostics.close(); cells.close();
    if (!support || !obstacles || !sources || !diagnostics || !cells || support_count == 0) {
      throw std::runtime_error("No supported ground or output write failed; no map exported");
    }
    RCLCPP_INFO(node->get_logger(), "Built %zu support points and %zu static hazards across %zu tiles",
      support_count, obstacle_count, buckets.size());
    rclcpp::shutdown(); return 0;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("terrain_map_builder"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
}
