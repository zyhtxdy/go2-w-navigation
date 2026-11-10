#include "astar_3d_planner/occupy_map.hpp"

#include <algorithm>
#include <cmath>

namespace astar_3d_planner
{
namespace
{
using XY = std::array<double, 2>;
double cross(const XY & a, const XY & b, const XY & c)
{return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]);}

std::vector<XY> convexHull(std::vector<XY> points)
{
  std::sort(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end()), points.end());
  if (points.size() < 3) {return {};}
  std::vector<XY> hull;
  for (const auto & p : points) {
    while (hull.size() >= 2 && cross(hull[hull.size()-2], hull.back(), p) <= 1e-12) {
      hull.pop_back();
    }
    hull.push_back(p);
  }
  const auto lower_size = hull.size();
  for (auto it = points.rbegin()+1; it != points.rend(); ++it) {
    while (hull.size() > lower_size && cross(hull[hull.size()-2], hull.back(), *it) <= 1e-12) {
      hull.pop_back();
    }
    hull.push_back(*it);
  }
  if (hull.size() < 4) {return {};}
  hull.pop_back();
  return hull;
}

bool covers(const std::vector<XY> & hull, const XY & position, double resolution)
{
  if (hull.size() < 3) {return false;}
  const double tolerance = resolution*1e-5;  // FLOAT32 raster transport only.
  for (std::size_t i=0; i<hull.size(); ++i) {
    const auto & a=hull[i]; const auto & b=hull[(i+1)%hull.size()];
    if (cross(a,b,position) < -tolerance*std::hypot(b[0]-a[0],b[1]-a[1])) {return false;}
  }
  return true;
}
}  // namespace

OccupyMap::InferredColumn OccupyMap::inferColumnUnlocked(
  const ColumnKey & column, double query_height) const
{
  InferredColumn result;
  result.query_height = query_height;
  if (!config_.support_inference_enabled) {return result;}
  std::lock_guard<std::mutex> lock(ground_cache_mutex_);
  const auto cached = ground_cache_.find(column);
  if (cached != ground_cache_.end()) {return cached->second;}
  if (ground_cache_.size() >= static_cast<std::size_t>(config_.support_inference_max_columns)) {
    result.reason = "inference_cache_limit";
    return result;  // Fail closed; no unbounded work inside a planning request.
  }

  struct Sample {double x, y, height; ColumnKey column;};
  std::vector<Sample> samples;
  const auto centre = keyToPos({column.x,column.y,0});
  const int cells = static_cast<int>(std::ceil(config_.support_inference_radius * inv_resolution_))+1;
  for (int dx = -cells; dx <= cells; ++dx) {
    for (int dy = -cells; dy <= cells; ++dy) {
      if (dx == 0 && dy == 0) {continue;}
      const ColumnKey neighbour{column.x + dx, column.y + dy};
      // Only actual measurements vote. Inferred cells never seed other cells.
      const auto measured = support_columns_.find(neighbour);
      if (measured == support_columns_.end()) {continue;}
      for (int z : measured->second) {
        std::array<double, 2> xy;
        if (!measuredSupportXYUnlocked({neighbour.x,neighbour.y,z},xy)) {continue;}
        const double x=xy[0]-centre[0], y=xy[1]-centre[1];
        if (std::hypot(x,y) > config_.support_inference_radius+1e-6) {continue;}
        samples.push_back({x, y, support_heights_.at({neighbour.x, neighbour.y, z}), neighbour});
      }
    }
  }
  std::sort(samples.begin(), samples.end(), [](const Sample & a, const Sample & b) {
      if (a.height != b.height) {return a.height < b.height;}
      if (a.column.x != b.column.x) {return a.column.x < b.column.x;}
      return a.column.y < b.column.y;
    });
  // Height bands remain separate. Never average a lower floor and an upper
  // surface, or cherry-pick just the flat side of a discontinuity.
  for (std::size_t first = 0; first < samples.size();) {
    std::size_t end = first + 1;
    while (end < samples.size() && samples[end].height - samples[end - 1].height <=
      config_.support_inference_max_height_span + 1e-9) {++end;}
    if (end - first < 3) {first = end; continue;}
    if (samples[end - 1].height - samples[first].height >
      config_.support_inference_max_height_span + 1e-9)
    {
      result.reason = "inconsistent_height_band"; first = end; continue;
    }
    std::unordered_set<ColumnKey, ColumnKeyHash> columns;
    std::vector<XY> geometry;
    bool ambiguous = false;
    double mx = 0, my = 0, mh = 0;
    for (std::size_t i = first; i < end; ++i) {
      const auto & s = samples[i];
      if (!columns.insert(s.column).second) {ambiguous = true; break;}
      mx += s.x; my += s.y; mh += s.height;
      geometry.push_back({s.x,s.y});
    }
    if (ambiguous) {result.reason = "ambiguous_ground_layers"; first = end; continue;}
    constexpr double pi = 3.14159265358979323846;
    auto polygon=convexHull(std::move(geometry));
    const double half=config_.resolution*.5;
    bool cell_covered=!polygon.empty();
    for (const XY corner : {XY{-half,-half},XY{-half,half},XY{half,-half},XY{half,half}}) {
      cell_covered=cell_covered && covers(polygon,corner,config_.resolution);
    }
    // Search and path resampling assume uniform coverage inside a column.
    // A centre-only fit can pass search, then fail after densification. Require
    // the entire cell, never a thin triangular fragment, before caching a layer.
    if (!cell_covered) {
      result.reason = "not_surrounded_by_support"; first = end; continue;
    }
    const double n = static_cast<double>(end - first);
    mx /= n; my /= n; mh /= n;
    double xx = 0, xy = 0, yy = 0, xh = 0, yh = 0;
    for (std::size_t i = first; i < end; ++i) {
      const auto & s = samples[i];
      const double x = s.x - mx, y = s.y - my, h = s.height - mh;
      xx += x*x; xy += x*y; yy += y*y; xh += x*h; yh += y*h;
    }
    const double determinant = xx*yy - xy*xy;
    if (determinant <= 1e-12) {
      result.reason = "degenerate_support_geometry"; first = end; continue;
    }
    const double a = (xh*yy - yh*xy) / determinant;
    const double b = (yh*xx - xh*xy) / determinant;
    const double height = mh - a*mx - b*my;
    if (std::hypot(a, b) > std::tan(config_.support_inference_max_slope_deg*pi/180) + 1e-9) {
      result.reason = "inference_slope_limit"; first = end; continue;
    }
    double residual = 0;
    for (std::size_t i = first; i < end; ++i) {
      const auto & s = samples[i];
      residual = std::max(residual, std::abs(s.height - (height + a*s.x + b*s.y)));
    }
    if (residual > config_.support_inference_max_residual + 1e-9) {
      result.reason = "inference_residual_limit"; first = end; continue;
    }
    // A nearby height discontinuity must not disappear merely because height
    // clustering left its observation in a small adjacent band. Distant floors
    // remain independent; near-layer contradictory observations veto the fit.
    bool contradicted=false;
    const double near_layer = config_.support_inference_max_height_span+config_.resolution;
    for (std::size_t i=0; i<samples.size(); ++i) {
      if (i>=first && i<end) {continue;}
      const double error=std::abs(samples[i].height-(height+a*samples[i].x+b*samples[i].y));
      if (error>config_.support_inference_max_residual+1e-9 && error<=near_layer+1e-9) {
        contradicted=true; break;
      }
    }
    if (contradicted) {
      result.reason="inconsistent_nearby_ground_layer"; first=end; continue;
    }
    const int z = posToKey(0, 0, height + height_datum_offset_).z;
    result.layers.push_back({z, height});
    first = end;
  }
  if (!result.layers.empty()) {result.reason = "inferred_enclosed_ground";}
  ground_cache_.emplace(column, result);
  return result;
}

std::vector<int> OccupyMap::groundLayersUnlocked(
  const ColumnKey & column, double query_height) const
{
  const auto measured = support_columns_.find(column);
  if (measured != support_columns_.end()) {return measured->second;}
  std::vector<int> layers;
  for (const auto & layer : inferColumnUnlocked(column, query_height).layers) {
    layers.push_back(layer.z);
  }
  return layers;
}

bool OccupyMap::inferredSupportUnlocked(const VoxelKey & ground) const
{
  if (support_heights_.count(ground) || support_columns_.count({ground.x, ground.y})) {return false;}
  const auto column = inferColumnUnlocked({ground.x, ground.y}, keyToPos(ground)[2]);
  return std::any_of(column.layers.begin(), column.layers.end(),
    [&](const InferredLayer & layer) {return layer.z == ground.z;});
}

std::vector<OccupyMap::GroundQuery> OccupyMap::groundQueriesUnlocked(std::size_t limit) const
{
  std::lock_guard<std::mutex> lock(ground_cache_mutex_);
  // Stable ordering makes a capped diagnostic repeatable across runs.
  std::vector<ColumnKey> columns;
  for (const auto & entry : ground_cache_) {columns.push_back(entry.first);}
  std::sort(columns.begin(), columns.end(), [](const ColumnKey & a, const ColumnKey & b) {
      return a.x != b.x ? a.x < b.x : a.y < b.y;
    });
  std::vector<GroundQuery> output;
  for (const auto & xy : columns) {
    if (output.size() >= limit) {break;}
    const auto & column = ground_cache_.at(xy);
    const auto position = keyToPos({xy.x, xy.y, 0});
    if (column.layers.empty()) {
      output.push_back({{position[0], position[1], column.query_height}, false, column.reason});
    } else {
      for (const auto & layer : column.layers) {
        if (output.size() >= limit) {break;}
        output.push_back({{position[0], position[1], layer.height + height_datum_offset_},
            true, column.reason});
      }
    }
  }
  return output;
}

}  // namespace astar_3d_planner
