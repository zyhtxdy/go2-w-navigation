#include "astar_3d_planner/path_refiner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace astar_3d_planner
{
PathRefiner::PathRefiner(const Config & config) : config_(config)
{
  if (config.maximum_checks < 1 || !std::isfinite(config.maximum_seconds) ||
    config.maximum_seconds <= 0 || !std::isfinite(config.corner_cut_distance) ||
    config.corner_cut_distance < 0 || !std::isfinite(config.turn_weight) ||
    config.turn_weight < 0 || !std::isfinite(config.clearance_weight) ||
    config.clearance_weight < 0 || !std::isfinite(config.grid_resolution) ||
    config.grid_resolution <= 0 || config.maximum_cost_samples < 1)
  {throw std::invalid_argument("Invalid path refinement budget or weights");}
}

double PathRefiner::score(const Points & points, double start_yaw, double goal_yaw,
  const ClearanceCost & clearance, double * integrated_cost) const
{
  double integral=0.0;
  std::size_t samples=0;
  const auto invalid=[&]() {
      if (integrated_cost) {*integrated_cost=std::numeric_limits<double>::infinity();}
      return std::numeric_limits<double>::infinity();
    };
  if (clearance && config_.clearance_weight > 0) {
    for (std::size_t i=1;i<points.size();++i) {
      const auto & a=points[i-1];const auto & b=points[i];
      const double length=std::hypot(b[0]-a[0],b[1]-a[1]);
      if (!std::isfinite(length)) {return invalid();}
      if (length<1e-12) {continue;}
      std::vector<double> cuts{0.0,1.0};
      for (std::size_t axis=0;axis<3;++axis) {
        if (!std::isfinite(a[axis]) || !std::isfinite(b[axis]) ||
          std::max(std::abs(a[axis]),std::abs(b[axis]))/config_.grid_resolution >
          std::numeric_limits<int>::max()/4.0) {return invalid();}
        const double delta=b[axis]-a[axis];
        if (std::abs(delta)<1e-12) {continue;}
        const int first=static_cast<int>(std::floor(std::min(a[axis],b[axis])/
          config_.grid_resolution))+1;
        const int last=static_cast<int>(std::ceil(std::max(a[axis],b[axis])/
          config_.grid_resolution))-1;
        if (last>=first && static_cast<std::size_t>(last-first+1)>
          config_.maximum_cost_samples) {return invalid();}
        for(int boundary=first;boundary<=last;++boundary) {
          const double t=(boundary*config_.grid_resolution-a[axis])/delta;
          if(t>1e-10 && t<1.0-1e-10) {cuts.push_back(t);}
        }
      }
      std::sort(cuts.begin(),cuts.end());
      cuts.erase(std::unique(cuts.begin(),cuts.end(),[](double x,double y){
          return std::abs(x-y)<1e-10;
        }),cuts.end());
      if (samples+cuts.size()-1>config_.maximum_cost_samples) {return invalid();}
      for(std::size_t j=1;j<cuts.size();++j) {
        const double t=.5*(cuts[j-1]+cuts[j]);
        OccupyMap::Point p;
        for(std::size_t axis=0;axis<3;++axis) {p[axis]=a[axis]+t*(b[axis]-a[axis]);}
        const double cost=clearance(p);++samples;
        if (!std::isfinite(cost) || cost<0.0) {return invalid();}
        integral+=length*(cuts[j]-cuts[j-1])*cost;
      }
    }
  }
  if (integrated_cost) {*integrated_cost=integral;}
  return objective(points,start_yaw,goal_yaw,config_.turn_weight)+config_.clearance_weight*integral;
}

double PathRefiner::objective(const Points & points, double start_yaw, double goal_yaw,
  double turn_weight)
{
  double result = 0.0, yaw = start_yaw;
  const auto turn = [&](double next) {
      const double delta = std::remainder(next - yaw, 2.0 * std::acos(-1.0));
      result += turn_weight * delta * delta;
      yaw = next;
    };
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double dx = points[i][0] - points[i-1][0], dy = points[i][1] - points[i-1][1];
    const double length = std::hypot(dx, dy);
    result += length;
    if (length > 1e-9) {turn(std::atan2(dy, dx));}
  }
  turn(goal_yaw);
  return std::isfinite(result) ? result : std::numeric_limits<double>::infinity();
}

PathRefiner::Points PathRefiner::refine(const Points & input, double start_yaw, double goal_yaw,
  bool retain_first_connector, const std::function<bool(Points &)> & validate,
  Diagnostics * diagnostics, const ClearanceCost & clearance) const
{
  Diagnostics result;
  Points best = input;
  const auto begin = std::chrono::steady_clock::now();
  const auto available = [&]() {
      if (result.checks >= config_.maximum_checks ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count() >=
        config_.maximum_seconds)
      {result.budget_exhausted = true; return false;}
      return true;
    };
  const auto finish = [&]() {if (diagnostics) {*diagnostics = result;} return best;};
  if (input.size() < 3 || !validate || !std::isfinite(start_yaw) ||
    !std::isfinite(goal_yaw)) {return finish();}
  ++result.checks;
  if (!validate(best)) {best = input; return finish();}
  double current_score = score(best, start_yaw, goal_yaw, clearance,&result.clearance_before);
  result.clearance_after=result.clearance_before;
  if (!std::isfinite(current_score)) {result.clearance_unavailable=true;return finish();}
  const auto accept = [&](Points & candidate) {
      if (!available()) {return false;}
      double candidate_cost=0.0;
      const double candidate_score=score(candidate,start_yaw,goal_yaw,clearance,&candidate_cost);
      if (!std::isfinite(candidate_score) || candidate_score >= current_score - 1e-9) {
        if (candidate_cost>result.clearance_after) {++result.clearance_rejected;}
        return false;
      }
      if (!available()) {return false;}
      ++result.checks;
      if (!validate(candidate)) {return false;}
      const double updated_score = score(candidate,start_yaw,goal_yaw,clearance,&candidate_cost);
      if (!std::isfinite(updated_score) || updated_score >= current_score - 1e-9) {return false;}
      best = std::move(candidate); current_score = updated_score;
      result.clearance_after=candidate_cost; return true;
    };
  for (std::size_t anchor = retain_first_connector ? 1 : 0;
    anchor + 2 < best.size() && available(); ++anchor)
  {
    for (std::size_t target = best.size() - 1; target > anchor + 1 && available(); --target) {
      Points candidate(best.begin(), best.begin() + anchor + 1);
      candidate.insert(candidate.end(), best.begin() + target, best.end());
      if (accept(candidate)) {++result.shortcuts; break;}
    }
  }
  // Replace a sharp corner by two points on its adjacent segments. There is
  // no spline extrapolation, endpoint motion, or assumed support in the gap.
  for (std::size_t i = retain_first_connector ? 2 : 1;
    i + 1 < best.size() && available(); ++i)
  {
    const auto a = best[i-1], b = best[i], c = best[i+1];
    const double ab = std::hypot(b[0]-a[0], b[1]-a[1]);
    const double bc = std::hypot(c[0]-b[0], c[1]-b[1]);
    const double cut = std::min({config_.corner_cut_distance, ab * .25, bc * .25});
    if (cut < 1e-4) {continue;}
    OccupyMap::Point left, right;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      left[axis] = b[axis] + (a[axis]-b[axis]) * cut / ab;
      right[axis] = b[axis] + (c[axis]-b[axis]) * cut / bc;
    }
    Points candidate = best;
    candidate[i] = left; candidate.insert(candidate.begin() + i + 1, right);
    if (accept(candidate)) {++result.bevels; ++i;}
  }
  return finish();
}
}  // namespace astar_3d_planner
