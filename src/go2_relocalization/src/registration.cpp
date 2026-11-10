#include "go2_relocalization/registration.hpp"

#include <Eigen/Eigenvalues>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#ifdef __linux__
#include <fcntl.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace go2_relocalization
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaxCloudPoints = 10000000;

bool finitePoint(const pcl::PointXYZ & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::abs(p.x) < 1e6 && std::abs(p.y) < 1e6 && std::abs(p.z) < 1e6;
}

bool rigidPose(const Eigen::Isometry3d & pose)
{
  const auto r = pose.linear();
  return pose.matrix().allFinite() && pose.translation().norm() < 1e6 &&
         (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-3 &&
         std::abs(r.determinant() - 1.0) < 1e-3 &&
         (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-6;
}

bool safeId(const std::string & id)
{
  return !id.empty() && id.size() <= 128 &&
         std::all_of(id.begin(), id.end(), [](unsigned char ch) {
           return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                  (ch >= '0' && ch <= '9') || ch == '_' || ch == '-';
         });
}

Cloud::Ptr finiteCloud(const Cloud & input)
{
  auto cloud = Cloud::Ptr(new Cloud);
  cloud->reserve(input.size());
  for (const auto & point : input) {
    if (finitePoint(point)) {
      cloud->push_back(point);
    }
  }
  cloud->is_dense = true;
  return cloud;
}

Cloud::Ptr voxelCloud(const Cloud & input, double leaf, double max_range = -1.0)
{
  auto finite = Cloud::Ptr(new Cloud);
  finite->reserve(input.size());
  for (const auto & point : input) {
    if (!finitePoint(point)) {
      continue;
    }
    if (max_range > 0.0 &&
      (std::hypot(point.x, point.y) > max_range || std::abs(point.z) > max_range))
    {
      continue;
    }
    finite->push_back(point);
  }
  auto output = Cloud::Ptr(new Cloud);
  if (!finite->empty()) {
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(finite);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.filter(*output);
  }
  output->is_dense = true;
  return output;
}

Cloud::Ptr deriveMap(const std::vector<Keyframe> & keyframes)
{
  auto map = Cloud::Ptr(new Cloud);
  for (const auto & keyframe : keyframes) {
    if (!keyframe.cloud || keyframe.cloud->empty() || !rigidPose(keyframe.T_map_body)) {
      throw std::invalid_argument("empty keyframe cloud or invalid keyframe pose");
    }
    if (map->size() + keyframe.cloud->size() > kMaxCloudPoints) {
      throw std::invalid_argument("derived map exceeds 10000000 points; voxelize before export");
    }
    Cloud transformed;
    pcl::transformPointCloud(*finiteCloud(*keyframe.cloud), transformed,
      keyframe.T_map_body.matrix().cast<float>());
    *map += transformed;
  }
  return map;
}

struct Descriptor
{
  Eigen::MatrixXf values;
  Eigen::VectorXf ring_key;
};

Descriptor describe(const Cloud & cloud, const RegistrationConfig & config)
{
  Descriptor descriptor;
  descriptor.values = Eigen::MatrixXf::Zero(config.ring_count, config.sector_count);
  for (const auto & point : cloud) {
    const double radius = std::hypot(point.x, point.y);
    if (radius <= 1e-6 || radius > config.max_range) {
      continue;
    }
    double angle = std::atan2(point.y, point.x);
    if (angle < 0.0) {
      angle += 2.0 * kPi;
    }
    const int ring = std::min(config.ring_count - 1,
      static_cast<int>(radius / config.max_range * config.ring_count));
    const int sector = std::min(config.sector_count - 1,
      static_cast<int>(angle / (2.0 * kPi) * config.sector_count));
    // A positive offset keeps low/negative floor heights distinguishable from
    // unobserved bins. This is a candidate descriptor, not a registration score.
    const float height = std::max(0.001, point.z + config.descriptor_height_offset);
    descriptor.values(ring, sector) = std::max(descriptor.values(ring, sector), height);
  }
  descriptor.ring_key = descriptor.values.rowwise().mean();
  return descriptor;
}

double descriptorDistance(const Descriptor & query, const Descriptor & target, int shift)
{
  double similarity = 0.0;
  int populated = 0;
  const int sectors = query.values.cols();
  for (int column = 0; column < sectors; ++column) {
    const auto q = query.values.col((column - shift + sectors) % sectors);
    const auto t = target.values.col(column);
    const double q_norm = q.norm();
    const double t_norm = t.norm();
    if (q_norm <= 1e-8 && t_norm <= 1e-8) {
      continue;
    }
    ++populated;
    if (q_norm > 1e-8 && t_norm > 1e-8) {
      similarity += q.dot(t) / (q_norm * t_norm);
    }
  }
  return populated > 0 ? 1.0 - similarity / populated : 1.0;
}

double yawOf(const Eigen::Isometry3d & pose)
{
  return std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
}

double angleDistance(double a, double b)
{
  return std::abs(std::remainder(a - b, 2.0 * kPi));
}

Eigen::Vector3d centroid(const Cloud & cloud)
{
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & point : cloud) {
    mean += point.getVector3fMap().cast<double>();
  }
  return cloud.empty() ? mean : mean / static_cast<double>(cloud.size());
}

double observedNormalRatio(
  const Cloud & target, pcl::KdTreeFLANN<pcl::PointXYZ> & tree,
  const std::vector<int> & matched, const RegistrationConfig & config,
  std::vector<Eigen::Vector3d> * normal_cache = nullptr,
  std::vector<std::uint8_t> * normal_states = nullptr)
{
  Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
  std::size_t normals = 0;
  std::set<int> visited;
  const std::size_t stride = std::max<std::size_t>(1, matched.size() / 1000);
  std::vector<int> neighborhood;
  std::vector<float> squared_distances;
  for (std::size_t i = 0; i < matched.size(); i += stride) {
    const int index = matched[i];
    if (!visited.insert(index).second) {
      continue;
    }
    if (normal_states) {
      if ((*normal_states)[index] == 2) {
        const auto & normal = (*normal_cache)[index];
        information.noalias() += normal * normal.transpose(); ++normals; continue;
      }
      if ((*normal_states)[index] == 1) {continue;}
      (*normal_states)[index] = 1;  // Cache rejection too; target/config are immutable.
    }
    if (tree.radiusSearch(target[index], config.normal_radius, neighborhood, squared_distances) <
      config.min_normal_neighbors)
    {
      continue;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int neighbor : neighborhood) {
      mean += target[neighbor].getVector3fMap().cast<double>();
    }
    mean /= neighborhood.size();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int neighbor : neighborhood) {
      const Eigen::Vector3d offset = target[neighbor].getVector3fMap().cast<double>() - mean;
      covariance.noalias() += offset * offset.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues().sum() <= 1e-9 ||
      eigen.eigenvalues()[1] <= 1e-8 ||
      std::max(0.0, eigen.eigenvalues()[0]) / eigen.eigenvalues().sum() >
      config.max_normal_curvature)
    {
      continue;
    }
    const Eigen::Vector3d normal = eigen.eigenvectors().col(0);
    if (normal_states) {(*normal_cache)[index] = normal; (*normal_states)[index] = 2;}
    information.noalias() += normal * normal.transpose();
    ++normals;
  }
  if (normals < static_cast<std::size_t>(config.min_normal_samples)) {
    return 0.0;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(information);
  if (eigen.info() != Eigen::Success || eigen.eigenvalues()[2] <= 1e-9) {
    return 0.0;
  }
  return std::max(0.0, eigen.eigenvalues()[0]) / eigen.eigenvalues()[2];
}

bool basicQuality(const CandidateScore & candidate, const RegistrationConfig & config)
{
  return candidate.converged && candidate.overlap >= config.min_overlap &&
         candidate.rmse <= config.max_rmse && candidate.score >= config.min_quality_score;
}

MatchResult decide(std::vector<CandidateScore> candidates, const RegistrationConfig & config,
  bool global)
{
  MatchResult result;
  std::sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
    return a.score > b.score;
  });
  result.candidates = std::move(candidates);
  if (result.candidates.empty()) {
    result.status = "empty_target";
    return result;
  }
  const auto & best = result.candidates.front();
  result.pose = best.pose;
  result.rmse = best.rmse;
  result.overlap = best.overlap;
  result.observability_ratio = best.observability_ratio;
  result.quality_score = best.score;
  result.best_candidate_id = best.id;
  for (std::size_t i = 1; i < result.candidates.size(); ++i) {
    const auto & alternative = result.candidates[i];
    if (!alternative.converged || alternative.overlap < config.min_overlap ||
      alternative.rmse > config.max_rmse) {continue;}
    if ((best.pose.translation() - alternative.pose.translation()).norm() >=
      config.ambiguity_distance ||
      angleDistance(yawOf(best.pose), yawOf(alternative.pose)) >= config.ambiguity_yaw)
    {
      result.alternative_score = alternative.score;
      result.score_gap = best.score - alternative.score;
      break;
    }
  }
  if (!best.converged) {
    result.status = "registration_failed";
  } else if (best.overlap < config.min_overlap) {
    result.status = "low_overlap";
  } else if (best.rmse > config.max_rmse) {
    result.status = "high_rmse";
  } else if (best.score < config.min_quality_score) {
    result.status = "low_quality";
  } else if (best.observability_ratio < config.min_observability_ratio) {
    result.status = "degenerate_geometry";
  } else {
    if (global) {
      for (std::size_t i = 1; i < result.candidates.size(); ++i) {
        const auto & alternative = result.candidates[i];
        // An almost equally plausible alternative is still ambiguous even if it
        // lies just below the final quality-score acceptance threshold.
        if (!alternative.converged || alternative.overlap < config.min_overlap ||
          alternative.rmse > config.max_rmse)
        {
          continue;
        }
        const bool different_place =
          (best.pose.translation() - alternative.pose.translation()).norm() >=
          config.ambiguity_distance;
        const bool different_heading = angleDistance(yawOf(best.pose), yawOf(alternative.pose)) >=
          config.ambiguity_yaw;
        if ((different_place || different_heading) &&
          best.score - alternative.score < config.min_score_gap)
        {
          result.status = "ambiguous_match";
          return result;
        }
      }
    }
    result.accepted = true;
    result.status = "accepted";
  }
  return result;
}

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

double parseNumber(const std::string & value)
{
  std::size_t consumed = 0;
  const double number = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(number)) {
    throw std::invalid_argument("non-finite or invalid numeric pose field");
  }
  return number;
}
}  // namespace

bool validateConfig(const RegistrationConfig & c, std::string & error)
{
  const std::array<double, 15> finite_values{{c.voxel_size, c.max_range,
    c.descriptor_height_offset, c.max_correspondence_distance, c.evaluation_distance,
    c.min_overlap, c.max_rmse, c.ambiguity_distance, c.ambiguity_yaw,
    c.min_score_gap, c.min_observability_ratio, c.normal_radius,
    c.max_normal_curvature, c.min_quality_score, static_cast<double>(c.max_iterations)}};
  if (!std::all_of(finite_values.begin(), finite_values.end(), [](double v) {
      return std::isfinite(v);
    }) || c.voxel_size <= 0 || c.max_range <= c.voxel_size || c.top_k < 1 ||
    c.ring_count < 1 || c.ring_count > 1000 || c.sector_count < 4 ||
    c.sector_count > 3600 || c.max_correspondence_distance <= 0 ||
    c.evaluation_distance <= 0 || c.evaluation_distance > c.max_correspondence_distance ||
    c.max_iterations < 1 || c.min_points < 20 || c.min_overlap <= 0 ||
    c.min_overlap > 1 || c.max_rmse <= 0 || c.min_quality_score <= 0 ||
    c.min_quality_score > 1 || c.ambiguity_distance <= 0 ||
    c.ambiguity_yaw <= 0 || c.ambiguity_yaw > kPi || c.min_score_gap < 0 ||
    c.min_observability_ratio <= 0 || c.min_observability_ratio > 1 ||
    c.normal_radius <= 0 || c.min_normal_neighbors < 3 ||
    c.max_normal_curvature <= 0 || c.max_normal_curvature >= 1.0 / 3.0 ||
    c.min_normal_samples < 3 || c.candidate_threads < 1 || c.candidate_threads > 4)
  {
    error = "invalid registration configuration";
    return false;
  }
  error.clear();
  return true;
}

struct Relocalizer::Impl
{
  // Covariances depend only on the exact point set, never on a pose seed.
  class ReusableGICP : public pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>
  {
  public:
    MatricesVectorPtr sourceCovariances() const {return input_covariances_;}
    MatricesVectorPtr targetCovariances() const {return target_covariances_;}
    MatricesVectorPtr prepareSource(const Cloud::Ptr & cloud, const KdTreeReciprocalPtr & tree)
    {
      auto covariances = MatricesVectorPtr(new MatricesVector);
      tree->setInputCloud(cloud);
      computeCovariances<pcl::PointXYZ>(cloud, tree, *covariances);
      return covariances;
    }
  };
  struct SourceFeatures
  {
    ReusableGICP::MatricesVectorPtr covariances;
    ReusableGICP::KdTreeReciprocalPtr tree;
  };
  struct TargetFeatures
  {
    std::vector<std::size_t> indices;
    Cloud::Ptr cloud{new Cloud};
    ReusableGICP::MatricesVectorPtr covariances;
    ReusableGICP::KdTreePtr registration_tree;
    pcl::KdTreeFLANN<pcl::PointXYZ> evaluation_tree;
    bool evaluation_ready{false};
    std::vector<Eigen::Vector3d> normals;
    std::vector<std::uint8_t> normal_states;
    std::mutex mutex;
  };
  struct PreparedKeyframe
  {
    Keyframe frame;
    Descriptor descriptor;
  };
  Database database;
  RegistrationConfig config;
  std::vector<PreparedKeyframe> prepared;
  Cloud::Ptr map;
  // Bounded by point count as well as entry count; no approximate crop reuse.
  mutable std::mutex cache_mutex;
  mutable std::deque<std::shared_ptr<TargetFeatures>> target_cache;
  mutable std::size_t cached_points{0};

  std::shared_ptr<TargetFeatures> targetAt(const Eigen::Isometry3d & pose) const
  {
    std::vector<std::size_t> indices;
    const auto center = pose.translation();
    const double radius = config.max_range + config.max_correspondence_distance;
    for (std::size_t index = 0; index < map->size(); ++index) {
      const auto & point = (*map)[index];
      if (std::hypot(point.x - center.x(), point.y - center.y()) <= radius &&
        std::abs(point.z - center.z()) <= radius)
      {
        indices.push_back(index);
      }
    }
    std::unique_lock<std::mutex> lock(cache_mutex);
    if (config.cache_registration_features) {
      const auto found = std::find_if(target_cache.begin(), target_cache.end(),
        [&indices](const auto & target) {return target->indices == indices;});
      if (found != target_cache.end()) {
        const auto target = *found;
        target_cache.erase(found); target_cache.push_back(target);
        return target;
      }
    }
    auto target = std::make_shared<TargetFeatures>();
    target->indices = std::move(indices);
    target->cloud->reserve(target->indices.size());
    for (const auto index : target->indices) {target->cloud->push_back((*map)[index]);}
    if (config.cache_registration_features) {
      target->normals.resize(target->cloud->size());
      target->normal_states.resize(target->cloud->size(), 0);
    }
    constexpr std::size_t point_budget = 200000;
    if (config.cache_registration_features && target->indices.size() <= point_budget) {
      while (!target_cache.empty() && (target_cache.size() >= 8 ||
        cached_points + target->indices.size() > point_budget))
      {
        cached_points -= target_cache.front()->indices.size(); target_cache.pop_front();
      }
      target_cache.push_back(target); cached_points += target->indices.size();
    }
    return target;
  }

  CandidateScore align(const Cloud::Ptr & source, const Eigen::Isometry3d & initial,
    const std::string & id, double descriptor_distance, SourceFeatures source_features) const
  {
    CandidateScore candidate;
    candidate.id = id;
    candidate.pose = initial;
    candidate.descriptor_distance = descriptor_distance;
    const auto features = targetAt(initial);
    std::unique_lock<std::mutex> lock(features->mutex);
    const auto & target = features->cloud;
    if (target->size() < static_cast<std::size_t>(config.min_points)) {
      return candidate;
    }
    try {
      ReusableGICP registration;
      registration.setInputSource(source);
      registration.setInputTarget(target);
      if (config.cache_registration_features) {
        if (source_features.covariances) {
          registration.setSourceCovariances(source_features.covariances);
          registration.setSearchMethodSource(source_features.tree, true);
        }
        if (features->covariances) {
          registration.setTargetCovariances(features->covariances);
          registration.setSearchMethodTarget(features->registration_tree, true);
          lock.unlock();  // Immutable prepared features; optimize candidates concurrently.
        }
      }
      registration.setMaxCorrespondenceDistance(config.max_correspondence_distance);
      registration.setMaximumIterations(config.max_iterations);
      registration.setTransformationEpsilon(1e-5);
      registration.setEuclideanFitnessEpsilon(1e-5);
      registration.setRANSACIterations(0);
      Cloud aligned;
      registration.align(aligned, initial.matrix().cast<float>());
      if (!lock.owns_lock()) {lock.lock();}
      if (config.cache_registration_features) {
        source_features.covariances = registration.sourceCovariances();
        source_features.tree = registration.getSearchMethodSource();
        features->covariances = registration.targetCovariances();
        features->registration_tree = registration.getSearchMethodTarget();
      }
      candidate.converged = registration.hasConverged();
      candidate.pose.matrix() = registration.getFinalTransformation().cast<double>();
      if (!rigidPose(candidate.pose)) {
        candidate.converged = false;
        return candidate;
      }
      auto & tree = features->evaluation_tree;
      if (!features->evaluation_ready) {tree.setInputCloud(target); features->evaluation_ready = true;}
      std::vector<int> matched;
      matched.reserve(aligned.size());
      std::vector<int> nearest(1);
      std::vector<float> distance(1);
      double squared_error = 0.0;
      const double max_squared = config.evaluation_distance * config.evaluation_distance;
      for (const auto & point : aligned) {
        if (!finitePoint(point)) {
          continue;
        }
        if (tree.nearestKSearch(point, 1, nearest, distance) > 0 && distance[0] <= max_squared) {
          squared_error += distance[0];
          matched.push_back(nearest[0]);
        }
      }
      candidate.overlap = static_cast<double>(matched.size()) / source->size();
      if (!matched.empty()) {
        candidate.rmse = std::sqrt(squared_error / matched.size());
        candidate.observability_ratio = observedNormalRatio(*target, tree, matched, config,
          config.cache_registration_features ? &features->normals : nullptr,
          config.cache_registration_features ? &features->normal_states : nullptr);
        candidate.score = candidate.overlap - 0.25 * candidate.rmse / config.evaluation_distance;
      }
      candidate.geometrically_valid = basicQuality(candidate, config) &&
        candidate.observability_ratio >= config.min_observability_ratio;
    } catch (const std::exception &) {
      candidate.converged = false;
    }
    return candidate;
  }

  struct Job {Eigen::Isometry3d initial; std::string id; double distance;};
  std::vector<CandidateScore> alignJobs(const Cloud::Ptr & source,
    const std::vector<Job> & jobs) const
  {
    SourceFeatures source_features;
    if (config.cache_registration_features) {
      ReusableGICP preparation;
      source_features.tree = preparation.getSearchMethodSource();
      source_features.covariances = preparation.prepareSource(source, source_features.tree);
    }
    std::vector<CandidateScore> candidates(jobs.size());
    std::atomic<std::size_t> next{0};
    std::exception_ptr failure;
    std::mutex failure_mutex;
    const auto work = [&]() {
        while (true) {
          const auto index = next.fetch_add(1);
          if (index >= jobs.size()) {return;}
          try {
            const auto & job = jobs[index];
            candidates[index] = align(source, job.initial, job.id, job.distance, source_features);
          } catch (...) {
            std::lock_guard<std::mutex> lock(failure_mutex);
            if (!failure) {failure = std::current_exception();}
            next.store(jobs.size()); return;
          }
        }
      };
    std::vector<std::thread> threads;
    try {
      for (int i = 1; i < config.candidate_threads; ++i) {threads.emplace_back(work);}
      work();
    } catch (...) {
      for (auto & thread : threads) {thread.join();}
      throw;
    }
    for (auto & thread : threads) {thread.join();}
    if (failure) {std::rethrow_exception(failure);}
    return candidates;
  }

};

Relocalizer::Relocalizer(Database database, RegistrationConfig config)
: impl_(std::make_shared<Impl>())
{
  std::string error;
  if (!validateConfig(config, error)) {
    throw std::invalid_argument(error);
  }
  if (database.keyframes.empty()) {
    throw std::invalid_argument("database contains no keyframes");
  }
  impl_->config = config;
  std::set<std::string> ids;
  for (auto & keyframe : database.keyframes) {
    if (!safeId(keyframe.id) || !ids.insert(keyframe.id).second ||
      !rigidPose(keyframe.T_map_body) || !keyframe.cloud ||
      keyframe.cloud->size() > kMaxCloudPoints)
    {
      throw std::invalid_argument("invalid keyframe id, pose or cloud");
    }
    keyframe.cloud = voxelCloud(*keyframe.cloud, config.voxel_size, config.max_range);
    if (keyframe.cloud->size() < static_cast<std::size_t>(config.min_points)) {
      throw std::invalid_argument("keyframe has insufficient finite points after filtering");
    }
    impl_->prepared.push_back({keyframe, describe(*keyframe.cloud, config)});
  }
  if (!database.global_cloud || database.global_cloud->empty()) {
    database.global_cloud = deriveMap(database.keyframes);
  }
  if (database.global_cloud->size() > kMaxCloudPoints) {
    throw std::invalid_argument("map exceeds 10000000 points");
  }
  impl_->map = voxelCloud(*database.global_cloud, config.voxel_size);
  if (impl_->map->size() < static_cast<std::size_t>(config.min_points)) {
    throw std::invalid_argument("map has insufficient finite points after filtering");
  }
  database.global_cloud = impl_->map;
  impl_->database = std::move(database);
}

const Database & Relocalizer::database() const
{
  return impl_->database;
}

MatchResult Relocalizer::globalMatch(const Cloud & local_body_cloud,
  const std::optional<Eigen::Isometry3d> & prediction) const
{
  const auto & config = impl_->config;
  const auto source = voxelCloud(local_body_cloud, config.voxel_size, config.max_range);
  if (source->size() < static_cast<std::size_t>(config.min_points)) {
    MatchResult result;
    result.status = "insufficient_points";
    return result;
  }
  const auto query = describe(*source, config);
  std::vector<std::pair<double, std::size_t>> retrieval;
  for (std::size_t i = 0; i < impl_->prepared.size(); ++i) {
    retrieval.emplace_back(
      (query.ring_key - impl_->prepared[i].descriptor.ring_key).squaredNorm(), i);
  }
  std::sort(retrieval.begin(), retrieval.end());
  retrieval.resize(std::min(retrieval.size(), static_cast<std::size_t>(config.top_k)));
  std::vector<Impl::Job> jobs;
  const Eigen::Vector3d source_center = centroid(*source);
  for (const auto & selected : retrieval) {
    const auto & keyframe = impl_->prepared[selected.second];
    std::vector<std::pair<double, int>> shifts;
    for (int shift = 0; shift < config.sector_count; ++shift) {
      shifts.emplace_back(descriptorDistance(query, keyframe.descriptor, shift), shift);
    }
    std::sort(shifts.begin(), shifts.end());
    std::vector<double> tested_yaws;
    for (const auto & hypothesis : shifts) {
      if (tested_yaws.size() >= 2 || hypothesis.first > shifts.front().first + 0.12) {
        break;
      }
      const double yaw = hypothesis.second * 2.0 * kPi / config.sector_count;
      if (std::any_of(tested_yaws.begin(), tested_yaws.end(), [yaw](double other) {
          return angleDistance(yaw, other) < 0.8;
        }))
      {
        continue;
      }
      tested_yaws.push_back(yaw);
      Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
      relative.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      const auto initial = keyframe.frame.T_map_body * relative;
      jobs.push_back({initial, keyframe.frame.id, hypothesis.first});
      // The centroid proposal helps near a recorded keyframe. It is only a seed;
      // the independent nearest-neighbor and observability gates still decide.
      Eigen::Isometry3d centered = initial;
      centered.translation() = keyframe.frame.T_map_body * centroid(*keyframe.frame.cloud) -
        initial.linear() * source_center;
      const double displacement = (centered.translation() - initial.translation()).norm();
      if (displacement > config.voxel_size / 2.0 && displacement <= 3.0) {
        jobs.push_back({centered, keyframe.frame.id, hypothesis.first});
      }
    }
  }
  if (prediction && rigidPose(*prediction)) {
    jobs.push_back({*prediction, "odometry_prior", 0.0});
  }
  return decide(impl_->alignJobs(source, jobs), config, true);
}

MatchResult Relocalizer::verifyGlobalProposal(const Cloud & local_body_cloud,
  const MatchResult & proposal, const Eigen::Isometry3d & odometry_delta) const
{
  MatchResult rejected;
  rejected.status = "invalid_global_proposal";
  if (!proposal.accepted || proposal.candidates.empty() || !rigidPose(odometry_delta)) {
    return rejected;
  }
  const auto & config = impl_->config;
  const auto source = voxelCloud(local_body_cloud, config.voxel_size, config.max_range);
  if (source->size() < static_cast<std::size_t>(config.min_points)) {
    rejected.status = "insufficient_points"; return rejected;
  }
  std::vector<Impl::Job> jobs;
  for (const auto & candidate : proposal.candidates) {
    if (!rigidPose(candidate.pose)) {return rejected;}
    jobs.push_back({candidate.pose * odometry_delta, candidate.id, candidate.descriptor_distance});
  }
  return decide(impl_->alignJobs(source, jobs), config, true);
}

MatchResult Relocalizer::track(const Cloud & local_body_cloud,
  const Eigen::Isometry3d & predicted_T_map_body) const
{
  if (!rigidPose(predicted_T_map_body)) {
    MatchResult result;
    result.status = "invalid_prediction";
    return result;
  }
  const auto source = voxelCloud(local_body_cloud, impl_->config.voxel_size, impl_->config.max_range);
  if (source->size() < static_cast<std::size_t>(impl_->config.min_points)) {
    MatchResult result;
    result.status = "insufficient_points";
    return result;
  }
  std::vector<CandidateScore> candidates;
  Impl::SourceFeatures source_features;
  candidates.push_back(impl_->align(source, predicted_T_map_body, "tracking", 0.0, source_features));
  return decide(std::move(candidates), impl_->config, false);
}

bool loadDatabase(const std::string & directory, Database & database, std::string & error)
{
  try {
    const auto root = std::filesystem::path(directory);
    std::ifstream manifest(root / "manifest.csv");
    if (!manifest) {
      throw std::runtime_error("cannot open manifest.csv");
    }
    std::string line;
    if (!std::getline(manifest, line)) {
      throw std::runtime_error("manifest.csv is empty");
    }
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line != "id,x,y,z,qx,qy,qz,qw") {
      throw std::runtime_error("manifest header must be id,x,y,z,qx,qy,qz,qw");
    }
    Database loaded;
    std::set<std::string> ids;
    std::size_t row = 1;
    while (std::getline(manifest, line)) {
      ++row;
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (line.empty()) {
        continue;
      }
      const auto fields = splitCsv(line);
      if (fields.size() != 8 || !safeId(fields[0]) || !ids.insert(fields[0]).second) {
        throw std::runtime_error("invalid or duplicate keyframe id/columns at row " +
          std::to_string(row));
      }
      Keyframe keyframe;
      keyframe.id = fields[0];
      keyframe.T_map_body.translation() =
        Eigen::Vector3d(parseNumber(fields[1]), parseNumber(fields[2]), parseNumber(fields[3]));
      Eigen::Quaterniond quaternion(parseNumber(fields[7]), parseNumber(fields[4]),
        parseNumber(fields[5]), parseNumber(fields[6]));
      if (quaternion.norm() < 1e-6 || std::abs(quaternion.norm() - 1.0) > 0.1) {
        throw std::runtime_error("invalid quaternion at row " + std::to_string(row));
      }
      keyframe.T_map_body.linear() = quaternion.normalized().toRotationMatrix();
      if (!rigidPose(keyframe.T_map_body)) {
        throw std::runtime_error("invalid pose at row " + std::to_string(row));
      }
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(
          (root / "keyframes" / (keyframe.id + ".pcd")).string(), *keyframe.cloud) < 0 ||
        keyframe.cloud->size() > kMaxCloudPoints)
      {
        throw std::runtime_error("cannot load bounded keyframe cloud: " + keyframe.id);
      }
      keyframe.cloud = finiteCloud(*keyframe.cloud);
      if (keyframe.cloud->empty()) {
        throw std::runtime_error("keyframe cloud has no finite points: " + keyframe.id);
      }
      loaded.keyframes.push_back(std::move(keyframe));
      if (loaded.keyframes.size() > 100000) {
        throw std::runtime_error("too many keyframes");
      }
    }
    if (loaded.keyframes.empty()) {
      throw std::runtime_error("manifest has no keyframes");
    }
    if (std::filesystem::exists(root / "map.pcd")) {
      if (pcl::io::loadPCDFile<pcl::PointXYZ>((root / "map.pcd").string(),
          *loaded.global_cloud) < 0 || loaded.global_cloud->size() > kMaxCloudPoints)
      {
        throw std::runtime_error("cannot load bounded map.pcd");
      }
      loaded.global_cloud = finiteCloud(*loaded.global_cloud);
      if (loaded.global_cloud->empty()) {
        throw std::runtime_error("map.pcd has no finite points");
      }
    } else {
      loaded.global_cloud = deriveMap(loaded.keyframes);
    }
    database = std::move(loaded);
    error.clear();
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }
}

bool writeDatabase(const std::string & directory, const Database & database, std::string & error)
{
  std::filesystem::path staging;
  bool owns_staging = false;
  try {
    if (directory.empty() || database.keyframes.empty()) {
      throw std::invalid_argument("output path and keyframes must be nonempty");
    }
    const auto destination = std::filesystem::path(directory);
    if (std::filesystem::exists(destination) || std::filesystem::is_symlink(destination)) {
      throw std::runtime_error("destination already exists; export refuses to overwrite");
    }
    const auto parent = destination.has_parent_path() ?
      destination.parent_path() : std::filesystem::current_path();
    std::filesystem::create_directories(parent);
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    staging = parent / (destination.filename().string() + ".partial-" + std::to_string(suffix));
    owns_staging = std::filesystem::create_directory(staging);
    if (!owns_staging) {
      throw std::runtime_error("cannot create unique export staging directory");
    }
    std::filesystem::create_directory(staging / "keyframes");
    std::ofstream manifest(staging / "manifest.csv");
    if (!manifest) {
      throw std::runtime_error("cannot create manifest.csv");
    }
    manifest << "id,x,y,z,qx,qy,qz,qw\n" << std::setprecision(17);
    std::set<std::string> ids;
    for (const auto & keyframe : database.keyframes) {
      if (!safeId(keyframe.id) || !ids.insert(keyframe.id).second ||
        !rigidPose(keyframe.T_map_body) || !keyframe.cloud ||
        keyframe.cloud->size() > kMaxCloudPoints)
      {
        throw std::runtime_error("invalid keyframe id, pose or cloud during export");
      }
      const auto cloud = finiteCloud(*keyframe.cloud);
      if (cloud->empty() || pcl::io::savePCDFileBinary(
          (staging / "keyframes" / (keyframe.id + ".pcd")).string(), *cloud) < 0)
      {
        throw std::runtime_error("cannot export nonempty keyframe cloud: " + keyframe.id);
      }
      const auto t = keyframe.T_map_body.translation();
      const Eigen::Quaterniond q(keyframe.T_map_body.linear());
      manifest << keyframe.id << ',' << t.x() << ',' << t.y() << ',' << t.z() << ',' <<
        q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << '\n';
    }
    manifest.flush();
    if (!manifest) {
      throw std::runtime_error("cannot flush manifest.csv");
    }
    manifest.close();
    const auto map = database.global_cloud && !database.global_cloud->empty() ?
      finiteCloud(*database.global_cloud) : deriveMap(database.keyframes);
    if (map->empty() || map->size() > kMaxCloudPoints ||
      pcl::io::savePCDFileBinary((staging / "map.pcd").string(), *map) < 0)
    {
      throw std::runtime_error("cannot export nonempty bounded map.pcd");
    }
#ifdef __linux__
    // Linux's no-replace rename prevents a race from overwriting an independently
    // created destination between the initial existence check and publication.
    if (syscall(SYS_renameat2, AT_FDCWD, staging.c_str(), AT_FDCWD,
        destination.c_str(), 1 /* RENAME_NOREPLACE */) != 0)
    {
      throw std::system_error(errno, std::generic_category(), "cannot publish database");
    }
#else
    if (std::filesystem::exists(destination)) {
      throw std::runtime_error("destination appeared during export; refusing overwrite");
    }
    std::filesystem::rename(staging, destination);
#endif
    owns_staging = false;
    error.clear();
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    if (owns_staging) {
      std::error_code cleanup_error;
      std::filesystem::remove_all(staging, cleanup_error);
    }
    return false;
  }
}
}  // namespace go2_relocalization
