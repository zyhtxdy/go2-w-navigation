#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace go2_relocalization
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct Keyframe
{
  std::string id;
  Eigen::Isometry3d T_map_body{Eigen::Isometry3d::Identity()};
  Cloud::Ptr cloud{new Cloud};  // Local points expressed in this keyframe's body frame.
};

struct Database
{
  std::vector<Keyframe> keyframes;
  Cloud::Ptr global_cloud{new Cloud};  // Points in map; derived from keyframes if absent.
};

struct RegistrationConfig
{
  double voxel_size{0.2};
  double max_range{20.0};
  int top_k{12};
  int ring_count{20};
  int sector_count{60};
  double descriptor_height_offset{2.0};
  double max_correspondence_distance{1.0};
  int max_iterations{40};
  int min_points{100};
  double evaluation_distance{0.4};
  double min_overlap{0.6};
  double max_rmse{0.25};
  double min_quality_score{0.70};  // overlap - 0.25 * rmse / evaluation_distance.
  double ambiguity_distance{2.0};
  double ambiguity_yaw{0.6};
  double min_score_gap{0.04};
  double min_observability_ratio{0.01};
  double normal_radius{0.8};
  int min_normal_neighbors{8};
  double max_normal_curvature{0.08};
  int min_normal_samples{30};
  bool cache_registration_features{true};
  int candidate_threads{1};
};

struct CandidateScore
{
  std::string id;
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  double descriptor_distance{std::numeric_limits<double>::infinity()};
  double rmse{std::numeric_limits<double>::infinity()};
  double overlap{0.0};
  double observability_ratio{0.0};
  double score{-std::numeric_limits<double>::infinity()};
  bool converged{false};
  bool geometrically_valid{false};
};

struct MatchResult
{
  bool accepted{false};
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  double rmse{std::numeric_limits<double>::infinity()};
  double overlap{0.0};
  double observability_ratio{0.0};
  double quality_score{-std::numeric_limits<double>::infinity()};
  double alternative_score{-std::numeric_limits<double>::infinity()};
  double score_gap{std::numeric_limits<double>::infinity()};
  std::string best_candidate_id;
  std::string status{"not_run"};
  std::vector<CandidateScore> candidates;
};

// No external Scan Context implementation is included: candidate retrieval uses a
// small ring/sector maximum-height descriptor written for this package.
class Relocalizer
{
public:
  explicit Relocalizer(Database database, RegistrationConfig config = {});
  // A fresh odometry prediction adds one seed; it never removes global
  // candidates or bypasses the quality, observability or ambiguity gates.
  MatchResult globalMatch(const Cloud & local_body_cloud,
    const std::optional<Eigen::Isometry3d> & prediction = std::nullopt) const;
  // A global result is a proposal, never a fresh pose by itself. Propagate
  // every original hypothesis using odometry and re-evaluate all of them on
  // the new scan, including the original ambiguity and observability checks.
  MatchResult verifyGlobalProposal(const Cloud & local_body_cloud,
    const MatchResult & proposal, const Eigen::Isometry3d & odometry_delta) const;
  MatchResult track(
    const Cloud & local_body_cloud, const Eigen::Isometry3d & predicted_T_map_body) const;
  const Database & database() const;

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

bool validateConfig(const RegistrationConfig & config, std::string & error);

// Format: manifest.csv with id,x,y,z,qx,qy,qz,qw; keyframes/<id>.pcd in body;
// map.pcd in map. Loading derives map from keyframes when map.pcd is absent.
// Both functions return false with a descriptive error on malformed data.
bool loadDatabase(const std::string & directory, Database & database, std::string & error);
// Export is atomic via a sibling staging directory and refuses an existing path.
bool writeDatabase(const std::string & directory, const Database & database, std::string & error);
}  // namespace go2_relocalization
