#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

#include "go2_relocalization/registration.hpp"

namespace
{
using go2_relocalization::Cloud;
using go2_relocalization::Database;
using go2_relocalization::Keyframe;
using go2_relocalization::MatchResult;
using go2_relocalization::RegistrationConfig;
using go2_relocalization::Relocalizer;

constexpr double kPi = 3.14159265358979323846;

// Every cloud in this file is synthetic. No result measures a physical robot,
// the XT16, a recorded bag, or the accuracy of the existing FAST-LIO front end.
Eigen::Isometry3d pose(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(x, y, z);
  result.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

void addPoint(Cloud & cloud, const Eigen::Vector3d & point)
{
  cloud.push_back(pcl::PointXYZ(
    static_cast<float>(point.x()), static_cast<float>(point.y()),
    static_cast<float>(point.z())));
}

void addPlane(
  Cloud & cloud, const Eigen::Vector3d & origin,
  const Eigen::Vector3d & first_axis, const Eigen::Vector3d & second_axis,
  double first_length, double second_length, double spacing = 0.24)
{
  const int first_count = static_cast<int>(std::ceil(first_length / spacing));
  const int second_count = static_cast<int>(std::ceil(second_length / spacing));
  for (int first = 0; first <= first_count; ++first) {
    for (int second = 0; second <= second_count; ++second) {
      addPoint(
        cloud, origin + first_axis * (first_length * first / first_count) +
        second_axis * (second_length * second / second_count));
    }
  }
}

void addBox(
  Cloud & cloud, double x_min, double x_max, double y_min, double y_max,
  double z_min, double z_max)
{
  const Eigen::Vector3d x = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  for (double face_x : {x_min, x_max}) {
    addPlane(cloud, {face_x, y_min, z_min}, y, z, y_max - y_min, z_max - z_min);
  }
  for (double face_y : {y_min, y_max}) {
    addPlane(cloud, {x_min, face_y, z_min}, x, z, x_max - x_min, z_max - z_min);
  }
  addPlane(cloud, {x_min, y_min, z_max}, x, y, x_max - x_min, y_max - y_min);
}

Cloud::Ptr asymmetricScene(int variant = 0)
{
  Cloud::Ptr cloud(new Cloud);
  const Eigen::Vector3d x = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  if (variant == 0) {
    addPlane(*cloud, {-3.4, -2.3, -0.5}, x, y, 7.6, 5.0);
    addPlane(*cloud, {-3.4, -2.3, -0.5}, x, z, 7.6, 2.3);
    addPlane(*cloud, {-3.4, 2.7, -0.5}, x, z, 7.6, 2.3);
    addPlane(*cloud, {4.2, -2.3, -0.5}, y, z, 5.0, 2.3);
    addPlane(*cloud, {-3.4, 1.2, -0.5}, y, z, 1.5, 1.5);
    addBox(*cloud, 0.9, 1.5, 1.1, 1.8, -0.5, 2.1);
    addBox(*cloud, -2.0, -1.3, 0.2, 0.9, -0.5, 0.8);
    addBox(*cloud, 2.4, 3.1, -1.6, -0.7, -0.5, 0.35);
  } else {
    addPlane(*cloud, {-2.7, -3.4, -0.5}, x, y, 6.3, 5.5);
    addPlane(*cloud, {-2.7, -3.4, -0.5}, y, z, 5.5, 1.7);
    addPlane(*cloud, {3.6, -3.4, -0.5}, y, z, 5.5, 2.9);
    addPlane(*cloud, {-2.7, 2.1, -0.5}, x, z, 6.3, 2.9);
    addBox(*cloud, 0.3, 1.1, -2.5, -1.5, -0.5, 2.5);
    addBox(*cloud, -1.5, -0.2, -0.3, 0.6, -0.5, 0.1);
    addBox(*cloud, 1.8, 2.5, 0.5, 1.1, -0.5, 1.4);
  }
  cloud->is_dense = true;
  return cloud;
}

Cloud::Ptr straightTunnel()
{
  Cloud::Ptr cloud(new Cloud);
  const Eigen::Vector3d x = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  // No end walls: all actual surface normals are perpendicular to the tunnel
  // axis. A finite point-to-point fit must not imply an observable axial pose.
  addPlane(*cloud, {-8.0, -1.5, -0.5}, x, y, 16.0, 3.0);
  addPlane(*cloud, {-8.0, -1.5, 2.0}, x, y, 16.0, 3.0);
  addPlane(*cloud, {-8.0, -1.5, -0.5}, x, z, 16.0, 2.5);
  addPlane(*cloud, {-8.0, 1.5, -0.5}, x, z, 16.0, 2.5);
  cloud->is_dense = true;
  return cloud;
}

Cloud::Ptr transformed(const Cloud & input, const Eigen::Isometry3d & transform)
{
  Cloud::Ptr result(new Cloud);
  result->reserve(input.size());
  for (const auto & point : input) {
    addPoint(*result, transform * Eigen::Vector3d(point.x, point.y, point.z));
  }
  result->is_dense = input.is_dense;
  return result;
}

Keyframe keyframe(
  const std::string & id, const Cloud::Ptr & local_cloud,
  const Eigen::Isometry3d & map_pose)
{
  Keyframe result;
  result.id = id;
  result.cloud = local_cloud;
  result.T_map_body = map_pose;
  return result;
}

Database database(std::vector<Keyframe> keyframes)
{
  Database result;
  result.keyframes = std::move(keyframes);
  for (const auto & frame : result.keyframes) {
    *result.global_cloud += *transformed(*frame.cloud, frame.T_map_body);
  }
  result.global_cloud->is_dense = true;
  return result;
}

std::string summarize(const MatchResult & result)
{
  std::ostringstream output;
  output << "status=" << result.status << ", accepted=" << result.accepted <<
    ", overlap=" << result.overlap << ", rmse=" << result.rmse <<
    ", observability=" << result.observability_ratio;
  for (const auto & candidate : result.candidates) {
    output << "\n  id=" << candidate.id << ", converged=" << candidate.converged <<
      ", geometry=" << candidate.geometrically_valid << ", overlap=" <<
      candidate.overlap << ", rmse=" << candidate.rmse << ", observable=" <<
      candidate.observability_ratio << ", score=" << candidate.score <<
      ", position=" << candidate.pose.translation().transpose() << ", yaw=" <<
      std::atan2(candidate.pose.linear()(1, 0), candidate.pose.linear()(0, 0));
  }
  return output.str();
}

void expectCorrectPose(const MatchResult & result, const Eigen::Isometry3d & expected)
{
  SCOPED_TRACE(summarize(result));
  ASSERT_TRUE(result.accepted);
  EXPECT_LT((result.pose.translation() - expected.translation()).norm(), 0.12);
  const double angle_error = Eigen::AngleAxisd(
    expected.linear().transpose() * result.pose.linear()).angle();
  EXPECT_LT(std::abs(angle_error), 0.06);
  const RegistrationConfig production;
  EXPECT_GE(result.overlap, production.min_overlap);
  EXPECT_LE(result.rmse, production.max_rmse);
  EXPECT_GE(result.observability_ratio, production.min_observability_ratio);
  ASSERT_FALSE(result.candidates.empty());
  EXPECT_GE(result.candidates.front().score, production.min_quality_score);
}

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    std::string pattern = (std::filesystem::temp_directory_path() /
      "go2_relocalization_test_XXXXXX").string();
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    char * result = ::mkdtemp(name.data());
    if (result) {
      root_ = result;
    }
  }

  ~TemporaryDirectory()
  {
    if (!root_.empty()) {
      std::error_code ignored;
      std::filesystem::remove_all(root_, ignored);
    }
  }

  std::filesystem::path root() const {return root_;}

private:
  std::filesystem::path root_;
};
}  // namespace

TEST(GlobalRelocalization, UnknownInitialPoseRecoversTranslationAnd120DegreeYaw)
{
  const auto scene = asymmetricScene();
  const auto map_keyframe = pose(12.0, -5.0, 0.5, 0.17);
  const auto keyframe_query = pose(0.22, -0.14, 0.0, 2.0 * kPi / 3.0);
  const auto scan = transformed(*scene, keyframe_query.inverse());
  const Relocalizer relocalizer(database({keyframe("junction", scene, map_keyframe)}));

  // No initial estimate is passed to globalMatch.
  expectCorrectPose(relocalizer.globalMatch(*scan), map_keyframe * keyframe_query);
}

TEST(GlobalRelocalization, ReverseFacingScanRecovers180DegreeYaw)
{
  const auto scene = asymmetricScene();
  const auto map_keyframe = pose(-3.0, 8.0, 0.5, -0.4);
  const auto keyframe_query = pose(-0.16, 0.12, 0.0, kPi);
  const auto scan = transformed(*scene, keyframe_query.inverse());
  const Relocalizer relocalizer(database({keyframe("fire_point", scene, map_keyframe)}));

  expectCorrectPose(relocalizer.globalMatch(*scan), map_keyframe * keyframe_query);
}

TEST(GlobalProposalVerification, RechecksFreshScanAtOdometryPropagatedPose)
{
  const auto scene = asymmetricScene();
  const auto frame = pose(12, -5, .5, .17);
  const auto old_query = pose(.12, -.08, 0, .3);
  const auto new_query = old_query * pose(.4, -.2, .02, .2);
  const Relocalizer registration(database({keyframe("junction", scene, frame)}));
  const auto proposal = registration.globalMatch(*transformed(*scene, old_query.inverse()));
  ASSERT_TRUE(proposal.accepted);
  const auto result = registration.verifyGlobalProposal(
    *transformed(*scene, new_query.inverse()), proposal, old_query.inverse() * new_query);
  expectCorrectPose(result, frame * new_query);
  EXPECT_EQ(result.candidates.size(), proposal.candidates.size());
}

TEST(GlobalProposalVerification, FreshAmbiguityCannotBeHiddenByOldCandidateScores)
{
  const auto scene = asymmetricScene();
  const auto first = pose(0, 0, .5, 0), second = pose(30, 5, .5, 0);
  const auto query = pose(.12, -.08, 0, kPi / 2);
  const auto scan = transformed(*scene, query.inverse());
  const Relocalizer registration(database({keyframe("a", scene, first), keyframe("b", scene, second)}));
  auto proposal = registration.globalMatch(*scan);
  ASSERT_FALSE(proposal.accepted);
  // Simulate a previously unique proposal. The old numeric scores and accepted
  // flag cannot bless the newly ambiguous observation: all poses are rescored.
  proposal.accepted = true;
  for (auto & candidate : proposal.candidates) {
    if (candidate.id == "b") {candidate.score = -100; candidate.geometrically_valid = false;}
  }
  const auto result = registration.verifyGlobalProposal(*scan, proposal, Eigen::Isometry3d::Identity());
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, "ambiguous_match");
  EXPECT_EQ(result.candidates.size(), proposal.candidates.size());
}

TEST(GlobalProposalVerification, InvalidProposalOrNewScanCannotPublishAnAcceptedPose)
{
  const auto scene = asymmetricScene();
  const Relocalizer registration(database({keyframe("junction", scene, pose(0, 0, .5, 0))}));
  EXPECT_FALSE(registration.verifyGlobalProposal(*scene, MatchResult{},
      Eigen::Isometry3d::Identity()).accepted);
  const auto proposal = registration.globalMatch(*scene);
  ASSERT_TRUE(proposal.accepted);
  EXPECT_FALSE(registration.verifyGlobalProposal(Cloud{}, proposal,
      Eigen::Isometry3d::Identity()).accepted);
  auto bad_delta = Eigen::Isometry3d::Identity();
  bad_delta.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(registration.verifyGlobalProposal(*scene, proposal, bad_delta).accepted);
}

TEST(GlobalRelocalization, CachedFeaturesPreserveCandidatesScoresAndPoseAcrossQueries)
{
  const auto scene = asymmetricScene();
  const auto frame = pose(12, -5, .5, .17);
  const auto saved = database({keyframe("junction", scene, frame)});
  RegistrationConfig baseline;
  baseline.cache_registration_features = false;
  const Relocalizer uncached(saved, baseline);
  RegistrationConfig optimized;
  optimized.candidate_threads = 2;
  const Relocalizer cached(saved, optimized);
  for (const auto & query : {pose(.22, -.14, 0, 2*kPi/3), pose(-.16, .12, 0, kPi)}) {
    const auto scan = transformed(*scene, query.inverse());
    const auto before = uncached.globalMatch(*scan);
    const auto after = cached.globalMatch(*scan);
    ASSERT_EQ(before.status, after.status);
    ASSERT_EQ(before.accepted, after.accepted);
    ASSERT_EQ(before.candidates.size(), after.candidates.size());
    EXPECT_LT((before.pose.matrix()-after.pose.matrix()).norm(), 1e-6);
    for (std::size_t i = 0; i < before.candidates.size(); ++i) {
      EXPECT_EQ(before.candidates[i].id, after.candidates[i].id);
      EXPECT_NEAR(before.candidates[i].score, after.candidates[i].score, 1e-6);
      EXPECT_NEAR(before.candidates[i].observability_ratio, after.candidates[i].observability_ratio, 1e-6);
    }
  }
}

TEST(DatabasePersistence, SavedDatabaseLoadsIntoAFreshRelocalizerWithoutAnInitialPose)
{
  TemporaryDirectory temporary;
  ASSERT_FALSE(temporary.root().empty());
  const auto scene = asymmetricScene();
  const auto keyframe_pose = pose(7.0, 11.0, 0.5, 0.37);
  const auto original = database({keyframe("saved_junction", scene, keyframe_pose)});
  std::string error;
  const auto target = temporary.root() / "database";
  ASSERT_TRUE(go2_relocalization::writeDatabase(target.string(), original, error)) << error;

  Database loaded;
  ASSERT_TRUE(go2_relocalization::loadDatabase(target.string(), loaded, error)) << error;
  ASSERT_EQ(loaded.keyframes.size(), 1u);
  EXPECT_EQ(loaded.keyframes.front().id, "saved_junction");
  EXPECT_NEAR(
    (loaded.keyframes.front().T_map_body.matrix() - keyframe_pose.matrix()).norm(),
    0.0, 1.0e-8);
  EXPECT_EQ(loaded.keyframes.front().cloud->size(), scene->size());
  EXPECT_FALSE(loaded.global_cloud->empty());
  const auto keyframe_query = pose(0.1, 0.17, 0.0, -2.0 * kPi / 3.0);
  const auto scan = transformed(*scene, keyframe_query.inverse());
  // Only persistent data survives this construction; no previous match state.
  expectCorrectPose(Relocalizer(std::move(loaded)).globalMatch(*scan),
    keyframe_pose * keyframe_query);
}

TEST(DatabasePersistence, MissingCombinedMapIsReconstructedFromKeyframes)
{
  TemporaryDirectory temporary;
  ASSERT_FALSE(temporary.root().empty());
  const auto scene = asymmetricScene();
  const auto frame_pose = pose(8.0, -2.0, 0.5, 0.2);
  const auto original = database({keyframe("rebuild", scene, frame_pose)});
  std::string error;
  const auto target = temporary.root() / "database";
  ASSERT_TRUE(go2_relocalization::writeDatabase(target.string(), original, error)) << error;
  ASSERT_TRUE(std::filesystem::remove(target / "map.pcd"));

  Database loaded;
  ASSERT_TRUE(go2_relocalization::loadDatabase(target.string(), loaded, error)) << error;
  ASSERT_EQ(loaded.global_cloud->size(), scene->size());
  const auto expected = transformed(*scene, frame_pose);
  for (std::size_t index = 0; index < expected->size(); ++index) {
    const auto & actual = loaded.global_cloud->at(index);
    const auto & point = expected->at(index);
    EXPECT_NEAR(actual.x, point.x, 1.0e-5);
    EXPECT_NEAR(actual.y, point.y, 1.0e-5);
    EXPECT_NEAR(actual.z, point.z, 1.0e-5);
  }
}

TEST(DatabasePersistence, ExistingDatabaseIsNotOverwritten)
{
  TemporaryDirectory temporary;
  ASSERT_FALSE(temporary.root().empty());
  const auto target = temporary.root() / "occupied";
  ASSERT_TRUE(std::filesystem::create_directory(target));
  {
    std::ofstream marker(target / "do_not_replace.txt");
    marker << "original contents";
  }
  std::string error;
  EXPECT_FALSE(go2_relocalization::writeDatabase(
      target.string(), database({keyframe("new", asymmetricScene(), pose(0, 0, 0, 0))}),
      error));
  EXPECT_FALSE(error.empty());
  std::ifstream marker(target / "do_not_replace.txt");
  std::string contents;
  std::getline(marker, contents);
  EXPECT_EQ(contents, "original contents");
  EXPECT_FALSE(std::filesystem::exists(target / "manifest.csv"));
}

TEST(DatabasePersistence, MissingKeyframeCloudFailsInsteadOfProducingAPartialMap)
{
  TemporaryDirectory temporary;
  ASSERT_FALSE(temporary.root().empty());
  const auto target = temporary.root() / "database";
  std::string error;
  ASSERT_TRUE(go2_relocalization::writeDatabase(target.string(),
    database({keyframe("required", asymmetricScene(), pose(0, 0, 0.5, 0))}), error)) << error;
  ASSERT_TRUE(std::filesystem::remove(target / "keyframes" / "required.pcd"));
  Database loaded;
  EXPECT_FALSE(go2_relocalization::loadDatabase(target.string(), loaded, error));
  EXPECT_FALSE(error.empty());
}

TEST(GlobalRelocalization, MovedRobotRejectsOldPoseThenFindsAnotherKeyframe)
{
  const auto first_scene = asymmetricScene(0);
  const auto second_scene = asymmetricScene(1);
  const auto first_pose = pose(-4.0, 2.0, 0.5, 0.3);
  const auto second_pose = pose(21.0, -12.0, 0.5, -0.4);
  const Relocalizer relocalizer(database({
    keyframe("first_place", first_scene, first_pose),
    keyframe("other_place", second_scene, second_pose)}));
  const auto first_query = pose(0.15, -0.1, 0, kPi / 2.0);
  const auto first_scan = transformed(*first_scene, first_query.inverse());
  expectCorrectPose(relocalizer.globalMatch(*first_scan), first_pose * first_query);

  const auto second_query = pose(-0.12, 0.15, 0, -2.0 * kPi / 3.0);
  const auto moved_scan = transformed(*second_scene, second_query.inverse());
  const auto stale_result = relocalizer.track(*moved_scan, first_pose * first_query);
  EXPECT_FALSE(stale_result.accepted) << summarize(stale_result);
  // The global search must escape a stale local estimate after the robot moves.
  expectCorrectPose(relocalizer.globalMatch(*moved_scan), second_pose * second_query);
}

TEST(GlobalRelocalization, IdenticalFarApartPlacesAreRejectedAsAmbiguous)
{
  const auto scene = asymmetricScene();
  const Relocalizer relocalizer(database({
    keyframe("repeated_a", scene, pose(0, 0, 0.5, 0)),
    keyframe("repeated_b", scene, pose(30, 5, 0.5, 0))}));
  const auto keyframe_query = pose(0.12, -0.08, 0, kPi / 2.0);
  const auto scan = transformed(*scene, keyframe_query.inverse());
  const auto result = relocalizer.globalMatch(*scan);

  SCOPED_TRACE(summarize(result));
  EXPECT_FALSE(result.accepted);
  ASSERT_GE(result.candidates.size(), 2u);
  std::vector<Eigen::Vector3d> valid_positions;
  for (const auto & candidate : result.candidates) {
    if (candidate.geometrically_valid) {
      valid_positions.push_back(candidate.pose.translation());
    }
  }
  ASSERT_GE(valid_positions.size(), 2u);
  bool separate_places = false;
  for (const auto & first : valid_positions) {
    for (const auto & second : valid_positions) {
      separate_places = separate_places || (first - second).norm() > 20.0;
    }
  }
  EXPECT_TRUE(separate_places);
  EXPECT_NE(result.status.find("ambig"), std::string::npos);
}

TEST(GlobalRelocalization, PredictionAddsASeedWithoutRemovingGlobalCandidates)
{
  const auto scene = asymmetricScene();
  const auto saved_pose = pose(12, -5, .5, .17);
  const auto query = pose(.22, -.14, 0, 2*kPi/3);
  const auto scan = transformed(*scene, query.inverse());
  const Relocalizer registration(database({keyframe("junction", scene, saved_pose)}));
  const auto baseline = registration.globalMatch(*scan);
  const auto expected = saved_pose * query;
  const auto predicted = registration.globalMatch(*scan, expected);
  expectCorrectPose(predicted, expected);
  ASSERT_EQ(predicted.candidates.size(), baseline.candidates.size() + 1);
  EXPECT_TRUE(std::any_of(predicted.candidates.begin(), predicted.candidates.end(),
    [](const auto & c) {return c.id == "odometry_prior";}));
  EXPECT_DOUBLE_EQ(predicted.quality_score, predicted.candidates.front().score);
  EXPECT_EQ(predicted.best_candidate_id, predicted.candidates.front().id);
  // A completely wrong odometry seed cannot replace the global search.
  expectCorrectPose(registration.globalMatch(*scan, pose(200, 200, .5, 0)), expected);
}

TEST(GlobalRelocalization, PredictionCannotResolveIdenticalPlacesByItself)
{
  const auto scene = asymmetricScene();
  const auto first = pose(0, 0, .5, 0);
  const auto second = pose(30, 5, .5, 0);
  const auto query = pose(.12, -.08, 0, kPi/2);
  const auto scan = transformed(*scene, query.inverse());
  const Relocalizer registration(database({keyframe("a", scene, first),
    keyframe("b", scene, second)}));
  const auto result = registration.globalMatch(*scan, first * query);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.status, "ambiguous_match");
  EXPECT_LT(result.score_gap, RegistrationConfig().min_score_gap);
}

TEST(GlobalRelocalization, StraightTunnelDoesNotClaimObservableAxialPosition)
{
  const auto tunnel = straightTunnel();
  const Relocalizer relocalizer(database({keyframe("featureless_tunnel", tunnel,
    pose(0, 0, 0.5, 0))}));
  const auto result = relocalizer.globalMatch(*tunnel);

  SCOPED_TRACE(summarize(result));
  EXPECT_FALSE(result.accepted);
  ASSERT_FALSE(result.candidates.empty());
  // Require the fixture to exercise degeneracy, rather than passing because
  // registration never converged or because the clouds did not overlap.
  bool fitted_but_unobservable = false;
  const RegistrationConfig production;
  for (const auto & candidate : result.candidates) {
    if (candidate.converged && candidate.overlap >= production.min_overlap &&
      candidate.rmse <= production.max_rmse &&
      candidate.observability_ratio < production.min_observability_ratio)
    {
      fitted_but_unobservable = true;
    }
  }
  EXPECT_TRUE(fitted_but_unobservable);
}

TEST(GlobalRelocalization, EmptyAndNonFiniteCloudsNeverProduceAnAcceptedPose)
{
  const Relocalizer relocalizer(database({keyframe("valid_map", asymmetricScene(),
    pose(0, 0, 0.5, 0))}));
  const Cloud empty;
  EXPECT_FALSE(relocalizer.globalMatch(empty).accepted);
  Cloud invalid;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  for (int index = 0; index < 200; ++index) {
    invalid.push_back(pcl::PointXYZ(nan, infinity, -infinity));
  }
  // Deliberately incorrect metadata: validation must inspect point values.
  invalid.is_dense = true;
  EXPECT_FALSE(relocalizer.globalMatch(invalid).accepted);
}

TEST(GlobalRelocalization, NonFiniteNoiseDoesNotCorruptAValidMatch)
{
  const auto scene = asymmetricScene();
  const auto map_pose = pose(4, -5, 0.5, -0.25);
  const auto keyframe_query = pose(0.12, 0.08, 0, 2.0 * kPi / 3.0);
  auto scan = transformed(*scene, keyframe_query.inverse());
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (int index = 0; index < 200; ++index) {
    scan->push_back(pcl::PointXYZ(nan, 0, 0));
  }
  scan->is_dense = true;
  const Relocalizer relocalizer(database({keyframe("finite_map", scene, map_pose)}));
  expectCorrectPose(relocalizer.globalMatch(*scan), map_pose * keyframe_query);
}

TEST(GlobalRelocalization, UnrelatedCurvedSceneIsRejected)
{
  const Relocalizer relocalizer(database({keyframe("rectangular_place", asymmetricScene(),
    pose(0, 0, 0.5, 0))}));
  Cloud sphere;
  // A complete sphere cannot simultaneously match the observed floor, walls,
  // and asymmetric pillars. Its full normal span avoids a planar-only fixture.
  for (int latitude = 1; latitude < 20; ++latitude) {
    const double polar = kPi * latitude / 20;
    for (int longitude = 0; longitude < 48; ++longitude) {
      const double azimuth = 2 * kPi * longitude / 48;
      addPoint(sphere, {1.2 * std::sin(polar) * std::cos(azimuth),
        1.2 * std::sin(polar) * std::sin(azimuth), 0.7 + 1.2 * std::cos(polar)});
    }
  }
  const auto result = relocalizer.globalMatch(sphere);
  EXPECT_FALSE(result.accepted) << summarize(result);
}

TEST(RegistrationConfiguration, InvalidThresholdsAreRejected)
{
  std::string error;
  RegistrationConfig config;
  EXPECT_TRUE(go2_relocalization::validateConfig(config, error)) << error;
  config.voxel_size = 0;
  EXPECT_FALSE(go2_relocalization::validateConfig(config, error));
  config = RegistrationConfig{};
  config.min_overlap = 1.1;
  EXPECT_FALSE(go2_relocalization::validateConfig(config, error));
  config = RegistrationConfig{};
  config.min_observability_ratio = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(go2_relocalization::validateConfig(config, error));
}
