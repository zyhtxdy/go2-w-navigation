#include "go2_relocalization/registration.hpp"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <stdexcept>

// Runs matching on saved local scans. Pose comparison uses the recorded LIO
// trajectory, not external ground truth. No ROS node, TF or command is created.
int main(int argc, char **argv)
{
  if (argc < 2) {std::cerr << "Usage: registration_benchmark database [cached|uncached] [threads]\n"; return 2;}
  try {
    go2_relocalization::Database database;
    std::string error;
    if (!go2_relocalization::loadDatabase(argv[1], database, error)) {throw std::runtime_error(error);}
    go2_relocalization::RegistrationConfig config;
    config.cache_registration_features = argc < 3 || std::string(argv[2]) != "uncached";
    if (argc > 3) {config.candidate_threads = std::stoi(argv[3]);}
    go2_relocalization::Relocalizer registration(database, config);
    std::cout << "keyframe,cache,milliseconds,status,candidates,overlap,rmse,translation_from_recorded_pose,verification_ms,verification_status,verification_candidates,verification_pose_change_m\n";
    for (const std::size_t index : {std::size_t(0), database.keyframes.size()/2, database.keyframes.size()-1}) {
      const auto & keyframe = database.keyframes[index];
      const auto start = std::chrono::steady_clock::now();
      const auto result = registration.globalMatch(*keyframe.cloud);
      const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      const auto verify_start = std::chrono::steady_clock::now();
      const auto verified = registration.verifyGlobalProposal(
        *keyframe.cloud, result, Eigen::Isometry3d::Identity());
      const double verify_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - verify_start).count();
      std::cout << std::setprecision(10) << keyframe.id << ',' << config.cache_registration_features << ','
                << ms << ',' << result.status << ',' << result.candidates.size() << ',' << result.overlap
                << ',' << result.rmse << ',' <<
                (result.pose.translation()-keyframe.T_map_body.translation()).norm() << ',' <<
                verify_ms << ',' << verified.status << ',' << verified.candidates.size() << ',' <<
                (verified.pose.translation()-result.pose.translation()).norm() << std::endl;
    }
    return 0;
  } catch (const std::exception & e) {std::cerr << e.what() << '\n'; return 1;}
}
