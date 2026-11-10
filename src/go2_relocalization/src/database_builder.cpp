#include "go2_relocalization/registration.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {return "";}
  return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> result;
  size_t begin = 0;
  do {
    const size_t end = line.find(',', begin);
    result.push_back(trim(line.substr(begin, end == std::string::npos ? end : end - begin)));
    if (end == std::string::npos) {break;}
    begin = end + 1;
  } while (true);
  return result;
}

double number(const std::string & value)
{
  size_t consumed = 0;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result)) {
    throw std::invalid_argument("Invalid finite numeric value: " + value);
  }
  return result;
}

bool validId(const std::string & id)
{
  return !id.empty() && std::all_of(id.begin(), id.end(), [](unsigned char character) {
    return (character >= 'a' && character <= 'z') ||
           (character >= 'A' && character <= 'Z') ||
           (character >= '0' && character <= '9') || character == '_' || character == '-';
  });
}

std::vector<go2_relocalization::Keyframe> readPoses(const std::string & filename)
{
  std::ifstream input(filename);
  if (!input) {throw std::runtime_error("Cannot open pose CSV: " + filename);}
  std::string line;
  if (!std::getline(input, line)) {throw std::runtime_error("Pose CSV is empty");}
  if (line.compare(0, 3, "\xEF\xBB\xBF") == 0) {line.erase(0, 3);}
  const auto headers = split(line);
  std::map<std::string, size_t> columns;
  for (size_t i = 0; i < headers.size(); ++i) {
    if (headers[i].empty() || !columns.emplace(headers[i], i).second) {
      throw std::runtime_error("Pose CSV contains empty or duplicate header");
    }
  }
  for (const auto & required : {"x", "y", "z", "qx", "qy", "qz", "qw"}) {
    if (columns.count(required) == 0) {
      throw std::runtime_error("Pose CSV missing required column: " + std::string(required));
    }
  }
  std::vector<go2_relocalization::Keyframe> poses;
  std::set<std::string> ids;
  size_t row = 1;
  while (std::getline(input, line)) {
    ++row;
    if (trim(line).empty()) {continue;}
    try {
      const auto cells = split(line);
      if (cells.size() != headers.size()) {throw std::runtime_error("Incorrect number of columns");}
      const auto value = [&cells, &columns](const std::string & name) {
          return number(cells.at(columns.at(name)));
        };
      go2_relocalization::Keyframe keyframe;
      Eigen::Quaterniond orientation(value("qw"), value("qx"), value("qy"), value("qz"));
      if (std::abs(orientation.norm() - 1.0) > 0.001) {
        throw std::runtime_error("Quaternion must be normalized and nonzero");
      }
      orientation.normalize();
      keyframe.T_map_body.translation() = Eigen::Vector3d(value("x"), value("y"), value("z"));
      keyframe.T_map_body.linear() = orientation.toRotationMatrix();
      if (columns.count("id")) {keyframe.id = cells.at(columns.at("id"));}
      else {
        std::ostringstream id;
        id << "k" << std::setw(6) << std::setfill('0') << poses.size();
        keyframe.id = id.str();
      }
      if (!validId(keyframe.id) || !ids.insert(keyframe.id).second) {
        throw std::runtime_error("Keyframe id is invalid or repeated");
      }
      poses.push_back(std::move(keyframe));
      if (poses.size() > 5000) {throw std::runtime_error("At most 5000 poses are supported");}
    } catch (const std::exception & error) {
      throw std::runtime_error("Pose CSV row " + std::to_string(row) + ": " + error.what());
    }
  }
  if (poses.empty()) {throw std::runtime_error("Pose CSV contains no poses");}
  return poses;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "Usage: database_builder MAP.pcd POSES.csv OUTPUT_DIR\n"
              << "POSES.csv header: [id,]x,y,z,qx,qy,qz,qw; poses are map -> body.\n"
              << "Uses a 20 m spherical crop and 0.2 m voxel filter at each pose.\n"
              << "These virtual views include unseen/occluded map points and do not replace "
              << "recorded Scan Context keyframes. OUTPUT_DIR must not exist.\n";
    return 2;
  }
  try {
    if (std::filesystem::exists(argv[3])) {
      throw std::runtime_error("Output directory exists; refusing overwrite");
    }
    go2_relocalization::Cloud raw;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], raw) != 0) {
      throw std::runtime_error("Cannot load map PCD: " + std::string(argv[1]));
    }
    go2_relocalization::Database database;
    database.global_cloud->reserve(raw.size());
    for (const auto & point : raw) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        database.global_cloud->push_back(point);
      }
    }
    if (database.global_cloud->size() < 100) {
      throw std::runtime_error("Map requires at least 100 finite XYZ points");
    }
    auto poses = readPoses(argv[2]);
    for (auto & keyframe : poses) {
      go2_relocalization::Cloud::Ptr crop(new go2_relocalization::Cloud);
      const Eigen::Isometry3d inverse = keyframe.T_map_body.inverse();
      for (const auto & point : *database.global_cloud) {
        const Eigen::Vector3d local = inverse * point.getVector3fMap().cast<double>();
        if (local.squaredNorm() <= 400.0) {
          crop->push_back(pcl::PointXYZ(
            static_cast<float>(local.x()), static_cast<float>(local.y()), static_cast<float>(local.z())));
        }
      }
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setLeafSize(0.2f, 0.2f, 0.2f);
      voxel.setInputCloud(crop);
      voxel.filter(*keyframe.cloud);
      if (keyframe.cloud->size() < 100) {
        throw std::runtime_error(
                "Pose " + keyframe.id + " yields fewer than 100 filtered map points");
      }
      database.keyframes.push_back(std::move(keyframe));
    }
    std::string error;
    if (!go2_relocalization::writeDatabase(argv[3], database, error)) {
      throw std::runtime_error(error);
    }
    std::cout << "Built " << database.keyframes.size() << " virtual keyframes and "
              << database.global_cloud->size() << " map points in " << argv[3] << "\n"
              << "Virtual map crops are not visibility-correct Scan Context templates; "
              << "use recorded body keyframes for dependable retrieval.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Database build failed: " << error.what() << "\n";
    return 1;
  }
}
