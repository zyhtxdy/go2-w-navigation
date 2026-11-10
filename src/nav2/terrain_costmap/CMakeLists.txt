cmake_minimum_required(VERSION 3.8)
project(terrain_costmap)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(terrain_analyzer
  src/terrain_analyzer.cpp
  src/terrain_hazard_stabilizer.cpp
  src/near_obstacle_detector.cpp
  src/evidence_clearing.cpp
)
target_compile_features(terrain_analyzer PUBLIC cxx_std_17)
target_include_directories(terrain_analyzer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(terrain_analyzer Eigen3::Eigen)

add_executable(terrain_costmap_node src/terrain_costmap_node.cpp)
target_compile_features(terrain_costmap_node PUBLIC cxx_std_17)
target_link_libraries(terrain_costmap_node terrain_analyzer)
ament_target_dependencies(
  terrain_costmap_node
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS terrain_analyzer terrain_costmap_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_terrain_analyzer test/test_terrain_analyzer.cpp)
  target_link_libraries(test_terrain_analyzer terrain_analyzer)
  ament_add_gtest(test_obstacle_observation_time test/test_obstacle_observation_time.cpp)
  target_link_libraries(test_obstacle_observation_time terrain_analyzer)
  ament_add_gtest(test_near_obstacle_detector test/test_near_obstacle_detector.cpp)
  target_link_libraries(test_near_obstacle_detector terrain_analyzer)
  ament_add_gtest(test_evidence_clearing test/test_evidence_clearing.cpp)
  target_link_libraries(test_evidence_clearing terrain_analyzer)
  ament_add_pytest_test(test_pose_waiting test/test_pose_waiting.py
    TIMEOUT 45
    ENV "TERRAIN_NODE_EXECUTABLE=$<TARGET_FILE:terrain_costmap_node>"
  )
  ament_add_pytest_test(test_obstacle_time_node test/test_obstacle_time_node.py
    TIMEOUT 30
    ENV "TERRAIN_NODE_EXECUTABLE=$<TARGET_FILE:terrain_costmap_node>"
  )
  ament_add_pytest_test(test_near_obstacle_node test/test_near_obstacle_node.py
    TIMEOUT 30
    ENV "TERRAIN_NODE_EXECUTABLE=$<TARGET_FILE:terrain_costmap_node>"
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(terrain_analyzer)
ament_package()
