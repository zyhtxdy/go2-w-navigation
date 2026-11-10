cmake_minimum_required(VERSION 3.16)
project(go2_relocalization LANGUAGES C CXX)

# GICP/Eigen instantiated in this package is too slow without optimization.
# Respect explicitly requested Debug/RelWithDebInfo configurations.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration features)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

add_library(relocalization_registration src/registration.cpp)
target_include_directories(relocalization_registration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(relocalization_registration SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(relocalization_registration PUBLIC Eigen3::Eigen Threads::Threads ${PCL_LIBRARIES})

add_executable(relocalization_node src/relocalization_node.cpp)
target_link_libraries(relocalization_node relocalization_registration)
ament_target_dependencies(relocalization_node rclcpp sensor_msgs nav_msgs geometry_msgs
  std_msgs std_srvs diagnostic_msgs pcl_conversions)
add_executable(database_recorder src/database_recorder.cpp)
target_link_libraries(database_recorder relocalization_registration)
ament_target_dependencies(database_recorder rclcpp sensor_msgs nav_msgs std_srvs pcl_conversions)
add_executable(database_builder src/database_builder.cpp)
target_link_libraries(database_builder relocalization_registration)
add_executable(registration_benchmark src/registration_benchmark.cpp)
target_link_libraries(registration_benchmark relocalization_registration)

install(TARGETS relocalization_registration relocalization_node database_recorder database_builder registration_benchmark
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_registration test/test_registration.cpp)
  target_link_libraries(test_registration relocalization_registration)
  ament_add_gtest(test_runtime test/test_runtime.cpp)
  target_link_libraries(test_runtime relocalization_registration)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_shadow.launch.py TARGET relocalization_shadow TIMEOUT 120)
  set_tests_properties(relocalization_shadow PROPERTIES ENVIRONMENT
    "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs;ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/ros_home")
endif()
ament_export_include_directories(include)
ament_export_libraries(relocalization_registration)
ament_export_dependencies(Eigen3)
ament_package()
