cmake_minimum_required(VERSION 3.8)
project(fast_lio_nav2)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(lio_base_odom_node src/lio_base_odom_node.cpp)
ament_target_dependencies(
  lio_base_odom_node
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(imu_bias_corrector_node src/imu_bias_corrector_node.cpp)
target_include_directories(imu_bias_corrector_node PRIVATE include)
ament_target_dependencies(
  imu_bias_corrector_node
  rclcpp
  sensor_msgs
)

install(TARGETS lio_base_odom_node imu_bias_corrector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# The offline planner does not need the navigation controller/BT library.
# Install the canonical navigation parameters here as well so its default
# launch works without compiling the full navigation_bringup package.
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../nav2/navigation_bringup/config/nav2_mppi_params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS scripts/rviz_goal_planner.py scripts/replay_starter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_gyro_bias_calibration test/test_gyro_bias_calibration.cpp)
  target_include_directories(test_gyro_bias_calibration PRIVATE include)
  ament_add_gtest(test_sensor_adapters test/test_sensor_adapters.cpp TIMEOUT 30)
  target_include_directories(test_sensor_adapters PRIVATE include)
  ament_target_dependencies(test_sensor_adapters
    nav_msgs rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
  # Unit tests call adapter callbacks locally with an in-memory TF buffer. The
  # private domain and localhost scope prevent discovery of live robot nodes.
  set_tests_properties(test_sensor_adapters PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=178;ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp;FASTRTPS_DEFAULT_PROFILES_FILE=;FASTDDS_DEFAULT_PROFILES_FILE=;ROS_DISCOVERY_SERVER=")
  ament_add_pytest_test(test_replay_starter test/test_replay_starter.py TIMEOUT 30)
endif()

ament_package()
