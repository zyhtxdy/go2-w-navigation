cmake_minimum_required(VERSION 3.8)
project(astar_3d_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(terrain_navigation_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)

add_library(${PROJECT_NAME} SHARED
  src/occupy_map.cpp
  src/ground_support.cpp
  src/astar_3d.cpp
  src/astar_3d_planner.cpp
  src/cloud_observations.cpp
  src/start_diagnostics.cpp
  src/always_current_layer.cpp
  src/swept_footprint.cpp
  src/goal_connections.cpp
  src/path_refiner.cpp
  src/path_reconnection.cpp
  src/fixed_map_guard.cpp
)
target_link_libraries(${PROJECT_NAME} OpenSSL::Crypto)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_msgs
  nav_msgs
  terrain_navigation_interfaces
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_layer_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(
  FILES
    global_planner_plugin.xml
    costmap_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_goal_connections test/test_goal_connections.cpp)
  target_compile_features(test_goal_connections PUBLIC cxx_std_17)
  target_link_libraries(test_goal_connections ${PROJECT_NAME})
  target_compile_definitions(test_goal_connections PRIVATE
    TEST_TERMINAL_MAP_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test/data/terminal_map_20261029")
  ament_add_gtest(test_path_reconnection test/test_path_reconnection.cpp)
  target_compile_features(test_path_reconnection PUBLIC cxx_std_17)
  target_link_libraries(test_path_reconnection ${PROJECT_NAME})
  ament_add_gtest(test_local_detour_budget test/test_local_detour_budget.cpp)
  target_compile_features(test_local_detour_budget PUBLIC cxx_std_17)
  target_include_directories(test_local_detour_budget PRIVATE include)
  ament_add_gtest(test_rectangular_prefilter test/test_rectangular_prefilter.cpp)
  target_compile_features(test_rectangular_prefilter PUBLIC cxx_std_17)
  target_link_libraries(test_rectangular_prefilter ${PROJECT_NAME})
  ament_add_gtest(test_goal_maneuvers test/test_goal_maneuvers.cpp)
  target_compile_features(test_goal_maneuvers PUBLIC cxx_std_17)
  target_link_libraries(test_goal_maneuvers ${PROJECT_NAME})
  ament_add_gtest(test_search_recovery test/test_search_recovery.cpp)
  target_compile_features(test_search_recovery PUBLIC cxx_std_17)
  target_link_libraries(test_search_recovery ${PROJECT_NAME})
  ament_add_gtest(test_ground_inference test/test_ground_inference.cpp)
  target_compile_features(test_ground_inference PUBLIC cxx_std_17)
  target_link_libraries(test_ground_inference ${PROJECT_NAME})
  ament_add_gtest(test_tunnel_maneuvers test/test_tunnel_maneuvers.cpp)
  target_compile_features(test_tunnel_maneuvers PUBLIC cxx_std_17)
  target_link_libraries(test_tunnel_maneuvers ${PROJECT_NAME})
  ament_add_gtest(test_live_obstacle_health test/test_live_obstacle_health.cpp)
  target_compile_features(test_live_obstacle_health PUBLIC cxx_std_17)
  target_link_libraries(test_live_obstacle_health ${PROJECT_NAME})
  ament_add_gtest(test_fixed_map_guard test/test_fixed_map_guard.cpp)
  target_compile_features(test_fixed_map_guard PUBLIC cxx_std_17)
  target_link_libraries(test_fixed_map_guard ${PROJECT_NAME})
  ament_add_gtest(test_fixed_map_integration test/test_fixed_map_integration.cpp)
  target_compile_features(test_fixed_map_integration PUBLIC cxx_std_17)
  target_link_libraries(test_fixed_map_integration ${PROJECT_NAME})
  ament_target_dependencies(test_fixed_map_integration
    rclcpp rclcpp_lifecycle nav2_costmap_2d nav2_core sensor_msgs std_msgs)
  ament_add_gtest(test_support_snapshot test/test_support_snapshot.cpp)
  target_compile_features(test_support_snapshot PUBLIC cxx_std_17)
  target_link_libraries(test_support_snapshot ${PROJECT_NAME})
  ament_target_dependencies(test_support_snapshot
    rclcpp rclcpp_lifecycle nav2_costmap_2d nav2_core sensor_msgs)
  ament_add_gtest(test_offline_goal_selection test/test_offline_goal_selection.cpp)
  target_compile_features(test_offline_goal_selection PUBLIC cxx_std_17)
  target_link_libraries(test_offline_goal_selection ${PROJECT_NAME})
  target_compile_definitions(test_offline_goal_selection PRIVATE
    TEST_MAP_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/../../../maps/lab_v1_navigation"
    TEST_SPARSE_MAP_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test/data/sparse_ground_20261028"
    TEST_TERMINAL_MAP_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test/data/terminal_map_20261029")
  ament_target_dependencies(test_offline_goal_selection
    rclcpp rclcpp_lifecycle nav2_costmap_2d nav2_core sensor_msgs)
  ament_add_gtest(test_swept_footprint test/test_swept_footprint.cpp)
  target_compile_features(test_swept_footprint PUBLIC cxx_std_17)
  target_link_libraries(test_swept_footprint ${PROJECT_NAME})
  ament_add_gtest(test_static_map test/test_static_map.cpp)
  target_compile_features(test_static_map PUBLIC cxx_std_17)
  target_link_libraries(test_static_map ${PROJECT_NAME})
  ament_add_gtest(test_support_updates test/test_support_updates.cpp)
  target_compile_features(test_support_updates PUBLIC cxx_std_17)
  target_link_libraries(test_support_updates ${PROJECT_NAME})
  ament_add_gtest(test_astar_3d test/test_astar_3d.cpp)
  target_compile_features(test_astar_3d PUBLIC cxx_std_17)
  target_include_directories(test_astar_3d PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_astar_3d ${PROJECT_NAME})

  ament_add_gtest(test_height_datum test/test_height_datum.cpp)
  target_compile_features(test_height_datum PUBLIC cxx_std_17)
  target_link_libraries(test_height_datum ${PROJECT_NAME})

  ament_add_gtest(test_terrain_inputs test/test_terrain_inputs.cpp)
  target_compile_features(test_terrain_inputs PUBLIC cxx_std_17)
  target_link_libraries(test_terrain_inputs ${PROJECT_NAME})

  ament_add_gtest(test_start_diagnostics test/test_start_diagnostics.cpp)
  target_compile_features(test_start_diagnostics PUBLIC cxx_std_17)
  target_link_libraries(test_start_diagnostics ${PROJECT_NAME})

  ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp)
  target_compile_features(test_plugin_loading PUBLIC cxx_std_17)
  ament_target_dependencies(test_plugin_loading nav2_core pluginlib)

endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs nav2_core nav2_costmap_2d nav2_util nav2_msgs nav_msgs terrain_navigation_interfaces pluginlib rclcpp
  rclcpp_lifecycle sensor_msgs std_msgs visualization_msgs tf2 tf2_geometry_msgs tf2_ros
)

ament_package()
