cmake_minimum_required(VERSION 3.8)
project(navigation_bringup)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(terrain_navigation_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_library(is_terrain_path_valid_condition_bt_node SHARED
  src/is_terrain_path_valid_condition.cpp
)
target_compile_features(is_terrain_path_valid_condition_bt_node PUBLIC cxx_std_17)
target_compile_definitions(
  is_terrain_path_valid_condition_bt_node PRIVATE BT_PLUGIN_EXPORT
)
ament_target_dependencies(is_terrain_path_valid_condition_bt_node
  behaviortree_cpp_v3
  nav2_behavior_tree
  nav2_msgs
  nav_msgs
  terrain_navigation_interfaces
  rclcpp
)

install(DIRECTORY
  behavior_trees
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS is_terrain_path_valid_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terrain_validation_policy test/test_terrain_validation_policy.cpp)
  target_compile_features(test_terrain_validation_policy PUBLIC cxx_std_17)
  target_compile_definitions(test_terrain_validation_policy PRIVATE
    TERRAIN_BT_PLUGIN_LIBRARY="$<TARGET_FILE:is_terrain_path_valid_condition_bt_node>")
  ament_target_dependencies(test_terrain_validation_policy
    behaviortree_cpp_v3 nav2_behavior_tree nav_msgs rclcpp terrain_navigation_interfaces)
  add_dependencies(test_terrain_validation_policy is_terrain_path_valid_condition_bt_node)
  ament_add_gtest(test_bt_plugin_loading test/test_bt_plugin_loading.cpp)
  target_compile_features(test_bt_plugin_loading PUBLIC cxx_std_17)
  target_compile_definitions(
    test_bt_plugin_loading PRIVATE
    TERRAIN_BT_PLUGIN_LIBRARY="$<TARGET_FILE:is_terrain_path_valid_condition_bt_node>"
  )
  ament_target_dependencies(test_bt_plugin_loading behaviortree_cpp_v3)
  add_dependencies(
    test_bt_plugin_loading is_terrain_path_valid_condition_bt_node
  )
endif()

ament_package()
