cmake_minimum_required(VERSION 3.8)
project(wheelchair_obstacle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

add_definitions(${PCL_DEFINITIONS})

add_executable(obstacle_cloud_filter_node
  src/obstacle_cloud_filter_node.cpp
)

target_include_directories(obstacle_cloud_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_compile_features(obstacle_cloud_filter_node PUBLIC cxx_std_17)

target_link_libraries(obstacle_cloud_filter_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(obstacle_cloud_filter_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

add_executable(joystick_local_path_node
  src/joystick_local_path_node.cpp
)

target_compile_features(joystick_local_path_node PUBLIC cxx_std_17)

ament_target_dependencies(joystick_local_path_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  nav2_msgs
  nav2_costmap_2d
  interface
  tf2
)

target_link_libraries(joystick_local_path_node
  nav2_costmap_2d::nav2_costmap_2d_core
)

add_executable(cmd_vel_safety_gate_node
  src/cmd_vel_safety_gate_node.cpp
)

target_compile_features(cmd_vel_safety_gate_node PUBLIC cxx_std_17)

ament_target_dependencies(cmd_vel_safety_gate_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  interface
)

# 方案 B：每周期整图重建的 ObstacleLayer 插件（nav2 costmap 层）。
add_library(rebuild_obstacle_layer SHARED
  src/rebuild_obstacle_layer.cpp
)

target_include_directories(rebuild_obstacle_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(rebuild_obstacle_layer PUBLIC cxx_std_17)

ament_target_dependencies(rebuild_obstacle_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
)

# 向 nav2_costmap_2d 导出插件描述，使其能被 pluginlib 发现。
pluginlib_export_plugin_description_file(nav2_costmap_2d rebuild_obstacle_layer.xml)

install(TARGETS
  obstacle_cloud_filter_node
  joystick_local_path_node
  cmd_vel_safety_gate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS rebuild_obstacle_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES rebuild_obstacle_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cyclic_rosbag_record.sh
  scripts/replay_mppi_bag.sh
  scripts/compare_mppi_bag_replay.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
