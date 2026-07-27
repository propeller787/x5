cmake_minimum_required(VERSION 3.8)
project(safety_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(common REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

add_definitions(${PCL_DEFINITIONS})

set(SAFETY_NODE_CORE_SOURCES
  src/core/avoidance_utils.cpp
  src/core/direction_mask.cpp
  src/core/vehicle_footprint.cpp
  src/core/motion_rollout.cpp
  src/core/swept_footprint_checker.cpp
  src/core/safety_filter.cpp
  src/core/emergency_brake.cpp
  src/core/sudden_obstacle_detector.cpp
  src/core/obstacle_field.cpp
  src/core/pulse_compensator.cpp
  src/core/narrow_passage_planner.cpp
  src/core/safety_verdict.cpp
  src/core/closing_speed_estimator.cpp
  src/core/degradation_policy.cpp
  src/core/front_lowpass_stop.cpp
  src/core/front_tilt_speed_cap.cpp
  src/core/warning_reporter.cpp
)

set(SAFETY_NODE_SOURCES
  ${SAFETY_NODE_CORE_SOURCES}
  src/safety_params.cpp
  src/rolling_local_map.cpp
  src/map_constraint_check.cpp
  src/mode_supervisor.cpp
  src/avoidance_controller.cpp
  src/output_shaper.cpp
  src/perception_utils.cpp
  src/joystick_utils.cpp
  src/lidar_perception.cpp
  src/vehicle_io.cpp
  src/safety_node.cpp
)

add_library(${PROJECT_NAME}_lib ${SAFETY_NODE_SOURCES})

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  geometry_msgs
  interface
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  common
  pcl_conversions
)

target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES})

add_executable(safety_node_node src/main.cpp)
target_link_libraries(safety_node_node ${PROJECT_NAME}_lib ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_lib safety_node_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rolling_local_map test/test_rolling_local_map.cpp)
  target_link_libraries(test_rolling_local_map ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_rolling_local_map rclcpp nav_msgs)

  ament_add_gtest(test_map_constraint_check test/test_map_constraint_check.cpp)
  target_link_libraries(test_map_constraint_check ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_map_constraint_check rclcpp)

  ament_add_gtest(test_avoidance_controller test/test_avoidance_controller.cpp)
  target_link_libraries(test_avoidance_controller ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_avoidance_controller rclcpp geometry_msgs)

  ament_add_gtest(test_output_shaper test/test_output_shaper.cpp)
  target_link_libraries(test_output_shaper ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_output_shaper geometry_msgs)

  ament_add_gtest(test_motion_rollout test/test_motion_rollout.cpp)
  target_link_libraries(test_motion_rollout ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_motion_rollout geometry_msgs)

  ament_add_gtest(test_vehicle_io test/test_vehicle_io.cpp)
  target_link_libraries(test_vehicle_io ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_vehicle_io rclcpp geometry_msgs interface)

  ament_add_gtest(test_safety_node_io test/test_safety_node_io.cpp)
  target_link_libraries(test_safety_node_io ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(
    test_safety_node_io
    rclcpp
    geometry_msgs
    interface
    nav_msgs
    sensor_msgs
    std_msgs
    tf2_ros
    visualization_msgs)

  ament_add_gtest(test_perception_utils test/test_perception_utils.cpp)
  target_link_libraries(test_perception_utils ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_perception_utils rclcpp)

  ament_add_gtest(test_joystick_utils test/test_joystick_utils.cpp)
  target_link_libraries(test_joystick_utils ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_joystick_utils interface geometry_msgs)

  ament_add_gtest(test_lidar_perception test/test_lidar_perception.cpp)
  target_link_libraries(test_lidar_perception ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_lidar_perception rclcpp sensor_msgs tf2_ros)

  ament_add_gtest(test_core_safety test/test_core_safety.cpp)
  target_link_libraries(test_core_safety ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_core_safety rclcpp geometry_msgs)

  ament_add_gtest(test_safety_filter test/test_safety_filter.cpp)
  target_link_libraries(test_safety_filter ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_safety_filter rclcpp geometry_msgs)

  ament_add_gtest(test_t3_sil test/test_t3_sil.cpp)
  target_link_libraries(test_t3_sil ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_t3_sil rclcpp geometry_msgs)

  ament_add_gtest(test_precise_stop_narrow test/test_precise_stop_narrow.cpp)
  target_link_libraries(test_precise_stop_narrow ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_precise_stop_narrow rclcpp geometry_msgs)

  ament_add_gtest(test_direction_mask test/test_direction_mask.cpp)
  target_link_libraries(test_direction_mask ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_direction_mask rclcpp geometry_msgs)

  # ── PRODUCTION_HARDENING_PLAN 新增测试 ──
  ament_add_gtest(test_param_validation test/test_param_validation.cpp)
  target_link_libraries(test_param_validation ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_param_validation rclcpp)

  ament_add_gtest(test_safety_arbiter test/test_safety_arbiter.cpp)
  target_link_libraries(test_safety_arbiter ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_safety_arbiter rclcpp geometry_msgs)

  ament_add_gtest(test_closing_speed_estimator test/test_closing_speed_estimator.cpp)
  target_link_libraries(test_closing_speed_estimator ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_closing_speed_estimator rclcpp)

  ament_add_gtest(test_degradation_policy test/test_degradation_policy.cpp)
  target_link_libraries(test_degradation_policy ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_degradation_policy rclcpp)

  ament_add_gtest(test_swept_cull test/test_swept_cull.cpp)
  target_link_libraries(test_swept_cull ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_swept_cull rclcpp geometry_msgs)

  ament_add_gtest(test_ego_motion_map test/test_ego_motion_map.cpp)
  target_link_libraries(test_ego_motion_map ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_ego_motion_map rclcpp nav_msgs)

  ament_add_gtest(test_retreat_escape test/test_retreat_escape.cpp)
  target_link_libraries(test_retreat_escape ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_retreat_escape rclcpp)

  ament_add_gtest(test_front_lowpass_stop test/test_front_lowpass_stop.cpp)
  target_link_libraries(test_front_lowpass_stop ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_front_lowpass_stop rclcpp geometry_msgs)

  ament_add_gtest(test_front_tilt_speed_cap test/test_front_tilt_speed_cap.cpp)
  target_link_libraries(test_front_tilt_speed_cap ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_front_tilt_speed_cap rclcpp geometry_msgs)

  ament_add_gtest(test_warning_reporter test/test_warning_reporter.cpp)
  target_link_libraries(test_warning_reporter ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
  ament_target_dependencies(test_warning_reporter rclcpp)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  interface
  nav_msgs
  sensor_msgs
  visualization_msgs
)

ament_package()
