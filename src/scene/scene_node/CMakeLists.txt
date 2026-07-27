cmake_minimum_required(VERSION 3.8)
project(scene_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(common REQUIRED)
find_package(interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# find_package(PCL 1.12 REQUIRED COMPONENTS common io filters)
include(../../common/cmake/add3rd.cmake)
add3rds(utils muduo)
add_definitions(${PCL_DEFINITIONS})

include_directories(include ${PCL_INCLUDE_DIRS})

link_directories(
  ${PCL_LIBRARIES}
)

# 阶段 A：虚拟导轨默认不编译，需要时 -DSCENE_NODE_BUILD_GUIDE_RAIL=ON
option(SCENE_NODE_BUILD_GUIDE_RAIL "Build virtual guide rail modules" OFF)

set(SCENE_NODE_CORE_SOURCES
  src/lidar_driver.cpp
  src/vehicle_interface.cpp
  src/clustering.cpp
  src/avoidance_types.cpp
  src/obstacle_field.cpp
  src/motion_rollout.cpp
  src/swept_footprint_checker.cpp
  src/safety_filter.cpp
  src/risk_assessment.cpp
  src/decision_maker_params_io.cpp
  src/decision_maker.cpp
  src/tracking.cpp
  src/smooth_filter.cpp
  src/speed_reduction_kalman_filter.cpp
  src/environment_detector.cpp
  src/obstacle_predictor.cpp
  src/evasion_detector.cpp
)

if(SCENE_NODE_BUILD_GUIDE_RAIL)
  list(APPEND SCENE_NODE_CORE_SOURCES
    src/wall_detector.cpp
    src/center_line_calculator.cpp
    src/path_tracker.cpp
    src/virtual_guide_rail_controller.cpp
  )
endif()

# 创建库文件
add_library(${PROJECT_NAME} ${SCENE_NODE_CORE_SOURCES})

if(SCENE_NODE_BUILD_GUIDE_RAIL)
  target_compile_definitions(${PROJECT_NAME} PUBLIC SCENE_NODE_BUILD_GUIDE_RAIL=1)
else()
  target_compile_definitions(${PROJECT_NAME} PUBLIC SCENE_NODE_BUILD_GUIDE_RAIL=0)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  common
  interface
  laser_geometry
  sensor_msgs
  pcl_conversions
)

# 创建可执行文件
add_executable(scene_node_node
  src/main.cpp
  src/scene_node.cpp
)
target_link_libraries(scene_node_node ${PROJECT_NAME} ${PCL_LIBRARIES})

# 安装目标
install(TARGETS ${PROJECT_NAME} scene_node_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
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

  # DecisionMaker 核心业务测试
  ament_add_gtest(test_decision_maker test/test_decision_maker.cpp)
  target_link_libraries(test_decision_maker ${PROJECT_NAME} ${PCL_LIBRARIES})
  ament_target_dependencies(test_decision_maker
    rclcpp geometry_msgs std_msgs common interface sensor_msgs pcl_conversions tf2)

  # SmoothFilter 单元测试
  ament_add_gtest(test_smooth_filter test/test_smooth_filter.cpp)
  target_link_libraries(test_smooth_filter ${PROJECT_NAME} ${PCL_LIBRARIES})
  ament_target_dependencies(test_smooth_filter
    rclcpp geometry_msgs std_msgs common interface sensor_msgs pcl_conversions tf2)

  # Swept-footprint avoidance pure-logic tests
  ament_add_gtest(test_swept_avoidance test/test_swept_avoidance.cpp)
  target_link_libraries(test_swept_avoidance ${PROJECT_NAME} ${PCL_LIBRARIES})
  ament_target_dependencies(test_swept_avoidance
    rclcpp geometry_msgs std_msgs common interface sensor_msgs pcl_conversions tf2)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  Eigen3
)

ament_package()
