cmake_minimum_required(VERSION 3.8)
project(lidar_imu_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/lidar_imu_fusion_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(three_lidar_imu_fusion_node src/main.cpp)
target_link_libraries(three_lidar_imu_fusion_node ${PROJECT_NAME}_lib)
target_compile_features(three_lidar_imu_fusion_node PUBLIC cxx_std_17)

install(TARGETS ${PROJECT_NAME}_lib three_lidar_imu_fusion_node
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

# 安装验证工具脚本，使其可通过 ros2 run lidar_imu_fusion fusion_validator.py 调用
install(PROGRAMS scripts/fusion_validator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_lidar_imu_fusion test/test_lidar_imu_fusion.cpp)
  target_link_libraries(test_lidar_imu_fusion ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_lidar_imu_fusion rclcpp sensor_msgs tf2)


  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
