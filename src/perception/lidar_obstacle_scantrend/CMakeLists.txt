cmake_minimum_required(VERSION 3.8)
project(lidar_obstacle_scantrend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -fno-omit-frame-pointer
  )
  add_compile_options(
    $<$<CONFIG:Debug>:-g>
    $<$<CONFIG:Debug>:-O0>
  )
  add_compile_options(
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Release>:-DNDEBUG>
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../utils/
  ${PROJECT_SOURCE_DIR}/../../common/
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

file(GLOB_RECURSE cpp_srcs ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE utils_cpp_srcs ${PROJECT_SOURCE_DIR}/../utils/*.cpp)

add_executable(lidar_obstacle_scantrend_node ${utils_cpp_srcs} ${cpp_srcs})
target_include_directories(lidar_obstacle_scantrend_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lidar_obstacle_scantrend_node PUBLIC c_std_99 cxx_std_17)

target_link_libraries(lidar_obstacle_scantrend_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp
)

ament_target_dependencies(
  lidar_obstacle_scantrend_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
)

install(TARGETS lidar_obstacle_scantrend_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
