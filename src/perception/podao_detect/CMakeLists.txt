cmake_minimum_required(VERSION 3.8)
project(podao_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(yaml-cpp REQUIRED)


include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../utils/
  ${PROJECT_SOURCE_DIR}/../../common/
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARIES}
)


file(GLOB_RECURSE cpp_srcs ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE utils_cpp_srcs ${PROJECT_SOURCE_DIR}/../utils/*.cpp)

add_executable(podao_detect_node ${utils_cpp_srcs} ${cpp_srcs})
target_include_directories(podao_detect_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(podao_detect_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(podao_detect_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp
)

ament_target_dependencies(
  podao_detect_node
  "rclcpp"
  cv_bridge
  sensor_msgs
  laser_geometry
  visualization_msgs
  geometry_msgs  
  tf2
  tf2_ros
  tf2_geometry_msgs 
  interface
  yaml-cpp
)

install(TARGETS podao_detect_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
