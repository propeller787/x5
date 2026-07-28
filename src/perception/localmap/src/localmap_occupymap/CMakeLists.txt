cmake_minimum_required(VERSION 3.8)
project(localmap_occupymap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(include)

add_library(occupymap_core
  src/probability_grid.cpp
  src/rolling_occupancy_map.cpp
)
target_include_directories(occupymap_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(occupymap_core
  rclcpp
  nav_msgs
  nav2_msgs
  map_msgs
  sensor_msgs
  geometry_msgs
  tf2
)

add_executable(occupymap_node
  src/occupymap_node.cpp
  src/main.cpp
)
target_link_libraries(occupymap_node occupymap_core)
ament_target_dependencies(occupymap_node
  rclcpp
  nav_msgs
  nav2_msgs
  map_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

install(TARGETS occupymap_core occupymap_node
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

ament_package()
