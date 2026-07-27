cmake_minimum_required(VERSION 3.5)
project(recharge)

# 默认使用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找 ROS2 包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common REQUIRED)
find_package(interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 1. 主节点可执行文件
add_executable(recharge_node
  src/main.cpp
  src/battery_manager.cpp
  src/event_input.cpp
  src/workflow.cpp
  src/ipad_manager.cpp
  src/lidascan_ctrl.cpp
  src/recharge_ctrl.cpp
  src/recharge_tool.cpp
  src/report.cpp
  src/rotation_manager.cpp
)

# 设置头文件路径
target_include_directories(recharge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Eigen3_INCLUDE_DIRS}
)

# 设置 ROS2 依赖
ament_target_dependencies(recharge_node
  rclcpp
  interface
  std_msgs
  common
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_geometry_msgs
)

# 安装规则
install(TARGETS
  recharge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()