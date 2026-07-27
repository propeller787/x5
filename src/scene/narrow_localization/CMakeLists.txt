cmake_minimum_required(VERSION 3.5)
project(narrow_localization)

# Find ROS2 packages
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_localization REQUIRED)

# 添加源文件
add_executable(narrow_localization src/narrow_localization.cpp)

# 链接依赖的库
ament_target_dependencies(narrow_localization
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  robot_localization
)

# 安装和注册可执行文件
install(TARGETS
  narrow_localization
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
