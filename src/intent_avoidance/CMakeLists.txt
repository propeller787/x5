cmake_minimum_required(VERSION 3.8)
project(intent_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interface REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/scan_processor.cpp
  src/free_space_analyzer.cpp
  src/intent_detector.cpp
  src/avoidance_engine.cpp
  src/control_generator.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  interface
)

add_executable(intent_avoidance_node
  src/main.cpp
  src/intent_avoidance_node.cpp
)
target_link_libraries(intent_avoidance_node ${PROJECT_NAME})
ament_target_dependencies(intent_avoidance_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  interface
)

install(TARGETS ${PROJECT_NAME} intent_avoidance_node
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

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_avoidance_engine test/test_avoidance_engine.cpp)
  target_link_libraries(test_avoidance_engine ${PROJECT_NAME})
  ament_target_dependencies(test_avoidance_engine
    rclcpp sensor_msgs geometry_msgs std_msgs interface)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs std_msgs interface)

ament_package()
