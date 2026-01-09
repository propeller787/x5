cmake_minimum_required(VERSION 3.8)
project(tft_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface REQUIRED)
find_package(common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# Gather all source files
file (GLOB SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "src/*.cpp" "src/control/*.cpp" "src/tft/*.cpp")
include(../common/cmake/add3rd.cmake)
add3rds(math utils muduo)
# Print out the sources found (for debugging)
message(STATUS "Source files found: ${SRCS}")

# Check if SRCS is empty
if(SRCS STREQUAL "")
  message(FATAL_ERROR "No source files found. Please check the source file paths.")
endif()

# Add executable with the source files
add_executable(tft_manager ${SRCS})

target_link_libraries(tft_manager 
    muduo_net 
    muduo_base
    utils
)

# Link the rclcpp library to the executable
ament_target_dependencies(tft_manager rclcpp interface common geometry_msgs
tf2
tf2_ros
tf2_geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the executable
install(TARGETS tft_manager
  DESTINATION lib/${PROJECT_NAME})

ament_package()
