cmake_minimum_required(VERSION 3.12)
project(tft_manager)
# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
add_compile_options(
  -O3                    # 最高速度优化
  -march=native          # 适配你的CPU
  -DNDEBUG               # 关闭调试
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface REQUIRED)
find_package(common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)

# 添加common库的头文件路径
include_directories(${CMAKE_SOURCE_DIR}/../common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(/usr/local/include/core)
# Gather all source files（CONFIGURE_DEPENDS：新增/删除 src 下 .cpp 时会重新配置，避免漏链 followme_manager 等）
file(GLOB SRCS CONFIGURE_DEPENDS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/*.cpp"
  "src/control/*.cpp"
  "src/tft/*.cpp"
  "src/dialog/*.cpp"
  "src/laneWarning/*.cpp"
)
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
    yaml-cpp
)

# Link the rclcpp library to the executable
ament_target_dependencies(tft_manager rclcpp interface common ament_index_cpp geometry_msgs
tf2
tf2_ros
tf2_geometry_msgs
std_msgs)

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

# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
