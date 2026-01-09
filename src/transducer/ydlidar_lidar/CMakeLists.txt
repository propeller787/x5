# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 3.5)
project(ydlidar_ros2_driver C CXX)

##################ros2#############################################
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################find package#####################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interface REQUIRED)
find_package(common REQUIRED)

############## YDLIDAR SDK START#####################################
# 使用工程中的 SDK
set(SDK_DIR ${PROJECT_SOURCE_DIR}/SDK)

# 检测系统架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ARCH_DIR "linux_aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(ARCH_DIR "linux_x64")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 设置 SDK 路径
set(YDLIDAR_SDK_INCLUDE_DIR ${SDK_DIR}/include)
set(YDLIDAR_SDK_LIB_DIR ${SDK_DIR}/lib/${ARCH_DIR})
set(YDLIDAR_SDK_LIBRARY ${YDLIDAR_SDK_LIB_DIR}/libydlidar_sdk.a)

# 检查库文件是否存在
if(NOT EXISTS ${YDLIDAR_SDK_LIBRARY})
  message(FATAL_ERROR "YDLIDAR SDK library not found: ${YDLIDAR_SDK_LIBRARY}")
endif()

message(STATUS "Using YDLIDAR SDK from project directory")
message(STATUS "  Include directory: ${YDLIDAR_SDK_INCLUDE_DIR}")
message(STATUS "  Library directory: ${YDLIDAR_SDK_LIB_DIR}")
message(STATUS "  Library file: ${YDLIDAR_SDK_LIBRARY}")
############## YDLIDAR SDK END#####################################

#Include directories
# Get common package source directory
get_filename_component(COMMON_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../common" ABSOLUTE)
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${COMMON_SOURCE_DIR}/..
  ${YDLIDAR_SDK_INCLUDE_DIR})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp)
#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_node
    "rclcpp"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "std_srvs"
    "common"
    "interface"
    )

target_link_libraries(${PROJECT_NAME}_node
    ${YDLIDAR_SDK_LIBRARY})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_client
    src/${PROJECT_NAME}_client.cpp)
#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_client
    "rclcpp"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "std_srvs"
    "common"
    "interface"
    )

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------
install(TARGETS
    ${PROJECT_NAME}_node ${PROJECT_NAME}_client
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

