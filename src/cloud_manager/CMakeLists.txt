cmake_minimum_required(VERSION 3.10)
project(cloud_manager)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface REQUIRED)
find_package(common REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(cpr REQUIRED) 
find_package(mqtt_module REQUIRED)
find_package(PkgConfig REQUIRED)

include(../common/cmake/add3rd.cmake)
add3rds(utils)

set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party)
set(CPR_USE_SYSTEM_CURL ON CACHE BOOL "Force CPR to use system curl" FORCE)
add_subdirectory(${THIRD_PARTY_DIR}/cpr cpr_build)

set(CPR_GENERATED_INCLUDES "${CMAKE_BINARY_DIR}/cpr_build/cpr_generated_includes")
set(_candidate_dirs
  "${CMAKE_CURRENT_SOURCE_DIR}/../../build/interface"
  "${CMAKE_CURRENT_SOURCE_DIR}/../interface/build"
  "${CMAKE_BINARY_DIR}/../interface/build"
  "${CMAKE_CURRENT_SOURCE_DIR}/../build/interface"
)

set(INTERFACE_BUILD_DIR "")
foreach(d ${_candidate_dirs})
  if(EXISTS "${d}/car_status.pb.cc")
    set(INTERFACE_BUILD_DIR "${d}")
    break()
  endif()
endforeach()

if(NOT INTERFACE_BUILD_DIR)
  message(WARNING "未在常见位置找到 interface 的 pb 源码（car_status.pb.cc）。将继续使用默认路径，并在配置时进行更严格的检查。")
  set(INTERFACE_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../build/interface")
endif()

set(INTERFACE_PROTO_SRCS
  "${INTERFACE_BUILD_DIR}/car_status.pb.cc"
  "${INTERFACE_BUILD_DIR}/vehicle_cmd_exe.pb.cc"
  "${INTERFACE_BUILD_DIR}/vehicle_cmd_ack.pb.cc"
  "${INTERFACE_BUILD_DIR}/gps_data.pb.cc"
  "${INTERFACE_BUILD_DIR}/domain_controller.pb.cc"
  "${INTERFACE_BUILD_DIR}/event_config.pb.cc"
  "${INTERFACE_BUILD_DIR}/vehicle_event.pb.cc"
  "${INTERFACE_BUILD_DIR}/event_device_bind.pb.cc"
)

# 循环检查每个proto文件是否存在（逐个检查，提供更明确的提示）
foreach(proto_src ${INTERFACE_PROTO_SRCS})
  if(NOT EXISTS "${proto_src}")
    message(FATAL_ERROR "Protobuf 源码文件不存在: ${proto_src}\n请执行：\n 1. 确认对应的 .proto 文件已放入 interface/proto 目录\n 2. 先编译 interface 包：colcon build --packages-select interface\n 3. 检查 workspace 根目录下的 build/interface 是否含有生成文件")
  endif()
endforeach()

# ========== 头文件路径（移除proto子目录，保留核心） ==========
include_directories(
  include
  ${INTERFACE_BUILD_DIR}          # interface的pb头文件在build根目录
  ${PROTOBUF_INCLUDE_DIRS}
  ${THIRD_PARTY_DIR}/paho.mqtt.cpp/src
  ${THIRD_PARTY_DIR}/cpr/include   # CPR 源码头文件路径
  ${CPR_GENERATED_INCLUDES}        # CPR 生成的头文件路径（包含 cprver.h）
  # 新增：OSS/UUID头文件
  ${CURL_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIRS}
)


add_executable(cloud_manager_node 
  src/cloud_manager_node.cpp
  ../common/utils/trigger_logger.cpp
  ${INTERFACE_PROTO_SRCS}
)
ament_target_dependencies(cloud_manager_node rclcpp interface common ament_index_cpp sensor_msgs mqtt_module)
target_include_directories(cloud_manager_node PRIVATE
  ${rclcpp_INCLUDE_DIRS} 
  ${interface_INCLUDE_DIRS} 
  ${common_INCLUDE_DIRS} 
  ${mqtt_module_INCLUDE_DIRS} 
  ${INTERFACE_BUILD_DIR} 
)
target_link_libraries(cloud_manager_node               
  http_singleton
  ${MOSQUITTO_LIBRARIES}
  ${MOSQUITTOPP_LIBRARIES}
  ${PROTOBUF_LIBRARIES}  # 仅链接系统Protobuf库，删除libproto_files_lib.a
  cpr::cpr
  CURL::libcurl
  mqtt_module::mqtt_module
  ${OPENSSL_LIBRARIES}
  ${interface_LIBRARIES}
)

add_library(http_singleton STATIC
  src/http_singleton.cpp
)
ament_target_dependencies(http_singleton rclcpp interface)
target_include_directories(http_singleton PUBLIC
  include
)
target_link_libraries(http_singleton
  cpr::cpr
  CURL::libcurl
  ${OPENSSL_LIBRARIES}
  ${interface_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS cloud_manager_node  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()