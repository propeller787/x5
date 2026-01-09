cmake_minimum_required(VERSION 3.14)
project(ota_agent)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 动态查找 interface 包生成的 Protobuf 源码目录
set(_candidate_dirs
  "${CMAKE_CURRENT_SOURCE_DIR}/../../build/interface"
  "${CMAKE_CURRENT_SOURCE_DIR}/../interface/build"
  "${CMAKE_BINARY_DIR}/../interface/build"
  "${CMAKE_CURRENT_SOURCE_DIR}/../build/interface"
)

set(INTERFACE_BUILD_DIR "")
foreach(d ${_candidate_dirs})
  if(EXISTS "${d}/ota_up.pb.h")
    set(INTERFACE_BUILD_DIR "${d}")
    break()
  endif()
endforeach()

if(NOT INTERFACE_BUILD_DIR)
  message(FATAL_ERROR "未找到 interface 的 pb 源码 (ota_up.pb.h)。请先编译 'interface' 包。")
endif()

message("INTERFACE_BUILD_DIR: ${INTERFACE_BUILD_DIR}")

include_directories(${INTERFACE_BUILD_DIR})

# 查找ROS 2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface REQUIRED)
find_package(common REQUIRED)
# Prefer system-installed cpr; fall back to bundled third_party if missing
find_package(cpr QUIET CONFIG)
if(NOT cpr_FOUND)
  message(STATUS "cpr not found; using bundled src/third_party/cpr")
  # Force CPR to use system libcurl to avoid network fetches
  set(CPR_USE_SYSTEM_CURL ON CACHE BOOL "Use system libcurl" FORCE)
  set(CPR_ENABLE_SSL ON CACHE BOOL "Enable SSL for curl" FORCE)
  # Ensure zlib external is disabled when using system curl
  set(CURL_ZLIB OFF CACHE BOOL "Disable zlib external" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/cpr ${CMAKE_BINARY_DIR}/third_party_cpr)
endif()
find_package(mqtt_module REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
pkg_check_modules(MOSQUITTOPP REQUIRED libmosquittopp)
pkg_check_modules(OPENSSL REQUIRED openssl)
find_package(nlohmann_json REQUIRED)
find_package(Protobuf REQUIRED)

# 新增：UUID和libsodium依赖
pkg_check_modules(UUID REQUIRED uuid)
pkg_check_modules(SODIUM REQUIRED libsodium)

message("rclcpp_INCLUDE_DIRS: ${rclcpp_INCLUDE_DIRS}")
message("interface_INCLUDE_DIRS: ${interface_INCLUDE_DIRS}")

# 1. 定义自定义CPR库
# add_library(cpr_module
#   src/cpr/mqtt_recovery.cpp
#   src/cpr/http_recovery.cpp
#   src/cpr/message_recovery.cpp
#   src/cpr/cpr_manager.cpp
# )
# CPR模块链接libcpr
# target_link_libraries(cpr_module PRIVATE cpr::cpr)

# 定义需要包含的proto源文件
set(INTERFACE_PROTO_SRCS
  "${INTERFACE_BUILD_DIR}/ota_up.pb.cc"
  "${INTERFACE_BUILD_DIR}/ota_ack.pb.cc"
  "${INTERFACE_BUILD_DIR}/ota_status.pb.cc"
  "${INTERFACE_BUILD_DIR}/tag_change.pb.cc"
)

# 检查所有proto文件是否存在
foreach(proto_src ${INTERFACE_PROTO_SRCS})
  if(NOT EXISTS "${proto_src}")
    message(FATAL_ERROR "Protobuf 源码文件不存在: ${proto_src}")
  endif()
endforeach()

# 2. 定义OTA Agent节点目标（ROS 2节点）
add_executable(ota_agent_node
  src/ota_agent_node.cpp
  src/controller_comm.cpp
  src/can_id_policy.cpp
  src/can_transport_socketcan.cpp
  src/can_transport_mock.cpp
  src/integrity_validator.cpp
  ${INTERFACE_PROTO_SRCS}
)
target_include_directories(ota_agent_node PRIVATE ${rclcpp_INCLUDE_DIRS} ${interface_INCLUDE_DIRS} ${common_INCLUDE_DIRS} ${mqtt_module_INCLUDE_DIRS} ${INTERFACE_BUILD_DIR} include include/cpr ${CMAKE_CURRENT_SOURCE_DIR}/../common/3rd/x86/utils/include ${MOSQUITTO_INCLUDE_DIRS} ${MOSQUITTOPP_INCLUDE_DIRS} ${SODIUM_INCLUDE_DIRS} ${Protobuf_INCLUDE_DIRS})
# OTA节点链接：ROS 2依赖 + CPR模块 + MQTT依赖
target_link_libraries(ota_agent_node
  cpr::cpr
  mqtt_module::mqtt_module
  ${MOSQUITTO_LIBRARIES}
  ${MOSQUITTOPP_LIBRARIES}
  ${OPENSSL_LIBRARIES}
  nlohmann_json::nlohmann_json
  ${PROTOBUF_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${interface_LIBRARIES}
  ${common_LIBRARIES}
  ${SODIUM_LIBRARIES}
  ${UUID_LIBRARIES}
)

ament_target_dependencies(ota_agent_node mqtt_module)

# Demo: Publish DBC-backed audio command to CAN via CanAdapter
add_executable(ota_dbc_demo_node
  src/ota_dbc_demo_node.cpp
)
ament_target_dependencies(ota_dbc_demo_node rclcpp std_msgs)

# 测试工具：发送 OTA 指令到 MQTT
add_executable(send_ota_test
  src/send_ota_test.cpp
  ${INTERFACE_PROTO_SRCS}
)
target_include_directories(send_ota_test PRIVATE ${INTERFACE_BUILD_DIR})
target_link_libraries(send_ota_test
  PRIVATE
  ${MOSQUITTO_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

# 安装目标（ROS 2标准）
install(TARGETS
  ota_agent_node
  ota_dbc_demo_node
  send_ota_test
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
