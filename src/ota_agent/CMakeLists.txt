cmake_minimum_required(VERSION 3.14)
project(ota_agent)

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
add_compile_options(
  -O3                    # 最高速度优化
  -march=native          # 适配你的CPU
  -DNDEBUG               # 关闭调试
)

# 动态查找 interface 包生成的 Protobuf 源码目录
set(_candidate_dirs
  "${CMAKE_CURRENT_SOURCE_DIR}/../../build/interface"
  "${CMAKE_CURRENT_SOURCE_DIR}/../interface/build"
  "${CMAKE_BINARY_DIR}/../interface/build"
  "${CMAKE_CURRENT_SOURCE_DIR}/../build/interface"
)

set(INTERFACE_BUILD_DIR "")
foreach(d ${_candidate_dirs})
  if(EXISTS "${d}/vehicle_ota_up.pb.h")
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
  set(CPR_USE_SYSTEM_CURL ON CACHE BOOL "Use system libcurl" FORCE)
  set(CPR_ENABLE_SSL ON CACHE BOOL "Enable SSL for curl" FORCE)
  # Ensure zlib external is disabled when using system curl
  set(CURL_ZLIB OFF CACHE BOOL "Disable zlib external" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/cpr ${CMAKE_BINARY_DIR}/third_party_cpr)
endif()


find_package(mqtt_module REQUIRED)
find_package(PkgConfig REQUIRED)
# 使用 pkg-config 查找系统的 curl，确保找到有版本标记的 curl 库
# mqtt_module 中的 libcpr.so 需要系统的 curl（有 @CURL_OPENSSL_4 版本标记）
pkg_check_modules(CURL REQUIRED libcurl)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
pkg_check_modules(MOSQUITTOPP REQUIRED libmosquittopp)
pkg_check_modules(OPENSSL REQUIRED openssl)
find_package(nlohmann_json REQUIRED)
find_package(Protobuf REQUIRED)

# 新增：UUID和libsodium依赖
pkg_check_modules(UUID REQUIRED uuid)
pkg_check_modules(SODIUM REQUIRED libsodium)

set(OSS_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aliyun-oss-cpp-sdk-1.10.0")
set(OSS_SDK_INCLUDE_DIR "${OSS_SDK_ROOT}/include")
set(OSS_SDK_LIB_DIR "${OSS_SDK_ROOT}/lib/x86_64")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
    set(OSS_SDK_LIB_DIR "${OSS_SDK_ROOT}/lib/aarch64")
else()
    set(OSS_SDK_LIB_DIR "${OSS_SDK_ROOT}/lib/x86_64")
endif()

message("rclcpp_INCLUDE_DIRS: ${rclcpp_INCLUDE_DIRS}")
message("interface_INCLUDE_DIRS: ${interface_INCLUDE_DIRS}")


# 定义需要包含的proto源文件
set(INTERFACE_PROTO_SRCS
  "${INTERFACE_BUILD_DIR}/vehicle_ota_up.pb.cc"
  "${INTERFACE_BUILD_DIR}/vehicle_ota_ack.pb.cc"
  "${INTERFACE_BUILD_DIR}/vehicle_ota_status.pb.cc"
  "${INTERFACE_BUILD_DIR}/tag_change.pb.cc"
  "${INTERFACE_BUILD_DIR}/vehicle_ecu_up.pb.cc"
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
  src/ota_agent/can_transport_socketcan.cpp
  src/ota_agent/can_transport_mock.cpp
  src/ota_agent/oss_manager.cpp
  ${INTERFACE_PROTO_SRCS}
)
target_include_directories(ota_agent_node PRIVATE 
  ${rclcpp_INCLUDE_DIRS} 
  ${interface_INCLUDE_DIRS} 
  ${common_INCLUDE_DIRS} 
  ${mqtt_module_INCLUDE_DIRS} 
  ${INTERFACE_BUILD_DIR} 
  include 
  include/cpr 
  ${CMAKE_CURRENT_SOURCE_DIR}/../common/3rd/x86/utils/include 
  ${MOSQUITTO_INCLUDE_DIRS} 
  ${MOSQUITTOPP_INCLUDE_DIRS} 
  ${SODIUM_INCLUDE_DIRS} 
  ${Protobuf_INCLUDE_DIRS}
  ${OSS_SDK_INCLUDE_DIR}
)
# OTA节点链接：ROS 2依赖 + CPR模块 + MQTT依赖
# 注意：mqtt_module 中的 libcpr.so 需要系统的 curl（有 @CURL_OPENSSL_4 版本标记）
# 使用 pkg-config 找到的系统 curl 库（${CURL_LIBRARIES}）
target_link_libraries(ota_agent_node
  cpr::cpr
  mqtt_module::mqtt_module
  ${CURL_LIBRARIES}
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
  ${OSS_SDK_LIB_DIR}/libalibabacloud-oss-cpp-sdk.a
)

ament_target_dependencies(ota_agent_node mqtt_module)

# 驱动器升级独立调试工具（不依赖ROS 2）
add_executable(driver_upgrade_debug
  src/test/driver_upgrade_debug.cpp
)
target_include_directories(driver_upgrade_debug PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_features(driver_upgrade_debug PUBLIC cxx_std_17)

# 安装目标（ROS 2标准）
install(TARGETS
  ota_agent_node
  driver_upgrade_debug
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
