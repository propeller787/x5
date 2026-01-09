cmake_minimum_required(VERSION 3.10)
project(cloud_manager)

# ========== 核心依赖查找（移除fmt，保留OSS/UUID） ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface REQUIRED)
find_package(common REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 新增：OSS上传依赖（libcurl + OpenSSL）
find_package(CURL REQUIRED)
find_package(OpenSSL REQUIRED)

# MQTT v5 由依赖自身头文件定义，无需全局宏

# 需添加UUID依赖（预留扩展）- 修复：仅查找依赖，暂不链接
find_package(PkgConfig REQUIRED)
pkg_check_modules(UUID REQUIRED uuid)
## libsodium (Ed25519)
pkg_check_modules(SODIUM REQUIRED libsodium)

# MQTT库：使用mosquitto
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
pkg_check_modules(MOSQUITTOPP REQUIRED libmosquittopp)

# ========== 第三方库配置（原有） ==========
include(../common/cmake/add3rd.cmake)
add3rds(utils)

set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party)
set(CPR_USE_SYSTEM_CURL ON CACHE BOOL "Force CPR to use system curl" FORCE)
add_subdirectory(${THIRD_PARTY_DIR}/paho.mqtt.cpp paho_mqtt_cpp_build)  # 为mqtt_data_verifier保留Paho
add_subdirectory(${THIRD_PARTY_DIR}/cpr cpr_build)

# ========== Protobuf 路径配置（核心修复：保留3个proto文件+循环检查） ==========
## 动态查找 interface 包生成的 Protobuf 源码目录（优先 workspace build/interface）
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
  # 新增：OSS/UUID头文件
  ${CURL_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

# ========== 1. 编译令牌管理器（原有） ==========
add_library(token_manager
  src/token_manager.cpp
)
ament_target_dependencies(token_manager rclcpp)
target_include_directories(token_manager
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(token_manager
  CURL::libcurl
  ${OPENSSL_LIBRARIES}
  cpr::cpr
)

# ========== 2. 编译签名管理器（原有） ==========
add_library(signature_manager
  src/signature_manager.cpp
)
ament_target_dependencies(signature_manager rclcpp)
target_include_directories(signature_manager PUBLIC
  include
)
target_include_directories(signature_manager PUBLIC ${SODIUM_INCLUDE_DIRS})
target_link_libraries(signature_manager ${SODIUM_LIBRARIES})

# ========== 3. 编译MQTT连接管理器（原有） ==========
add_library(mqtt_connection_manager
  src/mqtt_connection_manager.cpp
)
ament_target_dependencies(mqtt_connection_manager rclcpp)
target_link_libraries(mqtt_connection_manager
  ${MOSQUITTO_LIBRARIES}
  ${MOSQUITTOPP_LIBRARIES}
)
target_include_directories(mqtt_connection_manager PUBLIC
  include
)

# Domain controller monitor
add_library(domain_controller_monitor
  src/domain_controller_monitor.cpp
)
ament_target_dependencies(domain_controller_monitor rclcpp)
target_include_directories(domain_controller_monitor PUBLIC
  include
)

# ========== 4. 编译核心节点（保留3个proto文件，删除libproto_files_lib.a） ==========
add_executable(cloud_manager_node 
  src/cloud_manager_node.cpp
  ../common/utils/trigger_logger.cpp
  ${INTERFACE_PROTO_SRCS}
)
ament_target_dependencies(cloud_manager_node rclcpp interface common ament_index_cpp sensor_msgs)
target_link_libraries(cloud_manager_node
  token_manager          
  mqtt_connection_manager  
  signature_manager       
  domain_controller_monitor
  ${MOSQUITTO_LIBRARIES}
  ${MOSQUITTOPP_LIBRARIES}
  ${PROTOBUF_LIBRARIES}  # 仅链接系统Protobuf库，删除libproto_files_lib.a
  cpr::cpr
  CURL::libcurl
  ${OPENSSL_LIBRARIES}
)

# ========== 5. 编译指令接收节点（使用Paho MQTT v5 + TLS参数化） ==========
add_executable(mqtt_command_receiver
  src/mqtt_command_receiver.cpp
  ${INTERFACE_PROTO_SRCS}
)
ament_target_dependencies(mqtt_command_receiver rclcpp interface common ament_index_cpp)
target_link_libraries(mqtt_command_receiver
  token_manager
  signature_manager
  mqtt_connection_manager
  ${MOSQUITTO_LIBRARIES}
  ${MOSQUITTOPP_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  CURL::libcurl
  ${OPENSSL_LIBRARIES}
)

# ========== 6. 编译数据验证节点（核心修复：删除libproto_files_lib.a） ==========
# add_executable(mqtt_data_verifier
#   src/mqtt_data_verifier.cpp
#   ${INTERFACE_PROTO_SRCS}
# )
# ament_target_dependencies(mqtt_data_verifier rclcpp)
# target_link_libraries(mqtt_data_verifier
#   signature_manager
#   paho-mqttpp3  # 添加Paho链接
#   paho-mqtt3as  # 添加Paho异步库
#   ${PROTOBUF_LIBRARIES}  # 删除libproto_files_lib.a
# )

# ========== 7. 编译OSS上传核心库（原有） ==========
add_library(oss_uploader_lib
  src/oss_uploader.cpp
)
ament_target_dependencies(oss_uploader_lib rclcpp)
target_link_libraries(oss_uploader_lib
  CURL::libcurl
  ${OPENSSL_LIBRARIES}
  ${UUID_LIBRARIES}
  pthread
)
target_include_directories(oss_uploader_lib PUBLIC
  include
  ${CURL_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

# ========== 8. 编译独立OSS上传节点（原有） ==========
add_executable(oss_log_uploader_node
  src/oss_log_uploader_node.cpp
)
ament_target_dependencies(oss_log_uploader_node rclcpp)
target_link_libraries(oss_log_uploader_node
  oss_uploader_lib
  CURL::libcurl
  ${OPENSSL_LIBRARIES}
  ${UUID_LIBRARIES}
  pthread
)

# ========== 安装逻辑（原有） ==========
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS token_manager mqtt_connection_manager signature_manager domain_controller_monitor
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS cloud_manager_node mqtt_command_receiver mqtt_data_verifier
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS cloud_manager_node mqtt_command_receiver
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(mqtt_cmd_publisher
add_executable(mqtt_cmd_publisher
  src/mqtt_cmd_publisher.cpp
  ${INTERFACE_PROTO_SRCS}
)
ament_target_dependencies(mqtt_cmd_publisher rclcpp interface common ament_index_cpp)
target_link_libraries(mqtt_cmd_publisher
  signature_manager
  mqtt_connection_manager
  ${MOSQUITTO_LIBRARIES}
  ${MOSQUITTOPP_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  CURL::libcurl
  ${OPENSSL_LIBRARIES}
)
install(TARGETS mqtt_cmd_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS oss_uploader_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS oss_log_uploader_node
  DESTINATION lib/${PROJECT_NAME}
)

# ========== 9. 错误码查询服务节点 ==========
add_executable(error_code_service_node
  src/error_code_service_node.cpp
)
ament_target_dependencies(error_code_service_node rclcpp interface)
target_include_directories(error_code_service_node PRIVATE include)
target_link_libraries(error_code_service_node
  ${PROTOBUF_LIBRARIES}
)
install(TARGETS error_code_service_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ========== ament打包（必须） ==========
ament_package()