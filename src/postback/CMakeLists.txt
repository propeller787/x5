cmake_minimum_required(VERSION 3.10)
project(postback)

# 关键：启用 C++17 标准（支持 std::filesystem）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# 仅保留 OSS 上传所需依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(CURL REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(UUID REQUIRED uuid)
find_package(Threads REQUIRED)
## Ensure cpr::cpr target exists (required by mqtt_module's exports)
find_package(cpr QUIET CONFIG)
if(NOT cpr_FOUND)
  message(STATUS "cpr not found; using bundled src/third_party/cpr")
  # Force CPR to use system libcurl to avoid network fetches
  set(CPR_USE_SYSTEM_CURL ON CACHE BOOL "Force CPR to use system curl" FORCE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/cpr ${CMAKE_BINARY_DIR}/third_party_cpr)
endif()
find_package(mqtt_module REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Protobuf REQUIRED)

# 检查 C++17 支持（可选，增强鲁棒性）
if(NOT CMAKE_CXX_STANDARD EQUAL 17)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

include_directories(
  include
  ${CURL_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

# OSS 上传核心库
add_library(oss_uploader_lib
  src/oss_uploader.cpp
  # 如果有 token_manager.cpp，需要添加到这里（比如：src/token_manager.cpp）
)
ament_target_dependencies(oss_uploader_lib rclcpp)
target_include_directories(oss_uploader_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  # 安装后其他包能找到头文件
  ${CURL_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)
target_link_libraries(oss_uploader_lib
  CURL::libcurl
  ${OPENSSL_LIBRARIES}
  ${UUID_LIBRARIES}
  Threads::Threads
)

# 独立 OSS 上传节点
add_executable(oss_log_uploader_node
  src/oss_log_uploader_node.cpp
)
ament_target_dependencies(oss_log_uploader_node rclcpp)
target_link_libraries(oss_log_uploader_node
  oss_uploader_lib
  CURL::libcurl
  ${OPENSSL_LIBRARIES}
  ${UUID_LIBRARIES}
  Threads::Threads
)

# 安装头文件（保持 include/postback/ 目录结构）
install(DIRECTORY include/
  DESTINATION include
)

# 安装库文件，并导出（允许其他包依赖）
install(TARGETS oss_uploader_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 导出库的配置（关键：其他包依赖时需要）
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp CURL OpenSSL UUID Threads)
ament_export_dependencies(mqtt_module nlohmann_json)

# 安装节点（ROS2 节点必须安装到 lib/<包名>/ 目录）
install(TARGETS oss_log_uploader_node
  DESTINATION lib/${PROJECT_NAME}
)

# 事件报警上报节点
set(EVENT_PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../interface/proto)
set(EVENT_PROTO_FILES
  ${EVENT_PROTO_DIR}/event_config.proto
  ${EVENT_PROTO_DIR}/vehicle_event.proto
)
protobuf_generate_cpp(EVENT_PROTO_SRCS EVENT_PROTO_HDRS ${EVENT_PROTO_FILES})

add_executable(event_alarm_node
  src/event_alarm_node.cpp
  ${EVENT_PROTO_SRCS}
)
ament_target_dependencies(event_alarm_node rclcpp nlohmann_json std_msgs)
target_include_directories(event_alarm_node PRIVATE include ${CMAKE_CURRENT_BINARY_DIR} ${Protobuf_INCLUDE_DIRS})
target_link_libraries(event_alarm_node
  oss_uploader_lib
  mqtt_module::mqtt_module
  cpr::cpr
  CURL::libcurl
  ${OPENSSL_LIBRARIES}
  ${UUID_LIBRARIES}
  Threads::Threads
  nlohmann_json::nlohmann_json
  ${Protobuf_LIBRARIES}
)
install(TARGETS event_alarm_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 config 和 launch 目录（如果存在，没有则可以删除这行）
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()