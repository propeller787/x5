## 安装 libmosquitto（基础MQTT库）+ libmosquittopp（C++封装版）+ libsodium（加密库）
#sudo apt update
#sudo apt install -y libmosquitto-dev libmosquittopp-dev libsodium-dev

# 验证安装（确保3个依赖都能被pkg-config识别）
#pkg-config --exists libmosquitto libmosquittopp libsodium && echo "所有依赖安装成功" || echo "部分依赖缺失"

cmake_minimum_required(VERSION 3.10)
project(mqtt_module)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(cpr QUIET CONFIG)
if(NOT cpr_FOUND)
  message(STATUS "cpr not found; using bundled third_party/cpr")
  set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party)
  # Force CPR to use system libcurl to avoid network fetches
  set(CPR_USE_SYSTEM_CURL ON CACHE BOOL "Use system libcurl" FORCE)
  set(CPR_ENABLE_SSL ON CACHE BOOL "Enable SSL for curl" FORCE)
  set(CURL_ZLIB OFF CACHE BOOL "Disable zlib external" FORCE)
  add_subdirectory(${THIRD_PARTY_DIR}/cpr ${CMAKE_BINARY_DIR}/third_party_cpr)
  if(TARGET cpr AND NOT TARGET cpr::cpr)
    add_library(cpr::cpr ALIAS cpr)
  endif()
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
pkg_check_modules(MOSQUITTOWRAP REQUIRED libmosquittopp)
pkg_check_modules(SODIUM REQUIRED libsodium)

include_directories(
  include
  ${MOSQUITTO_INCLUDE_DIRS}
  ${MOSQUITTOWRAP_INCLUDE_DIRS}
  ${SODIUM_INCLUDE_DIRS}
)

add_library(mqtt_module
  src/mqtt_connection.cpp
  src/signature_manager.cpp
  src/token_manager.cpp
)

# ========== 链接逻辑不变，确保 cpr::cpr 目标有效 ==========
target_link_libraries(mqtt_module
  ${MOSQUITTO_LIBRARIES}
  ${MOSQUITTOWRAP_LIBRARIES}
  ${SODIUM_LIBRARIES}
  cpr::cpr
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(mqtt_module rclcpp)

target_include_directories(mqtt_module PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_mqtt_module HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp nlohmann_json cpr)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mqtt_module
  EXPORT export_mqtt_module
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()