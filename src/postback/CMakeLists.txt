cmake_minimum_required(VERSION 3.10)
project(postback)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
add_compile_options(
  -O3                    # 最高速度优化
  -march=native          # 适配你的CPU
  -DNDEBUG               # 关闭调试
)
# 仅保留必要的依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(CURL REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(ZIP REQUIRED libzip)

set(OSS_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/aliyun-oss-cpp-sdk-1.10.0")
set(OSS_SDK_INCLUDE_DIR "${OSS_SDK_ROOT}/include")
set(OSS_SDK_LIB_DIR "${OSS_SDK_ROOT}/lib/x86_64")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
    set(OSS_SDK_LIB_DIR "${OSS_SDK_ROOT}/lib/aarch64")
else()
    set(OSS_SDK_LIB_DIR "${OSS_SDK_ROOT}/lib/x86_64")
endif()

include_directories(
  include
  ${OSS_SDK_INCLUDE_DIR}
  ${interface_INCLUDE_DIRS} 
  ${common_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR}/../common/3rd/x86/utils/include 
  ${ZIP_INCLUDE_DIRS}
)

# 简单的postback节点
add_executable(postback_node
  src/main.cpp
  src/postback/postback_node.cpp
  src/postback/oss_manager.cpp
  src/postback/utils.cpp
)

target_link_libraries(postback_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${interface_LIBRARIES}
  ${OPENSSL_LIBRARIES}
  ${common_LIBRARIES}
  ${OSS_SDK_LIB_DIR}/libalibabacloud-oss-cpp-sdk.a
  CURL::libcurl
  Threads::Threads
  ${ZIP_LIBRARIES}
)

ament_target_dependencies(postback_node rclcpp std_msgs interface)

# 安装节点
install(TARGETS postback_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

ament_package()