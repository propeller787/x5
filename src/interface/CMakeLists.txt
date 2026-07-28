cmake_minimum_required(VERSION 3.8)
project(interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" msg/*.msg)
file(GLOB action_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" action/*.action)
file(GLOB srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" srv/*.srv)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# === Protobuf support ===
find_package(Protobuf REQUIRED)
file(GLOB proto_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" proto/*.proto)
# Legacy ota_*.proto duplicate vehicle_ota_*.proto (same package/messages → link errors).
list(FILTER proto_files EXCLUDE REGEX "^proto/ota_(status|up)\\.proto$")

if(proto_files)
  protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${proto_files})
  add_library(proto_files_lib ${PROTO_SRCS} ${PROTO_HDRS})
  target_include_directories(proto_files_lib PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(proto_files_lib PUBLIC ${Protobuf_LIBRARIES})
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "${msg_files}"
  "${action_files}"
  "${srv_files}"
  DEPENDENCIES std_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
