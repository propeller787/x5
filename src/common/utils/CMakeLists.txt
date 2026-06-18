cmake_minimum_required(VERSION 3.8)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

file(GLOB SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.cpp *.c)

add_library(utils STATIC ${SRCS})

target_include_directories(utils PUBLIC .)

# 链接依赖
ament_target_dependencies(utils rclcpp rcl_interfaces)
target_link_libraries(utils yaml-cpp)

file(GLOB INCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.h *.hpp)
install(FILES ${INCS} DESTINATION  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/x86/utils/include/utils)
install(TARGETS utils DESTINATION  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/x86/utils/lib)
