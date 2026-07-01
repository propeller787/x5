cmake_minimum_required(VERSION 3.5)
project(sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++11标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Debug RelWithDebInfo MinSizeRel
set(CMAKE_BUILD_TYPE Release)

include_directories(include) 

# 添加 muduo 库的包含目录和库目录（muduo 在 third_party 目录下）
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/muduo/include)

# 根据系统架构选择正确的库目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(MUDUO_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/muduo/lib/aarch64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(MUDUO_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/muduo/lib/x86_64)
else()
  set(MUDUO_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/muduo/lib)
endif()
link_directories(${MUDUO_LIB_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(interface REQUIRED)
find_package(common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(can_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interface REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# find_package(qxwz-pssdk-1.24.1-20240924-zqwl-ubuntu-production-release REQUIRED)

file(GLOB car_src "chassis_component/*.cpp" "chassis_component/chassis/*.cpp" "chassis_component/chassis/*.c"
  "chassis_component/handle/*.cpp" "chassis_component/handle/*.c" "chassis_component/dbc/*.c")
add_library(chassis_component SHARED  ${car_src})
ament_target_dependencies(chassis_component rclcpp can_msgs nav_msgs rosbag2_cpp rosbag2_storage interface common visualization_msgs std_msgs)
target_link_libraries(chassis_component)
rclcpp_components_register_nodes(chassis_component "sensor::chassis::ChassisComponent")

file(GLOB gps_src "gps_component/*.cpp" "gps_component/lc760z/*.cpp")
add_library(gps_component SHARED  ${gps_src})
ament_target_dependencies(gps_component rclcpp interface common )
rclcpp_components_register_nodes(gps_component "sensor::gps::GpsComponent")

# 查找FFmpeg库
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavcodec
    libavformat
    libswscale
    libavutil
)

file(GLOB camera_src "camera_component/*.cpp" "camera_component/v4l2/*.c")
add_library(camera_component SHARED  ${camera_src})
ament_target_dependencies(camera_component rclcpp interface common ffmpeg_image_transport_msgs std_srvs)
# 链接FFmpeg库
target_link_libraries(camera_component PkgConfig::LIBAV)
rclcpp_components_register_nodes(camera_component "sensor::camera::CameraComponent")

file(GLOB imu_src "imu_component/jy61P.cpp")
add_library(imu_component SHARED  ${imu_src})
ament_target_dependencies(imu_component rclcpp rclcpp_components interface common sensor_msgs tf2 geometry_msgs)
rclcpp_components_register_nodes(imu_component "sensor::imu::ImuComponent")

file(GLOB recorder_src "sensor_data_recorder/*.cpp")
add_library(recorder_component SHARED ${recorder_src})
ament_target_dependencies(recorder_component rclcpp interface common rosbag2_cpp rosbag2_storage 
                          sensor_msgs ffmpeg_image_transport_msgs can_msgs std_srvs)
target_link_libraries(recorder_component yaml-cpp)
rclcpp_components_register_nodes(recorder_component "sensor::recorder::SensorDataRecorder")

file(GLOB bluetooth_src "bluetooth_component/*.cpp")
add_library(bluetooth_component SHARED ${bluetooth_src})
ament_target_dependencies(bluetooth_component rclcpp rclcpp_components interface common std_msgs)
# 链接 GLib 和 GIO 库
pkg_check_modules(GLIB REQUIRED IMPORTED_TARGET glib-2.0 gio-2.0)
target_link_libraries(bluetooth_component PkgConfig::GLIB)
rclcpp_components_register_nodes(bluetooth_component "sensor::bluetooth::BluetoothComponent")
rclcpp_components_register_nodes(bluetooth_component "sensor::bluetooth::_3568::BluetoothBleComponent")

include(../common/cmake/add3rd.cmake)
add3rds(utils muduo)

add_executable(sensor main.cpp global.cpp)
ament_target_dependencies(sensor rclcpp rclcpp_components) 
target_link_libraries(sensor camera_component gps_component chassis_component imu_component recorder_component bluetooth_component muduo_net muduo_base utils)
install(TARGETS sensor gps_component chassis_component camera_component imu_component recorder_component bluetooth_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # IMU 组件核心业务测试（转换函数、验证逻辑、Madgwick 融合）
  ament_add_gtest(test_imu_component test/test_imu_component.cpp)
  ament_target_dependencies(test_imu_component rclcpp)
endif()

ament_package()
