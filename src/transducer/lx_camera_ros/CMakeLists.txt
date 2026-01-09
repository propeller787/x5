cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)
project(lx_camera_ros)

execute_process(COMMAND lsb_release -r COMMAND cut -f 2 OUTPUT_VARIABLE LX_DYNAMIC)
string(STRIP ${LX_DYNAMIC} ${LX_DYNAMIC})
if (${LX_DYNAMIC} VERSION_LESS_EQUAL "18.04")
	message(STATUS "Use Normal Link: ${LX_DYNAMIC}")
	SET(LAUNCH_DIR "${CMAKE_SOURCE_DIR}/src/launch/ubuntu18/")
else()
	add_definitions(-DLX_DYNAMIC)
	message(STATUS "Use Dnamic Link: ${LX_DYNAMIC}")
	SET(LAUNCH_DIR "${CMAKE_SOURCE_DIR}/src/launch/ubuntu22/")
endif ()
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/pallet.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/mapping.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/localization.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/obstacle.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/obstacleV2.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/sensor_sim.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/localization.launch.py ${CMAKE_SOURCE_DIR}/launch/)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${LAUNCH_DIR}/lx_camera_ros.launch.py ${CMAKE_SOURCE_DIR}/launch/)

if (NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif ()

#ament_cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/ObstacleBox.msg"
"msg/FrameRate.msg"
"msg/Obstacle.msg"
"msg/Pallet.msg"
"msg/Result.msg"

"srv/LxString.srv"
"srv/LxFloat.srv"
"srv/LxBool.srv"
"srv/LxCmd.srv"
"srv/LxInt.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs 
  ADD_LINTER_TESTS
)

set(dependencies rclcpp image_transport sensor_msgs
				cv_bridge nav_msgs pcl_conversions tf2 tf2_ros)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
ament_export_dependencies(rosidl_default_runtime)

include_directories(
        ${PROJECT_SOURCE_DIR}/src
)
#SDK
# 检测系统架构并设置对应的库路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(LX_ARCH "linux_x64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(LX_ARCH "linux_aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l|arm")
    set(LX_ARCH "linux_arm32")
else()
    message(WARNING "Unknown architecture ${CMAKE_SYSTEM_PROCESSOR}, defaulting to linux_x64")
    set(LX_ARCH "linux_x64")
endif()

set(LX_SDK_DIR ${PROJECT_SOURCE_DIR}/SDK)
set(LX_INCLUDE_DIR ${LX_SDK_DIR}/include)
set(LX_LIB_PATH ${LX_SDK_DIR}/lib/${LX_ARCH})

include_directories(${LX_INCLUDE_DIR})
link_directories(${LX_LIB_PATH})
set(LX_LIB ${LX_LIB_PATH}/libLxCameraApi.so)
add_definitions(-DLX_LIB="${LX_LIB}")
message(STATUS "LX SDK Directory: ${LX_SDK_DIR}")
message(STATUS "LX Architecture: ${LX_ARCH}")
message(STATUS "LX Library Path: ${LX_LIB_PATH}")
#pcl
find_package(PCL QUIET REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL_VERSION: " ${PCL_VERSION})
#opencv
find_package(OpenCV QUIET REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_DIR})
message(STATUS "OPENCV_VERSION " ${OpenCV_VERSION})

set(TARGET_NAME lx_camera_node)
add_executable(${TARGET_NAME}
				${PROJECT_SOURCE_DIR}/src/lx_camera/lx_camera.cpp 
				${PROJECT_SOURCE_DIR}/src/lx_camera/lx_camera_node.cpp 
				${PROJECT_SOURCE_DIR}/src/utils/dynamic_link.cpp)
# rosidl_target_interfaces(${TARGET_NAME} ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
ament_target_dependencies(${TARGET_NAME} ${dependencies} rosidl_default_runtime )
target_link_libraries(${TARGET_NAME}
                      ${PCL_LIBRARIES} 
					  "${cpp_typesupport_target}")
install(TARGETS ${TARGET_NAME} EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(FILES ${LX_LIB_PATH}/libLxDataProcess.so ${LX_LIB_PATH}/libLxCameraApi.so DESTINATION lib/)

set(TARGET_NAME lx_localization_node)
add_executable(${TARGET_NAME}
				${PROJECT_SOURCE_DIR}/src/lx_localization/lx_localization.cpp 
				${PROJECT_SOURCE_DIR}/src/lx_localization/lx_localization_node.cpp 
				${PROJECT_SOURCE_DIR}/src/utils/dynamic_link.cpp)
ament_target_dependencies(${TARGET_NAME} ${dependencies})
target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS} )
install(TARGETS ${TARGET_NAME} EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

set(TARGET_NAME sensor_sim_node)
add_executable(${TARGET_NAME}
				${PROJECT_SOURCE_DIR}/src/lx_localization/sensor_sim.cpp 
				${PROJECT_SOURCE_DIR}/src/lx_localization/sensor_sim_node.cpp)
ament_target_dependencies(${TARGET_NAME} ${dependencies})
install(TARGETS ${TARGET_NAME} EXPORT export_${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
