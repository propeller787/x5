ament_add_gtest(test_obstacle_cloud_filter_core
  test_obstacle_cloud_filter_core.cpp
)

target_include_directories(test_obstacle_cloud_filter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)

target_compile_features(test_obstacle_cloud_filter_core PUBLIC cxx_std_17)
