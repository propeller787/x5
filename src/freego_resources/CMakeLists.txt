cmake_minimum_required(VERSION 3.8)
project(freego_resources)

# find dependencies
find_package(ament_cmake REQUIRED)

# 获取工作空间根目录路径
# 从 src/freego_resources 向上两级到工作空间根目录
get_filename_component(WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# 计算install根目录路径
# CMAKE_INSTALL_PREFIX 在colcon构建时是 install/<package_name>
# 所以install根目录是其父目录
get_filename_component(INSTALL_ROOT_DIR "${CMAKE_INSTALL_PREFIX}" PATH)

# 使用 install(CODE ...) 来安装 etc 和 scripts 到 install 根目录
# 这样可以安装到 install/etc 和 install/scripts，而不是 install/<package_name>/etc
install(CODE "
  # 安装 etc 目录
  file(INSTALL \"${WORKSPACE_ROOT}/etc\"
    DESTINATION \"${INSTALL_ROOT_DIR}\"
    USE_SOURCE_PERMISSIONS
    FILES_MATCHING
    PATTERN \"*\"
  )
  
  # 安装 scripts 目录
  file(INSTALL \"${WORKSPACE_ROOT}/scripts\"
    DESTINATION \"${INSTALL_ROOT_DIR}\"
    USE_SOURCE_PERMISSIONS
    FILES_MATCHING
    PATTERN \"*\"
  )
")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

