include(${CMAKE_CURRENT_LIST_DIR}/PlatformConfig.cmake)

set(MUDUO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../muduo)

set(MUDUO_INCLUDE_DIR ${MUDUO_ROOT}/include)
set(MUDUO_LIB_DIR     ${MUDUO_ROOT}/lib/${TP_ARCH})

add_library(muduo_base STATIC IMPORTED GLOBAL)
set_target_properties(muduo_base PROPERTIES
  IMPORTED_LOCATION ${MUDUO_LIB_DIR}/libmuduo_base.a
)

add_library(muduo_net STATIC IMPORTED GLOBAL)
set_target_properties(muduo_net PROPERTIES
  IMPORTED_LOCATION ${MUDUO_LIB_DIR}/libmuduo_net.a
)
# Set dependency: muduo_net depends on muduo_base
set_target_properties(muduo_net PROPERTIES
  INTERFACE_LINK_LIBRARIES "muduo_base;pthread"
)

set(MUDUO_LIBRARIES
  muduo_base
  muduo_net
  pthread
)

set(Muduo_FOUND TRUE)
