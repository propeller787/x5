cmake_minimum_required(VERSION 3.8)

file(GLOB SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.cpp *.c)

include(../cmake/add3rd.cmake)
add3rds(muduo)

add_library(mmw STATIC ${SRCS})

target_include_directories(mmw PUBLIC .)

file(GLOB INCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.h *.hpp)
install(FILES ${INCS} DESTINATION  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/x86/mmw/include/mmw)
install(TARGETS mmw DESTINATION  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/x86/mmw/lib)

target_link_libraries(mmw PRIVATE muduo_net muduo_base)