cmake_minimum_required(VERSION 3.8)

file(GLOB SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.cpp *.c)

add_library(utils STATIC ${SRCS})

target_include_directories(utils PUBLIC .)

file(GLOB INCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.h *.hpp)
install(FILES ${INCS} DESTINATION  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/x86/utils/include/utils)
install(TARGETS utils DESTINATION  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/x86/utils/lib)
