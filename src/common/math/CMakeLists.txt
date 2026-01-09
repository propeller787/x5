cmake_minimum_required(VERSION 3.8)

file(GLOB SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.cpp)

add_library(math STATIC ${SRCS})

target_include_directories(math PUBLIC ..)

file(GLOB INCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.h *.hpp)
install(FILES ${INCS} DESTINATION  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/x86/math/include/math)
install(TARGETS math DESTINATION  ${CMAKE_CURRENT_SOURCE_DIR}/../3rd/x86/math/lib)