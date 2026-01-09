cmake_minimum_required(VERSION 3.8)

function(add3rd arg1)
  include_directories("${PROJECT_SOURCE_DIR}/../common/3rd/x86/${arg1}/include")
  link_directories("${PROJECT_SOURCE_DIR}/../common/3rd/x86/${arg1}/lib")
endfunction()

function(add3rds)
    foreach(v IN LISTS ARGV)
      add3rd(${v})
    endforeach()
endfunction()
