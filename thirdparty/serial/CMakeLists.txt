cmake_minimum_required(VERSION 3.5)
project(serial)

# 是否编译动态库
option(ENABLE_SHARED_LIB "enable compile with shared lib" OFF)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -w")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -w")

add_subdirectory(src)
