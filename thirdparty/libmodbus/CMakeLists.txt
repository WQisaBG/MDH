cmake_minimum_required(VERSION 3.5)
project(libmodbus)

# 是否编译动态库
option(ENABLE_SHARED_LIB "enable compile with shared lib" OFF)

# 使用x86还是arm
set(COMPILE_C_TYPES "x86" CACHE STRING "x86;ti3352;a40i;win32")

if (COMPILE_C_TYPES STREQUAL "a40i")
    message("compile a40i")
    SET(CMAKE_SYSTEM_NAME Linux)
    SET(CMAKE_C_COMPILER "/home/byd/A40i/lichee/out/sun8iw11p1/linux/common/buildroot/host/usr/bin/arm-linux-gnueabihf-gcc")
    SET(CMAKE_CXX_COMPILER "/home/byd/A40i/lichee/out/sun8iw11p1/linux/common/buildroot/host/usr/bin/arm-linux-gnueabihf-g++")
    set(CMAKE_AR "/home/byd/A40i/lichee/out/sun8iw11p1/linux/common/buildroot/host/usr/bin/arm-linux-gnueabihf-ar")
    set(CMAKE_RANLIB "/home/byd/A40i/lichee/out/sun8iw11p1/linux/common/buildroot/host/usr/bin/arm-linux-gnueabihf-ranlib")
    add_definitions(-D  __RTND_A40i__)

elseif(COMPILE_C_TYPES STREQUAL "ti3352")
    message("compile ti3352")
    SET(CMAKE_SYSTEM_NAME Linux)
    SET(CMAKE_C_COMPILER "/home/ubuntu/ti-processor-sdk-linux-rt-am335x-evm-04.03.00.05/linux-devkit/sysroots/x86_64-arago-linux/usr/bin/arm-linux-gnueabihf-gcc")
    SET(CMAKE_CXX_COMPILER "/home/ubuntu/ti-processor-sdk-linux-rt-am335x-evm-04.03.00.05/linux-devkit/sysroots/x86_64-arago-linux/usr/bin/arm-linux-gnueabihf-g++")

elseif(COMPILE_C_TYPES STREQUAL "x86")
    message("compile x86")

elseif(COMPILE_C_TYPES STREQUAL "win32")
    message("compile win32")

endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -w")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -w")

# 预编译宏定义
#add_definitions(-D EC_DEBUG)

add_subdirectory(src)
