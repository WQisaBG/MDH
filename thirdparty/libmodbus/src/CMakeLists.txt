cmake_minimum_required(VERSION 3.5)

# 添加包含目录，CMake将在这些目录寻找 .cpp 文件中引用到的头文件
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if (WIN32)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/win32)
else()
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/linux)
endif()

list(APPEND CPP_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/modbus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/modbus-data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/modbus-rtu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/modbus-tcp.c
    )

# 指定生成可执行程序 add_executable 必须放到 target_link_libraries 之前, 否则无法编译
if (ENABLE_SHARED_LIB)
    # 编译动态库
    add_library(modbus SHARED ${CPP_SOURCE_FILES})
else()
    # 编译静态库
    add_library(modbus STATIC ${CPP_SOURCE_FILES})
endif()

install(TARGETS modbus
    EXPORT ${PROJECT_NAME}Targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(FILES ./modbus.h
              ./modbus-rtu.h
              ./modbus-tcp.h
              ./modbus-version.h
        DESTINATION /usr/local/include/libmodbus )
