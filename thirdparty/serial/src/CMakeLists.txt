cmake_minimum_required(VERSION 3.5)

# 添加包含目录，CMake将在这些目录寻找 .cpp 文件中引用到的头文件
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if (WIN32)
    list(APPEND CPP_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/Serial.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SerialImplWin.cpp
        )
else()
    list(APPEND CPP_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/Serial.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SerialImplUnix.cpp
        )
endif()


if (ENABLE_SHARED_LIB)
    # 编译动态库
    add_library(serial SHARED ${CPP_SOURCE_FILES})
else()
    # 编译静态库
    add_library(serial STATIC ${CPP_SOURCE_FILES})
endif()

install(TARGETS serial
    EXPORT ${PROJECT_NAME}Targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if (WIN32)
    install(FILES ./Serial.h
        DESTINATION /usr/local/include/serial )
else ()
    install(FILES ./Serial.h
        DESTINATION /usr/local/include/serial )
endif()