# cmake_minimum_required(VERSION 3.8)
# project(claw_control)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)


# # 设置 serial 的头文件路径
# include_directories(
#   ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/libmodbus/src  # libmodus 的路径
# )

# include_directories(${CMAKE_SOURCE_DIR}/include)

# # 添加可执行文件
# add_executable(claw_control_node
#     src/main.cpp
#     src/clawControl.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/libmodbus/src/modbus.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/libmodbus/src/modbus-rtu.c
#     # ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/libmodbus/src/modbus-tcp.c
#     # ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/libmodbus/src/modbus-ascii.c

# )

# # 链接库
# ament_target_dependencies(claw_control_node
# "rclcpp"
# "std_msgs"
# )

# # 安装可执行文件
# install(TARGETS
#     claw_control_node
#     DESTINATION lib/${PROJECT_NAME}
# )

# # 安装头文件
# install(DIRECTORY
#     include/
#     DESTINATION include
# )

# # 导出依赖项
# ament_export_dependencies(rclcpp libmodbus)

# # 生成ament包
# ament_package()
cmake_minimum_required(VERSION 3.8)
project(claw_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 添加依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 设置头文件路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/libmodbus/src  # libmodbus 的头文件
    ${CMAKE_SOURCE_DIR}/include
)

# 构建 libmodbus 的源文件
set(LIBMODBUS_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/libmodbus/src/modbus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/libmodbus/src/modbus-rtu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/libmodbus/src/modbus-tcp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/libmodbus/src/modbus-data.c
    # 其他必要源文件
)

# 添加可执行文件并包含 libmodbus 源码
add_executable(claw_control_node
    src/main.cpp
    src/clawControl.cpp
    ${LIBMODBUS_SOURCES}
)

# 处理 C/C++ 混合编译
set_source_files_properties(${LIBMODBUS_SOURCES} PROPERTIES LANGUAGE C)

# 链接依赖项
ament_target_dependencies(claw_control_node
    "rclcpp"
    "std_msgs"
)

# 安装配置
install(TARGETS claw_control_node
    DESTINATION "lib/${PROJECT_NAME}"
)
install(DIRECTORY include/ DESTINATION include)

# 导出依赖项
ament_export_dependencies(rclcpp)

# 生成 ament 包
ament_package()