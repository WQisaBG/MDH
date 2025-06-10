cmake_minimum_required(VERSION 3.8)
project(motor_control_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(motor_control_command_msgs REQUIRED)


# 设置 serial 的头文件路径
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/cpp-httplib  # httplib.h 的路径
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/json/include  # json 的路径
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/serial/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../rmf_utils/rmf_utils/include


)

include_directories(${CMAKE_SOURCE_DIR}/include)

# 添加可执行文件
add_executable(motor_control_node
    src/main.cpp
    src/Node.cpp
    src/motor_config.cpp
    src/motor_command.cpp
    src/serial_communication.cpp
    src/ros2_communication.cpp
    src/http_server.cpp
    src/internal_http_server.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/serial/src/Serial.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/serial/src/SerialImplUnix.cpp
)

# 链接库
ament_target_dependencies(motor_control_node
"rclcpp"
"std_msgs"
"motor_control_command_msgs"
)

# 安装可执行文件
install(TARGETS
    motor_control_node
    DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY
    include/
    DESTINATION include
)

# 导出依赖项
ament_export_dependencies(rclcpp Serial httplib nlohmann_json)

# 生成ament包
ament_package()