cmake_minimum_required(VERSION 3.8)
project(motor_control)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motor_control_command_msgs REQUIRED)


# 设置 serial 的头文件路径
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/cpp-httplib  # httplib.h 的路径
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/json/include  # httplib.h 的路径
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/serial/src
)



#=======================================================
# add_executable(adapter_http src/adapter_http.cpp)
# ament_target_dependencies(
#   adapter_http
#   "rclcpp"
#   "std_msgs"
#   "motor_control_command_msgs"
# )





#=======================================================
# add_executable(deal_http src/deal_http.cpp)
# ament_target_dependencies(
#   deal_http
#   "rclcpp"
#   "std_msgs"
#   "motor_control_command_msgs"
# )

#=======================================================
# add_executable(send_motor_command src/send_motor_command.cpp)
# ament_target_dependencies(
#   send_motor_command
#   "rclcpp"
#   "std_msgs"
#   "motor_control_command_msgs"
#   "serial"

# )

#=======================================================
add_executable(motor_control_state 
src/motor_control_state.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/serial/src/Serial.cpp
${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/serial/src/SerialImplUnix.cpp
)
ament_target_dependencies(
  motor_control_state
  "rclcpp"
  "std_msgs"
  "motor_control_command_msgs"

)


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)


install(TARGETS
  # adapter_http
  # deal_http
  # send_motor_command
  motor_control_state
  DESTINATION lib/${PROJECT_NAME})
ament_package()
