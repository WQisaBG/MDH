# cmake_minimum_required(VERSION 3.8)
# project(motor_status)

# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# # find_package(serial REQUIRED)  # 添加这一行以找到 serial 库


# # 添加 cpp-httplib 的路径
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/cpp-httplib)
# # include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/)


# # include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/serial)
# include_directories(${serial_INCLUDE_DIRS})


# #=======================================================
# add_executable(motor_status src/motor_status.cpp)
# ament_target_dependencies(
#   motor_status
#   "rclcpp"
#   "std_msgs"
  
# )




# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()


# install(TARGETS
#   motor_status
#   DESTINATION lib/${PROJECT_NAME})

# ament_package()
cmake_minimum_required(VERSION 3.8)
project(motor_status)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)




# 设置 serial 的头文件路径
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/cpp-httplib  # httplib.h 的路径
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/json/include  # httplib.h 的路径
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/serial/src
)

# 添加可执行文件并包含 serial 的源文件
add_executable(motor_status
  src/motor_status.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/serial/src/Serial.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/serial/src/SerialImplUnix.cpp
)

# 链接 ROS 依赖
ament_target_dependencies(
  motor_status
  "rclcpp"
  "std_msgs"
)

# 如果 serial 需要额外链接（如动态库）
# target_link_libraries(motor_status serial)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS motor_status DESTINATION lib/${PROJECT_NAME})
ament_package()