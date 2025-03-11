#ifndef MOTOR_CONTROL_V2_ROS2_COMMUNICATION_HPP
#define MOTOR_CONTROL_V2_ROS2_COMMUNICATION_HPP

#include "motor_control_v2/ros2_communication.hpp"

ROS2Communication::ROS2Communication(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // 初始化发布者和订阅者
    RCLCPP_INFO(node_->get_logger(), "ROS2Communication initialized.");
}

#endif // MOTOR_CONTROL_V2_ROS2_COMMUNICATION_HPP