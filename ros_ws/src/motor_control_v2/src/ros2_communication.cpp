

#include "motor_control_v2/ros2_communication.hpp"

namespace motor_control_v2 {

ROS2Communication::ROS2Communication(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // 初始化发布者和订阅者

    pub_ = node_->create_publisher<motor_control_command_msgs::msg::MotorControlCommand>("motor_command", 10);
        status_publisher_ = node_->create_publisher<std_msgs::msg::String>("motor_status", 10);
    RCLCPP_INFO(node_->get_logger(), "ROS2Communication initialized.");
}

}

