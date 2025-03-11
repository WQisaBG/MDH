#ifndef ROS2_COMMUNICATION_HPP
#define ROS2_COMMUNICATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <motor_control_command_msgs/msg/motor_control_command.hpp>
#include <std_msgs/msg/string.hpp>

class ROS2Communication
{
public:
    ROS2Communication(rclcpp::Node::SharedPtr node);

private:
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Node::SharedPtr node_;
};

#endif // ROS2_COMMUNICATION_HPP