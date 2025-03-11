#include "motor_control_v2/http_server.hpp"
#include "motor_control_v2/motor_config.hpp"
#include "motor_control_v2/serial_communication.hpp"
#include "motor_control_v2/ros2_communication.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("motor_control_node");

    // Load configuration
    MotorConfig motor_config(node, "config.json");

    // Initialize components
    motor_control_v2::SerialCommunication serial_comm(node, motor_config.get_serial_port().c_str(), motor_config.get_baud_rate());
    ROS2Communication ros2_comm(node);
    HttpServerNode http_server;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(http_server.get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}