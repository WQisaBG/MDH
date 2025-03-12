#include "motor_control_v2/http_server.hpp"
#include "motor_control_v2/motor_config.hpp"
#include "motor_control_v2/motor_command.hpp"
#include "motor_control_v2/serial_communication.hpp"
#include "motor_control_v2/ros2_communication.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("motor_control_node");

    // 加载配置文件
    MotorConfig motor_config(node, "config.json");

    // 初始化 HTTP 服务器
    motor_control_v2::HttpServerNode http_server;

    // 注册 POST 数据回调函数
    http_server.registerPostDataCallback([](const nlohmann::json &json_data) {
        // 打印接收到的 POST 请求信息
        std::cout << "Received POST request data: " << json_data.dump() << std::endl;
    });

    // 初始化其他组件
    motor_control_v2::SerialCommunication serial_comm(node, motor_config.get_serial_port().c_str(), motor_config.get_baud_rate());
    motor_control_v2::ROS2Communication ros2_comm(node);
    motor_control_v2::MotorCommand motor_command(node, motor_config.get_serial_port().c_str(), motor_config.get_baud_rate());

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(http_server.get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}