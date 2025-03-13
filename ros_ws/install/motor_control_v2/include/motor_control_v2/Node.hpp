// motor_control_app.hpp
#ifndef NODE_HPP
#define NODE_HPP

#include "motor_control_v2/http_server.hpp"
#include "motor_control_v2/motor_config.hpp"
#include "motor_control_v2/motor_command.hpp"
#include "motor_control_v2/serial_communication.hpp"
#include "motor_control_v2/ros2_communication.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

namespace motor_control_v2
{

class MotorControlApp : public rclcpp::Node
{
public:
    MotorControlApp(const std::string &config_file_path);
    void initialize();

private:
    nlohmann::json motor_state_;
    std::shared_ptr<HttpServerNode> http_server_;
    std::shared_ptr<MotorConfig> motor_config_;
    std::shared_ptr<SerialCommunication> serial_comm_;
    std::shared_ptr<ROS2Communication> ros2_comm_;
    std::shared_ptr<MotorCommand> motor_command_;

    void initialize_http_server();
    void initialize_components();
};

} // namespace motor_control_v2

#endif // NODE_HPP