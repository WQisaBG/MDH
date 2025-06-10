#include "motor_control_v2/Node.hpp"

namespace motor_control_v2
{
    MotorControlApp::MotorControlApp(const std::string &config_file_path)
        : Node("motor_control_node"),
          stop_server_(false)
    {
        // 加载配置文件
        std::ifstream config_file(config_file_path);
        if (!config_file.is_open())
        {
            throw std::runtime_error("Failed to open config file: " + config_file_path);
        }
    }

    MotorControlApp::~MotorControlApp()
    {
    }

    void MotorControlApp::initialize()
    {
        motor_config_ = std::make_shared<MotorConfig>(this->shared_from_this(), "config.json");
        initialize_components();
    }

    void MotorControlApp::initialize_components()
    {
        auto node_shared_ptr = this->shared_from_this();
        ros2_comm_ = std::make_shared<ROS2Communication>(node_shared_ptr);
        http_server_ = std::make_shared<InternalHttpServer>(node_shared_ptr,
                                                            motor_config_->get_motor_count(),
                                                            motor_config_->get_serial_port().c_str(),
                                                            motor_config_->get_baud_rate(),
                                                            motor_config_->get_http_ip().c_str(),
                                                            motor_config_->get_http_port());
    }

} // namespace motor_control_v2