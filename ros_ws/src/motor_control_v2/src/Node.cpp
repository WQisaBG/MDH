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
        stop_server_ = true;
        request_cv_.notify_all();
        if (post_request_processing_thread_.joinable()) post_request_processing_thread_.join();
        if (query_motor_state_thread_.joinable()) query_motor_state_thread_.join();
        if (get_motor_state_thread_.joinable()) get_motor_state_thread_.join();
    }

    void MotorControlApp::initialize()
    {
        motor_config_ = std::make_shared<MotorConfig>(this->shared_from_this(), "config.json");
        motor_count_ = motor_config_->get_motor_count();
        initialize_components();
    }

    void MotorControlApp::initialize_components()
    {
        auto node_shared_ptr = this->shared_from_this();
        serial_comm_ = std::make_shared<SerialCommunication>(node_shared_ptr, motor_config_->get_serial_port().c_str(), motor_config_->get_baud_rate());
        ros2_comm_ = std::make_shared<ROS2Communication>(node_shared_ptr);
        http_server_ = std::make_shared<InternalHttpServer>(node_shared_ptr, motor_config_->get_serial_port().c_str(), motor_config_->get_baud_rate());
        motor_command_ = std::make_shared<MotorCommand>(node_shared_ptr, motor_config_->get_serial_port().c_str(), motor_config_->get_baud_rate());
    }

    
} // namespace motor_control_v2