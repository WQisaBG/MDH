#include "motor_control_v2/motor_config.hpp"
#include <fstream>

MotorConfig::MotorConfig(rclcpp::Node::SharedPtr node, const std::string &config_file)
    : node_(node)
{
    load_config(config_file);
    initialize_motor_status();
}

void MotorConfig::load_config(const std::string &config_file)
{
    std::ifstream f(config_file);
    if (!f.is_open())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open config file: %s", config_file.c_str());
        throw std::runtime_error("Failed to open config file");
    }

    json config = json::parse(f);
    serial_port_ = config.value("serial_port", "/dev/ttyUSB0");
    baud_rate_ = config.value("baud_rate", 921600);
    motor_count_ = config.value("motor_count", 2);
}

void MotorConfig::initialize_motor_status()
{
    motor_status_["motor"] = json::array();
    RCLCPP_INFO(node_->get_logger(), "Motor status initialization completed.");
}

json MotorConfig::get_motor_status()
{
    std::lock_guard<std::mutex> lock(motor_status_mutex_);
    return motor_status_;
}

void MotorConfig::update_motor_status(const std::string &feedback)
{
    std::lock_guard<std::mutex> lock(motor_status_mutex_);
    // Update motor status based on feedback
}