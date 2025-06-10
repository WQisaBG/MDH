#include "motor_control_v2/motor_config.hpp"
#include <fstream>

MotorConfig::MotorConfig(rclcpp::Node::SharedPtr node, const std::string &config_file)
    : node_(node)
{
    load_config(config_file);
}

void MotorConfig::load_config(const std::string &config_file)
{
    std::ifstream f(config_file);
    if (!f.is_open())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open config file: %s", config_file.c_str());
        throw std::runtime_error("Failed to open config file");
    }

    json config;
    try
    {
        config = json::parse(f);
    }
    catch (const json::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse config file: %s", e.what());
        throw std::runtime_error("Failed to parse config file");
    }
    json motor_config = config.value("motor_config", json::object());
    json http_config = config.value("http_config", json::object());
    if (motor_config.empty() || http_config.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load config file");
        throw std::runtime_error("Failed to load config file");
    }

    serial_port_ = motor_config.value("serial_port", "/dev/ttyUSB0");
    baud_rate_ = motor_config.value("baud_rate", 921600);
    motor_count_ = motor_config.value("motor_count", 2);
    http_port_ = http_config.value("port", 8080);
    http_ip_ = http_config.value("ip", "127.0.0.1");
}
