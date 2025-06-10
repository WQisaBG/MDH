#ifndef MOTOR_CONFIG_HPP
#define MOTOR_CONFIG_HPP

#include "nlohmann/json.hpp"
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <string>

using json = nlohmann::json;

class MotorConfig
{
public:
    MotorConfig(rclcpp::Node::SharedPtr node, const std::string &config_file);
    std::string get_serial_port() const { return serial_port_; }
    int get_baud_rate() const { return baud_rate_; }
    int get_motor_count() const { return motor_count_; }
    int get_http_port() const { return http_port_; }
    std::string get_http_ip() const { return http_ip_; }

private:
    void load_config(const std::string &config_file);

    json motor_status_;
    rclcpp::Node::SharedPtr node_;
    std::string serial_port_;
    int baud_rate_;
    int motor_count_;
    int http_port_;
    std::string http_ip_;
};

#endif // MOTOR_CONFIG_HPP