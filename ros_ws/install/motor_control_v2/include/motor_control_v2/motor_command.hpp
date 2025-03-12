#ifndef MOTOR_COMMAND_HPP
#define MOTOR_COMMAND_HPP

#include "nlohmann/json.hpp"
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include "rmf_utils/impl_ptr.hpp"

// 前置声明
namespace motor_control_v2
{
    class SerialCommunication;
}

using json = nlohmann::json;

namespace motor_control_v2
{

    class MotorCommand
    {
    public:
        MotorCommand(rclcpp::Node::SharedPtr node, const std::string &serial_port, int baud_rate);
        // 采用广播的方式：发送运动命令
        void send_moving_command(const json &tar_json_data, const json &cur_json_data);
        // 查询指定电机的状态
        void send_query_status_command(int motor_id);
        //读取串口反馈的信息   读取超时时间为秒
        std::vector<unsigned char> get_feedback_from_motor(int timeout_sce);
        //读取串口反馈的信息   读取超时时间为毫秒
        std::vector<unsigned char> get_feedback_from_motor_Ms(int timeout_millsce);

        // 指定电机停止运动和清除故障
        void send_stop_command(int motor_id);
        void send_clear_fault_command(int motor_id);

        // 这里的函数存在调用逻辑上的错误，需要修改
        char get_serial_port() const;
        void set_serial_port(const char* &serial_port);
        int get_baud_rate() const;
        void set_baud_rate(int baud_rate);

        class Implementation;

    private:
        // std::unique_ptr<SerialCommunication> serial_communication_; // 使用智能指针

        rmf_utils::impl_ptr<Implementation> _pimpl;
    };
}

#endif // MOTOR_CONFIG_HPP