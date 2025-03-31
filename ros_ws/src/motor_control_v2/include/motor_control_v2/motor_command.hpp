#ifndef MOTOR_COMMAND_HPP
#define MOTOR_COMMAND_HPP
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>
#include "nlohmann/json.hpp"
#include <rclcpp/rclcpp.hpp>
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
        /*
         * @brief 构造函数
         * @param serial_port 串口设备名
         * @param baud_rate 波特率
         */
        MotorCommand(rclcpp::Node::SharedPtr node, const std::string &serial_port, int baud_rate);
        /*
         * @brief 采用广播的方式：发送运动命令  为伺服模式，发送频率不低于50hz
         * @param tar_json_data 目标位置
         * @param cur_json_data 当前位置
         * @param step 步长
         * @param time 周期时长
         * @return 0 表示成功，其他表示失败
         */
        int send_moving_command(const json &tar_json_data, const json &cur_json_data, uint8_t step, uint8_t time);

        /*
         * @brief 查询指定电机的状态
         * @param motor_id 电机ID
         */
        void send_query_status_command(int motor_id);

        /*
         * @brief 读取串口反馈的信息
         * @param timeout_sec 读取串口反馈信息的超时时间，单位为秒*/
        std::vector<unsigned char> get_feedback_from_motor(int timeout_sce);

        // 读取串口反馈的信息   读取超时时间为毫秒
        /*
         * @brief 读取串口反馈的信息
         * @param timeout_millsec 读取串口反馈信息的超时时间，单位为毫秒
         */
        std::vector<unsigned char> get_feedback_from_motor_Ms(int timeout_millsce);

        /*
         * @brief 指定电机停止运动
         * @param motor_id 电机ID
         */
        void send_stop_command(int motor_id);

        /*
         * @brief 清除指定电机的故障
         * @param motor_id 电机ID
         */
        void send_clear_fault_command(int motor_id);

        // 这里的函数存在调用逻辑上的错误，需要修改
        // char get_serial_port() const;
        // void set_serial_port(const char *&serial_port);
        // int get_baud_rate() const;
        // void set_baud_rate(int baud_rate);

        class Implementation;

    private:
        rmf_utils::impl_ptr<Implementation> _pimpl;
    };
}

#endif // MOTOR_CONFIG_HPP