#ifndef SERIAL_COMMUNICATION_HPP
#define SERIAL_COMMUNICATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "Serial.h"
#include "rmf_utils/impl_ptr.hpp"

namespace motor_control_v2
{
    class SerialCommunication
    {
    public:
        /*
         * @brief 串口通信类
         * @param node 节点
         * @param serial_port 串口
         * @param baud_rate 波特率
         */
        SerialCommunication(rclcpp::Node::SharedPtr node, const char *serial_port, int baud_rate);
        ~SerialCommunication();
        /*
         * @brief 初始化串口
         *
        */
        void initSerialPort(); 
        /*
         * @brief 获取波特率
         * @return int 波特率
         */
        int get_baud_rate() const;        
        /*
         * @brief 设置波特率
         * @param int baud_rate 波特率
         */
        void set_baud_rate(int baud_rate); 
        /*
         * @brief 获取串口
         * @return char 串口
         */
        char get_serial_port() const;    
        /*
         * @brief 设置串口
         * @param char serial_port 串口
         */              
        void set_serial_port(const char *&serial_port); 




        /*
         * @brief 发送命令
         * @param unsigned char* command 命令
         */
        void send_serial_command(const unsigned char *command);                     
        /*
         * @brief 获取反馈
         * @param int timeout_millsec 超时时间  秒
         * @return std::vector<unsigned char> 反馈
         */
        std::vector<unsigned char> get_feedback_from_motor_Ms(int timeout_millsec); 
        /*
         * @brief 获取反馈
         * @param int timeout_sec 超时时间  秒
         * @return std::vector<unsigned char> 反馈
         */
        std::vector<unsigned char> get_feedback_from_motor(int timeout_sec);       




        class Implementation;

    private:
        rmf_utils::impl_ptr<Implementation> _pimpl;
    };
}

#endif // SERIAL_COMMUNICATION_HPP