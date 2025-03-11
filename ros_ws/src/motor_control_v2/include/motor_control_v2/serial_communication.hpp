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
        SerialCommunication(rclcpp::Node::SharedPtr node,  const char* serial_port, int baud_rate);
        ~SerialCommunication();
        void initSerialPort();   //  初始化串口
        

        void  send_serial_command(const unsigned char* &command);    //发送命令
        std::vector<unsigned char>  get_feedback_from_motor_Ms(int timeout_millsec);  // 超时时间  毫秒
        std::vector<unsigned char> get_feedback_from_motor(int timeout_sec);    //超时时间  秒 

        int get_baud_rate() const; //获取波特率
        void set_baud_rate(int baud_rate); //设置波特率

        char get_serial_port() const; //获取串口
        void set_serial_port(const char* &serial_port); //设置串口

        class Implementation;

    

    private:
        rmf_utils::impl_ptr<Implementation> _pimpl;
    

};
}

#endif // SERIAL_COMMUNICATION_HPP