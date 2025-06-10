#include <chrono>
#include <iostream>
#include <cstring>
#include <sstream>
#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "httplib.h"
#include "Serial.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

class MotorStatusNode : public rclcpp::Node
{
public:
    MotorStatusNode()
        : Node("motor_status_node"),
          serial_("/dev/ttyUSB0")
    {
        // 初始化串口
        initSerialPort();

        // 启动 HTTP 服务器
        start_http_server();

        // 定义发布者（可以用来发布电机状态）
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("motor_status", 10);

        // 创建定时器以定期更新电机状态
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MotorStatusNode::publish_motor_status, this));
    }

private:
    void start_http_server()
    {
        // 启动 HTTP 服务器的线程
        std::thread([this]()
                    {
                        httplib::Server svr; // 实例化http svr
                        // 状态查询的GET请求
                        svr.Get("/motor/state", [this](const httplib::Request &req, httplib::Response &res)
                                {
                    // 查询电机状态并发送 JSON 响应
                    json status = query_motor_status();
                    res.set_content(status.dump(), "application/json"); });

                        svr.listen("127.0.0.1", 10088); // 启动监听服务器
                    })
            .detach();
    }

    json query_motor_status()
    {
        while (1)
        {
            // 构造发送查询命令
            unsigned char command[] = {0x055, 0xAA, 0x03, 0x01, 0x04, 0x00, 0x22, 0x2A};
            // serial_.write(reinterpret_cast<char*>(command), sizeof(command));
            serial_.write(reinterpret_cast<char *>(command), sizeof(command));
            // 打印发送的命令
            std::stringstream ss_command;
            ss_command << "Sending command: ";
            for (auto byte : command)
            {
                ss_command << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(byte) << " ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", ss_command.str().c_str());

            // 读取串口响应  获取电机返回的结果
            unsigned char result[128];
            auto nrecv = serial_.readBlock(reinterpret_cast<char *>(result), sizeof(result), 1);
            if (nrecv == 0)
            {
                RCLCPP_WARN(this->get_logger(), "No response from motor");
                return json{};
            }
            else if (nrecv < 0)
            {
                RCLCPP_WARN(this->get_logger(), "Error reading from serial port");
                return json{};
            }

            std::stringstream ss;
            ss << "Commintting command: ";
            for (int i = 0; i < nrecv; i++)
            {
                ss << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(result[i]) << " ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());


            // if (serial_.setOpt(921600, 8, 'N', 1) == 0)
            // {

            json motor_status;
            motor_status["timestamp"] = get_current_time();
            int motor_index = static_cast<unsigned char>(result[3]);
            int current_position = (static_cast<unsigned char>(result[10]) << 8) | static_cast<unsigned char>(result[9]);
            int target_position = (static_cast<unsigned char>(result[8]) << 8) | static_cast<unsigned char>(result[7]);

            RCLCPP_INFO(this->get_logger(), "Motor %d: Current Position: %d, Target Position: %d", motor_index, current_position, target_position);

            // 解析结果并封装为 JSON
            std::vector<json> motors;
            uint16_t index;
            json motor;
            try
            {
                motor["index"] = motor_index;
                motor["currentPosition"] = current_position;
                motor["targetPosition"] = target_position;
                motor["error"] = "No error";
                motor["mode"] = "normal";
                motors.push_back(motor);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error parsing motor data: %s", e.what());
            }

            motor_status["motor"] = motors;
            last_motor_status_ = motor_status.dump(); // 保存最后状态
            // return motor_status;

            std::this_thread::sleep_for(std::chrono::microseconds(2));

        }
        // }
        // else
        // {
        //     RCLCPP_WARN(this->get_logger(), "Serial port is not open");
        //     return json{};
        // }
    }

    void publish_motor_status()
    {
        // 发布电机状态到 ROS 主题
        std_msgs::msg::String msg;
        msg.data = last_motor_status_;
        status_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published motor status: %s", msg.data.c_str());
    }

    std::string get_current_time()
    {
        // 获取当前时间，并返回为 ISO 8601 字符串，您可以用 chrono 库实现
        return "2025-01-06T07:11:08Z"; // 示例时间
    }

    void initSerialPort()
    {
        serial_.setOpt(921600, 8, 'N', 1);
        try
        {
            if (serial_.setOpt(921600, 8, 'N', 1) != 0)
            {
                throw std::runtime_error("Failed to set serial port options");
            }
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
        }
    }

    Serial serial_;
    std::string last_motor_status_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorStatusNode>());
    rclcpp::shutdown();
    return 0;
}