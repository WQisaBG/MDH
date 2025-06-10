#include "motor_control_v2/serial_communication.hpp"

namespace motor_control_v2
{
    class SerialCommunication::Implementation
    {
    public:
        Implementation(const char *serial_port, int baud_rate)
            : serial_(serial_port), baud_rate_(baud_rate)
        {
            // doing nothing
        }
        int initSerialPort();

        int send_serial_command(const unsigned char *command, size_t length);
        std::vector<unsigned char> get_feedback_from_motor_Ms(int timeout_millsec);
        std::vector<unsigned char> get_feedback_from_motor(int timeout_sec);

        // int get_baud_rate() const;
        // void set_baud_rate(int baud_rate);

        // char get_serial_port() const;
        // void set_serial_port(const char *&serial_port);

        Serial serial_;
        rclcpp::Node::SharedPtr node_;
        const char *serial_port_;
        int baud_rate_;
    };
    //========================================================================================================================
    SerialCommunication::SerialCommunication(rclcpp::Node::SharedPtr node, const char *serial_port, int baud_rate)
        : _pimpl(rmf_utils::make_impl<Implementation>(serial_port, baud_rate))
    {
        _pimpl->node_ = node; // 初始化 node_ 成员变量
        initSerialPort();
    }
    //========================================================================================================================

    int SerialCommunication::Implementation::initSerialPort()
    {
        int ret = serial_.setOpt(baud_rate_, 8, 'N', 1);

        try
        {
            if (ret == -1)
            {
                throw std::runtime_error("Failed to set serial port options");
                return ret;
            }

            RCLCPP_INFO(node_->get_logger(), "Serial port opened successfully");
            return ret;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
        }
    }
    //========================================================================================================================
    int SerialCommunication::Implementation::send_serial_command(const unsigned char *command, size_t length)
    {
        int ret = -1;
        if (serial_.setOpt(baud_rate_, 8, 'N', 1) != 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Serial port is not open");
            return ret;
        }
        // 打印 command 的内容
        std::ostringstream oss;
        for (size_t i = 0; i < length; ++i)
        {
            oss << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(static_cast<uint8_t>(command[i])) << " ";
        }
        RCLCPP_INFO(node_->get_logger(), "Sending command: %s", oss.str().c_str());

        size_t bytes_to_write = length;
        size_t bytes_written = serial_.write(const_cast<char *>(reinterpret_cast<const char *>(command)), bytes_to_write);
        if (bytes_written != bytes_to_write)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to write all bytes to serial port. Expected %zu, wrote %zu",
                         bytes_to_write, bytes_written);
        }
        if(bytes_written == bytes_to_write)
        {
            RCLCPP_INFO(node_->get_logger(), "Successfully wrote %zu bytes to serial port", bytes_written);
            return 0;
        }
    }
    //========================================================================================================================
    std::vector<unsigned char> SerialCommunication::Implementation::get_feedback_from_motor(int timeout_sec)
    {
        if (serial_.setOpt(baud_rate_, 8, 'N', 1) != 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Serial port is not open");
            return std::vector<unsigned char>();
        }

        const std::vector<unsigned char> sync_bytes = {0xAA, 0x55}; // 同步头
        std::vector<unsigned char> feedback;                        // 存储完整的反馈数据帧

        unsigned char buffer[128];
        int bytes_read = serial_.readBlock(reinterpret_cast<char *>(buffer), sizeof(buffer), timeout_sec);
        if (bytes_read == 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "No response from motor");
            return std::vector<unsigned char>();
        }
        else if (bytes_read < 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error reading from serial port");
            return std::vector<unsigned char>();
        }

        std::vector<unsigned char> new_data(buffer, buffer + bytes_read);

        // 查找同步头
        auto sync_header = std::search(new_data.begin(), new_data.end(), sync_bytes.begin(), sync_bytes.end());
        if (sync_header == new_data.end())
        {
            RCLCPP_WARN(node_->get_logger(), "Sync header not found");
            return std::vector<unsigned char>(); // 未找到同步头，返回空数据
        }

        // 处理数据帧
        for (auto it = sync_header; it < new_data.end();)
        {
            if (it + 3 >= new_data.end())
                break; // 数据不足，跳出循环

            uint8_t length = *(it + 2); // 数据帧长度
            if (it + 3 + length + 1 > new_data.end())
                break; // 数据帧不完整，跳出循环

            // 提取反馈数据
            std::vector<unsigned char> frame(it, it + 3 + length + 1);
            feedback = frame;
            break; // 只处理第一个有效帧，后续帧可以留待下次调用处理
        }

        return feedback;
    }
    //========================================================================================================================
    std::vector<unsigned char> SerialCommunication::Implementation::get_feedback_from_motor_Ms(int timeout_millsec)
    {
        if (serial_.setOpt(baud_rate_, 8, 'N', 1) != 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Serial port is not open");
            return std::vector<unsigned char>();
        }

        const std::vector<unsigned char> sync_bytes = {0xAA, 0x55}; // 同步头
        std::vector<unsigned char> feedback;                        // 存储完整的反馈数据帧

        unsigned char buffer[128];
        int bytes_read = serial_.readBlockMs(reinterpret_cast<char *>(buffer), sizeof(buffer), timeout_millsec);
        if (bytes_read == 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "No response from motor");
            return std::vector<unsigned char>();
        }
        else if (bytes_read < 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error reading from serial port");
            return std::vector<unsigned char>();
        }

        std::vector<unsigned char> new_data(buffer, buffer + bytes_read);

        // 查找同步头
        auto sync_header = std::search(new_data.begin(), new_data.end(), sync_bytes.begin(), sync_bytes.end());
        if (sync_header == new_data.end())
        {
            RCLCPP_WARN(node_->get_logger(), "Sync header not found");
            return std::vector<unsigned char>(); // 未找到同步头，返回空数据
        }

        // 处理数据帧
        for (auto it = sync_header; it < new_data.end();)
        {
            if (it + 3 >= new_data.end())
                break; // 数据不足，跳出循环

            uint8_t length = *(it + 2); // 数据帧长度
            if (it + 3 + length + 1 > new_data.end())
                break; // 数据帧不完整，跳出循环

            // 提取反馈数据
            std::vector<unsigned char> frame(it, it + 3 + length + 1);
            // 返回有效的反馈数据帧
            feedback = frame;
            break; // 只处理第一个有效帧，后续帧可以留待下次调用处理
        }

        return feedback;
    }


    //========================================================================================================================
    // int SerialCommunication::Implementation::get_baud_rate() const
    // {
    //     return baud_rate_;
    // }
    // //========================================================================================================================
    // void SerialCommunication::Implementation::set_baud_rate(int baud_rate)
    // {
    //     baud_rate_ = baud_rate;
    // }
    // //========================================================================================================================
    // char SerialCommunication::Implementation::get_serial_port() const
    // {
    //     return *serial_port_;
    // }
    // //========================================================================================================================
    // void SerialCommunication::Implementation::set_serial_port(const char *&serial_port)
    // {
    //     serial_port_ = serial_port;
    // }
    //========================================================================================================================
    // int SerialCommunication::get_baud_rate() const
    // {
    //     return _pimpl->get_baud_rate();
    // }
    // //========================================================================================================================

    // void SerialCommunication::set_baud_rate(int baud_rate)
    // {
    //     _pimpl->set_baud_rate(baud_rate);
    // }
    // //========================================================================================================================

    // char SerialCommunication::get_serial_port() const
    // {
    //     return _pimpl->get_serial_port();
    // }
    // //========================================================================================================================

    // void SerialCommunication::set_serial_port(const char *&serial_port)
    // {
    //     _pimpl->set_serial_port(serial_port);
    // }
    //========================================================================================================================
    int SerialCommunication::initSerialPort()
    {
        _pimpl->initSerialPort();
    }
    //========================================================================================================================

    int SerialCommunication::send_serial_command(const unsigned char *command, size_t command_length)
    {
       return _pimpl->send_serial_command(command, command_length);
    }
    //========================================================================================================================
    std::vector<unsigned char> SerialCommunication::get_feedback_from_motor(int timeout_sec)
    {
        return _pimpl->get_feedback_from_motor(timeout_sec);
    }
    //========================================================================================================================

    std::vector<unsigned char> SerialCommunication::get_feedback_from_motor_Ms(int timeout_millsec)
    {
        return _pimpl->get_feedback_from_motor_Ms(timeout_millsec);
    }

    //========================================================================================================================

    SerialCommunication::~SerialCommunication()
    {
        // doing noting
    }
} // namespace motor_control_v2
