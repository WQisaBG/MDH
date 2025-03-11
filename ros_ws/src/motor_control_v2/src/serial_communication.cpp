#include "motor_control_v2/serial_communication.hpp"

namespace motor_control_v2
{
    class SerialCommunication::Implementation
    {
    public:
        void initSerialPort();

        void send_serial_command(const unsigned char *&command);
        std::vector<unsigned char> get_feedback_from_motor_Ms(int timeout_millsec);
        std::vector<unsigned char> get_feedback_from_motor(int timeout_sec);

        int get_baud_rate() const;
        void set_baud_rate(int baud_rate);

        char get_serial_port() const;
        void set_serial_port(const char* &serial_port);

        Serial serial_;
        rclcpp::Node::SharedPtr node_;
        const char *serial_port_;
        int baud_rate_;
    };
    //========================================================================================================================
    SerialCommunication::SerialCommunication(rclcpp::Node::SharedPtr node, const char *serial_port, int baud_rate)
        : _pimpl(rmf_utils::make_impl<Implementation>(
              Implementation{
                  Serial(serial_port),
                  node,
                  serial_port,
                  baud_rate

              }))
    {
        // doing noting
    }
    //========================================================================================================================

    void SerialCommunication::Implementation::initSerialPort()
    {
        serial_.setOpt(baud_rate_, 8, 'N', 1);
        try
        {
            if (serial_.setOpt(baud_rate_, 8, 'N', 1) != 0)
            {
                throw std::runtime_error("Failed to set serial port options");
            }
            RCLCPP_INFO(node_->get_logger(), "Serial port opened successfully at %s with baud rate %d");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
        }
    }
    //========================================================================================================================
    void SerialCommunication::Implementation::send_serial_command(const unsigned char *&command)
    {
        if (serial_.setOpt(baud_rate_, 8, 'N', 1) != 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Serial port is not open");
            return;
        }
        size_t bytes_to_write = sizeof(command);
        size_t bytes_written = serial_.write(const_cast<char *>(reinterpret_cast<const char *>(command)), bytes_to_write);
        if (bytes_written != bytes_to_write)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to write all bytes to serial port. Expected %zu, wrote %zu",
                         bytes_to_write, bytes_written);
        }
    }
    //========================================================================================================================
    std::vector<unsigned char> SerialCommunication::Implementation::get_feedback_from_motor(int timeout_sec)
    {
        if (serial_.setOpt(baud_rate_, 8, 'N', 1) != 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Serial port is not open");
            return std::vector<unsigned char>() ;
        }
        unsigned char buffer[128];
        int bytes_read = serial_.readBlockMs(reinterpret_cast<char *>(buffer), sizeof(buffer), timeout_sec);
        if (bytes_read == 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "No response from motor");
            return std::vector<unsigned char>();
        }
        else if (bytes_read < 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error reading from serial port");
        }

        std::vector<unsigned char> feedback(buffer, buffer + bytes_read);
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
        }

        std::vector<unsigned char> feedback(buffer, buffer + bytes_read);
        return feedback;
    }
    //========================================================================================================================
    int SerialCommunication::Implementation::get_baud_rate() const
    {
        return baud_rate_;
    }
    //========================================================================================================================
    void SerialCommunication::Implementation::set_baud_rate(int baud_rate)
    {
        baud_rate_ = baud_rate;
    }
    //========================================================================================================================
    char SerialCommunication::Implementation::get_serial_port() const
    {
        return *serial_port_;
    }
    //========================================================================================================================
    void SerialCommunication::Implementation::set_serial_port(const char* &serial_port)
    {
        serial_port_ = serial_port;
    }
    //========================================================================================================================
    int SerialCommunication::get_baud_rate() const
    {
        return _pimpl->get_baud_rate();
    }
    //========================================================================================================================

    void SerialCommunication::set_baud_rate(int baud_rate)
    {
        _pimpl->set_baud_rate(baud_rate);
    }
    //========================================================================================================================

    char SerialCommunication::get_serial_port() const
    {
        return _pimpl->get_serial_port();
    }
    //========================================================================================================================

    void SerialCommunication::set_serial_port(const char* &serial_port)
    {
        _pimpl->set_serial_port(serial_port);
    }
    //========================================================================================================================
    void SerialCommunication::initSerialPort()
    {
        _pimpl->initSerialPort();
    }
    //========================================================================================================================

    void SerialCommunication::send_serial_command(const unsigned char *&command)
    {
        _pimpl->send_serial_command(command);
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
}// namespace motor_control_v2
