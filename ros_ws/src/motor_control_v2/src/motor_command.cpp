#include "motor_control_v2/motor_command.hpp"
#include "motor_control_v2/serial_communication.hpp" // 添加这行代码

namespace motor_control_v2
{
    class MotorCommand::Implementation
    {
    public:
        rclcpp::Node::SharedPtr node_;
        std::string serial_port_;
        int baud_rate_;
        std::unique_ptr<SerialCommunication> serial_communication_;

        int send_moving_command(const json &tar_json_data, const json &cur_json_data, uint8_t step, uint8_t time);
        void runWithParam(const json &tar_json_param);
        void send_query_status_command(int motor_id);
        std::vector<unsigned char> get_feedback_from_motor(int timeout_sec);
        std::vector<unsigned char> get_feedback_from_motor_Ms(int timeout_millsce);

        void send_stop_command(int motor_id);
        void send_clear_fault_command(int motor_id);

        uint8_t calculateChecksum(const uint8_t *data, size_t length);
        bool are_all_motors_reached(const std::unordered_map<uint8_t, uint16_t> &target_positions,
                                    const std::unordered_map<uint8_t, uint16_t> &current_positions);
        void send_serial_command_async(const std::vector<uint8_t> command, size_t command_size);

        // char get_serial_port() const;
        // void set_serial_port(const char *&serial_port);
        // int get_baud_rate() const;
        // void set_baud_rate(int baud_rate);
    };

    MotorCommand::MotorCommand(rclcpp::Node::SharedPtr node, const std::string &serial_port, int baud_rate)
        : _pimpl(std::make_unique<Implementation>(
              Implementation{node, serial_port.c_str(), baud_rate}))
    {

        try
        {

            _pimpl->serial_communication_ = std::make_unique<SerialCommunication>(node, serial_port.c_str(), baud_rate);
            RCLCPP_INFO(node->get_logger(), "Serial communication initialized successfully.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "Error initializing serial communication: %s", e.what());
            rclcpp::shutdown();
        }
    }

    int MotorCommand::Implementation::send_moving_command(const json &tar_json_data, const json &cur_json_data, uint8_t step, uint8_t time)
    {
        // 1、获取电机列表
        const auto &motors = tar_json_data["motor"];
        size_t motor_num = motors.size();

        // 2、初始化目标位置映射
        std::unordered_map<uint8_t, uint16_t> target_positions;
        for (const auto &motor : motors)
        {
            uint8_t motor_index = motor["index"];
            uint16_t target_position = motor["targetPosition"];
            target_positions[motor_index] = target_position;
            // RCLCPP_INFO(this->get_logger(), "Target position of motor %d: %d", motor_index, target_position);
        }

        // 3、获取当前电机状态
        std::unordered_map<uint8_t, uint16_t> current_positions;

        auto &motor_list = cur_json_data["motor"];
        for (const auto &motor : motor_list)
        {
            uint8_t motor_index = motor["motor_id"];
            uint16_t current_position = motor["currentPosition"];
            current_positions[motor_index] = current_position;
            // RCLCPP_INFO(this->get_logger(), "Current position of motor %d: %d", motor_index, current_position);
        }
        // 判断是否所有电机都已到达目标位置
        while (!are_all_motors_reached(current_positions, target_positions))
        {
            // 5、构造广播指令
            std::vector<unsigned char> trans_cmd = {0x55, 0xAA};
            trans_cmd.push_back((motor_num * 3 + 1) & 0xFF); // 指令长度
            trans_cmd.push_back(0xFF);                       // 固定字段
            trans_cmd.push_back(0xF2);                       // 指令类型

            for (const auto &[motor_index, target_position] : target_positions)
            {
                uint16_t current_position = current_positions[motor_index];
                // 计算下一步目标位置
                int16_t position_diff = target_position - current_position;
                int step_target;
                if(std::abs(position_diff) <= step)
                {
                    step_target = target_position;

                }
                else 
                {
                    step_target = current_position + (position_diff > 0 ? step : -step);
                }
                step_target = std::clamp(step_target, 0, 2000);
                current_positions[motor_index] = step_target;
                // RCLCPP_INFO(node_->get_logger(), "Motor %d: current_position=%d, target_position=%d, step_target=%d",
                //             motor_index, current_position, target_position, step_target);

                // 添加电机索引和目标位置
                trans_cmd.push_back(motor_index);
                trans_cmd.push_back(step_target & 0xFF);        // 目标位置低字节
                trans_cmd.push_back((step_target >> 8) & 0xFF); // 目标位置高字节
            }
            // 7、计算校验和
            uint8_t checkSum = calculateChecksum(trans_cmd.data() + 2, trans_cmd.size() - 2);

            trans_cmd.push_back(checkSum);
            std::stringstream ss;
            ss << "Commintting command: ";
            for (auto byte : trans_cmd)
            {
                ss << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(byte) << " ";
            }
            RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
            size_t trans_cmd_size = trans_cmd.size();

            send_serial_command_async(trans_cmd, trans_cmd_size);
            std::this_thread::sleep_for(std::chrono::milliseconds(time));
        }

        // 4、检查是否所有电机都已到达目标位置
        if (are_all_motors_reached(current_positions, target_positions))
        {
            return 0; // 所有电机都已到达目标位置，退出循环
        }
    }

    void MotorCommand::Implementation::runWithParam(const json &tar_json_param)
    {
        const auto &motors = tar_json_param["motor"];

        for (const auto &motor : motors)
        {
            uint8_t motor_index = motor["index"];
            uint16_t target_position = motor["targetPosition"];
            uint16_t target_velocity = motor["targetVelocity"];
            uint16_t target_force = motor["targetForce"];

            std::vector<unsigned char> trans_cmd = {0x55, 0xAA, 0x0D};
            trans_cmd.push_back(static_cast<unsigned char>(motor_index));
            trans_cmd.insert(trans_cmd.end(), {0x32,0x25,0x00,0x05, 0x00,0x00,0x00});
            trans_cmd.push_back(static_cast<unsigned char>(target_force & 0xFF));
            trans_cmd.push_back(static_cast<unsigned char>((target_force >> 8) & 0xFF)); 
            trans_cmd.push_back(static_cast<unsigned char>(target_velocity & 0xFF));
            trans_cmd.push_back(static_cast<unsigned char>(target_velocity >> 8) & 0xFF);
            trans_cmd.push_back(static_cast<unsigned char>(target_position & 0xFF));
            trans_cmd.push_back(static_cast<unsigned char>(target_position >> 8) & 0xFF);

            uint8_t checkSum = calculateChecksum(trans_cmd.data() + 2, trans_cmd.size() - 2);
            trans_cmd.push_back(checkSum);
            size_t trans_cmd_size = trans_cmd.size();
            send_serial_command_async(trans_cmd, trans_cmd_size);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void MotorCommand::Implementation::send_query_status_command(int motor_id)
    {
        std::vector<unsigned char> command = {0x55, 0xAA, 0x03, static_cast<unsigned char>(motor_id), 0x04, 0x00, 0x22};
        uint8_t checkSum = calculateChecksum(command.data() + 2, command.size() - 2);
        command.push_back(checkSum);
        size_t command_size = command.size();
        send_serial_command_async(command, command_size);
    }
    std::vector<unsigned char> MotorCommand::Implementation::get_feedback_from_motor(int timeout_sec)
    {
        return serial_communication_->get_feedback_from_motor(timeout_sec);
    }

    std::vector<unsigned char> MotorCommand::Implementation::get_feedback_from_motor_Ms(int timeout_millsec)
    {
        return serial_communication_->get_feedback_from_motor_Ms(timeout_millsec);
    }

    void MotorCommand::Implementation::send_stop_command(int motor_id)
    {
        std::vector<unsigned char> command = {0x55, 0xAA, 0x05, static_cast<unsigned char>(motor_id), 0x1A, 0x00, 0x01, 0x00};
        uint8_t checkSum = calculateChecksum(command.data() + 2, command.size() - 2);
        command.push_back(checkSum);
        size_t command_size = command.size();
        send_serial_command_async(command, command_size);
    }

    void MotorCommand::Implementation::send_clear_fault_command(int motor_id)
    {
        std::vector<unsigned char> command = {0x55, 0xAA, 0x05, static_cast<unsigned char>(motor_id), 0x32, 0x18, 0x00, 0x01, 0x00};
        uint8_t checkSum = calculateChecksum(command.data() + 2, command.size() - 2);
        command.push_back(checkSum);
        size_t command_size = command.size();
        send_serial_command_async(command, command_size);
    }

    uint8_t MotorCommand::Implementation::calculateChecksum(const uint8_t *data, size_t length)
    {
        uint16_t checksum = 0;
        for (size_t i = 0; i < length; ++i)
        {
            checksum += data[i];
        }
        return static_cast<uint8_t>(checksum & 0xFF);
    }

    bool MotorCommand::Implementation::are_all_motors_reached(const std::unordered_map<uint8_t, uint16_t> &target_positions,
                                                              const std::unordered_map<uint8_t, uint16_t> &current_positions)
    {
        for (const auto &[motor_index, target_position] : target_positions)
        {
            // 检查当前电机是否存在
            if (current_positions.find(motor_index) == current_positions.end())
            {
                RCLCPP_ERROR(node_->get_logger(), "Motor index %d not found in current positions", motor_index);
                return false; // 如果某个电机的状态缺失，直接返回 false
            }

            // 计算位置差
            int16_t position_diff = target_position - current_positions.at(motor_index);
            if (std::abs(position_diff) > 1)
            {
                return false; // 如果任意一个电机未到达目标位置，返回 false
            }
        }
        return true; // 所有电机都已到达目标位置
    }

    void MotorCommand::Implementation::send_serial_command_async(const std::vector<unsigned char> command, size_t command_size)
    {
        try
        {
            std::async(std::launch::async, [this, command, command_size]()
                       {  serial_communication_->send_serial_command(command.data(), command_size); });
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error sending serial command: %s", e.what());
        }
    }

    //========================================================================================================================
    // char MotorCommand::Implementation::get_serial_port() const
    // {
    //     return serial_communication_->get_serial_port();
    // }

    // void MotorCommand::Implementation::set_serial_port(const char *&serial_port)
    // {
    //     serial_communication_->set_serial_port(serial_port);
    // }

    // int MotorCommand::Implementation::get_baud_rate() const
    // {
    //     return serial_communication_->get_baud_rate();
    // }

    // void MotorCommand::Implementation::set_baud_rate(int baud_rate)
    // {
    //     serial_communication_->set_baud_rate(baud_rate);
    // } //    这里不对     设定串口波特率   需要给电机写指令   需要串口通信模块   这里混乱了

    //========================================================================================================================

    int MotorCommand::send_moving_command(const json &tar_json_data, const json &cur_json_data, uint8_t step, uint8_t time)
    {
        return _pimpl->send_moving_command(tar_json_data, cur_json_data, step, time);
    }

    void MotorCommand::runWithParam(const json &tar_json_param)
    {
        _pimpl->runWithParam(tar_json_param);
    }
    //========================================================================================================================

    void MotorCommand::send_query_status_command(int motor_id)
    {
        _pimpl->send_query_status_command(motor_id);
    }

    std::vector<unsigned char> MotorCommand::get_feedback_from_motor(int timeout_sec)
    {
        return _pimpl->get_feedback_from_motor(timeout_sec);
    }

    std::vector<unsigned char> MotorCommand::get_feedback_from_motor_Ms(int timeout_millsec)
    {
        return _pimpl->get_feedback_from_motor_Ms(timeout_millsec);
    }
    //========================================================================================================================

    void MotorCommand::send_stop_command(int motor_id)
    {
        _pimpl->send_stop_command(motor_id);
    }
    //========================================================================================================================

    void MotorCommand::send_clear_fault_command(int motor_id)
    {
        _pimpl->send_clear_fault_command(motor_id);
    }

    // char MotorCommand::get_serial_port() const
    // {
    //     return _pimpl->get_serial_port();
    // }

    // void MotorCommand::set_serial_port(const char *&serial_port)
    // {
    //     _pimpl->set_serial_port(serial_port);
    // }

    // int MotorCommand::get_baud_rate() const
    // {
    //     return _pimpl->get_baud_rate();
    // }

    // void MotorCommand::set_baud_rate(int baud_rate)
    // {
    //     _pimpl->set_baud_rate(baud_rate);
    // }

}
