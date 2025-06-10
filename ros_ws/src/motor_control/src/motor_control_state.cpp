#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <cstring>
#include <sstream>
#include <thread>
#include <memory>
#include <queue>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <condition_variable>
#include "nlohmann/json.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <std_msgs/msg/string.hpp>
#include "httplib.h"
#include "Serial.h"
#include "motor_control_command_msgs/msg/motor_control_command.hpp"
#include "motor_control_command_msgs/msg/motor.hpp"

using json = nlohmann::json;

struct RequestWithResponse
{
    nlohmann::json json_data;
    httplib::Response *response = nullptr;
};

class HttpServerNode : public rclcpp::Node
{
public:
    HttpServerNode()
        : Node("http_server_node"),
          stop_server_(false),
          serial_("/dev/ttyUSB0")
    {
        pub_ = this->create_publisher<motor_control_command_msgs::msg::MotorControlCommand>("motor_command", 10);
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("motor_status", 10);

        RCLCPP_INFO(this->get_logger(), "HTTP server starting on http://127.0.0.1:10088/motor/task");

        // 1、初始化
        initialize_motor_status();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        initSerialPort();

        // 2、启动线程
        http_server_thread_ = std::thread(&HttpServerNode::start_http_server, this); // http server thread
        feedback_thread_ = std::thread(&HttpServerNode::feedback_listener, this);
    }

    ~HttpServerNode()
    {
        stop_server_ = true;
        if (http_server_thread_.joinable())
            http_server_thread_.join();
        if (feedback_thread_.joinable())
            feedback_thread_.join();
    }

    struct MotorTarget
    {
        uint16_t position;
        uint16_t velocity;
        uint16_t force;
    };

    struct MotorStatus
    {
        uint16_t current_position;
        uint16_t current_force;
    };

private:
    void start_http_server()
    {
        httplib::Server server;

        server.Post("/motor/task", [this](const httplib::Request &req, httplib::Response &res)
                    { handle_post(req, res); });
        server.Post("/motor/e_stop", [this](const httplib::Request &req, httplib::Response &res)
                    { handle_pause_motor(req, res); });
        server.Post("/motor/clear_fault", [this](const httplib::Request &req, httplib::Response &res)
                    { handle_clear_fault_motor(req, res); });

        server.Get("/motor/state", [this](const httplib::Request &req, httplib::Response &res)
                   {
                        {
                            std::lock_guard<std::mutex> lock(status_update_mutex_);
                            status_update_needed_ = true; // 标记需要更新
                        }
                    
                        // 发送查询命令获取最新状态
                        for (uint8_t motor_index = 1; motor_index < 11; motor_index++) 
                        {
                            std::vector<uint8_t> command = create_motor_status_query_command(motor_index);
                            send_serial_command(command);
                            std::this_thread::sleep_for(std::chrono::milliseconds(5));
                            send_serial_command(command);
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        }
                    
                        // 等待状态更新完成
                        std::unique_lock<std::mutex> lock(status_update_mutex_);
                        status_updated_.wait(lock, [this]{ return !status_update_needed_; });
                    
                        // 构建完整 JSON 并作为响应返回
                        json response_json;
                        {
                            std::lock_guard<std::mutex> lock(motor_status_mutex_);

                            uint64_t current_id = res_state_task_id_counter_.fetch_add(1);
                            response_json["id"] = "RES_STATE_TASK_" + std::to_string(current_id);
                            response_json["timestamp"] = get_current_iso_time();
                    
                            json motor_array = json::array();
                            auto &motors = motor_status_["motor"];
                    
                            for (const auto &motor : motors)
                            {
                                json motor_obj;
                                motor_obj["index"] = motor["index"];
                                motor_obj["currentPosition"] = motor["currentPosition"];
                                motor_obj["targetPosition"] = motor["targetPosition"];
                                motor_obj["currentElectricity"] = motor["currentElectricity"];
                                motor_obj["currentForce"] = motor["currentForce"];
                                motor_obj["initForce"] = motor["initForce"];
                                motor_obj["currentTemperature"] = motor["currentTemperature"];
                                motor_obj["faultCode"] = motor["faultCode"];
                                motor_array.push_back(motor_obj);
                            }
                            response_json["motor"] = motor_array;
                        }
                    
                        res.set_content(response_json.dump(), "application/json"); });

        while (!stop_server_)
        {
            server.listen("127.0.0.1", 10088);
        }
    }

    //======================================================================================================================
    void initialize_motor_status()
    {
        // 初始化电机状态
        motor_status_["motor"] = json::array(); // 确保 "motor" 是一个数组

        for (uint8_t motor_index = 1; motor_index < 11; motor_index++)
        {
            std::vector<uint8_t> command = create_motor_status_query_command(motor_index);
            send_serial_command(command);
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 等待响应
        }
        RCLCPP_INFO(this->get_logger(), "Motor status initialization completed.");
    }
    //======================================================================================================================
    void initSerialPort() /// 初始化串口
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
    //======================================================================================================================

    void handle_post(const httplib::Request &req, httplib::Response &res)
    {
        std::lock_guard<std::mutex> lock(request_mutex_);
        try
        {
            auto json_data = json::parse(req.body);
            RCLCPP_INFO(this->get_logger(), "Received JSON data: %s", json_data.dump().c_str());

            if (json_data.contains("id") && json_data.contains("timestamp") && json_data.contains("motor"))
            {
                send_request_motors_with_completion(json_data, &res);
            }
            else
            {
                throw std::invalid_argument("Invalid JSON structure");
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", ex.what());
            res.set_content("{\"status\":\"error\",\"message\":\"Invalid JSON\"}", "application/json");
        }
    }
    //======================================================================================================================
    void handle_pause_motor(const httplib::Request &req, httplib::Response &res)
    {
        json json_data = json::parse(req.body);
        try
        {
            std::string TASK_id = json_data["id"];
            bool is_all_pause = json_data["is_emergency_stop"];
            if (is_all_pause)
            {
                pause_requested_ = true; // 设置暂停标志
                for (int i = 1; i <= 11; i++)
                {
                    send_e_stop_command(i);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            res.set_content("{\"status\":\"success\"}", "application/json");
        }
        catch (const std::exception &e)
        {
            res.status = 400;
            res.set_content(e.what(), "text/plain");
        }
    }
    void send_e_stop_command(int motor_id)
    {
        std::vector<unsigned char> command = {0x55, 0xAA, 0x05, static_cast<unsigned char>(motor_id), 0x32, 0x1A, 0x00, 0x01, 0x00};
        uint8_t checkSum = calculateChecksum(command.data() + 2, command.size() - 2);
        command.push_back(checkSum);
        send_serial_command(command);
        // RCLCPP_INFO(this->get_logger(), "Sent clear e_stop to motor %d", motor_id);
    }

    //======================================================================================================================

    void handle_clear_fault_motor(const httplib::Request &req, httplib::Response &res)
    {
        json json_data = json::parse(req.body);
        try
        {
            std::string TASK_id = json_data["id"];
            bool is_all_clear = json_data["is_clear_fault"];
            if (is_all_clear)
            {
                for (int i = 1; i <= 11; i++)
                {
                    send_clear_fault_command(i);
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
            }
            res.set_content("{\"status\":\"success\"}", "application/json");
        }
        catch (const std::exception &e)
        {
            res.status = 400;
            res.set_content(e.what(), "text/plain");
        }
    }

    void send_clear_fault_command(int motor_id)
    {
        std::vector<unsigned char> command = {0x55, 0xAA, 0x05, static_cast<unsigned char>(motor_id), 0x32, 0x18, 0x00, 0x01, 0x00};
        uint8_t checkSum = calculateChecksum(command.data() + 2, command.size() - 2);
        command.push_back(checkSum);
        send_serial_command(command);
        RCLCPP_INFO(this->get_logger(), "Sent clear fault command to motor %d", motor_id);
    }

    //======================================================================================================================

    void process_request_with_response(const json &json_data, httplib::Response *res)
    {
        // send_request_topic(json_data);
        bool success = send_request_motors_with_completion(json_data, res);

        if (!success)
        {
            if (res)
            {
                RCLCPP_WARN(this->get_logger(), "[RESPONSE] Failed to complete request.");
                res->set_content("{\"status\":\"FAILED\"}", "application/json");
            }
        }
    }
    //======================================================================================================================

    std::string get_current_iso_time()
    {
        auto now = std::chrono::system_clock::now();
        auto t_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&t_c), "%Y-%m-%dT%H:%M:%SZ") << "+08:00";
        return ss.str();
    }
    //======================================================================================================================

    bool send_request_motors_with_completion(const json &json_data, httplib::Response *res)
    {
        const auto &motors = json_data["motor"];
        size_t motor_num = motors.size();

        std::unordered_map<uint8_t, MotorTarget> motor_targets;
        for (const auto &motor : motors)
        {
            uint8_t motor_index = motor["index"];
            float raw_position = motor["targetPosition"].get<float>();
            float raw_velocity = motor["targetVelocity"].get<float>();
            float raw_force = motor["targetForce"].get<float>();

            if (raw_position < 0.0f || raw_position > 100.0f ||
                raw_velocity < 0.0f || raw_velocity > 100.0f ||
                raw_force < 0.0f || raw_force > 100.0f)
            {
                // RCLCPP_WARN(this->get_logger(), "Invalid targetPosition value: %.3f", raw_position);
                raw_position = std::clamp(raw_position, 0.0f, 100.0f);
                raw_velocity = std::clamp(raw_velocity, 0.0f, 100.0f);
                raw_force = std::clamp(raw_force, 0.0f, 100.0f);
            }

            MotorTarget target{
                static_cast<uint16_t>(std::roundf(raw_position * 20)),
                static_cast<uint16_t>(std::roundf(raw_velocity * 20)),
                static_cast<uint16_t>(std::roundf(raw_force * 100))};

            motor_targets[motor_index] = target;
        }

        using namespace std::chrono_literals;

        for (const auto &[motor_index, target] : motor_targets)
        {
            RCLCPP_INFO(this->get_logger(), "Motor %d:  target_position=%d; target_velocity=%d; target_force=%d",
                        motor_index, target.position, target.velocity, target.force);

            std::vector<uint8_t> trans_cmd = {0x55, 0xAA, 0x0D, motor_index, 0x32, 0x25, 0x00, 0x05, 0x00, 0x00, 0x00};
            trans_cmd.push_back(target.force & 0xFF);
            trans_cmd.push_back((target.force >> 8) & 0xFF);
            trans_cmd.push_back(target.velocity & 0xFF);
            trans_cmd.push_back((target.velocity >> 8) & 0xFF);
            trans_cmd.push_back(target.position & 0xFF);
            trans_cmd.push_back((target.position >> 8) & 0xFF);
            uint8_t checkSum = calculateChecksum(trans_cmd.data() + 2, trans_cmd.size() - 2);
            trans_cmd.push_back(checkSum);
            send_serial_command_async(trans_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(status_update_mutex_);
                status_update_needed_ = true;
            }

            for (uint8_t motor_index = 1; motor_index < 11; motor_index++)
            {
                std::vector<uint8_t> command = create_motor_status_query_command(motor_index);
                send_serial_command(command);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

            std::unique_lock<std::mutex> lock(status_update_mutex_);
            status_updated_.wait(lock, [this]
                                 { return !status_update_needed_; });

            std::unordered_map<uint8_t, MotorStatus> motor_status;
            {
                std::lock_guard<std::mutex> lock(motor_status_mutex_);
                auto &motor_list = motor_status_["motor"];
                for (const auto &motor : motor_list)
                {
                    uint8_t motor_index = motor["index"];
                    uint16_t current_position = motor["currentPosition"];
                    uint16_t current_force = motor["currentForce"];
                    MotorStatus status{current_position, current_force};
                    motor_status[motor_index] = status;
                }
            }

            if (pause_requested_)
            {
                RCLCPP_INFO(this->get_logger(), "[RESPONSE] Command canceled due to pause.");
                pause_requested_ = false;
                if (res)
                {
                    res->set_content("{\"status\":\"PAUSED\"}", "application/json");
                }
                return false;
            }

            if (reach_max_force(motor_status, motor_targets))
            {
                if (res)
                {
                    RCLCPP_INFO(this->get_logger(), "[RESPONSE] Reach max force, sending FAILED.");
                    json response_json = {
                        {"id", json_data["id"].get<std::string>()},
                        {"timestamp", get_current_iso_time()},
                        {"actioncompleted", false},
                        {"error", "Reach max force"}};
                    res->set_content(response_json.dump(), "application/json");
                }
                return false ; 
            }

            if (are_all_motors_reached(motor_status, motor_targets))
            {
                if (res)
                {
                    RCLCPP_INFO(this->get_logger(), "[RESPONSE] All motors reached, sending SUCCESS.");
                    json response_json = {
                        {"id", json_data["id"].get<std::string>()},
                        {"timestamp", get_current_iso_time()},
                        {"actioncompleted", true},
                        {"error", "NULL"}};
                    res->set_content(response_json.dump(), "application/json");
                }
                pause_requested_ = false; // 重置标志
                return true;
            }
        }
    }

    //=====================================================================================================================

    bool are_all_motors_reached(const std::unordered_map<uint8_t, MotorStatus> &status,
                                const std::unordered_map<uint8_t, MotorTarget> &targets)
    {
        for (const auto &[motor_index, target] : targets)
        {
            auto it = status.find(motor_index);
            // 检查当前电机是否存在
            if (it == status.end())
            {
                RCLCPP_ERROR(this->get_logger(), "Motor index %d not found in current positions", motor_index);
                return false; // 如果某个电机的状态缺失，直接返回 false
            }

            // 计算位置差
            uint16_t current_position = it->second.current_position;
            int16_t position_diff = static_cast<int16_t>(target.position) - static_cast<int16_t>(current_position);
            if (std::abs(position_diff) > 5)
            {
                return false; // 如果任意一个电机未到达目标位置，返回 false
            }
        }
        return true; // 所有电机都已到达目标位置
    }
    //======================================================================================================================
    bool reach_max_force(const std::unordered_map<uint8_t, MotorStatus> &status,
                         const std::unordered_map<uint8_t, MotorTarget> &targets)
    {
        for (const auto &[motor_index, target] : targets)
        {
            auto it = status.find(motor_index);
            // 检查当前电机是否存在
            if (it == status.end())
            {
                RCLCPP_ERROR(this->get_logger(), "Motor index %d not found in current positions", motor_index);
                return false; // 如果某个电机的状态缺失，直接返回 false
            }
            uint16_t current_force = it->second.current_force;
            if(std::abs(current_force) > target.force/100)
            {
                RCLCPP_INFO(this->get_logger(), "Motor index %d reach max force", motor_index);
                for (int i = 1; i <= 11; i++)
                {
                    send_e_stop_command(i);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                return true; // 如果任意一个电机未到达目标位置，返回 false
            }
        }
        return false;
    }

    //======================================================================================================================

    void send_serial_command_async(const std::vector<uint8_t> &command)
    {
        std::async(std::launch::async, [this, command]()
                   { send_serial_command(command); });
    }

    void send_serial_command(const std::vector<uint8_t> &command)
    {
        if (serial_.setOpt(921600, 8, 'N', 1) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
            return;
        }

        size_t bytes_to_write = command.size();
        char *data_ptr = const_cast<char *>(reinterpret_cast<const char *>(command.data())); // 数据转换置为char*类型

        size_t bytes_written = serial_.write(data_ptr, bytes_to_write);

        if (bytes_written != bytes_to_write)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write all bytes to serial port. Expected %zu, wrote %zu",
                         bytes_to_write, bytes_written);
        }
    }

    std::vector<uint8_t> create_motor_status_query_command(uint8_t motor_index)
    {
        std::vector<uint8_t> command = {0x55, 0xAA, 0x01, motor_index, 0x30};
        uint8_t checkSum = calculateChecksum(command.data() + 2, command.size() - 2);
        command.push_back(checkSum);
        return command;
    }

    uint8_t calculateChecksum(const uint8_t *data, size_t length)
    {
        uint16_t checksum = 0; // 初始值为 0
        for (size_t i = 0; i < length; ++i)
        {
            checksum += data[i];
            if (checksum > 0xFF) // 处理溢出
            {
                checksum &= 0xFF;
            }
        }
        return static_cast<uint8_t>(checksum & 0xFF);
    }
    void feedback_listener() // 监听反馈的状态
    {
        std::vector<uint8_t> buffer;
        while (!stop_server_)
        {
            process_feedback_from_serial(buffer);
        }
    }

    void process_feedback_from_serial(std::vector<uint8_t> &buffer)
    {
        if (serial_.setOpt(921600, 8, 'N', 1) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
            return;
        }

        // 读取新数据
        unsigned char result[128];
        auto bytes_read = serial_.readBlock(reinterpret_cast<char *>(result), sizeof(result), 1);
        if (bytes_read == 0)
        {
            // RCLCPP_WARN(this->get_logger(), "No response from motor");
            return;
        }
        else if (bytes_read < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Error reading from serial port");
            return;
        }

        std::vector<uint8_t> new_data(result, result + bytes_read);

        // // 输出调试信息
        // std::stringstream ss;
        // ss << "Received data: ";
        // for (auto byte : new_data)
        // {
        //     ss << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(byte) << " ";
        // }
        // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

        // 查找同步头
        auto sync_header = std::search(new_data.begin(), new_data.begin() + bytes_read,
                                       std::begin(sync_bytes), std::end(sync_bytes));
        if (sync_header == new_data.begin() + bytes_read)
            return; // 没有找到同步头，直接返回

        // 处理数据帧
        for (auto it = sync_header; it < new_data.begin() + bytes_read;)
        {
            if (it + 3 >= new_data.begin() + bytes_read)
                break; // 数据不足，跳出循环

            uint8_t length = *(it + 2);
            if (it + 3 + length + 1 > new_data.begin() + bytes_read)
                break; // 数据帧不完整，跳出循环

            // 提取反馈数据
            std::vector<uint8_t> feedback(it, it + 3 + length + 1);
            it += 3 + length + 1;

            if (is_valid_feedback(feedback))
            {
                int motor_index = feedback[3];
                std::string feedback_str = std::move(std::string(feedback.begin(), feedback.end()));
                {
                    std::lock_guard<std::mutex> lock(lastest_feedback_mutex_);
                    lastest_feedback_ = {motor_index, std::move(feedback_str)};
                    update_motor_status(lastest_feedback_.second);
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Invalid feedback detected and discarded");
            }
        }

        // 更新缓冲区
        buffer.insert(buffer.end(), new_data.begin() + bytes_read, new_data.end());
    }

    bool is_valid_feedback(const std::vector<uint8_t> &feedback)
    {
        if (feedback.size() < 4)
            return false;
        uint8_t length = feedback[2];
        if (feedback.size() != 3 + length + 1)
            return false;

        if (feedback[0] != 0xAA || feedback[1] != 0x55)
            return false;

        int motor_id = static_cast<unsigned char>(feedback[3]);
        int current_position = (static_cast<unsigned char>(feedback[10]) << 8) | static_cast<unsigned char>(feedback[9]);
        int target_position = (static_cast<unsigned char>(feedback[8]) << 8) | static_cast<unsigned char>(feedback[7]);
        if (motor_id < 1 || motor_id > 6)
            return false;
        if (current_position < 0 || current_position > 2000)
            return false;
        if (target_position < 0 || target_position > 2000)
            return false;

        return true;
    }

    void update_motor_status(const std::string &feedback)
    {
        if (feedback.size() < 8)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid feedback length: %zu", feedback.size());
            return;
        }

        int motor_index = static_cast<unsigned char>(feedback[3]);
        int current_position = (static_cast<unsigned char>(feedback[10]) << 8) | static_cast<unsigned char>(feedback[9]);
        int target_position = (static_cast<unsigned char>(feedback[8]) << 8) | static_cast<unsigned char>(feedback[7]);
        int current_electricity = (static_cast<unsigned char>(feedback[12]) << 8) | static_cast<unsigned char>(feedback[11]);
        int current_force = (static_cast<unsigned char>(feedback[14]) << 8) | static_cast<unsigned char>(feedback[13]);
        int init_force = (static_cast<unsigned char>(feedback[16]) << 8) | static_cast<unsigned char>(feedback[15]);
        int current_temperature = static_cast<unsigned char>(feedback[17]);
        int fault_code = static_cast<unsigned char>(feedback[18]);

        if (current_force > 0xC350)
        {
            current_force = (current_force - 0xFFFF) / 100;
        }
        else
        {
            current_force = current_force / 100;
        }

        std::lock_guard<std::mutex> lock(motor_status_mutex_);

        auto &motors = motor_status_["motor"];
        bool found = false;

        for (auto &motor : motors)
        {
            if (motor["index"] == motor_index)
            {
                motor["currentPosition"] = current_position;
                motor["targetPosition"] = target_position;
                motor["currentElectricity"] = current_electricity;
                motor["currentForce"] = current_force;
                motor["initForce"] = init_force;
                motor["currentTemperature"] = current_temperature;
                motor["faultCode"] = fault_code;
                found = true;
                break;
            }
        }

        if (!found)
        {
            // 如果没有找到，则添加新的电机状态
            motors.push_back({{"index", motor_index},
                              {"currentPosition", current_position},
                              {"targetPosition", target_position},
                              {"currentElectricity", current_electricity},
                              {"currentForce", current_force},
                              {"initForce", init_force},
                              {"currentTemperature", current_temperature},
                              {"faultCode", fault_code}});
        }

        // RCLCPP_INFO(this->get_logger(), "Updated motor %d status: current=%d, target=%d", motor_index, current_position, target_position);
        {
            std::lock_guard<std::mutex> lock(status_update_mutex_);
            status_update_needed_ = false;
            status_updated_.notify_all();
        }
    }

    std::pair<int, std::string> lastest_feedback_;
    std::mutex lastest_feedback_mutex_;
    rclcpp::Publisher<motor_control_command_msgs::msg::MotorControlCommand>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    Serial serial_;
    std::thread http_server_thread_, feedback_thread_, request_processing_thread_, motor_status_query_thread_;
    std::atomic<bool> stop_server_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex command_mutex_;
    std::condition_variable command_cv_;
    std::mutex request_mutex_;
    std::condition_variable request_cv_;
    std::atomic<int> current_motor_index_ = -1;
    const std::vector<uint8_t> sync_bytes = {0xAA, 0x55};
    std::mutex motor_status_mutex_;
    std::mutex trans_cmd_mutex_;
    json motor_status_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    json current_request_;
    std::mutex current_request_mutex_;
    std::mutex status_update_mutex_;
    std::condition_variable status_updated_;
    bool status_update_needed_ = false;
    std::queue<RequestWithResponse> request_queue_;
    std::atomic<bool> pause_requested_ = false; // 是否请求紧急停止
    std::atomic<uint64_t> res_state_task_id_counter_{1};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto http_server_node = std::make_shared<HttpServerNode>();
    executor.add_node(http_server_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}