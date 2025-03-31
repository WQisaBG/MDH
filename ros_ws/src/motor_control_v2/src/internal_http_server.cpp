#include "motor_control_v2/internal_http_server.hpp"

namespace motor_control_v2
{
    class InternalHttpServer::Implementation
    {
    public:     
        Implementation(rclcpp::Node::SharedPtr node,
                       const std::string &serial_port,
                       int baud_rate,
                       const std::string &ip,
                       int port)
            : node_(node),
              request_queue_mutex_(),
              motor_state_mutex_(),
              port(port),
              ip(ip),
              serial_port(serial_port),
              stop_server_(false)
        {
            motor_command_ptr = std::make_shared<MotorCommand>(node_, serial_port, baud_rate);
            http_server_node_ptr = std::make_shared<HttpServerNode>(node_ ,ip, port);
        }
        // 禁用拷贝构造函数
        Implementation(const Implementation &) = delete;

        // 允许移动构造函数
        Implementation(Implementation &&) = default;

        // 禁用赋值操作符
        Implementation &operator=(const Implementation &) = delete;

        // 允许移动赋值操作符
        Implementation &operator=(Implementation &&) = default;

        ~Implementation() = default;

        void start_http_server();
        void deal_post_request();

        void handle_motor_task(const httplib::Request &req, httplib::Response &res);
        void handle_motor_status(const httplib::Request &req, httplib::Response &res);
        void handle_single_motor_status(int motor_id, httplib::Response &res);
        void handle_all_motors_status(httplib::Response &res);
        void initialize_motor_state();
        void update_motor_status();

        void set_stop_server(bool value) { stop_server_ = value; }

        void update_motor_state(std::vector<unsigned char> &buffer);

    private:
        std::shared_ptr<MotorCommand> motor_command_ptr;
        std::shared_ptr<HttpServerNode> http_server_node_ptr;

        std::queue<json> request_queue_;
        std::mutex request_queue_mutex_;
        std::condition_variable request_cv_;

        rclcpp::Node::SharedPtr node_;
        const std::string serial_port;
        int baud_rate;
        const std::string ip;
        int port;
        bool stop_server_;

        nlohmann::json motor_state_;
        std::mutex motor_state_mutex_;
    };

    InternalHttpServer::InternalHttpServer(rclcpp::Node::SharedPtr node,
                                           const std::string &serial_port,
                                           int baud_rate,
                                           const std::string &ip,
                                           int port)
        : _pimpl(std::make_unique<Implementation>(
              node, serial_port, baud_rate, ip, port))
    {
        if (!node)
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid node pointer in InternalHttpServer");
            throw std::invalid_argument("Invalid node pointer in InternalHttpServer");
        }
        RCLCPP_INFO(node->get_logger(), "Node pointer is valid in InternalHttpServer");


        http_server_thread = std::thread(&InternalHttpServer::start_http_server, this);
        deal_post_request_thread = std::thread(&InternalHttpServer::deal_post_request, this);
        update_motor_status_thread = std::thread(&InternalHttpServer::update_motor_status, this);
        _pimpl->initialize_motor_state();
    }
    InternalHttpServer::~InternalHttpServer()
    {
        _pimpl->set_stop_server(true);
        if (http_server_thread.joinable())
        {
            http_server_thread.join();
        }
        if (deal_post_request_thread.joinable())
        {
            deal_post_request_thread.join();
        }
        if (update_motor_status_thread.joinable())
        {
            update_motor_status_thread.join();
        }
    }

    void InternalHttpServer::Implementation::start_http_server()
    {
        http_server_node_ptr->registerPostHandler("/motor/task", [this](const httplib::Request &req, httplib::Response &res)
                                                  { handle_motor_task(req, res); });

        http_server_node_ptr->registerGetHandler("/motor/state", [this](const httplib::Request &req, httplib::Response &res)
                                                 { handle_motor_status(req, res); });

        // 调用HttpServerNode的start方法启动服务
        if (!http_server_node_ptr->start(ip, port))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to start HTTP server on %s:%d", ip.c_str(), port);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "HTTP server started successfully on %s:%d", ip.c_str(), port);
        }
    }

    void InternalHttpServer::Implementation::handle_motor_task(const httplib::Request &req, httplib::Response &res)
    {
        json json_data = json::parse(req.body);
        RCLCPP_INFO(node_->get_logger(), "Received POST request data: %s", json_data.dump().c_str());
        try
        {
            if (json_data.contains("id") && json_data.contains("timestamp") && json_data.contains("motor"))
            {
                {
                    std::lock_guard<std::mutex> lock(request_queue_mutex_);
                    request_queue_.push(json_data);
                }
                request_cv_.notify_one();
                res.set_content("Received POST request data", "text/plain");
            }
            else
            {
                throw std::invalid_argument("Invalid JSON data");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error handling motor task: %s", e.what());
            res.set_content("{\"status\":\"error\",\"message\":\"Invalid JSON\"}", "application/json");
        }
    }

    void InternalHttpServer::Implementation::deal_post_request()
    {
        while (!stop_server_)
        {
            std::unique_lock<std::mutex> lock(request_queue_mutex_);
            request_cv_.wait(lock, [this]()
                             { return !request_queue_.empty() || stop_server_; });
            if (stop_server_)
                break;
            if (!request_queue_.empty())
            {
                json json_data = request_queue_.front();
                request_queue_.pop();
                lock.unlock();
                motor_command_ptr ->send_moving_command(json_data, motor_state_, 10, 20);
            }
        }
    }



    // void Implementation::send_request_topic(json json_data)
    // {
    //     motor_control_command_msgs::msg::MotorControlCommand msg;
    //     msg.id = json_data["id"];
    //     msg.timestamp = json_data["timestamp"];

    //     const auto &motors = json_data["motor"];
    //     for (const auto &motor : motors)
    //     {
    //         motor_control_command_msgs::msg::Motor motor_msg;
    //         motor_msg.index = motor["index"];
    //         motor_msg.target_position = motor["targetPosition"];
    //         msg.motors.push_back(motor_msg);
    //     }

    //     pub_->publish(msg);
    // }

    void InternalHttpServer::Implementation::handle_motor_status(const httplib::Request &req, httplib::Response &res)
    {
        const auto &params = req.params;
        auto motor_id_it = params.find("motor_id");

        if (motor_state_.empty())
        {
            res.status = 400;
            res.set_content("No motor states available", "text/plain");
            return;
        }

        if (motor_id_it != params.end())
        {
            try
            {
                int motor_id = std::stoi(motor_id_it->second);
                if (motor_id <= 0 || motor_id > static_cast<int>(motor_state_.size()))
                    throw std::out_of_range("motor_id out of range");

                handle_single_motor_status(motor_id - 1, res);
            }
            catch (const std::exception &e)
            {
                res.status = 400;
                res.set_content(e.what(), "text/plain");
            }
        }
        else
        {
            handle_all_motors_status(res);
        }
    }

    void InternalHttpServer::Implementation::handle_single_motor_status(int motor_id, httplib::Response &res)
    {
        std::lock_guard<std::mutex> lock(motor_state_mutex_);

        if (motor_id < 0 || motor_id >= static_cast<int>(motor_state_.size()))
        {
            res.status = 400;
            res.set_content("Invalid motor_id", "text/plain");
            return;
        }

        nlohmann::json json_response = motor_state_[motor_id];
        res.set_content(json_response.dump(), "application/json");
    }

    void InternalHttpServer::Implementation::handle_all_motors_status(httplib::Response &res)
    {
        for (uint8_t motor_index = 1; motor_index <= 5; motor_index++)
        {
            motor_command_ptr->send_query_status_command(motor_index);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        // 等待状态更新
        std::lock_guard<std::mutex> lock(motor_state_mutex_);
        nlohmann::json json_response = motor_state_;
        res.set_content(json_response.dump(), "application/json");
    }

    void InternalHttpServer::Implementation::initialize_motor_state()
    {
        motor_state_["motor"] = json::array(); // 确保 "motor" 是一个数组

        for (int i = 0; i < 5; ++i)
        {
            motor_command_ptr->send_query_status_command(i + 1);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void InternalHttpServer::Implementation::update_motor_status()
    {
        RCLCPP_INFO(node_->get_logger(), "Entering update_motor_status...");
        while (!stop_server_)
        {
            auto feedback = motor_command_ptr->get_feedback_from_motor(1);
            if (!feedback.empty())
            {
                update_motor_state(feedback);
            }

        }
        RCLCPP_INFO(node_->get_logger(), "Exiting update_motor_status...");
    }
    void InternalHttpServer::Implementation::update_motor_state(std::vector<unsigned char> &feedback)
    {
        if (feedback.size() < 8)
        {
            RCLCPP_ERROR(node_->get_logger(), "Invalid feedback data size: %zu", feedback.size());
            return;
        }
        std::ostringstream ss;
        for (int i = 0; i < feedback.size(); ++i)
        {
            ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(feedback[i]) << " ";
        }
        RCLCPP_INFO(node_->get_logger(), "Received feedback: %s", ss.str().c_str());

        int motor_index = feedback[3];
        int current_position = (feedback[10] << 8) | feedback[9];
        int target_position = (feedback[8] << 8) | feedback[7];

        std::lock_guard<std::mutex> lock(motor_state_mutex_);

        auto &motors = motor_state_["motor"]; // 直接操作数组
        bool found = false;

        for (auto &motor : motors)
        {
            if (motor["motor_id"] == motor_index)
            { // 使用正确的键名
                motor["currentPosition"] = current_position;
                motor["targetPosition"] = target_position;
                motor["error"] = "No error";
                motor["mode"] = "normal";
                found = true;
                break;
            }
        }

        if (!found)
        {
            motors.push_back({{"motor_id", motor_index},
                              {"currentPosition", current_position},
                              {"targetPosition", target_position},
                              {"error", "No error"},
                              {"mode", "normal"}});
        }

        RCLCPP_INFO(node_->get_logger(), "Updated motor %d status: current=%d, target=%d", motor_index, current_position, target_position);
    }

    void InternalHttpServer::start_http_server()
    {
        _pimpl->start_http_server();
    }

    void InternalHttpServer::handle_motor_task(const httplib::Request &req, httplib::Response &res)
    {
        _pimpl->handle_motor_task(req, res);
    }

    void InternalHttpServer::handle_motor_status(const httplib::Request &req, httplib::Response &res)
    {
        _pimpl->handle_motor_status(req, res);
    }
    void InternalHttpServer::deal_post_request()
    {
        _pimpl->deal_post_request();
    }

    void InternalHttpServer::update_motor_status()
    {
        _pimpl->update_motor_status();
    }

} // namespace motor_control_v2