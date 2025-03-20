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
              motor_command_ptr(std::make_shared<MotorCommand>(node_, serial_port, baud_rate)),
              http_server_node_ptr(std::make_shared<HttpServerNode>(ip, port)),
              request_queue_mutex_(),
              motor_state_mutex_(),
              port(port),
              ip(ip),
              serial_port(serial_port)
        {
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
        void handle_motor_task(const httplib::Request &req, httplib::Response &res);
        void handle_motor_status(const httplib::Request &req, httplib::Response &res);
        void handle_single_motor_status(int motor_id, httplib::Response &res);
        void handle_all_motors_status(httplib::Response &res);
        void initialize_motor_state();

    private:
        std::shared_ptr<MotorCommand> motor_command_ptr;
        std::shared_ptr<HttpServerNode> http_server_node_ptr;

        std::queue<json> request_queue_;
        std::mutex request_queue_mutex_;

        rclcpp::Node::SharedPtr node_;
        const std::string serial_port;
        int baud_rate;
        const std::string ip;
        int port;

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
        http_server_thread = std::thread(&InternalHttpServer::start_http_server, this);
        _pimpl->initialize_motor_state();
    }

    InternalHttpServer::~InternalHttpServer()
    {
        if (http_server_thread.joinable())
        {
            http_server_thread.join();
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

        {
            std::lock_guard<std::mutex> lock(request_queue_mutex_);
            request_queue_.push(json_data);
        }

        RCLCPP_INFO(node_->get_logger(), "Received POST request data: %s", json_data.dump().c_str());
        res.set_content("Received POST request data", "text/plain");
    }

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
        std::lock_guard<std::mutex> lock(motor_state_mutex_);
        nlohmann::json json_response = motor_state_;
        res.set_content(json_response.dump(), "application/json");
    }

    void InternalHttpServer::Implementation::initialize_motor_state()
    {
        motor_state_ = nlohmann::json::array();
        for (size_t i = 0; i < 5; ++i)
        {
            nlohmann::json motor = {
                {"motor_id", i},
                {"current_speed", 0},
                {"target_speed", 0},
                {"temperature", 25.0},
                {"error_code", 0}};
            motor_state_.push_back(motor);
        }
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

} // namespace motor_control_v2