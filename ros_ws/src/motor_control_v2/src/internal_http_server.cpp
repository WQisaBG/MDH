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
              serial_port(serial_port.c_str()),
              baud_rate(baud_rate),
              ip(ip),
              port(port),
              motor_command_ptr(std::make_shared<MotorCommand>(node_, serial_port.c_str(), baud_rate)),
              http_server_node_ptr(std::make_shared<HttpServerNode>(ip, port))
        {
        }
        void start_http_server();
        void handle_motor_task(const httplib::Request &req, httplib::Response &res);
        void handle_motor_status(const httplib::Request &req, httplib::Response &res);
        void handle_single_motor_status(int motor_id, httplib::Response &res);
        void handle_all_motors_status(httplib::Response &res);

    private:
        std::shared_ptr<MotorCommand> motor_command_ptr;
        std::shared_ptr<HttpServerNode> http_server_node_ptr;

        rclcpp::Node::SharedPtr node_;
        const std::string ip;
        int port;
        const std::string serial_port;
        int baud_rate;
        nlohmann::json motor_state_;
        std::mutex request_mutex_, motor_state_mutex_;
    };

    InternalHttpServer::InternalHttpServer(rclcpp::Node::SharedPtr node,
                                           const std::string &serial_port,
                                           int baud_rate,
                                           const std::string &ip,
                                           int port)
        : _pimpl(rmf_utils::make_impl<Implementation>(Implementation{node, serial_port.c_str(), baud_rate, ip, port}))
    {
        // doing noting
        http_server_thread =std::thread(&InternalHttpServer::start_http_server,this);
    }

    void InternalHttpServer::Implementation::start_http_server()
    {

        http_server_node_ptr->registerPostHandler("/motor/task", [this](const httplib::Request &req, httplib::Response &res)
                                                  { handle_motor_task(req, res); });

        http_server_node_ptr->registerGetHandler("/motor/state", [this](const httplib::Request &req, httplib::Response &res)
                                                 { handle_motor_status(req, res); });
    }

    void InternalHttpServer::Implementation::handle_motor_task(const httplib::Request &req, httplib::Response &res)
    {
        json json_data = json::parse(req.body);
        motor_command_ptr->send_moving_command(json_data, motor_state_);
        res.set_content("Received POST request data", "text/plain");
    }

    void InternalHttpServer::Implementation::handle_motor_status(const httplib::Request &req, httplib::Response &res)
    {
        const auto &params = req.params;
        auto motor_id_it = params.find("motor_id");

        if (motor_id_it != params.end())
        {
            try
            {
                int motor_id = std::stoi(motor_id_it->second);
                if (motor_id <= 0 || motor_id > motor_state_.size())
                    throw std::out_of_range("motor_id out of range");

                handle_single_motor_status(motor_id - 1, res); // 转换为0-based索引
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
        std::lock_guard<std::mutex> lock(motor_state_mutex_); // 线程安全访问

        if (motor_id < 0 || motor_id >= motor_state_.size())
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

    

}
