#include "motor_control_v2/http_server.hpp"

namespace motor_control_v2
{

    HttpServerNode::HttpServerNode(const std::string &ip, int port)
        : Node("http_server_node"), ip_(ip), port_(port), stop_server_(false)
    {
        http_server_thread_ = std::thread(&HttpServerNode::start_http_server, this);
    }

    HttpServerNode::~HttpServerNode()
    {
        stop_server_ = true;
        request_cv_.notify_all();
        if (http_server_thread_.joinable())
            http_server_thread_.join();
    }

    void HttpServerNode::start_http_server()
    {
        httplib::Server server;

        server.Post("/motor/task", [this](const httplib::Request &req, httplib::Response &res)
                    { handle_post(req, res); });

        server.Get("/motor/state", [this](const httplib::Request &req, httplib::Response &res)
                   { handle_get(req, res); });

        try
        {
            RCLCPP_INFO(rclcpp::get_logger("http_server"), "Starting HTTP server on %s:%d", ip_.c_str(), port_);
            while (!stop_server_)
            {
                if (!server.listen(ip_.c_str(), port_))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("http_server"), "Failed to start HTTP server on %s:%d", ip_.c_str(), port_);
                    break;
                }
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(rclcpp::get_logger("http_server"), "HTTP server error: %s", ex.what());
        }

        RCLCPP_INFO(rclcpp::get_logger("http_server"), "HTTP server stopped");
    }

    void HttpServerNode::registerPostDataCallback(PostDataCallback callback)
    {
        post_data_callback_ = callback; // 注册回调函数
    }

    void HttpServerNode::handle_post(const httplib::Request &req, httplib::Response &res)
    {
        std::lock_guard<std::mutex> lock(request_mutex_);
        try
        {
            auto json_data = json::parse(req.body);
            RCLCPP_INFO(rclcpp::get_logger("http_server"), "Received JSON data: %s", json_data.dump().c_str());

            if (!json_data.contains("id") || !json_data.contains("timestamp") || !json_data.contains("motor"))
            {
                throw std::invalid_argument("JSON missing required fields: id, timestamp, or motor");
            }
            // 调用回调函数处理 POST 数据
            if (post_data_callback_)
            {
                post_data_callback_(json_data);
            }

            request_queue_.push(std::move(json_data));
            request_cv_.notify_one();
            res.set_content("{\"status\":\"received\"}", "application/json");
        }
        catch (const json::parse_error &ex)
        {
            RCLCPP_ERROR(rclcpp::get_logger("http_server"), "JSON parse error: %s", ex.what());
            res.set_content("{\"status\":\"error\",\"message\":\"Invalid JSON format\"}", "application/json");
        }
        catch (const std::invalid_argument &ex)
        {
            RCLCPP_ERROR(rclcpp::get_logger("http_server"), "Invalid JSON structure: %s", ex.what());
            res.set_content("{\"status\":\"error\",\"message\":\"Invalid JSON structure\"}", "application/json");
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(rclcpp::get_logger("http_server"), "Unexpected error: %s", ex.what());
            res.set_content("{\"status\":\"error\",\"message\":\"Internal server error\"}", "application/json");
        }
    }

    void HttpServerNode::handle_get(const httplib::Request &req, httplib::Response &res)
    {
        json response_body;

        if (get_handlers_.find(req.path) != get_handlers_.end())
        {
            get_handlers_[req.path](req, res);
            res.set_content(response_body.dump(), "application/json");
        }
        else
        {
            response_body = {{"error", "Path not found"}};
            res.status = 404;
            res.set_content(response_body.dump(), "application/json");
        }
    }

} // namespace motor_control_v2