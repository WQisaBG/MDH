#include "motor_control_v2/http_server.hpp"

namespace motor_control_v2
{

    HttpServerNode::HttpServerNode(const std::string &ip, int port)
        : Node("http_server_node"),
          ip_(ip),
          port_(port),
          stop_server_(false)
    {
        // 启动 HTTP 服务器线程
        http_server_thread_ = std::thread(&HttpServerNode::start_http_server, this);

        // 启动请求处理线程
        request_processing_thread_ = std::thread(&HttpServerNode::process_requests, this);
    }

    HttpServerNode::~HttpServerNode()
    {
        stop_server_ = true;
        request_cv_.notify_all(); // 唤醒所有等待的线程
        if (http_server_thread_.joinable())
            http_server_thread_.join();
        if (request_processing_thread_.joinable())
            request_processing_thread_.join();
    }

    void HttpServerNode::registerPostHandler(const std::string &path, PostHandler handler)
    {
        post_handlers_[path] = handler;
    }

    void HttpServerNode::registerGetHandler(const std::string &path, GetHandler handler)
    {
        get_handlers_[path] = handler;
    }

    void HttpServerNode::start_http_server()
    {
        httplib::Server server;

        // 注册 POST 请求处理函数
        server.Post("/motor/task", [this](const httplib::Request &req, httplib::Response &res)
                    { handle_post(req, res); });

        // 注册 GET 请求处理函数
        server.Get("/motor/state", [this](const httplib::Request &req, httplib::Response &res)
                   { handle_get(req, res); });

        // 启动服务器
        while (!stop_server_)
        {
            server.listen(ip_.c_str(), port_);
        }
    }

    void HttpServerNode::stop_http_server()
    {
        stop_server_ = true;
    }

    void HttpServerNode::handle_post(const httplib::Request &req, httplib::Response &res)
    {
        std::lock_guard<std::mutex> lock(request_mutex_);
        try
        {
            auto json_data = json::parse(req.body);
            RCLCPP_INFO(rclcpp::get_logger("http_server"), "Received JSON data: %s", json_data.dump().c_str());

            if (json_data.contains("id") && json_data.contains("timestamp") && json_data.contains("motor"))
            {
                request_queue_.push(std::move(json_data)); // 将获取的POST请求 放到队列中
                request_cv_.notify_one();
                res.set_content("{\"status\":\"received\"}", "application/json");
            }
            else
            {
                throw std::invalid_argument("Invalid JSON structure");
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(rclcpp::get_logger("http_server"), "Failed to parse JSON: %s", ex.what());
            res.set_content("{\"status\":\"error\",\"message\":\"Invalid JSON\"}", "application/json");
        }
    }

    void HttpServerNode::handle_get(const httplib::Request &req, httplib::Response &res)
    {
        json response_body;
    
        // 查找并调用对应的处理函数
        if (get_handlers_.find(req.path) != get_handlers_.end())
        {
            get_handlers_[req.path](req, res);
            res.set_content(response_body.dump(), "application/json");
        }
        else
        {
            response_body = {{"error", "Path not found"}};
            res.set_content(response_body.dump(), "application/json");
        }
    }

    void HttpServerNode::process_requests()
    {
        while (!stop_server_)
        {
            std::unique_lock<std::mutex> lock(request_mutex_);
            request_cv_.wait(lock, [this]
                             { return !request_queue_.empty() || stop_server_; });

            if (stop_server_)
                break;

            json request = request_queue_.front();
            request_queue_.pop();
            lock.unlock();

        }
    }

 

} // namespace motor_control_v2




/*

class HttpServerNode : public rclcpp::Node
{
public:
    HttpServerNode()
        : Node("http_server_node"),
          http_server_("127.0.0.1", 10088),
          stop_server_(false),
          serial_("/dev/ttyUSB0")
    {
        pub_ = this->create_publisher<motor_control_command_msgs::msg::MotorControlCommand>("motor_command", 10);
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("motor_status", 10);

        RCLCPP_INFO(this->get_logger(), "HTTP server starting on http://127.0.0.1:10088/motor/task");

        // 1、初始化电机状态
        initialize_motor_status();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        initSerialPort(); // 初始化串口

        // 2、注册 HTTP 请求处理函数
        http_server_.registerPostHandler("/motor/task", [this](const json &request, json &response) {
            // 处理 POST 请求逻辑
            response = {{"status", "received"}};
        });

        http_server_.registerGetHandler("/motor/state", [this](json &response) {
            // 返回电机状态
            response = get_motor_status();
        });

        // 3、启动线程
        feedback_thread_ = std::thread(&HttpServerNode::feedback_listener, this);
        motor_status_query_thread_ = std::thread(&HttpServerNode::heartbeat_motor_status_query_thread, this);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&HttpServerNode::publish_motor_status, this));

        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                                 std::bind(&HttpServerNode::control_timer_callback, this));
    }

    ~HttpServerNode()
    {
        stop_server_ = true;
        if (feedback_thread_.joinable())
            feedback_thread_.join();
        if (motor_status_query_thread_.joinable())
            motor_status_query_thread_.join();
    }

    json get_motor_status()
    {
        std::lock_guard<std::mutex> lock(motor_status_mutex_);
        return motor_status_;
    }

private:
    HttpServerWrapper http_server_;
    std::atomic<bool> stop_server_;
    // 其他成员变量和方法保持不变
};



*/