#ifndef HTTP_SERVER_HPP
#define HTTP_SERVER_HPP

#include "httplib.h"
#include <rclcpp/rclcpp.hpp>
#include "nlohmann/json.hpp"
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>
#include <thread>
#include "motor_control_v2/motor_config.hpp"

using json = nlohmann::json;

namespace motor_control_v2
{

    class HttpServerNode : public rclcpp::Node
    {
    public:
        using PostHandler = std::function<void(const httplib::Request &, httplib::Response &)>;
        using GetHandler = std::function<void(const httplib::Request &, httplib::Response &)>;
        using PostDataCallback = std::function<void(const json &)>;
        // 构造函数，允许配置 IP 地址和端口号
        HttpServerNode(const std::string &ip = "127.0.0.1", int port = 8080);

        // 析构函数，确保线程安全退出
        ~HttpServerNode();

        // 注册 POST 请求处理函数
        void registerPostHandler(const std::string &path, PostHandler handler);

        // 注册 GET 请求处理函数
        void registerGetHandler(const std::string &path, GetHandler handler);
        void registerPostDataCallback(PostDataCallback callback); // 新增：注册回调函数
        // 启动 HTTP 服务器
        void start_http_server();

        void stop_http_server();

    private:
        // 处理 POST 请求
        void handle_post(const httplib::Request &req, httplib::Response &res);

        // 处理 GET 请求
        void handle_get(const httplib::Request &req, httplib::Response &res);

        // 处理请求队列
        void process_requests();


        // 服务器配置
        std::string ip_;
        int port_;
        std::atomic<bool> stop_server_;

      

        // 线程控制
        std::thread http_server_thread_;
        std::thread request_processing_thread_;

        // 请求队列
        std::queue<json> request_queue_;
        std::mutex request_mutex_;
        std::condition_variable request_cv_;

        // 路由表
        std::unordered_map<std::string, PostHandler> post_handlers_;
        std::unordered_map<std::string, GetHandler> get_handlers_;  

        PostDataCallback post_data_callback_;   // 回调函数
    };

}// namespace motor_control_v2

#endif // HTTP_SERVER_HPP