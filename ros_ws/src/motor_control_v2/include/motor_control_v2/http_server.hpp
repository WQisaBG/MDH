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
        ~HttpServerNode();

        // 注册 POST 请求处理函数
        void registerPostHandler(const std::string &path, PostHandler handler);

        // 注册 GET 请求处理函数
        void registerGetHandler(const std::string &path, GetHandler handler);


    private:
        std::string ip_;
        int port_;

        // 路由表
        std::unordered_map<std::string, PostHandler> post_handlers_;
        std::unordered_map<std::string, GetHandler> get_handlers_;  
    };

}// namespace motor_control_v2

#endif // HTTP_SERVER_HPP