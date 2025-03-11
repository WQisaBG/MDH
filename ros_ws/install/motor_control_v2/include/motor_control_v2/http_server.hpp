#ifndef HTTP_SERVER_HPP
#define HTTP_SERVER_HPP

#include "httplib.h"
#include <rclcpp/rclcpp.hpp>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

class HttpServerNode : public rclcpp::Node
{
public:
    HttpServerNode();
    ~HttpServerNode();

    json get_motor_status();

private:
    void start_http_server();
    void handle_post(const httplib::Request &req, httplib::Response &res);

    std::atomic<bool> stop_server_;
    std::thread http_server_thread_;
};

#endif // HTTP_SERVER_HPP