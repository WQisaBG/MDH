#include "motor_control_v2/http_server.hpp"
#include "motor_control_v2/http_server.hpp"

namespace motor_control_v2
{
    HttpServerNode::HttpServerNode(const std::string &ip, int port)
        : Node("http_server_node_"), ip_(ip), port_(port), server_()
    {
        //
    }

    HttpServerNode::~HttpServerNode()
    {
    }

    bool HttpServerNode::start(const std::string &ip, int port)
    {
        // 启动HTTP服务器
        server_.Get("/", [](const httplib::Request &, httplib::Response &res)
                    { res.set_content("Hello World!", "text/plain"); });

        // 监听指定IP和端口
        return server_.listen(ip.c_str(), port);
    }

    /**
       * @brief 注册POST请求处理函数
       *
       * @param path 请求路径
       * @param handler 处理函数
       *
       * path：字符串类型，表示 POST 请求的路径（例如 /motor/task）。
        handler：PostHandler 类型，是一个函数对象，用于处理指定路径的 POST 请求。
       */
    void HttpServerNode::registerPostHandler(const std::string &path, PostHandler handler)
    {
       server_.Post(path, handler);
    }
    void HttpServerNode::registerGetHandler(const std::string &path, GetHandler handler)
    {
        server_.Get(path, handler);
    }
} // namespace motor_control_v2