#include "motor_control_v2/http_server.hpp"
#include "motor_control_v2/http_server.hpp"

namespace motor_control_v2
{

    HttpServerNode::HttpServerNode(const std::string &ip, int port)
        : Node("http_server_node_"), ip_(ip), port_(port)
    {
        
    }

    HttpServerNode::~HttpServerNode()
    {
       
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
        post_handlers_[path] = handler;
    }

    void HttpServerNode::registerGetHandler(const std::string &path, GetHandler handler)
    {
        get_handlers_[path] = handler;
    }
} // namespace motor_control_v2