

// namespace motor_control_v2
// {
//     class InternalHttpServerNode : public HttpServerNode
//     {
//     public:
//         InternalHttpServerNode(const std::string &node_name,
//                                const std::string &ip = "127.0.0.1", int port = 8080,
//                                const std::string &serial_port, int baud_rate);

//

//     private:
//         MotorCommand motor_command_;
//     };

// }

#ifndef MOTOR_CONTROL_V2_INTERNAL_HTTP_SERVER_HPP_
#define MOTOR_CONTROL_V2_INTERNAL_HTTP_SERVER_HPP_

#include <string>
#include "motor_control_v2/http_server.hpp"
#include "motor_control_v2/motor_command.hpp"
#include "rmf_utils/impl_ptr.hpp"

namespace motor_control_v2
{

    class InternalHttpServer
    {
    public:
        InternalHttpServer(rclcpp::Node::SharedPtr node,
                           const std::string &serial_port, int baud_rate,
                           const std::string &ip = "127.0.0.1", int port = 8080);

        ~InternalHttpServer();
            //初始化http
        void start_http_server();
            //POST请求
        void handle_motor_task(const httplib::Request &req, httplib::Response &res);

            //GET请求
        void handle_motor_status(const httplib::Request &req, httplib::Response &res);

        void deal_post_request();

        void update_motor_status();




        class Implementation;

        private:


        std::thread http_server_thread;
        std::thread deal_post_request_thread;
        std::thread update_motor_status_thread;



        rmf_utils::impl_ptr<Implementation> _pimpl;

    };

} // namespace motor_control_v2

#endif // MOTOR_CONTROL_V2_INTERNAL_HTTP_SERVER_HPP_