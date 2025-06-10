#ifndef NODE_HPP
#define NODE_HPP

#include "clawControl.hpp"

#include "httplib.h"
#include "nlohmann/json.hpp"

class httpSeverNode : public rclcpp::Node
{
public:
    httpSeverNode(std::string device, int baud, int parity, int dataBits, int stopBits, int slaveId);
    ~httpSeverNode();

    void start_http_server();
    void handle_post(const httplib::Request &req, httplib::Response &res);
    void process_requests(nlohmann::json json_data);
    nlohmann::json get_claw_status();

private:
    std::atomic<bool> stop_server_;

    std::shared_ptr<clawControl> claw_control_;

    int slaveId;
    std::string device;
    int baudrate;
    int parity;
    int dataBits;
    int stopBits;


    std::mutex status_update_mutex_, request_mutex_;
    std::thread http_server_thread_;
};

#endif