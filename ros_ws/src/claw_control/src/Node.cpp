#include "Node.hpp"
#include "nlohmann/json.hpp"

httpSeverNode::httpSeverNode(std::string device, int baud, int parity, int dataBits, int stopBits, int slaveId)
    : Node("claw_control_node"),
      slaveId(slaveId), baudrate(baud), parity(parity), dataBits(dataBits), stopBits(stopBits), device(device),
      stop_server_(false)
{
    // Initialize the serial port
    claw_control_ = std::make_shared<clawControl>(device, baud, parity, dataBits, stopBits, slaveId);
    claw_control_ ->clawEnable(slaveId, false);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    claw_control_->clawEnable(slaveId, true);
    http_server_thread_ = std::thread(&httpSeverNode::start_http_server, this); // http server thread

}

void httpSeverNode::start_http_server()
{
    httplib::Server server;

    server.Post("/claw/task", [this](const httplib::Request &req, httplib::Response &res)
                { handle_post(req, res); });

    server.Get("/claw/state", [this](const httplib::Request &req, httplib::Response &res)
               {
                    {
                        std::lock_guard<std::mutex> lock(status_update_mutex_);
                    }



                    // 等待状态更新完成
                    std::unique_lock<std::mutex> lock(status_update_mutex_);

                    nlohmann::json status = get_claw_status();
                    res.set_content(status.dump(), "application/json"); });

    while (!stop_server_)
    {
        server.listen("127.0.0.1", 10088);
    }
}

httpSeverNode::~httpSeverNode()
{
    stop_server_ = true;
    if(http_server_thread_.joinable())
    http_server_thread_.join();
}

void httpSeverNode::handle_post(const httplib::Request &req, httplib::Response &res)
{
    std::lock_guard<std::mutex> lock(request_mutex_);
    try
    {
        auto json_data = nlohmann::json::parse(req.body);
        RCLCPP_INFO(this->get_logger(), "Received JSON data: %s", json_data.dump().c_str());

        if (json_data.contains("id") && json_data.contains("timestamp") && json_data.contains("claw"))
        {
            process_requests(json_data);
            res.set_content("{\"status\":\"received\"}", "application/json");
        }
        else
        {
            throw std::invalid_argument("Invalid JSON structure");
        }
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", ex.what());
        res.set_content("{\"status\":\"error\",\"message\":\"Invalid JSON\"}", "application/json");
    }
}

void httpSeverNode::process_requests(nlohmann::json json_data)
{
    const auto &claws = json_data["claw"];
    for (const auto &claw : claws)
    {
        uint8_t slaveId = claw["slaveId"];
        uint16_t target_position = claw["targetPosition"];
        uint16_t target_speed = claw["targetSpeed"];
        uint16_t target_torque = claw["targetTorque"];

        claw_control_->runWithParameter(slaveId, target_position, target_speed, target_torque);
    }
    
}

nlohmann::json httpSeverNode::get_claw_status()
{
    nlohmann::json status;
    status["claw"] = claw_control_->readClawStatus(slaveId);
    return status;
}


