#include "motor_control_v2/Node.hpp"

namespace motor_control_v2
{

    MotorControlApp::MotorControlApp(const std::string &config_file_path)
        : Node("motor_control_node"),
          stop_server_(false)
    {

        // 加载配置文件
        std::ifstream config_file(config_file_path);
        if (!config_file.is_open())
        {
            throw std::runtime_error("Failed to open config file: " + config_file_path);
        }

        post_request_processing_thread_ = std::thread(&MotorControlApp::process_post_request, this);
        query_motor_state_thread_ = std::thread(&MotorControlApp::query_motor_state, this);
        get_motor_state_thread_ = std::thread(&MotorControlApp::get_motor_state, this);

    }
    MotorControlApp::~MotorControlApp()
    {
        stop_server_ = true;
        if (post_request_processing_thread_.joinable())
        {
            post_request_processing_thread_.detach();
        }
        if (query_motor_state_thread_.joinable())
        {
            query_motor_state_thread_.detach();
        }
        if (get_motor_state_thread_.joinable())
        {
            get_motor_state_thread_.detach();
        }
    }

    void MotorControlApp::initialize()
    {
        motor_config_ = std::make_shared<MotorConfig>(this->shared_from_this(), "config.json");
        motor_count_ = motor_config_->get_motor_count();
        initialize_http_server();
        initialize_components();
    }

    void MotorControlApp::initialize_http_server()
    {
        http_server_ = std::make_shared<HttpServerNode>();
        http_server_->registerPostDataCallback([this](const nlohmann::json &json_data)
                                               { request_queue_.push(json_data);
                                                request_cv_.notify_one();

                                                std::cout << "Received POST request data: " << json_data.dump() << std::endl; });

        http_server_->registerGetHandler("/motor/state", [this](const httplib::Request &, httplib::Response &res)
                                         {
                                                auto motor_status = motor_state_;
                                                nlohmann::json json_status;
                                                json_status["motor_status"] = motor_status;
                                                res.set_content(json_status.dump(), "application/json"); });
    }

    void MotorControlApp::initialize_components()
    {
        // 获取 shared_ptr 形式的 this 指针
        auto node_shared_ptr = this->shared_from_this();
        serial_comm_ = std::make_shared<SerialCommunication>(node_shared_ptr, motor_config_->get_serial_port().c_str(), motor_config_->get_baud_rate());
        ros2_comm_ = std::make_shared<ROS2Communication>(node_shared_ptr);
        motor_command_ = std::make_shared<MotorCommand>(node_shared_ptr, motor_config_->get_serial_port().c_str(), motor_config_->get_baud_rate());
    }

    void MotorControlApp::initialize_motor_state(int motor_count)
    {
        motor_state_ = nlohmann::json::array(); // 初始化为 JSON 数组
        for (int i = 1; i < motor_count + 1; ++i)
        {
            nlohmann::json motor = {
                {"motor_id", i + 1},
                {"current_speed", 0},
                {"target_speed", 0},
                {"temperature", 25.0},
                {"error_code", 0}};
            motor_state_.push_back(motor);
        }
    }
    void MotorControlApp::process_post_request()
    {
        while (!stop_server_)
        {
            std::unique_lock<std::mutex> lock(request_mutex_);
            request_cv_.wait(lock, [this]
                             { return !request_queue_.empty() || stop_server_; });
            if (stop_server_)
                break;

            auto json_data = request_queue_.front(); // 从队列中取出一个请求
            request_queue_.pop();                    // 从队列中移除该请求
            lock.unlock();
            std::lock_guard<std::mutex> lock2(current_request_mutex_);
            current_request_ = json_data; // 将当前请求赋值给current_request_
            if (!current_request_.empty())
            {
                motor_command_->send_moving_command(current_request_, motor_state_);
            }
        }
    }

    void MotorControlApp::query_motor_state()
    {
        while (!stop_server_)
        {
            for (int i = 0; i < motor_state_.size() + 1; i++)
            {
                motor_command_->send_query_status_command(i);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

        }
    }

    void MotorControlApp::get_motor_state()
    {
        std::vector<unsigned char> result = motor_command_ ->get_feedback_from_motor(1);
        update_motor_state(result);
    }

    void MotorControlApp::update_motor_state(std::vector<unsigned char> feedback)
    {
       

    }

} // namespace motor_control_v2