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
        request_cv_.notify_all();
        if (post_request_processing_thread_.joinable()) post_request_processing_thread_.join();
        if (query_motor_state_thread_.joinable()) query_motor_state_thread_.join();
        if (get_motor_state_thread_.joinable()) get_motor_state_thread_.join();
    }

    void MotorControlApp::initialize()
    {
        motor_config_ = std::make_shared<MotorConfig>(this->shared_from_this(), "config.json");
        motor_count_ = motor_config_->get_motor_count();
        initialize_components();
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



   

    
    void MotorControlApp::query_motor_state()
    {
        while (!stop_server_)
        {
            for (int i = 0; i < motor_state_.size() ; i++)
            {
                motor_command_->send_query_status_command(i+1);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
    }

    void MotorControlApp::get_motor_state()
    {
        std::vector<unsigned char> result = motor_command_->get_feedback_from_motor(1);
        update_motor_state(result);
    }

    void MotorControlApp::update_motor_state(std::vector<unsigned char> feedback)
    {
    }

} // namespace motor_control_v2