// motor_control_app.hpp
#ifndef NODE_HPP
#define NODE_HPP

#include "motor_control_v2/http_server.hpp"
#include "motor_control_v2/motor_config.hpp"
#include "motor_control_v2/motor_command.hpp"
#include "motor_control_v2/serial_communication.hpp"
#include "motor_control_v2/ros2_communication.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

namespace motor_control_v2
{

    class MotorControlApp : public rclcpp::Node
    {
    public:
        MotorControlApp(const std::string &config_file_path);
        ~MotorControlApp();
        void initialize();

    private:
        std::mutex request_mutex_, current_request_mutex_;

        json current_request_;
        
        std::atomic<bool> stop_server_;
        nlohmann::json motor_state_;
        std::queue<json> request_queue_;
        std::condition_variable request_cv_;

        std::thread post_request_processing_thread_;
        std::thread query_motor_state_thread_;
        std::thread get_motor_state_thread_;


        int motor_count_;



        void process_post_request();
        void query_motor_state();
        void get_motor_state();
        void update_motor_state(std::vector<unsigned char> feedback);


        std::shared_ptr<HttpServerNode> http_server_;
        std::shared_ptr<MotorConfig> motor_config_;
        std::shared_ptr<SerialCommunication> serial_comm_;
        std::shared_ptr<ROS2Communication> ros2_comm_;
        std::shared_ptr<MotorCommand> motor_command_;

        void initialize_http_server();
        void initialize_components();
        void initialize_motor_state(int motor_count);
    };

} // namespace motor_control_v2

#endif // NODE_HPP