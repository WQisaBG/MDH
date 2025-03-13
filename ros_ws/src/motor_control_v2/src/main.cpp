// #include "motor_control_v2/http_server.hpp"
// #include "motor_control_v2/motor_config.hpp"
// #include "motor_control_v2/motor_command.hpp"
// #include "motor_control_v2/serial_communication.hpp"
// #include "motor_control_v2/ros2_communication.hpp"
// #include <rclcpp/rclcpp.hpp>

// int main(int argc, char *argv[])
// {


//     if (argc < 2)
//     {
//         std::cerr << "Usage: " << argv[0] << " <config_file_path>" << std::endl;
//         return 1;
//     }
//     std::string config_file_path = argv[1];
//     // 加载配置文件
//     std::ifstream config_file(config_file_path);
//     if (!config_file.is_open())
//     {
//         throw std::runtime_error("Failed to open config file: " + config_file_path);
//     }

//     nlohmann::json motor_state_ = {
//         {"motor_id", 1},
//         {"current_speed", 0},
//         {"target_speed", 0},
//         {"temperature", 25.0},
//         {"error_code", 0}};



//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("motor_control_node");

//     // 加载配置文件
//     MotorConfig motor_config(node, "config.json");

//     motor_control_v2::HttpServerNode http_server;

//     http_server.registerPostDataCallback([](const nlohmann::json &json_data)
//                                          {
//         // 打印接收到的 POST 请求信息
//         std::cout << "Received POST request data: " << json_data.dump() << std::endl; });


//     // 注册GET请求处理函数
//     http_server.registerGetHandler("/motor/state", [&motor_state_](const httplib::Request &, httplib::Response &res)
//     {
//         auto motor_status = motor_state_;
//         nlohmann::json json_status;
//         json_status["motor_status"] = motor_status;
//         res.set_content(json_status.dump(), "application/json");
//     });

//     // 初始化其他组件
//     motor_control_v2::SerialCommunication serial_comm(node, motor_config.get_serial_port().c_str(), motor_config.get_baud_rate());
//     motor_control_v2::ROS2Communication ros2_comm(node);
//     motor_control_v2::MotorCommand motor_command(node, motor_config.get_serial_port().c_str(), motor_config.get_baud_rate());

//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.add_node(http_server.get_node_base_interface());
//     executor.spin();
//     rclcpp::shutdown();
//     return 0;
// }



// main.cpp
#include "motor_control_v2/Node.hpp"

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <config_file_path>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    // 创建并初始化MotorControlApp
    auto motor_control_app = std::make_shared<motor_control_v2::MotorControlApp>(argv[1]);
    motor_control_app->initialize();

    // 添加节点到执行器
    executor.add_node(motor_control_app);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}