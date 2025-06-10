#include "Node.hpp"
#include "thread"
#include <chrono>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto claw_control_node = std::make_shared<httpSeverNode>(
        "/dev/ttyUSB0", // std::string device
        115200,         // int baud
        'N',            // char parity（例如：'N'表示无校验）
        8,              // int data_bit
        1,              // int stop_bit
        9               // int slaveId
    );
    executor.add_node(claw_control_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
