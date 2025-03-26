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

    // claw_control_node->clawEnable(9, true);

    // while (true)
    // {
    //     for (int i = 0; i < 100; i++)       
    //     {
    //         claw_control_node->runWithParameter(9, 255, 255, 255);
    //         std::this_thread::sleep_for(std::chrono::seconds(2));
    //         claw_control_node->readClawStatus(9,0x07D0,3);
    //         std::this_thread::sleep_for(std::chrono::seconds(2));
    //         claw_control_node->runWithParameter(9, 0, 255, 255);
    //         std::this_thread::sleep_for(std::chrono::seconds(2));
    //         claw_control_node->readClawStatus(9,0x07D0,3);
    //         std::this_thread::sleep_for(std::chrono::seconds(2));
    //     }
    // }

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
