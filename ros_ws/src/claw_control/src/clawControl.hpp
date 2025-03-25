
#ifndef CLAW_CONTROL_HPP
#define CLAW_CONTROL_HPP

#include "modbus.h"
#include <mutex>
#include <vector>
#include <iostream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

class clawControl : public rclcpp::Node
{
public:
    clawControl(std::string device, int baud, char parity, int data_bit, int stop_bit, int slaveId);
    ~clawControl();

    void initSerial();

    int clawEnable(int slaveId, bool status);

    /*
     * @brief  runWithParameter  带参数控制
     * @param  slaveId    从站地址
     * @param  pos        目标位置 范围[0x00,0xFF]  与真实的位置关系成线性映射关系
     * @param  speed      速度 范围[0x00,0xFF]  与真实的速度关系成线性映射关系
     * @param  torque     力矩 范围[0x00,0xFF]  与真实的力矩关系成线性映射关系
     * @return 1 成功, -1 失败
     * */
    int runWithParameter(int slaveId, int pos, int speed, int torque);

    /*
     * @brief  readClawStatus  读取 Claw 状态
     * @param  slaveId    从站地址
     * @param  startAddr  起始地址
     * @param  count      读取寄存器数量
     * @return 1 成功, -1 失败
     * */
    int readClawStatus(int slaveId, uint16_t startAddr, int count);

protected:
    int safe_modbus_write_raw_req(modbus_t *ctx, uint8_t raw_req[], int raw_req_length);
    int safe_modbus_read_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest);

private:
    modbus_t *d_ctx;
    std::string device;
    int baud;
    char parity;
    int data_bit;
    int stop_bit;
    int slaveId;
    std::mutex d_mutex;
};

#endif
