
#ifndef CLAW_CONTROL_HPP
#define CLAW_CONTROL_HPP

#include "modbus.h"
#include <mutex>
#include <vector>
#include <iostream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

class clawControl
{
public:
    // 构造函数完成初始化化、断使能 间隔0.1s 再上使能（使能后为带参数控制模式）
    clawControl(std::string device, int baud, char parity, int data_bit, int stop_bit, int slaveId);
    ~clawControl();

    void initSerial();

    /*
     * @brief  clawEnable  使能/断使能
     * @param  slaveId    从站地址
     * @param  status      true:使能 false:断使能
     * @return 1 操作成功, -1 操作失败
     */
    int clawEnable(int slaveId, bool status); 

    /*
     * @brief  runWithParameter  带参数控制   
     * @param  slaveId    从站地址
     * @param  pos        目标位置 范围[0x00,0xFF]  
     * @param  speed      速度 范围[0x00,0xFF]    
     * @param  torque     力矩 范围[0x00,0xFF]
     * @return 1 操作成功, -1 操作失败
     * */
    int runWithParameter(int slaveId, int pos, int speed, int torque);

    /*
     * @brief  getClawCurrentPosition  读取 Claw 的位置状态
     * @param  slaveId    从站地址
     * @return 读取失败：-1  读取成功：返回当前位置值
     * */
    int getClawCurrentPosition(int slaveId);

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
