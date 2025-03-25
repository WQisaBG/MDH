
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

    // int *getStatus(int slaveId, int address, int readMode, int count);
    // int runWithoutParameter(int slaveId, int cmdId);
    int runWithParameter(int slaveId, int pos, int speed, int torque);
    int readClawStatus(int slaveId, uint16_t startAddr, int count);

    

protected:
    int safe_modbus_write_raw_req(modbus_t *ctx, uint8_t raw_req[], int raw_req_length);
    int safe_modbus_read_bits(modbus_t *ctx, int addr, int nb, uint8_t *dest);


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
