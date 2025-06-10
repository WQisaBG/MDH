#include "clawControl.hpp"

clawControl::clawControl(std::string device, int baud, char parity, int data_bit, int stop_bit, int slaveId)
    : device(device), baud(baud), parity(parity), data_bit(data_bit), stop_bit(stop_bit), slaveId(slaveId)

{
    initSerial();
    clawEnable(slaveId, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    clawEnable(slaveId, true);
}

clawControl::~clawControl()
{
    if (d_ctx)
    {
        modbus_close(d_ctx);
        modbus_free(d_ctx);
    }
}

void clawControl::initSerial()
{
    try
    {
        // 波特率115200，数据位8位，无校验，停止位1位
        d_ctx = modbus_new_rtu(device.c_str(), baud, parity, data_bit, stop_bit);
        if (d_ctx == NULL)
        {
            RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "initSerial error: %s", modbus_strerror(errno));
            exit(EXIT_FAILURE); // 程序直接终止
        }
        if (modbus_connect(d_ctx) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "initSerial error: %s", modbus_strerror(errno));
            modbus_free(d_ctx);
            exit(EXIT_FAILURE);
        }
        // modbus_set_slave(d_ctx, slaveId);    //这里 这样的话，多个从站时还可以这样吗？

        RCLCPP_INFO(rclcpp::get_logger("clawControl"), "initSerial success,:device: %s, baud: %d, parity: %c, data_bit: %d, stop_bit: %d", device.c_str(), baud, parity, data_bit, stop_bit);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "initSerial error: %s", e.what());
    }
}

int clawControl::clawEnable(int slaveId, bool status)
{
    int ret = 0;
    int able = status ? 1 : 0;
    try
    {
        modbus_set_slave(d_ctx, slaveId);

        uint8_t raw_req[9] = {
            // 总长度为 9 字节（不含 CRC，由库自动计算）
            static_cast<uint8_t>(slaveId),   // 从站地址
            0x10,                            // 功能码 0x10（写入多个寄存器）
            0x03, 0xE8,                      // 起始地址 0x03E8（十进制 1000）
            0x00, 0x01,                      // 寄存器数量 1（0x0001）
            0x02,                            // 数据字节数（2 字节 = 1 个寄存器）
            0x00, static_cast<uint8_t>(able) // 数据值（0 或 1）
        };
        int raw_req_length = sizeof(raw_req) / sizeof(uint8_t);
        ret = safe_modbus_write_raw_req(d_ctx, raw_req, raw_req_length);

        if (ret == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "clawEnable error: %s", modbus_strerror(errno));
            return -1;
        }
        if (ret == raw_req_length + 2)
        {
            RCLCPP_INFO(rclcpp::get_logger("clawControl"), "clawEnable success: %s", status ? "enable" : "disable");
            return 1;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "clawEnable error: %s", e.what());
    }
}

int clawControl::runWithParameter(int slaveId, int pos, int speed, int torque)
{
    int ret = 0;
    try
    {
        // 参数范围
        if (pos < 0 || pos > 255 ||
            speed < 0 || speed > 255 ||
            torque < 0 || torque > 255)
        {
            RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "runWithParameter error: 参数范围错误");
            return -1;
        }

        modbus_set_slave(d_ctx, slaveId);
        uint8_t raw_req[13] = {
            // 总长度为 13 字节（不含 CRC）
            static_cast<uint8_t>(slaveId),                            // 从站地址
            0x10,                                                     // 功能码 0x10（写入多个寄存器）
            0x03, 0xE8,                                               // 起始地址 0x03E8（十进制1000）
            0x00, 0x03,                                               // 寄存器数量 3（0x0003）
            0x06,                                                     // 数据字节数（6 = 2 字节 * 3 个寄存器）
            0x00, 0x09,                                               // 寄存器 03E8 的内容(激活请求：0x0009)
            static_cast<uint8_t>(pos), 0x00,                          // 寄存器 03E9 的内容(目标位置：0x00~0xFF)
            static_cast<uint8_t>(torque), static_cast<uint8_t>(speed) // 寄存器 03EA 的内容(目标速度：0x00~0xFF。力矩：0x00~0xFF )
        };
        int raw_req_length = sizeof(raw_req) / sizeof(uint8_t);
        ret = safe_modbus_write_raw_req(d_ctx, raw_req, raw_req_length);
        if (ret == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "runWithParameter error: %s", modbus_strerror(errno));
            return -1;
        }
        if (ret == raw_req_length + 2)
        {
            RCLCPP_INFO(rclcpp::get_logger("clawControl"), "runWithParameter success: %d", pos);
            return 1;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "runWithParameter error: %s", e.what());
    }
}

int clawControl::getClawCurrentPosition(int slaveId)
{
    try
    {
        if (slaveId <= 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "readClawStatus error: 参数无效");
            return -1;
        }

        // 检查modbus_set_slave返回值
        if (modbus_set_slave(d_ctx, slaveId) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "设置从设备ID失败: %s", modbus_strerror(errno));
            return -1;
        }

        // 检查modbus_set_response_timeout返回值
        if (modbus_set_response_timeout(d_ctx, 1, 0) == -1)
        {
            RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "设置超时失败: %s", modbus_strerror(errno));
            return -1;
        }

        std::vector<uint16_t> buffer(3);
        int retryCount = 10;
        int ret = -1;
        while (retryCount-- > 0)
        {
            ret = safe_modbus_read_registers(d_ctx, 0x07D0, 3, buffer.data());
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }

        if (ret != 3)
        {
            RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "readClawStatus error: %s", modbus_strerror(errno));
            return -1;
        }

        // 在读取成功后添加：
        if (ret == 3)
        { // 确保读取到3个寄存器
            uint16_t raw_value = buffer[1];
            uint8_t high_byte = (raw_value >> 8) & 0xFF;
            int decimal_value = static_cast<int>(high_byte);

            RCLCPP_INFO(rclcpp::get_logger("clawControl"), "夹爪当前位置: %d ", decimal_value);
            return decimal_value;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("clawControl"), "readClawStatus error: %s", e.what());
        return -1;
    }
}
int clawControl::safe_modbus_write_raw_req(modbus_t *ctx, uint8_t raw_req[], int raw_req_length)
{
    std::lock_guard<std::mutex> guard(d_mutex);
    return modbus_send_raw_request(ctx, raw_req, raw_req_length);
}

int clawControl::safe_modbus_read_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest)
{
    std::lock_guard<std::mutex> guard(d_mutex);
    return modbus_read_input_registers(ctx, addr, nb, dest);
}
