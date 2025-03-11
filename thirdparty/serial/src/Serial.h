#ifndef __SERIAL_H__
#define __SERIAL_H__

/****************** 串口操作类 *****************/

/*
 *	usage:
 *  Serial uart("/dev/ttyS1");
 *  uart.SetOpt(115200, 8, 'N', 1);
 */
#include <stdint.h>
#include <stddef.h>

class SerialImpl;
class Serial
{
public:
    Serial(const char* dev);
    ~Serial();
    static Serial& getInstance();

    /********************
     * 设置串口属性值
     * 参数:
     * 		speed: 波特率
     *             50
     *             75
     *             110
     *             134
     *             150
     *             200
     *             300
     *             600
     *             1200
     *             1800
     *             2400
     *             4800
     *             9600
     *             19200
     *             38400
     *             57600
     *             115200
     *             230400
     *             460800
     *             500000
     *             576000
     *             921600
     *             1000000
     *             1152000
     *             1500000
     *             2000000
     *             2500000
     *             3000000
     *             3500000
     *             4000000
     * 		bits: 数据位
     *            5
     *            6
     *            7
     *            8
     *      parity: 校验方式
     *           'N'   (o)
     *           'O'   (dd)
     *           'E'   (ven)
     * 		stopbit: 停止位
     *            1
     *            2
     *
     * 返回值:
     * 		成功返回0, 失败返回-1
     ********************/
    int setOpt(int speed, int bits, char parity, int stopbit);

    /********************
     * 串口数据读取函数(非阻塞)
     * 参数:
     * 		buf: 数据指针
     * 		size: buf大小
     * 返回值:
     * 		成功返回读取字节数, 失败返回-1
     ********************/
    int read(char* buf, size_t size);

    /********************
     * 串口数据读取函数(阻塞)
     * 参数:
     * 		buf: 数据指针
     * 		size: data长度
     * 		sec: 超时时间(秒)
     * 返回值:
     * 		成功返回读取字节数, 失败返回-1, 超时返回0
     ********************/
    int readBlock(char* buf, size_t size, int sec);

    /********************
     * 串口数据读取函数(阻塞)
     * 参数:
     * 		buf: 数据指针
     * 		size: data长度
     * 		msec: 超时时间(毫秒)
     * 返回值:
     * 		成功返回读取字节数, 失败返回-1, 超时返回0
     ********************/
    int readBlockMs(char* buf, size_t size, int msec);

    /********************
     * 串口数据发送函数
     * 参数:
     * 		buf: 数据指针
     * 		size: 数据有效长度
     * 返回值:
     * 		成功返回写入的字节数, 失败返回-1
     ********************/
    int write(char * buf, size_t size);

private:
    SerialImpl* m_impl;
};


#if 0

/********************** 串口例程 *****************/
void uartSend(void* argv)
{
    Serial* serial = (Serial*)argv;

    int i = 0;
    while(1)
    {
        char buffer[100] = {};
        sprintf(buffer, "+++++++++++++++++++ hello %d", i++);
        serial->write(buffer, strlen(buffer));

        this_thread::sleep_for(chrono::seconds(2));
    }
}

void uartRecv(void* argv)
{
    Serial* serial = (Serial*)argv;

    char buffer[100] = {};
    while(1)
    {
        serial->readBlock(buffer, sizeof(buffer), 3600);
        LOGD << "---------------------------------------- recv: " << buffer;
    }
}

int main(int argc, char** argv)
{
    /**** 串口测试 ****/
    char dev[] = "ttyS0";
    Serial serial(dev);
    serial.SetOpt(115200, 8, 'N', 1);

    std::thread uartSendThread(uartSend, &serial);
    std::thread uartRecvThread(uartRecv, &serial);
}

#endif

#endif // __SERIAL_H__
