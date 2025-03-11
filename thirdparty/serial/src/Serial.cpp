#include "Serial.h"

#include "SerialImplUnix.h"

#define DEFAULT_UART_DEV	"ttyS0"

Serial::Serial(const char* dev)
    : m_impl(new SerialImpl(dev))
{

}

Serial::~Serial()
{
    if (m_impl) {
        delete m_impl;
        m_impl = nullptr;
    }
}

Serial& Serial::getInstance()
{
    static Serial instance(DEFAULT_UART_DEV);
    instance.setOpt(115200, 8, 'N', 1);
    return instance;
}

int Serial::setOpt(int speed, int bits, char parity, int stopbit)
{
    return m_impl->setOpt(speed, bits, parity, stopbit);
}

int Serial::write(char* buf, size_t size)
{
    return m_impl->write(buf, size);
}

int Serial::readBlock(char* buf, size_t size, int sec)
{
    return m_impl->readBlock(buf, size, sec);
}

int Serial::readBlockMs(char* buf, size_t size, int msec)
{
    return m_impl->readBlockMs(buf, size, msec);
}

int Serial::read(char* buf, size_t size)
{
    return m_impl->read(buf, size);
}

