#ifndef __SERIALIMPL_UNIX_H__
#define __SERIALIMPL_UNIX_H__

#include <stdint.h>
#include <stddef.h>

class SerialImpl
{
public:
    SerialImpl(const char* dev);
    ~SerialImpl();

    int setOpt(int speed, int bits, char parity, int stopbit);
    int read(char* buf, size_t size);
    int readBlock(char* buf, size_t size, int sec);
    int readBlockMs(char* buf, size_t size, int msec);
    int write(char * buf, size_t size);

private:
    int m_fd;
};

#endif // __SERIALIMPL_UNIX_H__
