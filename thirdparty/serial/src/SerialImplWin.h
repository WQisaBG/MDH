#ifndef __SERIALIMPL_WIN_H__
#define __SERIALIMPL_WIN_H__

#include <WinSock2.h>
#include <windows.h>

class SerialImpl
{
public:
    SerialImpl(const char* dev);
    ~SerialImpl();

    int setOpt(int speed, int bits, char parity, int stopbit);
    int read(char* buf, int size);
    int readBlock(char* buf, int size, int sec);
    int write(char * buf, int size);

private:
    HANDLE m_fd;
};

#endif // __SERIALIMPL_WIN_H__
