#include "SerialImplWin.h"

#include <stdio.h>
#include <string.h>


SerialImpl::SerialImpl(const char* dev)
{

}

SerialImpl::~SerialImpl()
{

}

int SerialImpl::setOpt(int speed, int bits, char parity, int stop)
{
    return 0;
}

int SerialImpl::write(char* buf, int size)
{
    return 0;
}

int SerialImpl::readBlock(char* buf, int size, int sec)
{
    return 0;
}

int SerialImpl::read(char* buf, int size)
{
    return 0;
}
