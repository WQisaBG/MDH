#include "SerialImplUnix.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>


SerialImpl::SerialImpl(const char* dev)
{
    m_fd = open(dev, O_RDWR | O_NOCTTY);
    //m_fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (m_fd <= 0) {
        printf("Failed to open %s!\n", dev);
        m_fd = -1;
    }
}

SerialImpl::~SerialImpl()
{
    if (m_fd > 0)
        close(m_fd);
}

int SerialImpl::setOpt(int speed, int bits, char parity, int stopbit)
{
    if (m_fd < 0)
    {
        printf("Serial Init failed\n");
        return -1;
    }

    fcntl(m_fd, F_SETFL, 0);

    struct termios newtio, oldtio;
    if (tcgetattr( m_fd, &oldtio ) != 0)
    {
        printf("tcgetattr error!\n");
        return -1;
    }

    tcflush(m_fd, TCIOFLUSH);

    bzero( &newtio, sizeof(newtio) );
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    newtio.c_lflag &= ~(ICANON | ISIG | ECHO | ECHOE | IEXTEN);
    newtio.c_oflag &= ~OPOST;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | BRKINT | INPCK | ISTRIP);
    newtio.c_cflag &= ~CRTSCTS;

    /****** 设置数据位 *******/
    switch ( bits ) {
        case 5:
            newtio.c_cflag |= CS5;
            break;
        case 6:
            newtio.c_cflag |= CS6;
            break;
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }


    /******* 设置校验 ********/
    switch (parity) {
        case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
    }

    /******* 设置波特率 ********/
    switch (speed) {
        case 50:
            cfsetispeed(&newtio, B50);
            cfsetospeed(&newtio, B50);
            break;
        case 75:
            cfsetispeed(&newtio, B75);
            cfsetospeed(&newtio, B75);
            break;
        case 110:
            cfsetispeed(&newtio, B110);
            cfsetospeed(&newtio, B110);
            break;
        case 134:
            cfsetispeed(&newtio, B134);
            cfsetospeed(&newtio, B134);
            break;
        case 150:
            cfsetispeed(&newtio, B150);
            cfsetospeed(&newtio, B150);
            break;
        case 200:
            cfsetispeed(&newtio, B200);
            cfsetospeed(&newtio, B200);
            break;
        case 300:
            cfsetispeed(&newtio, B300);
            cfsetospeed(&newtio, B300);
            break;
        case 600:
            cfsetispeed(&newtio, B600);
            cfsetospeed(&newtio, B600);
            break;
        case 1200:
            cfsetispeed(&newtio, B1200);
            cfsetospeed(&newtio, B1200);
            break;
        case 1800:
            cfsetispeed(&newtio, B1800);
            cfsetospeed(&newtio, B1800);
            break;
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 19200:
            cfsetispeed(&newtio, B19200);
            cfsetospeed(&newtio, B19200);
            break;
        case 38400:
            cfsetispeed(&newtio, B38400);
            cfsetospeed(&newtio, B38400);
            break;
        case 57600:
            cfsetispeed(&newtio, B57600);
            cfsetospeed(&newtio, B57600);
            break;
        case 100000:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 230400:
            cfsetispeed(&newtio, B230400);
            cfsetospeed(&newtio, B230400);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        case 500000:
            cfsetispeed(&newtio, B500000);
            cfsetospeed(&newtio, B500000);
            break;
        case 576000:
            cfsetispeed(&newtio, B576000);
            cfsetospeed(&newtio, B576000);
            break;
        case 921600:
            cfsetispeed(&newtio, B921600);
            cfsetospeed(&newtio, B921600);
            break;
        case 1000000:
            cfsetispeed(&newtio, B1000000);
            cfsetospeed(&newtio, B1000000);
            break;
        case 1152000:
            cfsetispeed(&newtio, B1152000);
            cfsetospeed(&newtio, B1152000);
            break;
        case 1500000:
            cfsetispeed(&newtio, B1500000);
            cfsetospeed(&newtio, B1500000);
            break;
        case 2000000:
            cfsetispeed(&newtio, B2000000);
            cfsetospeed(&newtio, B2000000);
            break;
        case 2500000:
            cfsetispeed(&newtio, B2500000);
            cfsetospeed(&newtio, B2500000);
            break;
        case 3000000:
            cfsetispeed(&newtio, B3000000);
            cfsetospeed(&newtio, B3000000);
            break;
        case 3500000:
            cfsetispeed(&newtio, B3500000);
            cfsetospeed(&newtio, B3500000);
            break;
        case 4000000:
            cfsetispeed(&newtio, B4000000);
            cfsetospeed(&newtio, B4000000);
            break;
        default:
            printf("Failed to set speed!\n");
            return -1;
    }

    /******* 设置停止位 ********/
    if (stopbit == 1) {
        newtio.c_cflag &= ~CSTOPB;
    }
    else if (stopbit == 2) {
        newtio.c_cflag |= CSTOPB;
    }

    newtio.c_cc[VTIME] = 5;
    newtio.c_cc[VMIN] = 0;

    if ((tcsetattr(m_fd, TCSANOW, &newtio)) != 0)
    {
        printf("set opt error!\n");
        return -1;
    }

    tcflush(m_fd, TCIFLUSH);

    return 0;
}

int SerialImpl::write(char* buf, size_t size)
{
    if (m_fd < 0)
    {
        return -1;
    }

    int ret = ::write(m_fd, buf, size);
    if (ret < 0)
    {
        printf("Uart write failed!\n");
        return -1;
    }

    return ret;
}

int SerialImpl::readBlock(char* buf, size_t size, int sec)
{
    int ret = -1;
    int retval;
    fd_set rfds;
    struct timeval tv;

    if (m_fd < 0)
    {
        printf("Serial Init failed!\n");
        return -1;
    }

    FD_ZERO(&rfds);
    FD_SET(m_fd, &rfds);

    tv.tv_sec = sec;
    tv.tv_usec = 0;
    retval = ::select(m_fd + 1, &rfds, NULL, NULL, &tv);
    if (retval == -1)
    {
        printf("select()\n");
        return -1;
    }
    else if (retval)
    {
        {
            ret = ::read(m_fd, buf, size);
            if (ret <= 0)
            {
                printf("Error read data!\n");
                return -1;
            }
            else
            {
                //LOGV << __FUNCTION__ << " " << __LINE__ << " len: " << ret;
                // printf("len: %d \n", ret);

                return ret;
            }
        }
    }
    else
    {
        //LOGD << "No data within time_sec: " << sec << " seconds";
        return 0;
    }

}

int SerialImpl::readBlockMs(char* buf, size_t size, int msec)
{
    int ret = -1;
    int retval;
    fd_set rfds;
    struct timeval tv;

    if (m_fd < 0)
    {
        printf("Serial Init failed!\n");
        return -1;
    }

    FD_ZERO(&rfds);
    FD_SET(m_fd, &rfds);

    tv.tv_sec = msec/1000;
    tv.tv_usec = msec % 1000 * 1000;
    retval = ::select(m_fd + 1, &rfds, NULL, NULL, &tv);
    if (retval == -1)
    {
        printf("select()\n");
        return -1;
    }
    else if (retval)
    {
        {
            ret = ::read(m_fd, buf, size);
            if (ret <= 0)
            {
                printf("Error read data!\n");
                return -1;
            }
            else
            {
                //LOGV << __FUNCTION__ << " " << __LINE__ << " len: " << ret;
                // printf("len: %d \n", ret);

                return ret;
            }
        }
    }
    else
    {
        //LOGD << "No data within time_sec: " << sec << " seconds";
        return 0;
    }

}

int SerialImpl::read(char* buf, size_t size)
{
    if (m_fd < 0)
    {
        printf("Serial Init failed!\n");
        return -1;
    }
    //cout << __PRETTY_FUNCTION__ << " " << __LINE__ << endl;
    return ::read(m_fd, buf, size);
}

