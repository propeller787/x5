/*
 * 文件功能：串口参数设置、数据读写等功能的实现
 * 创建人：yangshuai15
 * 日  期：2021/6/16
 */
#include "serial_port.h"
#include <dirent.h>
#include <fcntl.h>
#include <iostream>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
namespace utils {

SerialPort::SerialPort(const std::string& devpath, const OpenOptions& options, bool is_block)
    : _path(devpath)
    , _open_options(options)
    , _is_block(is_block)
{
}

SerialPort::~SerialPort()
{
    close(_tty_fd);
}

bool SerialPort::start()
{
    return serial_port_init();
}

//::read(_tty_fd, data, length)
int SerialPort::read(char* buffer, int buf_len)
{
    return ::read(_tty_fd, buffer, buf_len);
}

// int SerialPort::write(char* buffer, int buf_len)
// {
//     return ::write(_tty_fd, buffer, buf_len);
// }

int SerialPort::write(char* buffer, int buf_len)
{
    int offset = 0, total = buf_len;
    ssize_t wcnt = 0;

    do {
        wcnt = ::write(_tty_fd, buffer + offset, total);
        if (wcnt > 0) {
            offset += wcnt;
            total -= wcnt;
        } else if (wcnt < 0 && errno == EINTR) {
            usleep(1000);
            continue;
        } else {
            printf("write data to serial failed. errno:%d \n", errno);
            break;
        }
    } while (total > 0);
    // std::cout << " wcnt " << wcnt << std::endl;
    return 0;
    // int wcnt = ::write(_tty_fd, buffer, buf_len);
    // std::cout << " 1wcnt2 " << wcnt << std::endl;
    // return wcnt;
    // return ::write(_tty_fd, buffer, buf_len);
}

int SerialPort::get_tty_fd()
{
    if (_tty_fd > 0) {
        return _tty_fd;
    }
    return -1;
}

bool SerialPort::serial_port_init()
{
    if (!_path.size()) {
        return false;
    }
    if (_is_block) {
        _tty_fd = open(_path.c_str(), O_RDWR | O_NOCTTY);
    } else {
        _tty_fd = open(_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    }
    if (_tty_fd < 0) {
        std::cout << "open serial device " << _path << " failed!" << std::endl;
        return false;
    }

    struct termios tios;
    termios_options(tios, _open_options);
    tcsetattr(_tty_fd, TCSANOW, &tios);
    tcflush(_tty_fd, TCIOFLUSH);
    return true;
}

void SerialPort::termios_options(termios& tios, const OpenOptions& options)
{
    tcgetattr(_tty_fd, &tios);

    cfmakeraw(&tios);
    tios.c_cflag &= ~(CSIZE | CRTSCTS);
    tios.c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR);
    tios.c_lflag &= ~(ECHOK | ECHOCTL | ECHOKE);
    tios.c_oflag &= ~(OPOST | ONLCR);

    cfsetispeed(&tios, options.baud_rate);
    cfsetospeed(&tios, options.baud_rate);

    tios.c_iflag |= (options.xon ? IXON : 0) | (options.xoff ? IXOFF : 0) | (options.xany ? IXANY : 0);

    // data bits
    int databits[] = { CS5, CS6, CS7, CS8 };
    tios.c_cflag &= ~0x30;
    tios.c_cflag |= databits[options.data_bits];

    // stop bits
    if (options.stop_bits == STOP_BITS_2) {
        tios.c_cflag |= CSTOPB;
    } else {
        tios.c_cflag &= ~CSTOPB;
    }

    // parity  奇偶校验
    if (options.parity == PARITY_NONE) {
        tios.c_cflag &= ~PARENB;
    } else {
        tios.c_cflag |= PARENB;

        if (options.parity == PARITY_MARK) {
            tios.c_cflag |= PARMRK;
        } else {
            tios.c_cflag &= ~PARMRK;
        }

        if (options.parity == PARITY_ODD) {
            tios.c_cflag |= PARODD;
        } else {
            tios.c_cflag &= ~PARODD;
        }
    }

    tios.c_cc[VMIN] = options.vmin;
    tios.c_cc[VTIME] = options.vtime;
}

} // namespace sensor