/*
 * @copyright: Copyright (c) 2024 zhengqi.com, Inc. All Rights Reserved
 * @Author: likang(likang@baidu.com)
 * @Date: 2024-01-05 16:32:38
 * @FilePath: mem/src/sensor/serial_port/serial_port.h
 * @brief:
 */
#ifndef SERIALPORT_H
#define SERIALPORT_H
#include <functional>
#include <memory>
#include <string>
#include <vector>

struct termios;
namespace sensor::gps {

enum BaudRate {
    BR0 = 0000000,
    BR50 = 0000001,
    BR75 = 0000002,
    BR110 = 0000003,
    BR134 = 0000004,
    BR150 = 0000005,
    BR200 = 0000006,
    BR300 = 0000007,
    BR600 = 0000010,
    BR1200 = 0000011,
    BR1800 = 0000012,
    BR2400 = 0000013,
    BR4800 = 0000014,
    BR9600 = 0000015,
    BR19200 = 0000016,
    BR38400 = 0000017,
    BR57600 = 0010001,
    BR115200 = 0010002,
    BR230400 = 0010003,
    BR460800 = 0010004,
    BR500000 = 0010005,
    BR576000 = 0010006,
    BR921600 = 0010007,
    BR1000000 = 0010010,
    BR1152000 = 0010011,
    BR1500000 = 0010012,
    BR2000000 = 0010013,
    BR2500000 = 0010014,
    BR3000000 = 0010015,
    BR3500000 = 0010016,
    BR4000000 = 0010017
};

enum DataBits {
    DATA_BITS_5,
    DATA_BITS_6,
    DATA_BITS_7,
    DATA_BITS_8,
};

enum StopBits { STOP_BITS_1,
    STOP_BITS_2 };

enum Parity { PARITY_NONE,
    PARITY_EVEN,
    PARITY_MARK,
    PARITY_ODD,
    PARITY_SPACE };

struct OpenOptions {
    bool auto_open;
    BaudRate baud_rate;
    DataBits data_bits;
    StopBits stop_bits;
    Parity parity;
    bool xon; // 当一端的数据连接不再能接受更多数据（或者接近这个状态），它发送XOFF字节给另一端。另一端收到XOFF字节，挂起数据发送。
    bool xoff; // 一端如果准备好继续接收数据，它发送XON字节给另一端，另一端恢复数据发送。
    bool xany;
    int vmin;
    int vtime;
};

class SerialPort {
public:
    static BaudRate baud_rate_make(unsigned long baud_rate);
    static const OpenOptions default_options;
    SerialPort(const std::string& devpath, const OpenOptions& options, bool isblock = false);
    ~SerialPort();
    bool start();
    int read(char* buffer, int buf_len);
    int write(char* buffer, int buf_len);
    int get_tty_fd();

private:
    bool _is_block = true;
    bool serial_port_init();
    void termios_options(termios& tios, const OpenOptions& options);
    std::string _path;
    OpenOptions _open_options;
    int _tty_fd = 0;
};

} // namespace sensor
#endif // SERIALPORT_H