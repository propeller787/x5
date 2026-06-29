#include "bluetooth.hpp"
#include <utils/topic.h>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <sstream>
#include <thread>
#include <time.h>
#include <algorithm>
#include <fstream>
#include <sys/stat.h>

using namespace sensor::bluetooth;

// 辅助函数：带重试的写入操作
ssize_t write_with_retry(int fd, const void* buf, size_t count) {
    size_t total_written = 0;
    const char* buffer = static_cast<const char*>(buf);
    int retries = 5;
    struct timespec delay = {0, 30000000}; // 30ms

    while (total_written < count && retries > 0) {
        ssize_t written = write(fd, buffer + total_written, count - total_written);
        if (written > 0) {
            total_written += written;
        } else if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // 资源暂时不可用，重试
            nanosleep(&delay, nullptr);
            retries--;
        } else {
            // 其他错误
            return -1;
        }
    }

    return total_written;
}

// 辅助函数：带重试的读取操作
ssize_t read_with_retry(int fd, void* buf, size_t count) {
    size_t total_read = 0;
    char* buffer = static_cast<char*>(buf);
    int retries = 5;
    struct timespec delay = {0, 30000000}; // 30ms

    while (total_read < count && retries > 0) {
        ssize_t read_bytes = read(fd, buffer + total_read, count - total_read);
        if (read_bytes > 0) {
            total_read += read_bytes;
        } else if (read_bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // 资源暂时不可用，重试
            nanosleep(&delay, nullptr);
            retries--;
        } else {
            // 其他错误或读取结束
            break;
        }
    }

    return total_read;
}

namespace {

std::string read_serial_response(int fd, int total_timeout_ms = 160, int idle_timeout_ms = 80) {
    std::string response;
    response.reserve(1024);

    auto start = std::chrono::steady_clock::now();
    auto last_data_time = start;
    char tmp[256];

    while (true) {
        ssize_t n = read(fd, tmp, sizeof(tmp));
        if (n > 0) {
            response.append(tmp, tmp + n);
            last_data_time = std::chrono::steady_clock::now();
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        auto total_elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        auto idle_elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_data_time).count();

        if (total_elapsed_ms >= total_timeout_ms) {
            break;
        }
        if (!response.empty() && idle_elapsed_ms >= idle_timeout_ms) {
            break;
        }

        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    return response;
}
// 

bool has_terminal_line(const std::string& response) {
    return response.find("\r\nOK\r\n") != std::string::npos ||
           response.find("\nOK\n") != std::string::npos ||
           response.find("\r\nERROR\r\n") != std::string::npos ||
           response.find("\nERROR\n") != std::string::npos ||
           response.find("+QBLEGATTCIND:") != std::string::npos;
}

}  // namespace

BluetoothComponent::BluetoothComponent(const rclcpp::NodeOptions& options) 
    : rclcpp::Node("bluetooth", options), device_fd_(-1),  bluetooth_device_path_("/dev/ttyAS8"){
    auto logger = rclcpp::get_logger("sensor::bluetooth");
    RCLCPP_INFO(logger, "Bluetooth Component starting...");
    
    // 声明参数
    this->declare_parameter("bluetooth_device_path", bluetooth_device_path_);
    // 获取参数
    bluetooth_device_path_ = this->get_parameter("bluetooth_device_path").as_string();
    RCLCPP_INFO(logger, "Device path: %s", bluetooth_device_path_.c_str());
    
    // 设置设备
    if (!setupDevice()) {
        RCLCPP_ERROR(logger, "Failed to setup Bluetooth device");
        return;
    }
    
    // 创建发布器
    bluetooth_status_publisher_ = this->create_publisher<interface::msg::BluetoothStatus>(bluetoothStatusTopic, 10);
    
    // 初始化上一次状态的智能指针
    last_status_ = std::make_shared<interface::msg::BluetoothStatus>();
    // 加载保存的MAC地址
    if (loadMacAddressFromConfig()) {
        RCLCPP_INFO(logger, "Loaded saved MAC address: %s", saved_mac_address_.c_str());
    } else {
        RCLCPP_INFO(logger, "No saved MAC address found, will search for devices");
    }
    worker_running_ = true;
    worker_thread_ = std::thread(&BluetoothComponent::workerThreadFunction, this);
    RCLCPP_INFO(logger, "Bluetooth Component initialized successfully!");
}

BluetoothComponent::~BluetoothComponent() {
    worker_running_ = false;
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    if (device_fd_ >= 0) {
        close(device_fd_);
    }
}

bool BluetoothComponent::setupDevice() {
    auto logger = rclcpp::get_logger("sensor::bluetooth");
    device_fd_ = open(bluetooth_device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (device_fd_ < 0) {
        RCLCPP_ERROR(logger, "Failed to open device: %s", bluetooth_device_path_.c_str());
        return false;
    }
    
    // 配置串口
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(device_fd_, &tty) != 0) {
        RCLCPP_ERROR(logger, "Failed to get serial port attributes %s", strerror(errno));
        close(device_fd_);
        device_fd_ = -1;
        return false;
    }
    
    // 设置波特率为115200
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    
    // 8位数据，无校验，1位停止位
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    // 原始模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    // 设置读取超时
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    if (tcsetattr(device_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(logger, "Failed to set serial port attributes");
        close(device_fd_);
        device_fd_ = -1;
        return false;
    }
    RCLCPP_INFO(logger, "Successfully opened and configured device: %s", bluetooth_device_path_.c_str());
    return true;
}

void BluetoothComponent::workerThreadFunction() {
   /**
    *  @brief 
    *  1.优先尝试连接保存的MAC地址
    *  2.若连接失败则搜索附近蓝牙设备 只保留Freego_CS的设备 并将信号强度最强的设备进行排序
    *  3.连接信号最强的设备
    *  4.如果连接成功，请求基站状态 发送AA55B1A1C0000212命令
    *  5.获取应答并解析状态信息
    *  6.将状态信息发布到ROS2主题 BluetoothStatus
    *  7.搜索到新设备时更新配置文件
    */
    auto logger = rclcpp::get_logger("sensor::bluetooth");
    RCLCPP_INFO(logger, "配置文件内保存的MAC地址: %s", saved_mac_address_.c_str());
    while(worker_running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 延迟300ms  进程执行时间 300ms
        if (!isConnected()) {
            bool connected = false;
            BluetoothDevice device;
            if (!saved_mac_address_.empty()) {
                device.address = saved_mac_address_;
                if (toConnected(device)) {
                    connected = true;
                }
            }
            if (!connected) {
                RCLCPP_INFO(logger, "尝试连接保存的MAC地址连接失败，开始搜索设备");
                if (!getBluetoothStatus(device) && !toConnected(device)) {
                    RCLCPP_ERROR(logger, "搜索/连接 设备失败");
                    auto msg = std::make_shared<interface::msg::BluetoothStatus>();
                    msg->status = false;
                    bluetooth_status_publisher_->publish(*msg);
                    continue;
                }
                RCLCPP_INFO(logger, "搜索到设备，连接到设备, 更新配置文件内保存的MAC地址 %s", device.address.c_str());
                if (device.address != saved_mac_address_) {
                    saveMacAddressToConfig(device.address);
                    saved_mac_address_ = device.address;
                }
            }
        }
        if(!isSubscribed_ && !subscriberUUID()) {
            continue;
        }
        isSubscribed_ = true;
        if (!requestConnect()) {
            request_connect_fail_count_++;
            if (request_connect_fail_count_ >= 3) {
                isSubscribed_ = false;
                request_connect_fail_count_ = 0;
                continue;
            }
        } else {
            request_connect_fail_count_ = 0;
        }
    }
}
bool BluetoothComponent::isConnected() {
    auto logger = rclcpp::get_logger("sensor::bluetooth");
    RCLCPP_INFO(logger, "连接状态检测阶段");
    // 检查设备文件描述符是否有效
    if (device_fd_ < 0) {
        RCLCPP_ERROR(logger, "Device file descriptor is invalid");
        return false;
    }
    
    tcflush(device_fd_, TCIFLUSH);
    std::string cmd = "AT+QBLESTAT\r\n";
    ssize_t written = write_with_retry(device_fd_, cmd.c_str(), cmd.size());
    if (written != static_cast<ssize_t>(cmd.size())) {
        RCLCPP_ERROR(logger, "Failed to write AT+QBLESTAT command: %s, written: %zd, expected: %zu",            
            strerror(errno), written, cmd.size());
        return false;
    }
    RCLCPP_INFO(logger, "检测连接阶段请求: %s", cmd.c_str());
    
    std::string response = read_serial_response(device_fd_, 200, 120);
    RCLCPP_INFO(logger, "检测连接阶段响应: %s", response.c_str());
    if (response.empty()) {
        /* 可能是请求太快导致没有响应 默认使用上一次连接状态 */
        RCLCPP_WARN(logger, "AT+QBLESTAT no response");
        return isSubscribed_;
    }
    if (response.find("DISCONNECTED") != std::string::npos) {
        isSubscribed_ = false;
        return false;
    }
    if (response.find("CONNECTED") != std::string::npos) {
        return true;
    }
    return false;
}
bool BluetoothComponent::toConnected(const BluetoothDevice& device) { 
    auto logger = rclcpp::get_logger("sensor::bluetooth");
    RCLCPP_INFO(logger, "连接请求阶段");
    // 检查设备文件描述符是否有效
    if (device_fd_ < 0) {
        RCLCPP_ERROR(logger, "Device file descriptor is invalid");
        return false;
    }
    
    tcflush(device_fd_, TCIFLUSH);
    std::string cmd = "AT+QBLECONN=0,\"" + device.address + "\"\r\n";
    RCLCPP_INFO(logger, "蓝牙连接请求命令: %s", cmd.c_str());
    
    ssize_t written = write_with_retry(device_fd_, cmd.c_str(), cmd.size());
    if (written != static_cast<ssize_t>(cmd.size())) {
        RCLCPP_ERROR(logger, "Failed to write AT+QBLECONN command: %s, written: %zd, expected: %zu", 
            strerror(errno), written, cmd.size());
        return false;
    }
    
    std::string response = read_serial_response(device_fd_, 2500, 200);
    RCLCPP_INFO(logger, "连接请求结果: %s", response.c_str());
    if (response.empty()) {
        RCLCPP_WARN(logger, "AT+QBLECONN no response");
        return false;
    }
    if (response.find("CONNECTED") != std::string::npos) {
        isSubscribed_ = true;  // 连接成功后重置订阅状态
        return true;
    }
    return false;
}
bool BluetoothComponent::getBluetoothStatus(BluetoothDevice& device) {    
    auto logger = rclcpp::get_logger("sensor::bluetooth");
    RCLCPP_INFO(logger, "蓝牙搜索阶段");
    // 检查设备文件描述符是否有效
    if (device_fd_ < 0) {
        RCLCPP_ERROR(logger, "Device file descriptor is invalid");
        return false;
    }
    
    tcflush(device_fd_, TCIFLUSH);
    // 发送AT+QBLESCAN=1命令设置为扫描状态
    std::string cmd = "AT+QBLESCAN=1\r\n";
    ssize_t written = write_with_retry(device_fd_, cmd.c_str(), cmd.size());
    if (written != static_cast<ssize_t>(cmd.size())) {
        RCLCPP_ERROR(logger, "Failed to write AT+QBLESCAN=1 command: %s, written: %zd, expected: %zu", 
            strerror(errno), written, cmd.size());
        // return false;
    }
    
    // 等待3秒让蓝牙模块有足够时间扫描设备
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // 读取所有返回的数据
    std::string response = read_serial_response(device_fd_, 1500, 250);
    // 发送AT+QBLESCAN=0命令停止扫描
    cmd = "AT+QBLESCAN=0\r\n";
    written = write_with_retry(device_fd_, cmd.c_str(), cmd.size());
    if (written != static_cast<ssize_t>(cmd.size())) {
        RCLCPP_ERROR(logger, "Failed to write AT+QBLESCAN=0 command: %s, written: %zd, expected: %zu", 
            strerror(errno), written, cmd.size());
        // return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // 解析扫描结果  响应格式为: +QBLESCAN: "Freego_CS",1,"3B124B48447E"
    if (!response.empty()) {
        std::stringstream ss(response);
        std::string line;
        while (std::getline(ss, line)) {
            // 查找包含+QBLESCAN的行
            if (line.find("Freego_CS") == std::string::npos) {
                continue;
            }
            // 解析设备信息
            // 格式: +QBLESCAN: "Freego_CS",0,"E4EF2DF17F2B"
            size_t first_quote = line.find('"');
            size_t second_quote = line.find('"', first_quote + 1);
            size_t comma1 = line.find(',', second_quote + 1);
            size_t comma2 = line.find(',', comma1 + 1);
            size_t third_quote = line.find('"', comma2 + 1);
            size_t fourth_quote = line.find('"', third_quote + 1);
            
            if (first_quote != std::string::npos && second_quote != std::string::npos && 
                comma1 != std::string::npos && comma2 != std::string::npos && 
                third_quote != std::string::npos && fourth_quote != std::string::npos) {
                
                std::string device_name = line.substr(first_quote + 1, second_quote - first_quote - 1);
                
                // 只处理Freego_CS设备
                if (device_name.find("Freego_CS") == std::string::npos) {
                    continue;
                }
                try {
                    int rssi = std::stoi(line.substr(comma1 + 1, comma2 - comma1 - 1));
                    std::string mac_address = line.substr(third_quote + 1, fourth_quote - third_quote - 1);
                    // 更新信号强度最强的设备
                    if (rssi > device.rssi) {
                        device.name = device_name;
                        device.rssi = rssi;
                        device.address = mac_address;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(logger, "Failed to parse device info: %s", e.what());
                    continue;
                }
            }
        }
    }
    return device.rssi > -1;
}
bool BluetoothComponent::subscriberUUID() {
    auto logger = rclcpp::get_logger("sensor::bluetooth");
    // 检查设备文件描述符是否有效
    if (device_fd_ < 0) {
        RCLCPP_ERROR(logger, "Device file descriptor is invalid");
        return false;
    }

    tcflush(device_fd_, TCIFLUSH);
    // 订阅UUID
    std::string cmd = "AT+QBLEGATTCNTFCFG=0,\"FFF1\",1\r\n";
    ssize_t written = write_with_retry(device_fd_, cmd.c_str(), cmd.size());
    if (written != static_cast<ssize_t>(cmd.size())) {
        RCLCPP_ERROR(logger, "Failed to write AT+QBLEGATTCNTFCFG command: %s, written: %zd, expected: %zu", 
            strerror(errno), written, cmd.size());
        return false;
    }
    std::string response = read_serial_response(device_fd_, 200, 80);
    if (!response.empty()) {
        RCLCPP_DEBUG(logger, "Response: %s", response.c_str());
        if (response.find("OK") != std::string::npos) {
            return true;
        }
    }
    return false;
}
bool BluetoothComponent::requestConnect() { 
    auto logger = rclcpp::get_logger("sensor::bluetooth");
    // 检查设备文件描述符是否有效
    if (device_fd_ < 0) {
        RCLCPP_ERROR(logger, "Device file descriptor is invalid");
        return false;
    }
    tcflush(device_fd_, TCIFLUSH);
    std::string cmd = "AT+QBLEGATTCWRCMD=0,\"FFF2\",8,\"AA55B1A1C0000212\"\r\n";
    ssize_t written = write_with_retry(device_fd_, cmd.c_str(), cmd.size());
    if (written != static_cast<ssize_t>(cmd.size())) {
        RCLCPP_ERROR(logger, "Failed to write AT+QBLETRANMODE command: %s, written: %zd, expected: %zu", 
            strerror(errno), written, cmd.size());
        return false;
    }
    // 读取响应：等待直到出现终止行(OK/ERROR)或通知行(+QBLEGATTCIND)
    std::string response;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start)
               .count() < 200) {
        response += read_serial_response(device_fd_, 250, 100);
        if (has_terminal_line(response)) {
            break;
        }
    }
    bool result = false;
    if (!response.empty()) {
        RCLCPP_DEBUG(logger, "Response: %s", response.c_str());
        std::stringstream ss(response);
        std::string line;
        while (std::getline(ss, line)) {
            // 去除首尾空白字符
            line.erase(0, line.find_first_not_of(" \t\r\n"));
            line.erase(line.find_last_not_of(" \t\r\n") + 1);
            if (!line.empty()) {
                // 将二进制数据转换为16进制字符串
                std::string hex_data;
                hex_data.reserve(line.size() * 2);
                for (unsigned char c : line) {
                    char buf[3];
                    snprintf(buf, sizeof(buf), "%02X", c);
                    hex_data += buf;
                }
                // 移除at无效回执数据
                if(hex_data.substr(0, 4) == "4F4B" || hex_data.substr(0, 4) == "2B51") {
                    continue;
                }
                if(processBluetoothHexData(hex_data)) {
                    result = true;
                }
            }
        }
    }
    else {
        RCLCPP_ERROR(logger, "No response received");
    }
    return result;
}

bool BluetoothComponent::processBluetoothHexData(const std::string& hex_data) {  
    auto logger = rclcpp::get_logger("sensor::bluetooth");
    try {
        // 帧头 AA 55
        if (hex_data.substr(0, 4) != "AA55") {
            RCLCPP_ERROR(logger, "检验帧头失败 接收数据: %s", hex_data.c_str());
            return false;
        }
        // 获取数值长度
        int data_len = std::stoi(hex_data.substr(10, 2));
        // 检查数据长度是否正确
        if (12 + data_len * 2 + 4 != int(hex_data.size())) {
            RCLCPP_ERROR(logger, "检验数据长度失败 接收数据: %s", hex_data.c_str());
            return false;
        }
        // data 第一个字节 电压值
        std::string voltage_hex = hex_data.substr(12, 2);
        uint16_t voltage_uint = std::stoul(voltage_hex, nullptr, 16);
        int16_t voltage = static_cast<int16_t>(voltage_uint);
        // data 第二个字节 按键状态位
        std::string status_hex = hex_data.substr(14, 2);
        uint16_t status = std::stoul(status_hex, nullptr, 16);
        // data 第三个字节 充电状态位
        std::string charging_hex = hex_data.substr(16, 2);
        uint16_t charging = std::stoul(charging_hex, nullptr, 16);
        // data 第四个字节 保留位
        // std::string reserved_hex = hex_data.substr(18, 2);
        // uint16_t reserved = std::stoul(reserved_hex, nullptr, 16);
        // 案件状态 00未按下 false 01按下 true
        bool key_status = status & 0x0001; // 按键状态
        // 充电状态 00未充电 false 01充电中 true
        bool charging_status = charging & 0x0001; // 充电状态
        
        // 检查状态是否变化
        // bool status_changed = false;
        // if (last_status_->key_status != key_status ||
        //     last_status_->charging_status != charging_status) {
        //     status_changed = true;
        // }
        
        // if (status_changed) {
        //     // 发布BluetoothStatus消息
        //     auto msg = std::make_shared<interface::msg::BluetoothStatus>();
        //     msg->status = true; // 已连接
        //     msg->voltage = voltage;
        //     msg->key_status = key_status;
        //     msg->charging_status = charging_status;
        //     bluetooth_status_publisher_->publish(*msg);
            
        //     // 更新上一次的状态
        //     last_status_ = msg;
        // }
        auto msg = std::make_shared<interface::msg::BluetoothStatus>();
        msg->status = true; // 已连接
        msg->voltage = voltage;
        msg->key_status = key_status;
        msg->charging_status = charging_status;
        bluetooth_status_publisher_->publish(*msg);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool BluetoothComponent::loadMacAddressFromConfig() {
    auto logger = rclcpp::get_logger("sensor::bluetooth");
    
    std::ifstream config_file(config_file_path_);
    if (!config_file.is_open()) {
        RCLCPP_DEBUG(logger, "Config file not found: %s", config_file_path_.c_str());
        return false;
    }
    
    std::string line;
    if (std::getline(config_file, line)) {
        // 去除首尾空白字符
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if (!line.empty()) {
            saved_mac_address_ = line;
            return true;
        }
    }
    return false;
}

bool BluetoothComponent::saveMacAddressToConfig(const std::string& mac_address) {
    auto logger = rclcpp::get_logger("sensor::bluetooth");
    std::ofstream config_file(config_file_path_);
    if (!config_file.is_open()) {
        RCLCPP_ERROR(logger, "无法打开配置文件进行写入: %s", config_file_path_.c_str());
        return false;
    }
    
    config_file << mac_address << std::endl;
    config_file.close();
    
    RCLCPP_INFO(logger, "蓝牙MAC地址已保存到配置文件: %s", mac_address.c_str());
    return true;
}
