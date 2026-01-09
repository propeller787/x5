#include "global.h"
#include <arpa/inet.h>
#include <cstdlib>
#include <fcntl.h>
#include <iomanip>
#include <netdb.h>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>
#include <utils/topic.h>
using std::placeholders::_1;
namespace tft_service {

// 辅助函数：将数据转换为16进制字符串
std::string bytesToHexString(const char *data, size_t length) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < length; ++i) {
        ss << std::setw(2) << static_cast<int>(static_cast<unsigned char>(data[i]));
        if (i < length - 1) {
            ss << " ";
        }
    }
    return ss.str();
}
Global *Global::instance = nullptr;
std::mutex Global::mtx;
Global::Global(void) {
    node_ = std::make_shared<rclcpp::Node>("tftservice");
    tft_module_ = std::make_shared<TftModule>("/dev/ttyS1");
    command_queue_ = std::make_shared<TftCommandQueue>(tft_module_);
    command_queue_->start(); // 启动命令队列工作线程
    date_buf_ = getCurrentDateString();
    named_status_pub_ = node_->create_publisher<interface::msg::NamedStatus>(tftModeTopic, 10);

    // 启动网络检测
    startNetworkCheck();
}

Global::~Global() {
    // 停止命令队列
    if (command_queue_) {
        command_queue_->stop();
    }
    // 停止网络检测线程
    stopNetworkCheck();
}

std::string Global::getCurrentDateString() {
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    // 转换为 time_t
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    // 转换为 tm 结构体
    std::tm *local_tm = std::localtime(&now_time);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d/%02d/%02d", local_tm->tm_year + 1900, local_tm->tm_mon + 1, local_tm->tm_mday);
    date_general_buf_ = std::string(buf);
    std::string result{""};
    switch (la_type_) {
    case LINGUISTIC_TYPE::CN: {
        // 中文格式：2025年1月1日 周三
        const char *week_days_cn[] = {"周日", "周一", "周二", "周三", "周四", "周五", "周六"};

        // 构造日期字符串
        char date_buf[64];
        std::strftime(date_buf, sizeof(date_buf), "%Y年%m月%d日", local_tm);

        // 拼接最终字符串
        result = std::string(date_buf) + " " + week_days_cn[local_tm->tm_wday];
        break;
    }
    case LINGUISTIC_TYPE::EN: {
        // 英文格式：Wednesday, April 20
        const char *week_days_en[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
        const char *months_en[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sept", "Oct", "Nov", "Dec"};
        std::stringstream ss;
        ss << std::setw(2) << std::setfill('0') << local_tm->tm_mday;
        std::string day_str = ss.str();
        // 构造日期字符串
        result = week_days_en[local_tm->tm_wday] + std::string(",") + months_en[local_tm->tm_mon] + "." + day_str;
        break;
    }
    default:
        break;
    }
    return result;
}

std::string Global::getCurrentTimeString() {
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    // 转换为 time_t
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    // 转换为 tm 结构体
    std::tm *local_tm = std::localtime(&now_time);
    // 格式：12 : 30
    char time_buf[16];
    std::strftime(time_buf, sizeof(time_buf), "%H:%M", local_tm);

    return std::string(time_buf);
}

void Global::setElectricQuantity(int16_t electricquantity) {
    if (electricquantity == electricquantity_) {
        return; // 状态未改变，不需要发布
    }
    electricquantity_ = electricquantity;
    sendAddressData(BATTERY_LEVEL_FRAME_ID, electricquantity);
    if (electricquantity >= 10 && electricquantity < 100) {
        sendElectricIndex(LOW_BATTERY_LEVEL_FRAME_ID_3, ELECTRIC_INDEX_2, ELECTRIC_Y_2);
    }
    if (electricquantity < 10) {
        sendElectricIndex(LOW_BATTERY_LEVEL_FRAME_ID_3, ELECTRIC_INDEX_1, ELECTRIC_Y_1);
    }
    if (electricquantity == 100) {
        sendElectricIndex(LOW_BATTERY_LEVEL_FRAME_ID_3, ELECTRIC_INDEX_3, ELECTRIC_INDEX_3);
    }
    if (electricquantity < 20) {
        sendAddressData(LOW_BATTERY_LEVEL_FRAME_ID, RED_COLOR);
    }
}
void Global::setHealthStatus(bool status) {
    if (status == health_status_) {
        return; // 状态未改变，不需要发布
    }
    health_status_ = status;
    interface::msg::NamedStatus status_msg;
    status_msg.name = "health";
    status_msg.status = health_status_ ? 1 : 0;
    named_status_pub_->publish(status_msg);
}

void Global::setCruiseStatus(bool status) {
    if (status == cruise_status_) {
        return; // 状态未改变，不需要发布
    }
    cruise_status_ = status;
    interface::msg::NamedStatus status_msg;
    status_msg.name = "cruise";
    status_msg.status = cruise_status_ ? 1 : 0;
    named_status_pub_->publish(status_msg);
}

void Global::setAutomaticRecharge(bool status) {
    if (status == automatic_recharge_) {
        return; // 状态未改变，不需要发布
    }
    automatic_recharge_ = status;
    interface::msg::NamedStatus status_msg;
    status_msg.name = "automatic_recharge";
    status_msg.status = automatic_recharge_ ? 1 : 0;
    named_status_pub_->publish(status_msg);
}

void Global::setSafeMode(bool status) {
    if (status == safe_mode_) {
        return; // 状态未改变，不需要发布
    }
    safe_mode_ = status;
    interface::msg::NamedStatus status_msg;
    status_msg.name = "safe_mode";
    status_msg.status = safe_mode_ ? 1 : 0;
    named_status_pub_->publish(status_msg);
}

void Global::setSetting(bool status) {
    if (status == setting_) {
        return; // 状态未改变，不需要发布
    }
    setting_ = status;
    interface::msg::NamedStatus status_msg;
    status_msg.name = "setting";
    status_msg.status = setting_ ? 1 : 0;
    named_status_pub_->publish(status_msg);
}

// 网络检测相关方法实现
void Global::setNetworkStatus(bool status) {
    if (status == network_status_) {
        return; // 状态未改变，不需要发布
    }
    network_status_ = status;
    interface::msg::NamedStatus status_msg;
    status_msg.name = "network";
    status_msg.status = network_status_ ? 1 : 0;
    named_status_pub_->publish(status_msg);

    // 输出网络状态变化日志
    if (network_status_) {
        RCLCPP_INFO(node_->get_logger(), "网络连接已恢复");
    } else {
        RCLCPP_WARN(node_->get_logger(), "网络连接已断开");
    }
}

void Global::startNetworkCheck() {
    if (network_check_running_.load()) {
        RCLCPP_INFO(node_->get_logger(), "网络检测已在运行中");
        return;
    }

    network_check_running_.store(true);
    network_check_stop_requested_.store(false);

    network_check_thread_ = std::thread([this]() {
        RCLCPP_INFO(node_->get_logger(), "网络检测线程已启动，每%d秒检测一次", NETWORK_CHECK_INTERVAL);

        while (network_check_running_.load() && !network_check_stop_requested_.load()) {
            bool current_network_status = checkNetworkConnectivity();
            setNetworkStatus(current_network_status);

            // 等待指定的检测间隔
            std::this_thread::sleep_for(std::chrono::seconds(NETWORK_CHECK_INTERVAL));
        }

        RCLCPP_INFO(node_->get_logger(), "网络检测线程已停止");
    });
}

void Global::stopNetworkCheck() {
    if (!network_check_running_.load()) {
        return;
    }

    network_check_stop_requested_.store(true);

    if (network_check_thread_.joinable()) {
        network_check_thread_.join();
    }

    network_check_running_.store(false);
    network_check_stop_requested_.store(false);
    RCLCPP_INFO(node_->get_logger(), "网络检测已停止");
}

bool Global::checkNetworkConnectivity() {
    // 方法1：尝试连接到一个可靠的服务器（如8.8.8.8:53 DNS服务器）
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        return false;
    }

    // 设置非阻塞模式
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(53);                   // DNS端口
    server_addr.sin_addr.s_addr = inet_addr("8.8.8.8"); // Google DNS

    // 尝试连接
    int result = connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    if (result < 0 && errno == EINPROGRESS) {
        // 连接正在进行中，等待连接完成
        fd_set write_fds;
        struct timeval timeout;

        FD_ZERO(&write_fds);
        FD_SET(sock, &write_fds);
        timeout.tv_sec = 3; // 3秒超时
        timeout.tv_usec = 0;

        result = select(sock + 1, NULL, &write_fds, NULL, &timeout);

        if (result > 0 && FD_ISSET(sock, &write_fds)) {
            // 检查连接是否成功
            int error = 0;
            socklen_t len = sizeof(error);
            if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len) == 0 && error == 0) {
                close(sock);
                return true; // 连接成功
            }
        }
    }

    close(sock);

    return false;
}

void Global::sendPageId(int16_t id) {
    if (id != current_page_id_) {
        current_page_id_ = id; // 更新上个页面ID
        last_page_id_ = current_page_id_;
    } else {
        return; // 如果页面ID没有变化，则不发送
    }
    current_page_id_ = id; // 更新当前页面ID
    
    // 使用命令队列异步发送
    TftCommand cmd(TftCommand::Type::PAGE_ID);
    cmd.param1 = id;
    command_queue_->enqueue(std::move(cmd));
    
    RCLCPP_DEBUG(node_->get_logger(), "sendPageId - 页面ID: %d", id);
}

void Global::sendAddressData(uint16_t address, int16_t data) {
    // 使用命令队列异步发送
    TftCommand cmd(TftCommand::Type::ADDRESS_DATA);
    cmd.param1 = address;
    cmd.param2 = data;
    command_queue_->enqueue(std::move(cmd));
    
    RCLCPP_DEBUG(node_->get_logger(), "sendAddressData - 地址: 0x%04X, 数据: %d", address, data);
}

void Global::sendTextData(uint16_t address, std::string text) {
    // 使用命令队列异步发送
    TftCommand cmd(TftCommand::Type::TEXT_DATA);
    cmd.param1 = address;
    cmd.text = text; // 先复制用于日志
    command_queue_->enqueue(std::move(cmd));
    
    RCLCPP_DEBUG(node_->get_logger(), "sendTextData - 地址: 0x%04X, 文本: '%s'", address, text.c_str());
}

void Global::sendTextDemo() {
    // 5A A5 09 82 6100 B1B1 3132 4142
    std::vector<char> buf(12);
    buf[0] = 0x5A;
    buf[1] = 0xA5;
    buf[2] = 0x09;
    buf[3] = 0x82;
    buf[4] = 0x61;
    buf[5] = 0x00;
    buf[6] = 0xB1;
    buf[7] = 0xB1;
    buf[8] = 0x31;
    buf[9] = 0x32;
    buf[10] = 0x41;
    buf[11] = 0x42;
    tft_module_->startWrite(buf.data(), 12);
}

void Global::clearText() {
    // 5A A5 07 82 6100 B1B1 FFFF
    std::vector<char> buf(8);
    buf[0] = 0x5A;
    buf[1] = 0xA5;
    buf[2] = 0x07;
    buf[3] = 0x82;
    buf[4] = 0x61;
    buf[5] = 0x00;
    buf[6] = 0x20;
    buf[7] = 0x20;
    buf[8] = 0x20;
    buf[9] = 0x20;
    buf[10] = 0x20;
    buf[11] = 0x20;
    buf[12] = 0x20;
    buf[13] = 0x20;
    tft_module_->startWrite(buf.data(), 14);
}

void Global::sendDate() { sendTextData(PARKING_DATE_FRAME_ID, date_buf_); }

void Global::sendTime() {
    time_buf_ = getCurrentTimeString();
    sendTextData(PARKING_TIME_FRAME_ID, time_buf_);
}

std::shared_ptr<TftModule> Global::getTftModule() { return tft_module_; }
std::shared_ptr<TftCommandQueue> Global::getCommandQueue() { return command_queue_; }

void Global::sendNetworkStatus() {
    if (network_status_) {
        sendAddressData(NETWORK_FRAME_ID, 2); // 网络连接正常
    } else {
        sendAddressData(NETWORK_FRAME_ID, 1); // 网络连接异常
    }
}

void Global::sendPageHeader() {
    // 发送日期
    sendTextData(TIME_FRAME_ID, date_general_buf_);
    // 发送时间
    sendTextData(PARKING_TIME_FRAME_ID, getCurrentTimeString());
    // 发送电量
    // sendAddressData(BATTERY_LEVEL_FRAME_ID, electricquantity_);
}
// 改变数据坐标，将变量位移到（100，100）起始显示坐标点。
//  5AA5 07 82 8001 0064 0064
//  0x5AA5：帧头；
//  0x07：数据长度；
//  0x82：写指令；
//  0x8001：描述指针 0x8000 地址基础上加上 1 个地址作为起始地址，改变变量坐标；
//  0x0064 0064：将控件左上角起始显示坐标移到（100，100）坐标点。
void Global::sendElectricIndex(int16_t index, int16_t x, int16_t y) {
    // 使用命令队列异步发送
    TftCommand cmd(TftCommand::Type::ELECTRIC_INDEX);
    cmd.param1 = index;
    cmd.param2 = x;
    cmd.param3 = y;
    command_queue_->enqueue(std::move(cmd));
}

} // namespace tft_service
