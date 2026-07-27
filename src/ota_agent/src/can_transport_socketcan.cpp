#include "ota_agent/can_transport_socketcan.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace ota_agent {

SocketCanTransport::SocketCanTransport(const std::string& ifname, rclcpp::Logger logger)
    : ifname_(ifname), logger_(logger) {
    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        RCLCPP_ERROR(logger_, "SocketCAN 打开socket失败");
        return;
    }
    int err_mask = CAN_ERR_MASK;
    (void)setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

    struct ifreq ifr{};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname_.c_str());
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(logger_, "SocketCAN 获取接口索引失败: %s", ifname_.c_str());
        ::close(sock_); sock_ = -1; return;
    }
    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(logger_, "SocketCAN 绑定接口失败: %s", ifname_.c_str());
        ::close(sock_); sock_ = -1; return;
    }
    running_ = true;
    rx_thread_ = std::thread([this]() { rx_loop(); });
}

SocketCanTransport::~SocketCanTransport() {
    close();
}

bool SocketCanTransport::send(const CanFrame& frame, std::string& err) {
    if (sock_ < 0) { err = "socket not ready"; return false; }
    const uint32_t& can_id = frame.id;
    const std::vector<uint8_t>& data = frame.data;
    struct can_frame cf{};

    bool is_extended = (can_id & CAN_EFF_FLAG) != 0;
    if (is_extended) {
        // 扩展帧：29位ID，提取实际ID并设置标志位
        uint32_t actual_id = can_id & CAN_EFF_MASK;
        cf.can_id = actual_id | CAN_EFF_FLAG;
    } else {
        // 标准帧：11位ID
        cf.can_id = can_id & CAN_SFF_MASK;
    }
    
    cf.can_dlc = std::min<uint8_t>(static_cast<uint8_t>(data.size()), Constants::CAN_FRAME_MAX_DLC);
    for (int i = 0; i < cf.can_dlc; ++i) {
        cf.data[i] = data[i];
    }
    
    // 重试机制
    ssize_t n = 0;
    for (int retry = 0; retry < Constants::CAN_SEND_MAX_RETRIES; ++retry) {
        n = ::write(sock_, &cf, sizeof(cf));
        if (n == sizeof(cf)) {
            break;
        }
        
        if (retry < Constants::CAN_SEND_MAX_RETRIES - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(Constants::CAN_SEND_RETRY_DELAY_MS));
        } else {
            if (n < 0) {
                std::ostringstream oss;
                oss << "CAN 发送失败 (已重试" << Constants::CAN_SEND_MAX_RETRIES << "次): " << strerror(errno);
                err = oss.str();
            } else {
                std::ostringstream oss;
                oss << "CAN 发送不完整 (已重试" << Constants::CAN_SEND_MAX_RETRIES << "次): 期望" << sizeof(cf) << "字节，实际" << n << "字节";
                err = oss.str();
            }
            return false;
        }
    }
    return true;
}

void SocketCanTransport::set_receive_handler(std::function<void(const CanFrame&)> cb) {
    rx_cb_ = std::move(cb);
}
void SocketCanTransport::set_receive_handler(std::function<void(uint32_t can_id, const std::vector<uint8_t>& data)> cb){
    std::lock_guard<std::mutex> lk(rx_cb_mutex_);
    if (cb){
        rx_cb_wrapper_ =std::make_shared<std::function<void(uint32_t, const std::vector<uint8_t>&)>>(std::move(cb));
    } else {
        rx_cb_wrapper_ = nullptr;
    }
    
}

void SocketCanTransport::close() {
    // 幂等关闭：停止接收线程并释放 socket
    running_ = false;
    if (sock_ >= 0) {
        ::shutdown(sock_, SHUT_RDWR);
        ::close(sock_);
        sock_ = -1;
    }
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
}

void SocketCanTransport::rx_loop() {
    while (running_) {
        struct can_frame cf{};
        ssize_t n = ::read(sock_, &cf, sizeof(cf));
        if (n == sizeof(cf)) {
            // 检测是否为扩展帧
            bool is_extended = (cf.can_id & CAN_EFF_FLAG) != 0;
            CanFrame frame;
            frame.id = is_extended ? (cf.can_id & CAN_EFF_MASK) : (cf.can_id & CAN_SFF_MASK);
            frame.data.assign(cf.data, cf.data + cf.can_dlc);
            // 处理接收到的帧
            std::shared_ptr<std::function<void(uint32_t, const std::vector<uint8_t>&)>> local_cb_wrapper;
            {
                std::lock_guard<std::mutex> lk(rx_cb_mutex_);
                local_cb_wrapper = rx_cb_wrapper_;
            }
            if (local_cb_wrapper && *local_cb_wrapper) {
                (*local_cb_wrapper)(frame.id, frame.data);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(Constants::CAN_RX_LOOP_DELAY_MS));
        }
    }
} 
}// namespace ota_agent