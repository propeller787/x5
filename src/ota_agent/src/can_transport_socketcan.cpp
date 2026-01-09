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
    struct can_frame cf{};
    cf.can_id = frame.id & can_id_mask_;
    if (frame.is_extended) cf.can_id |= CAN_EFF_FLAG;
    cf.can_dlc = std::min<uint8_t>(frame.dlc, 8);
    for (int i = 0; i < cf.can_dlc; ++i) cf.data[i] = frame.data[i];
    ssize_t n = ::write(sock_, &cf, sizeof(cf));
    if (n != sizeof(cf)) { err = "can write failed"; return false; }
    // 额外调试：记录实际写出的帧（DEBUG级别）
    {
        std::ostringstream oss;
        oss << std::hex << std::uppercase;
        for (int i = 0; i < cf.can_dlc; ++i) {
            oss << std::setw(2) << std::setfill('0') << static_cast<int>(cf.data[i]) << ' ';
        }
        RCLCPP_DEBUG(logger_, "[SocketCAN-TX] id=0x%X ext=%d dlc=%u data=[%s]",
                    (cf.can_id & CAN_EFF_FLAG) ? (cf.can_id & CAN_EFF_MASK) : (cf.can_id & CAN_SFF_MASK),
                    (cf.can_id & CAN_EFF_FLAG) ? 1 : 0,
                    cf.can_dlc,
                    oss.str().c_str());
    }
    return true;
}

void SocketCanTransport::set_receive_handler(std::function<void(const CanFrame&)> cb) {
    rx_cb_ = std::move(cb);
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
            // 打印收到的帧
            std::ostringstream oss;
            oss << std::hex << std::uppercase;
            for (int i = 0; i < cf.can_dlc; ++i) {
                oss << std::setw(2) << std::setfill('0') << static_cast<int>(cf.data[i]) << ' ';
            }
            uint32_t rid = (cf.can_id & CAN_EFF_FLAG) ? (cf.can_id & CAN_EFF_MASK) : (cf.can_id & CAN_SFF_MASK);
            RCLCPP_DEBUG(logger_, "[SocketCAN-RX] id=0x%X ext=%d dlc=%u data=[%s]",
                        rid,
                        (cf.can_id & CAN_EFF_FLAG) ? 1 : 0,
                        cf.can_dlc,
                        oss.str().c_str());

            if (rx_cb_) {
            if (cf.can_id & CAN_ERR_FLAG) {
                RCLCPP_ERROR(logger_, "[CAN-ERR] can_id=0x%X", cf.can_id);
                continue;
            }
            CanFrame f;
            f.id = (cf.can_id & CAN_EFF_FLAG) ? (cf.can_id & CAN_EFF_MASK) : (cf.can_id & CAN_SFF_MASK);
            f.is_extended = (cf.can_id & CAN_EFF_FLAG);
            f.dlc = cf.can_dlc;
            f.data.assign(cf.data, cf.data + cf.can_dlc);
            rx_cb_(f);
            }
        } else {
            std::this_thread::sleep_for(5ms);
        }
    }
}

} // namespace ota_agent
