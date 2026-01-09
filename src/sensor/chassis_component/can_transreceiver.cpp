#include "can_transreceiver.h"
#include "sensor_utils.hpp"
#include "../global.h"
#include "interface/msg/hex_data.hpp"
#include "../../common/utils/error_event.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace sensor::chassis;
// 根据CAN设备名获取错误码
inline uint32_t getCanErrorCode(const std::string& can_dev) {
    if (can_dev == "can0") {
        return CAN0_DISCONNECTED;
    } else if (can_dev == "can1") {
        return CAN1_DISCONNECTED;
    } else if (can_dev == "can2") {
        return CAN2_DISCONNECTED;
    }
    return 0;
}

CanTransreceiver::CanTransreceiver(std::string dev, muduo::net::EventLoop* loop)
    : can_dev_(std::move(dev))
    , loop_(loop)
    , logger_(rclcpp::get_logger("CanTransreceiver"))
{
    signal(SIGPIPE, SIG_IGN);

    /* 打开套接字 */
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        throw std::runtime_error(sensor::utils::stringFormat("can socket create error [%d]%s", errno, strerror(errno)));
    }

    RCLCPP_INFO(logger_, "can socket %s create done!, fd %d", can_dev_.c_str(), fd);

    /* 指定 can0 设备 */
    struct ifreq ifr;
    bzero(&ifr, sizeof(ifr));

    struct sockaddr_can can_addr;
    bzero(&can_addr, sizeof(can_addr));

    strcpy(ifr.ifr_name, can_dev_.c_str());
    ioctl(fd, SIOCGIFINDEX, &ifr);
    can_addr.can_family = AF_CAN;
    can_addr.can_ifindex = ifr.ifr_ifindex;

    // int enable_candf = 1;
    // if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_candf, sizeof(enable_candf)) < 0) {
    //     close(fd);
    //     throw std::runtime_error(sensor::utils::stringFormat("can socket bind error [%d]%s", errno, strerror(errno)));
    // }

    /* 将 can0 与套接字进行绑定 */
    if (bind(fd, (struct sockaddr*)&can_addr, sizeof(can_addr)) < 0) {
        close(fd);
        uint32_t error_code = getCanErrorCode(can_dev_);
        interface::msg::HexData hex_msg;
        hex_msg.hex_value = encodeErrorLevel(error_code, ErrorEventLevel::OCCUR);
        auto* global = sensor::Global::getInstance();
        if (global) {
            global->sendHexData(hex_msg);
        }
        throw std::runtime_error(sensor::utils::stringFormat("can socket bind error [%d]%s", errno, strerror(errno)));
    }
    
    // 绑定成功，如果之前处于错误状态，现在恢复，发布恢复消息
    if (current_error_code_ != 0) {
        auto* global = sensor::Global::getInstance();
        if (global) {
            RCLCPP_INFO(logger_, "CAN %s recovered from error (0x%08X)", can_dev_.c_str(), current_error_code_);
            interface::msg::HexData recover_msg;
            recover_msg.hex_value = encodeErrorLevel(current_error_code_, ErrorEventLevel::RECOVER);
            global->sendHexData(recover_msg);
        }
        current_error_code_ = 0; // 清除错误状态
    }

    RCLCPP_INFO(logger_, "can socket %s bind done!", can_dev_.c_str());

    chnl_ = std::make_unique<muduo::net::Channel>(loop_, fd);
    chnl_->setReadCallback(std::bind(&CanTransreceiver::onDataReceived, this));
    RCLCPP_INFO(logger_, "can socket %s channel create done!", can_dev_.c_str());
}

CanTransreceiver::~CanTransreceiver()
{
    if (chnl_) {
        // 获取文件描述符（在销毁 Channel 之前）
        int fd = chnl_->fd();
        
        // 禁用所有事件监听并从 EventLoop 中移除
        // 注意：Channel 的 remove() 应该在 EventLoop 线程中调用
        if (loop_ && loop_->isInLoopThread()) {
            // 在 EventLoop 线程中，可以直接操作
            chnl_->disableAll();
            chnl_->remove();
        } else if (loop_) {
            // 如果不在 EventLoop 线程中，尝试同步移除
            // 注意：这可能需要 EventLoop 仍在运行
            chnl_->disableAll();
            // remove() 需要在 EventLoop 线程中调用，这里先禁用事件
            // 实际的移除会在 Channel 析构时由 muduo 处理
        }
        
        // 销毁 Channel（会从 EventLoop 中移除，如果还未移除的话）
        chnl_.reset();
        
        // 关闭文件描述符（Channel 不拥有 fd，需要手动关闭）
        if (fd >= 0) {
            close(fd);
            RCLCPP_INFO(logger_, "can socket %s fd %d closed", can_dev_.c_str(), fd);
        }
    }
    RCLCPP_INFO(logger_, "can socket %s destroy done!", can_dev_.c_str());
}

void CanTransreceiver::start()
{
    loop_->runInLoop([&] { chnl_->enableReading(); });
    RCLCPP_INFO(logger_, "can socket %s channel start enableReading!", can_dev_.c_str());
}

void CanTransreceiver::filter(std::map<unsigned uint16_t, unsigned int> canIdMap)
{
    struct can_filter rfilter[canIdMap.size()];
    int index = 0;
    for (auto& it : canIdMap) {
        rfilter[index].can_id = it.first | CAN_EFF_FLAG;
        rfilter[index].can_mask = it.second | CAN_EFF_FLAG;
        index++;
    }
    if (setsockopt(chnl_->fd(), SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
        throw std::runtime_error(sensor::utils::stringFormat("can socket bind error [%d]%s", errno, strerror(errno)));
    }
}

void CanTransreceiver::filter()
{
}

void CanTransreceiver::send(int frameid, muduo::net::Buffer& buf)
{
    struct can_frame frame;
    bzero(&frame, sizeof(frame));
    frame.can_id = frameid;
    frame.can_dlc = buf.readableBytes();
    const char* data = buf.peek();
    size_t len = buf.readableBytes();
    for (size_t i = 0; i < len; ++i) {
        frame.data[i] = static_cast<unsigned char>(data[i]);
    }
    if (sizeof(frame) == write(chnl_->fd(), &frame, sizeof(frame))) { // 如果 ret 不等于帧长度，就说明发送失败
        RCLCPP_DEBUG(logger_, "can socket %s send fid: %04x, fsz:%d done!", can_dev_.c_str(), frame.can_id, frame.can_dlc);
        
        // 发送成功，如果之前处于错误状态，现在恢复，发布恢复消息
        if (current_error_code_ != 0) {
            auto* global = sensor::Global::getInstance();
            if (global) {
                RCLCPP_INFO(logger_, "CAN %s recovered from error (0x%08X)", can_dev_.c_str(), current_error_code_);
                interface::msg::HexData recover_msg;
                recover_msg.hex_value = encodeErrorLevel(current_error_code_, ErrorEventLevel::RECOVER);
                global->sendHexData(recover_msg);
            }
            current_error_code_ = 0; // 清除错误状态
        }
    } else {
        RCLCPP_WARN(logger_, "can socket %s write error [%d]%s send fid: %04x, fsz:%d done!", can_dev_.c_str(), errno, strerror(errno), frame.can_id, frame.can_dlc);
        // 上报 CAN 连接异常
        uint32_t error_code = getCanErrorCode(can_dev_);
        interface::msg::HexData hex_msg;
        hex_msg.hex_value = encodeErrorLevel(error_code, ErrorEventLevel::OCCUR);
        auto* global = sensor::Global::getInstance();
        if (global) {
            global->sendHexData(hex_msg);
        }
        current_error_code_ = error_code; // 记录错误状态
    }
}

void CanTransreceiver::sendLoog(int frameid)
{
    struct can_frame frame;
    frame.can_id = frameid; // 设置目标CAN ID
    frame.can_dlc = 0; // 数据长度为0（RTR帧特征）
    frame.can_id |= CAN_RTR_FLAG; // 设置RTR标志
    if (sizeof(frame) == write(chnl_->fd(), &frame, sizeof(frame))) { // 如果 ret 不等于帧长度，就说明发送失败
        RCLCPP_DEBUG(logger_, "can socket %s send fid: %04x, fsz:%d done!", can_dev_.c_str(), frame.can_id, frame.can_dlc);
        
        // 发送成功，如果之前处于错误状态，现在恢复，发布恢复消息
        if (current_error_code_ != 0) {
            auto* global = sensor::Global::getInstance();
            if (global) {
                RCLCPP_INFO(logger_, "CAN %s recovered from error (0x%08X)", can_dev_.c_str(), current_error_code_);
                interface::msg::HexData recover_msg;
                recover_msg.hex_value = encodeErrorLevel(current_error_code_, ErrorEventLevel::RECOVER);
                global->sendHexData(recover_msg);
            }
            current_error_code_ = 0; // 清除错误状态
        }
    } else {
        RCLCPP_WARN(logger_, "can socket %s write error [%d]%s send fid: %04x, fsz:%d done!", can_dev_.c_str(), errno, strerror(errno), frame.can_id, frame.can_dlc);
        // 上报 CAN 连接异常
        uint32_t error_code = getCanErrorCode(can_dev_);
        interface::msg::HexData hex_msg;
        hex_msg.hex_value = encodeErrorLevel(error_code, ErrorEventLevel::OCCUR);
        auto* global = sensor::Global::getInstance();
        if (global) {
            global->sendHexData(hex_msg);
        }
        current_error_code_ = error_code; // 记录错误状态
    }
}

void CanTransreceiver::setReceiveCallback(CanReceiveCallback cb)
{
    slots_.insert(signal_.connect(std::move(cb)));
}

void CanTransreceiver::onDataReceived()
{
    struct canfd_frame frame;

    int ret = read(chnl_->fd(), &frame, sizeof(struct canfd_frame));
    if (ret < 0) {
        RCLCPP_WARN(logger_, "can socket %s reade error [%d]%s", can_dev_.c_str(), errno, strerror(errno));
        // 上报 CAN 连接异常
        uint32_t error_code = getCanErrorCode(can_dev_);
        interface::msg::HexData hex_msg;
        hex_msg.hex_value = encodeErrorLevel(error_code, ErrorEventLevel::OCCUR);
        auto* global = sensor::Global::getInstance();
        if (global) {
            global->sendHexData(hex_msg);
        }
        current_error_code_ = error_code; // 记录错误状态
        return;
    }
    
    // 读取成功，如果之前处于错误状态，现在恢复，发布恢复消息
    if (current_error_code_ != 0) {
        auto* global = sensor::Global::getInstance();
        if (global) {
            RCLCPP_INFO(logger_, "CAN %s recovered from error (0x%08X)", can_dev_.c_str(), current_error_code_);
            interface::msg::HexData recover_msg;
            recover_msg.hex_value = encodeErrorLevel(current_error_code_, ErrorEventLevel::RECOVER);
            global->sendHexData(recover_msg);
        }
        current_error_code_ = 0; // 清除错误状态
    }

    muduo::net::Buffer buffer;
    for (size_t i = 0; i < frame.len; i++) {
        buffer.appendInt8(frame.data[i]);
    }
    // if (can_dev_ == "can0") {
    signal_.call(frame.can_id & CAN_EFF_MASK, buffer);
    // RCLCPP_INFO(logger_, "can socket %s recv fid: %05x : %05x, fsz:%d  done!", can_dev_.c_str(), (frame.can_id & CAN_EFF_MASK), frame.can_id, frame.len);
    //  } else {
    //      signal_.call(frame.can_id, buffer);
    //      RCLCPP_INFO(logger_, "can socket %s recv fid: %05x, fsz:%d  done!", can_dev_.c_str(), (frame.can_id), frame.len);
    //  }
}
