#include "tft_command_queue.h"
#include "tft_module.h"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

namespace tft_service {

TftCommandQueue::TftCommandQueue(std::shared_ptr<TftModule> tft_module)
    : tft_module_(tft_module)
{
}

TftCommandQueue::~TftCommandQueue()
{
    stop();
}

void TftCommandQueue::start()
{
    if (running_.load()) {
        RCLCPP_WARN(rclcpp::get_logger("TftCommandQueue"), "Command queue is already running");
        return;
    }

    running_.store(true);
    stop_requested_.store(false);

    worker_thread_ = std::thread(&TftCommandQueue::workerThread, this);
    RCLCPP_INFO(rclcpp::get_logger("TftCommandQueue"), "TFT command queue started");
}

void TftCommandQueue::stop()
{
    if (!running_.load()) {
        return;
    }

    stop_requested_.store(true);
    queue_cv_.notify_all();

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    running_.store(false);
    RCLCPP_INFO(rclcpp::get_logger("TftCommandQueue"), "TFT command queue stopped");
}

void TftCommandQueue::enqueue(const TftCommand& cmd)
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.push(cmd);
    }
    queue_cv_.notify_one();
}

void TftCommandQueue::enqueue(TftCommand&& cmd)
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.push(std::move(cmd));
    }
    queue_cv_.notify_one();
}

void TftCommandQueue::clear()
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    std::queue<TftCommand> empty;
    queue_.swap(empty);
}

size_t TftCommandQueue::size() const
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return queue_.size();
}

void TftCommandQueue::workerThread()
{
    while (running_.load() && !stop_requested_.load()) {
        TftCommand cmd;
        bool has_command = false;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            // 等待命令或停止信号
            queue_cv_.wait(lock, [this] {
                return !queue_.empty() || stop_requested_.load();
            });

            if (stop_requested_.load() && queue_.empty()) {
                break;
            }

            if (!queue_.empty()) {
                cmd = std::move(queue_.front());
                queue_.pop();
                has_command = true;
            }
        }

        if (has_command) {
            processCommand(cmd);
            // 命令之间的延迟，避免串口过载
            usleep(COMMAND_DELAY_US);
        }
    }
}

void TftCommandQueue::processCommand(const TftCommand& cmd)
{
    if (!tft_module_) {
        RCLCPP_ERROR(rclcpp::get_logger("TftCommandQueue"), "TFT module is null, cannot process command");
        return;
    }

    switch (cmd.type) {
    case TftCommand::Type::PAGE_ID: {
        // 构造页面ID命令：5AA5078200845A010001
        std::vector<uint8_t> buf(10);
        buf[0] = 0x5A;
        buf[1] = 0xA5;
        buf[2] = 0x07;
        buf[3] = 0x82;
        buf[4] = 0x00;
        buf[5] = 0x84;
        buf[6] = 0x5A;
        buf[7] = 0x01;
        buf[8] = (cmd.param1 >> 8) & 0xFF;
        buf[9] = cmd.param1 & 0xFF;
        tft_module_->startWrite(reinterpret_cast<char*>(buf.data()), buf.size());
        break;
    }
    case TftCommand::Type::ADDRESS_DATA: {
        // 构造地址数据命令：5AA5 05 82 1000 0002
        std::vector<uint8_t> buf(8);
        buf[0] = 0x5A;
        buf[1] = 0xA5;
        buf[2] = 0x05;
        buf[3] = 0x82;
        buf[4] = (cmd.param1 >> 8) & 0xFF; // 地址高字节
        buf[5] = cmd.param1 & 0xFF;        // 地址低字节
        buf[6] = (cmd.param2 >> 8) & 0xFF; // 数据高字节
        buf[7] = cmd.param2 & 0xFF;        // 数据低字节
        tft_module_->startWrite(reinterpret_cast<char*>(buf.data()), buf.size());
        break;
    }
    case TftCommand::Type::TEXT_DATA: {
        // 构造文本数据命令
        size_t total_length = 4 + 2 + cmd.text.length();
        std::vector<uint8_t> buf(total_length);
        buf[0] = 0x5A;
        buf[1] = 0xA5;
        buf[2] = total_length - 3;
        buf[3] = 0x82;
        buf[4] = (cmd.param1 >> 8) & 0xFF;
        buf[5] = cmd.param1 & 0xFF;
        for (size_t i = 0; i < cmd.text.length(); i++) {
            buf[6 + i] = cmd.text[i];
        }
        tft_module_->startWrite(reinterpret_cast<char*>(buf.data()), buf.size());
        break;
    }
    case TftCommand::Type::ELECTRIC_INDEX: {
        // 构造电量索引命令：5AA5 07 82 8001 0064 0064
        std::vector<uint8_t> buf(10);
        buf[0] = 0x5A;
        buf[1] = 0xA5;
        buf[2] = 0x07;
        buf[3] = 0x82;
        buf[4] = (cmd.param1 >> 8) & 0xFF;
        buf[5] = cmd.param1 & 0xFF;
        buf[6] = (cmd.param2 >> 8) & 0xFF;
        buf[7] = cmd.param2 & 0xFF;
        buf[8] = (cmd.param3 >> 8) & 0xFF;
        buf[9] = cmd.param3 & 0xFF;
        tft_module_->startWrite(reinterpret_cast<char*>(buf.data()), buf.size());
        break;
    }
    case TftCommand::Type::RAW_DATA: {
        // 直接发送原始数据
        if (!cmd.data.empty()) {
            tft_module_->startWrite(reinterpret_cast<const char*>(cmd.data.data()), cmd.data.size());
        }
        break;
    }
    default:
        RCLCPP_WARN(rclcpp::get_logger("TftCommandQueue"), "Unknown command type: %d", static_cast<int>(cmd.type));
        break;
    }
}

} // namespace tft_service

