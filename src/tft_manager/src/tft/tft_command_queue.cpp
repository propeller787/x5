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

    const uint8_t station = tft_module_->stationAddr();

    switch (cmd.type) {
    case TftCommand::Type::PAGE_ID: {
        // 关键字 0x5A01 + 页面ID 写入寄存器 0x0084
        std::vector<uint8_t> payload{
            0x5A, 0x01,
            static_cast<uint8_t>((cmd.param1 >> 8) & 0xFF),
            static_cast<uint8_t>(cmd.param1 & 0xFF)};
        tft_module_->sendDcBusWritePayload(station, 0x0084, payload);
        break;
    }
    case TftCommand::Type::ADDRESS_DATA: {
        // 固定写入寄存器 0x1000，下发电量/数值
        std::vector<uint8_t> payload{
            static_cast<uint8_t>((cmd.param2 >> 8) & 0xFF),
            static_cast<uint8_t>(cmd.param2 & 0xFF)};
        tft_module_->sendDcBusWritePayload(station, 0x1000, payload);
        break;
    }
    case TftCommand::Type::READ_REGISTER: {
        // 读寄存器，param1=寄存器，param2=读取数量
        const uint16_t reg = static_cast<uint16_t>(cmd.param1);
        const uint8_t count = static_cast<uint8_t>(cmd.param2 & 0xFF);
        tft_module_->sendDcBusRead(station, reg, count);
        break;
    }
    case TftCommand::Type::TEXT_DATA: {
        std::vector<uint8_t> payload(cmd.text.begin(), cmd.text.end());
        tft_module_->sendDcBusWritePayload(station, static_cast<uint16_t>(cmd.param1), payload);
        break;
    }
    case TftCommand::Type::ELECTRIC_INDEX: {
        std::vector<uint8_t> payload{
            static_cast<uint8_t>((cmd.param2 >> 8) & 0xFF), static_cast<uint8_t>(cmd.param2 & 0xFF),
            static_cast<uint8_t>((cmd.param3 >> 8) & 0xFF), static_cast<uint8_t>(cmd.param3 & 0xFF)};
        tft_module_->sendDcBusWritePayload(station, static_cast<uint16_t>(cmd.param1), payload);
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

