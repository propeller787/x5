#pragma once

#include <condition_variable>
#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <string>

namespace tft_service {

/**
 * @brief TFT 命令结构
 */
struct TftCommand {
    enum class Type {
        PAGE_ID,      // 发送页面ID
        ADDRESS_DATA, // 发送地址数据
        TEXT_DATA,    // 发送文本数据
        ELECTRIC_INDEX, // 发送电量索引
        READ_REGISTER, // 读取寄存器
        RAW_DATA      // 原始数据
    };

    Type type;
    std::vector<uint8_t> data; // 命令数据
    int16_t param1 = 0;        // 参数1（页面ID、地址等）
    int16_t param2 = 0;        // 参数2（数据值、x坐标等）
    int16_t param3 = 0;        // 参数3（y坐标等）
    std::string text;           // 文本数据

    // 默认构造用于队列线程占位
    TftCommand() : type(Type::RAW_DATA) {}
    TftCommand(Type t) : type(t) {}
};

/**
 * @brief TFT 命令队列，用于异步发送命令
 * 移除阻塞式 usleep()，使用工作线程异步处理命令
 */
class TftCommandQueue {
public:
    TftCommandQueue(std::shared_ptr<class TftModule> tft_module);
    ~TftCommandQueue();

    /**
     * @brief 启动命令队列工作线程
     */
    void start();

    /**
     * @brief 停止命令队列工作线程
     */
    void stop();

    /**
     * @brief 将命令加入队列
     * @param cmd 命令对象
     */
    void enqueue(const TftCommand& cmd);

    /**
     * @brief 将命令加入队列（移动语义，更高效）
     * @param cmd 命令对象
     */
    void enqueue(TftCommand&& cmd);

    /**
     * @brief 清空队列
     */
    void clear();

    /**
     * @brief 获取队列大小
     */
    size_t size() const;

private:
    /**
     * @brief 工作线程函数，处理队列中的命令
     */
    void workerThread();

    /**
     * @brief 处理单个命令
     * @param cmd 命令对象
     */
    void processCommand(const TftCommand& cmd);

    /**
     * @brief 命令之间的延迟（微秒）
     */
    static constexpr int COMMAND_DELAY_US = 100;

private:
    std::shared_ptr<class TftModule> tft_module_;
    std::queue<TftCommand> queue_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
};

} // namespace tft_service

