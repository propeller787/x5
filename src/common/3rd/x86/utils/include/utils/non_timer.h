#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
namespace utils {
// 定义定时器类
class NonBlockingTimer {
public:
    // 构造函数，接受定时器时间（毫秒）和回调函数
    NonBlockingTimer(int milliseconds, std::function<void()> callback)
        : duration(std::chrono::milliseconds(milliseconds))
        , callback(callback)
        , is_running(false)
    {
    }

    // 启动定时器
    void start()
    {
        if (is_running)
            return; // 如果定时器已经在运行，返回
        is_running = true;
        timer_thread = std::thread([this]() {
            while (is_running) {
                std::this_thread::sleep_for(duration); // 睡眠指定的时间
                if (is_running) {
                    callback(); // 时间到后执行回调函数
                }
            }
        });
        timer_thread.detach(); // 分离线程，非阻塞
    }

    // 停止定时器
    void stop()
    {
        is_running = false;
    }

    // 检查定时器是否正在运行
    bool running() const
    {
        return is_running;
    }

    ~NonBlockingTimer()
    {
        stop(); // 确保在销毁时停止定时器
        if (timer_thread.joinable()) {
            timer_thread.join(); // 如果线程可加入，加入线程
        }
    }

private:
    std::chrono::milliseconds duration; // 定时器的时间（毫秒）
    std::function<void()> callback; // 回调函数
    std::atomic<bool> is_running; // 定时器是否正在运行
    std::thread timer_thread; // 定时器线程
};
}