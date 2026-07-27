// recharge_ctrl.hpp
#ifndef RECHARGE_CTRL_HPP
#define RECHARGE_CTRL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "wheelchair_types.hpp"
#include "recharge_status.hpp"
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <functional>

// RechargeCtrl 运行所需的外部依赖
struct RechargeCtrlDeps
{
    rclcpp::Node *node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_ctrl;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub1;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub2;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_station_detected;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_charging_voltage;
    geometry_msgs::msg::Twist *ctrl_twist;
    rclcpp::Time *ctrl_twist_time;
    bool *active;
    RechargeResult *last_result;
    std::function<void()> stop_navigation;
    std::function<void()> start_navigation;
    std::function<void(bool)> set_charging_voltage;
    std::function<bool()> get_charging_voltage;
};

class RechargeCtrl
{
public:
    RechargeCtrl(const RechargeCtrlDeps &deps);
    ~RechargeCtrl();

    // 生产者接口：由 LidarScanController 的 procData 调用
    void pushResult(const RechargeResult &result);

private:
    RechargeCtrlDeps deps_;

    // 结果队列（生产者：LidarScanController::procData，消费者：controlWorkerLoop）
    std::queue<RechargeResult> result_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread control_worker_thread_;
    std::atomic<bool> control_worker_running_{false};

    // 核心控制算法
    void controlRechargeMotion(const RechargeResult &result);
    bool checkStationAlignment(double dx, double dy, double yaw_diff);

        // 历史距离监控（卡住检测）
    std::deque<std::pair<rclcpp::Time, double>> dx_history_;
    const double STUCK_TIMEOUT = 3.0;       // 判定卡住的持续时长（秒）
    const double DX_MIN_DECREASE = 0.01;     // 3秒内 dx 至少减少 0.01m
    bool checkStuck(double dx);
    void resetStuckMonitor();

    // 工作线程
    void controlWorkerLoop();
    void startWorker();
    void stopWorker();
};

#endif // RECHARGE_CTRL_HPP
