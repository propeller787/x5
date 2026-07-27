// recharge_ctrl.cpp
#include "recharge_ctrl.hpp"
#include "recharge_tool.hpp"
#include <cmath>
#include <sstream>
#include <deque>

RechargeCtrl::RechargeCtrl(const RechargeCtrlDeps &deps)
    : deps_(deps)
{
    startWorker();
}

RechargeCtrl::~RechargeCtrl()
{
    stopWorker();
}

void RechargeCtrl::pushResult(const RechargeResult &result)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    // 只保留最新的结果
    while (!result_queue_.empty())
        result_queue_.pop();
    result_queue_.push(result);
    queue_cv_.notify_one();
}

// ==================== 工作线程 ====================

void RechargeCtrl::startWorker()
{
    if (control_worker_running_.exchange(true))
        return;
    control_worker_thread_ = std::thread(&RechargeCtrl::controlWorkerLoop, this);
    RCLCPP_INFO(deps_.node->get_logger(), "[RechargeCtrl] 异步控制工作线程已启动");
}

void RechargeCtrl::stopWorker()
{
    if (!control_worker_running_.exchange(false))
        return;
    queue_cv_.notify_all();
    if (control_worker_thread_.joinable())
        control_worker_thread_.join();
    RCLCPP_INFO(deps_.node->get_logger(), "[RechargeCtrl] 异步控制工作线程已停止");
}

void RechargeCtrl::controlWorkerLoop()
{
    while (control_worker_running_)
    {
        RechargeResult result;
        bool have_result = false;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() {
                return !result_queue_.empty() || !control_worker_running_;
            });

            if (!control_worker_running_)
                break;

            if (!result_queue_.empty())
            {
                result = result_queue_.front();
                result_queue_.pop();
                have_result = true;
            }
        }

        if (have_result)
        {
            controlRechargeMotion(result);
        }
    }
}

// ==================== 核心控制算法 ====================

bool RechargeCtrl::checkStuck(double dx)
{
    auto now = deps_.node->now();
    dx_history_.emplace_back(now, dx);

    // 清理超过 STUCK_TIMEOUT 的旧记录
    while (!dx_history_.empty() &&
           (now - dx_history_.front().first).seconds() > STUCK_TIMEOUT)
    {
        dx_history_.pop_front();
    }

    // 需要满 STUCK_TIMEOUT 时长的数据才判定
    if (dx_history_.size() < 2)
        return false;
    auto &oldest = dx_history_.front();
    auto &latest = dx_history_.back();
    double elapsed = (latest.first - oldest.first).seconds();
    if (elapsed < STUCK_TIMEOUT - 0.1)
        return false;

    double dx_diff = oldest.second - latest.second;
    RCLCPP_DEBUG(deps_.node->get_logger(),
                 "卡住监控: 旧dx=%.4f, 最新dx=%.4f, 变化=%.4f, 历时=%.2fs",
                 oldest.second, latest.second, dx_diff, elapsed);

    // 3 秒内 dx 减少不足 0.01 → 卡住
    return dx_diff < DX_MIN_DECREASE;
}

void RechargeCtrl::resetStuckMonitor()
{
    dx_history_.clear();
}

bool RechargeCtrl::checkStationAlignment(double dx, double dy, double yaw_diff)
{
    bool isstop = false;
    const double ALIGNMENT_DISTANCE_THRESHOLD = 0.55;
    const double ALIGNMENT_ANGLE_THRESHOLD = 12.0 * M_PI / 180.0;
    if (std::fabs(yaw_diff * 180.0 / M_PI) > 23.0) {
        return isstop;
    }

    if (dx <= ALIGNMENT_DISTANCE_THRESHOLD)
    {
        RCLCPP_INFO(deps_.node->get_logger(), "进入对准区域 - 距离基站: %.4f m, 夹角: %.2f°",
                    dx, yaw_diff * 180.0 / M_PI);

        if (fabs(yaw_diff) <= ALIGNMENT_ANGLE_THRESHOLD && std::fabs(dy) <= 0.055)
        {
            RCLCPP_INFO(deps_.node->get_logger(), "✓ 已对准基站，继续行驶 - 夹角: %.2f°,dx=%.2f,dy=%.2f",
                        yaw_diff * 180.0 / M_PI, dx, dy);
            RCLCPP_DEBUG(deps_.node->get_logger(), "对准行驶 - 线速度: %.4f", deps_.ctrl_twist->linear.x);
        }
        else
        {
            RCLCPP_WARN(deps_.node->get_logger(), "✗ 未对准基站，停止行驶 - 夹角: %.2f° > 8°,dx=%.2f,dy=%.2f",
                        yaw_diff * 180.0 / M_PI, dx, dy);
            isstop = true;
        }
    }
    return isstop;
}

void RechargeCtrl::controlRechargeMotion(const RechargeResult &result)
{
    auto get_rechargePose = [this]() {
        return deps_.last_result->rechargePose;
    };

    // 发布基站检测信号
    {
        static bool last_detected = false;
        static rclcpp::Time last_pub_time = deps_.node->now();
        auto now = deps_.node->now();
        bool detected = true;
        if (last_detected != detected)
        {
            auto msg = std_msgs::msg::Bool();
            msg.data = detected;
            deps_.pub_station_detected->publish(msg);
            last_detected = detected;
        }
        last_pub_time = now;
    }

    bool in_rotation_delay = deps_.node->get_parameter("in_rotation_delay").as_bool();
    if (in_rotation_delay)
    {
        RCLCPP_INFO(deps_.node->get_logger(), "旋转延迟期间，仅发布检测信号，不执行运动控制");
        double angular_velocity = 0.15;
        deps_.ctrl_twist->linear.x = 0.0;
        deps_.ctrl_twist->angular.z = angular_velocity;
        *deps_.ctrl_twist_time = deps_.node->now();
        deps_.pub_ctrl->publish(*deps_.ctrl_twist);
        return;
    }

    // 充电电压检测
    if (*deps_.active && deps_.get_charging_voltage())
    {
        RCLCPP_INFO(deps_.node->get_logger(), "检测到充电电压，停止移动");
        deps_.ctrl_twist->linear.x = 0.0;
        deps_.ctrl_twist->angular.z = 0.0;
        *deps_.ctrl_twist_time = deps_.node->now();
        deps_.stop_navigation();
        return;
    }

    RCLCPP_DEBUG(deps_.node->get_logger(), "调整后 dy: %.4f", get_rechargePose().dy);
    RCLCPP_DEBUG(deps_.node->get_logger(), "控制参数 - dx: %.4f, dy: %.4f, k: %.4f",
                 get_rechargePose().dx, get_rechargePose().dy, get_rechargePose().k);
    RCLCPP_INFO(deps_.node->get_logger(), "偏航角误差: %.4f rad (%.2f 度)",
                get_rechargePose().yaw_diff, get_rechargePose().yaw_diff * 180.0 / M_PI);

    if (std::fabs(get_rechargePose().yaw_diff * 180.0 / M_PI) > 23.0)
        return;

    if (*deps_.active && checkStationAlignment(get_rechargePose().dx, get_rechargePose().dy, get_rechargePose().yaw_diff))
    {
        RCLCPP_WARN(deps_.node->get_logger(), "===== 开始对准误差调整 =====");
        RCLCPP_WARN(deps_.node->get_logger(), "当前误差 - 夹角:%.2f°, dy:%.3fm",
                    get_rechargePose().yaw_diff * 180.0 / M_PI, get_rechargePose().dy);

        deps_.ctrl_twist->linear.x = 0.0;
        deps_.ctrl_twist->angular.z = 0.0;
        *deps_.ctrl_twist_time = deps_.node->now();
        deps_.pub_ctrl->publish(*deps_.ctrl_twist);
        rclcpp::sleep_for(std::chrono::milliseconds(300));

        double adjust_duration = 0.0;
        double adjust_duration_reverse = 0.0;
        double adjust_angular = 0.0;
        std::string adjust_type;

        if (1)
        {
            adjust_type = "角度调整";
            float dy_yaw = (get_rechargePose().dy / 0.5) * 0.78;
            double a = -get_rechargePose().yaw_diff / 0.15;
            double b = dy_yaw / 0.15;
            a = a * 1.2;
            b = b * 1.5;
            adjust_duration = std::abs(a + b) + 0.3;
            adjust_duration_reverse = std::abs(b) + 0.3;
            adjust_angular = (a + b > 0) ? 0.15 : -0.15;
            RCLCPP_WARN(deps_.node->get_logger(), "dy:%.2f dy_yaw:%.2f", get_rechargePose().dy, dy_yaw);
            RCLCPP_WARN(deps_.node->get_logger(), "a:%.2f b:%.2f adjust_duration:%.2f adjust_duration_reverse:%.2f adjust_angular:%.2f",
                        a, b, adjust_duration, adjust_duration_reverse, adjust_angular);
            RCLCPP_WARN(deps_.node->get_logger(), "旋转 %.2f 秒", adjust_duration);
        }

        // 执行旋转调整
        if (*deps_.active)
        {
            if (adjust_duration > 0.1)
            {
                geometry_msgs::msg::Twist adjust_cmd;
                adjust_cmd.linear.x = 0.0;
                adjust_cmd.angular.z = adjust_angular;

                rclcpp::Time adjust_start = deps_.node->now();
                int publish_count = 0;
                RCLCPP_WARN(deps_.node->get_logger(), "矫正- 旋转 z:%.2ff", adjust_cmd.angular.z);

                while (rclcpp::ok() && *deps_.active &&
                       (deps_.node->now() - adjust_start).seconds() < adjust_duration)
                {
                    deps_.pub_ctrl->publish(adjust_cmd);
                    publish_count++;
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                }

                RCLCPP_WARN(deps_.node->get_logger(), "%s完成: 发布%d次命令, 实际旋转%.2f秒",
                            adjust_type.c_str(), publish_count, adjust_duration);

                adjust_cmd.angular.z = 0.0;
                deps_.pub_ctrl->publish(adjust_cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                RCLCPP_WARN(deps_.node->get_logger(), "矫正- 直行 z:%.2ff", adjust_cmd.angular.z);
                RCLCPP_WARN(deps_.node->get_logger(), "sleep 执行旋转调整-2 500 ms");
            }

            // 向前走
            const double FORWARD_DISTANCE = 0.5;
            const double FORWARD_SPEED = 0.09;
            double forward_duration = FORWARD_DISTANCE / FORWARD_SPEED;

            RCLCPP_WARN(deps_.node->get_logger(), "frameid:%d 开始向前调整: 距离=%.2fm, 速度=%.2fm/s, 时间=%.2fs",
                        result.frameid, FORWARD_DISTANCE, FORWARD_SPEED, forward_duration);

            geometry_msgs::msg::Twist forward_cmd;
            forward_cmd.linear.x = FORWARD_SPEED;
            forward_cmd.angular.z = 0.0;

            rclcpp::Time forward_start = deps_.node->now();
            int forward_count = 0;

            while (rclcpp::ok() && *deps_.active &&
                   (deps_.node->now() - forward_start).seconds() < forward_duration)
            {
                deps_.pub_ctrl->publish(forward_cmd);
                forward_count++;
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }

            forward_cmd.linear.x = 0.0;
            deps_.pub_ctrl->publish(forward_cmd);

            RCLCPP_INFO(deps_.node->get_logger(), "前进调整完成: 发布%d次命令, 实际前进%.2f秒",
                        forward_count, forward_duration);

            if (adjust_duration_reverse > 0.1)
            {
                geometry_msgs::msg::Twist adjust_cmd;
                adjust_cmd.linear.x = 0.0;
                adjust_cmd.angular.z = -adjust_angular;

                rclcpp::Time adjust_start = deps_.node->now();
                int publish_count = 0;
                RCLCPP_WARN(deps_.node->get_logger(), "矫正- 反向旋转 z:%.2ff", adjust_cmd.angular.z);

                while (rclcpp::ok() && *deps_.active &&
                       (deps_.node->now() - adjust_start).seconds() < adjust_duration_reverse)
                {
                    deps_.pub_ctrl->publish(adjust_cmd);
                    publish_count++;
                    rclcpp::sleep_for(std::chrono::milliseconds(50));
                }

                RCLCPP_WARN(deps_.node->get_logger(), "%s完成: 发布%d次命令, 实际旋转%.2f秒",
                            adjust_type.c_str(), publish_count, adjust_duration_reverse);

                adjust_cmd.angular.z = 0.0;
                deps_.pub_ctrl->publish(adjust_cmd);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                RCLCPP_WARN(deps_.node->get_logger(), "矫正- 直行 z:%.2ff", adjust_cmd.angular.z);
                RCLCPP_WARN(deps_.node->get_logger(), "sleep 执行旋转调整-2 500 ms");
            }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(450));
        RCLCPP_WARN(deps_.node->get_logger(), "===== 对准误差调整完成 =====");
        return;
    }

    // 正常控制速度计算
    double dx = get_rechargePose().dx;
    double dy = get_rechargePose().dy;
    double yaw_diff = get_rechargePose().yaw_diff;
    float ady = 2.0 * dy;
    float dy_abs = -0.00;
    if (dy < 0.0) dy_abs = 0.0;
    if (dy > 1.5)
    {
        float abs_dy = std::fabs(dy);
        if (abs_dy > 0.1)        ady = 2.0 * dy + 1.0 * dy_abs;
        else if (abs_dy > 0.4)   ady = 4.0 * dy + dy_abs;
        else if (abs_dy > 0.2)   ady = 5.0 * dy + 1.0 * dy_abs;
    }
    else if (dx > 1.2)
    {
        float abs_dy = std::fabs(dy);
        if (abs_dy > 0.1)        ady = 2.0 * dy + dy_abs;
        else if (abs_dy > 0.4)   ady = 4.0 * dy - dy_abs;
        else if (abs_dy > 0.2)   ady = 5.0 * dy + dy_abs;
    }
    else if (dx < 0.58)
    {
        ady = 0.0;
    }

    auto [linear_velocity, angular_velocity] = RechargeTool::computeDockingVelocity((dx - 0.2), ady, yaw_diff);
    RCLCPP_INFO(deps_.node->get_logger(), "基础控制速度 - 线速度: %.3f, 角速度: %.3f",
                linear_velocity, angular_velocity);
    RCLCPP_INFO(deps_.node->get_logger(), "dx=%.3f,dy=%.3f yaw_diff=%.3f", dx, dy, yaw_diff);

    if (angular_velocity > 0.0)
        angular_velocity = std::min(fabs(angular_velocity), 0.09);
    else
        angular_velocity = -std::min(fabs(angular_velocity), 0.09);

    double radio_x = 1.0;
    double radio_z = 1.0;

    if (dx > 1.5)
    {
        radio_x = 1.0;
    }
    else if (dx > 0.7)
    {
        radio_x = 0.7;
        radio_z = 1.4;
    }
    else if (dx > 0.5)
    {
        radio_x = 0.5;
        radio_z = 1.4;
    }
    else if (dx > 0.55)
    {
        radio_x = 0.42;
        radio_z = 1.6;
    }
    else if (dx > 0.45)
    {
        radio_x = 0.33;
        radio_z = 1.8;
    }
    else if (dx > 0.46)
    {
        radio_x = 0.2;
        radio_z = 1.8;
    }
    else if (dx > 0.2)
    {
        radio_x = 0.15;
        radio_z = 2.0;
    }
    RCLCPP_DEBUG(deps_.node->get_logger(), "距离调整系数 - radio_x: %.2f, radio_z: %.2f", radio_x, radio_z);

    if (!*deps_.active) {
        resetStuckMonitor();
        return;
    }

    // 卡住检测：长时间跟踪 dx 无进展
    if (std::abs(linear_velocity) < 0.001 && std::abs(angular_velocity) < 0.001) {
        resetStuckMonitor();
    } else if (checkStuck(dx)) {
        RCLCPP_WARN(deps_.node->get_logger(), "===== 检测到卡住 (dx 长时间无变化) ===");
        deps_.ctrl_twist->linear.x = 0.0;
        deps_.ctrl_twist->angular.z = 0.0;
        *deps_.ctrl_twist_time = deps_.node->now();
        deps_.pub_ctrl->publish(*deps_.ctrl_twist);
        auto msg = std_msgs::msg::Int32();
        rclcpp::sleep_for(std::chrono::milliseconds(300));
        msg.data = RECHARGE_FEEDBACK_SUCCESS_DOCKED;
        deps_.pub1->publish(msg);
        RCLCPP_WARN(deps_.node->get_logger(), "运行上报状态 2");
        deps_.stop_navigation();
        deps_.set_charging_voltage(true);
        resetStuckMonitor();
        return;
    }
    if (dx > 0.50 - 0.065)
    {
        double final_linear_vel = linear_velocity * radio_x;
        deps_.ctrl_twist->linear.x = final_linear_vel;
        deps_.ctrl_twist->angular.z = angular_velocity * radio_z;
        *deps_.ctrl_twist_time = deps_.node->now();
        RCLCPP_DEBUG(deps_.node->get_logger(), "控制命令 - 线速度: %.4f, 角速度: %.4f",
                     deps_.ctrl_twist->linear.x, deps_.ctrl_twist->angular.z);
    }
    else
    {
        RCLCPP_INFO(deps_.node->get_logger(), "到达目标位置 (dx=%.4f <= 0.54), 停止移动", dx);
        deps_.ctrl_twist->linear.x = 0.0;
        deps_.ctrl_twist->angular.z = 0.2 * deps_.ctrl_twist->angular.z;
        *deps_.ctrl_twist_time = deps_.node->now();
        auto msg = std_msgs::msg::Int32();
        rclcpp::sleep_for(std::chrono::seconds(3));
        msg.data = RECHARGE_FEEDBACK_SUCCESS_DOCKED;
        deps_.pub1->publish(msg);
        RCLCPP_WARN(deps_.node->get_logger(), "运行上报状态 2");
        deps_.stop_navigation();
        deps_.set_charging_voltage(true);
    }
}
