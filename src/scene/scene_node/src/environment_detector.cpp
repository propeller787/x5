/**
 * @file environment_detector.cpp
 * @author likang (likang@freego.com)
 * @brief 自动环境检测器实现
 *
 * 信号说明：
 *   open_win_  : 水平激光开放度（低=室内有墙，高=室外开阔）
 *   ceil_win_  : 倾斜激光开放度（低=有天花板=室内，高=无顶=室外）
 *   wall_win_  : 双侧墙对称检测布尔序列
 *
 * 评分公式（各信号归一化到室内得分 [0,1]，加权求和）：
 *   s_open = 1 - clamp((avg_open - OPEN_INDOOR_MAX)
 *                    / (OPEN_OUTDOOR_MIN - OPEN_INDOOR_MAX), 0, 1)
 *   s_ceil = 1 - clamp((avg_ceil - TILT_INDOOR_MAX)
 *                    / (TILT_OUTDOOR_MIN - TILT_INDOOR_MAX), 0, 1)
 *   s_wall = clamp(wall_ratio / WALL_WIN_RATIO, 0, 1)
 *   score  = s_open * W_OPEN + s_ceil * W_CEIL + s_wall * W_WALL
 */

#include "environment_detector.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "../../../common/utils/topic.h"

// ==================== 构造函数 ====================

EnvironmentDetector::EnvironmentDetector()
    : rclcpp::Node("environment_detector")
{
    // 水平激光（开放度信号：室内有墙→低开放度）
    left_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        leftLaserTopic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { onLeftScan(msg); });

    right_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        rightLaserTopic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { onRightScan(msg); });

    // 倾斜激光（天花板信号：室内有顶→向上射线返回→低开放度）
    left_tilt_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        leftTiltLaserTopic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { onLeftTilt(msg); });

    right_tilt_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        rightTiltLaserTopic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { onRightTilt(msg); });

    RCLCPP_INFO(get_logger(),
        "[EnvDetector] 初始化完成 | 默认: 室外 | "
        "得分阈值: 室内>=%.2f / 室外<=%.2f | "
        "确认时间: 入室内 %.0fs / 入室外 %.0fs",
        INDOOR_SCORE_HIGH, INDOOR_SCORE_LOW,
        ENTER_INDOOR_S, ENTER_OUTDOOR_S);
}

// ==================== 信号计算（纯函数，无锁） ====================

double EnvironmentDetector::computeOpenness(
    const sensor_msgs::msg::LaserScan& scan)
{
    if (scan.ranges.empty()) return 0.5; // 无数据时返回中性值

    // range >= range_max * 0.93 或 nan/inf 均视为"无遮挡/无返回"
    const float thresh = scan.range_max * 0.93f;
    int open_count = 0;
    for (float r : scan.ranges) {
        if (!std::isfinite(r) || r >= thresh) ++open_count;
    }
    return static_cast<double>(open_count) / static_cast<double>(scan.ranges.size());
}

// ==================== 订阅回调 ====================

void EnvironmentDetector::onLeftScan(
    sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    double openness = computeOpenness(*msg);
    std::optional<EnvironmentType> changed;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        pushWindow(open_win_, openness, WIN_SIZE);
        changed = evaluate();
    }
    if (changed && on_changed_) on_changed_(*changed);
}

void EnvironmentDetector::onRightScan(
    sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    double openness = computeOpenness(*msg);
    std::optional<EnvironmentType> changed;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        // 与上一帧左侧数据合并取均值，避免窗口膨胀
        if (!open_win_.empty()) {
            open_win_.back() = (open_win_.back() + openness) * 0.5;
        } else {
            pushWindow(open_win_, openness, WIN_SIZE);
        }
        changed = evaluate();
    }
    if (changed && on_changed_) on_changed_(*changed);
}

void EnvironmentDetector::onLeftTilt(
    sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    // 倾斜激光：开放度低 = 向上射线被天花板反射 = 室内
    double openness = computeOpenness(*msg);
    std::optional<EnvironmentType> changed;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        pushWindow(ceil_win_, openness, WIN_SIZE);
        changed = evaluate();
    }
    if (changed && on_changed_) on_changed_(*changed);
}

void EnvironmentDetector::onRightTilt(
    sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    double openness = computeOpenness(*msg);
    std::optional<EnvironmentType> changed;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        // 与上一帧左侧倾斜数据合并取均值
        if (!ceil_win_.empty()) {
            ceil_win_.back() = (ceil_win_.back() + openness) * 0.5;
        } else {
            pushWindow(ceil_win_, openness, WIN_SIZE);
        }
        changed = evaluate();
    }
    if (changed && on_changed_) on_changed_(*changed);
}

void EnvironmentDetector::updateLateralDistances(
    double left_dist, double right_dist)
{
    // 两侧均 < 阈值，且左右距离比 > 对称阈值，视为对称双侧墙（走廊特征）
    bool bilateral = (left_dist < WALL_DIST_THRESH) && (right_dist < WALL_DIST_THRESH);
    if (bilateral) {
        double mn = std::min(left_dist, right_dist);
        double mx = std::max(left_dist, right_dist) + 1e-9;
        bilateral = (mn / mx) > WALL_SYM_RATIO;
    }
    std::optional<EnvironmentType> changed;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        pushWindow(wall_win_, bilateral, WIN_SIZE);
        changed = evaluate();
    }
    if (changed && on_changed_) on_changed_(*changed);
}

// ==================== 滑动窗口工具 ====================

double EnvironmentDetector::windowMean(const std::deque<double>& w)
{
    if (w.empty()) return 0.0;
    return std::accumulate(w.begin(), w.end(), 0.0) / static_cast<double>(w.size());
}

double EnvironmentDetector::windowTrueRatio(const std::deque<bool>& w)
{
    if (w.empty()) return 0.0;
    auto cnt = std::count(w.begin(), w.end(), true);
    return static_cast<double>(cnt) / static_cast<double>(w.size());
}

// ==================== 融合评分 + 滞后状态机 ====================

std::optional<EnvironmentType> EnvironmentDetector::evaluate()
{
    // ── 1. 各信号窗口均值 ─────────────────────────────────────────
    double avg_open   = windowMean(open_win_);   // 水平开放度，低=室内
    double avg_ceil   = windowMean(ceil_win_);   // 倾斜开放度，低=有天花板=室内
    double wall_ratio = windowTrueRatio(wall_win_);

    // ── 2. 归一化为室内得分 [0, 1] ──────────────────────────────
    // 水平开放度：avg_open 低 → s_open 高（室内）
    double s_open = 1.0 - std::clamp(
        (avg_open - OPEN_INDOOR_MAX) / (OPEN_OUTDOOR_MIN - OPEN_INDOOR_MAX),
        0.0, 1.0);

    // 倾斜开放度：avg_ceil 低（天花板存在）→ s_ceil 高（室内）
    double s_ceil = 1.0 - std::clamp(
        (avg_ceil - TILT_INDOOR_MAX) / (TILT_OUTDOOR_MIN - TILT_INDOOR_MAX),
        0.0, 1.0);

    // 双侧墙：wall_ratio 达到 WALL_WIN_RATIO → s_wall = 1.0
    double s_wall = std::clamp(wall_ratio / WALL_WIN_RATIO, 0.0, 1.0);

    // ── 3. 加权融合 ──────────────────────────────────────────────
    double score = s_open * W_OPEN + s_ceil * W_CEIL + s_wall * W_WALL;

    // ── 4. 判定目标环境 ──────────────────────────────────────────
    EnvironmentType target;
    if (score >= INDOOR_SCORE_HIGH) {
        target = EnvironmentType::INDOOR;
    } else if (score <= INDOOR_SCORE_LOW) {
        target = EnvironmentType::OUTDOOR;
    } else {
        // 模糊区间 [0.25, 0.60]：维持现状，取消计时
        if (transition_armed_) {
            transition_armed_ = false;
            RCLCPP_DEBUG(get_logger(),
                "[EnvDetector] 得分进入模糊区 %.2f，取消切换计时", score);
        }
        return std::nullopt;
    }

    // ── 5. 目标与当前相同，无需切换 ─────────────────────────────
    if (target == current_env_) {
        transition_armed_ = false;
        return std::nullopt;
    }

    // ── 6. 非对称滞后计时 ────────────────────────────────────────
    auto now = this->now();

    if (!transition_armed_ || armed_target_ != target) {
        transition_armed_ = true;
        armed_target_     = target;
        arm_time_         = now;
        RCLCPP_INFO(get_logger(),
            "[EnvDetector] 开始观察切换→%s | "
            "score=%.2f (open=%.2f ceil=%.2f wall=%.2f) | "
            "分解: s_open=%.2f*%.2f + s_ceil=%.2f*%.2f + s_wall=%.2f*%.2f",
            target == EnvironmentType::INDOOR ? "室内" : "室外", score,
            avg_open, avg_ceil, wall_ratio,
            s_open, W_OPEN, s_ceil, W_CEIL, s_wall, W_WALL);
        return std::nullopt;
    }

    double confirm_s = (target == EnvironmentType::INDOOR)
        ? ENTER_INDOOR_S : ENTER_OUTDOOR_S;

    if ((now - arm_time_).seconds() < confirm_s) {
        return std::nullopt; // 持续时间不足，继续等待
    }

    // ── 7. 确认切换 ──────────────────────────────────────────────
    current_env_      = target;
    transition_armed_ = false;
    RCLCPP_INFO(get_logger(),
        "[EnvDetector] *** 环境切换 → %s *** "
        "score=%.2f open=%.2f ceil=%.2f wall=%.2f",
        target == EnvironmentType::INDOOR ? "室内" : "室外",
        score, avg_open, avg_ceil, wall_ratio);

    return target;
}
