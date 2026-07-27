/**
 * @file environment_detector.h
 * @author likang (likang@freego.com)
 * @brief 自动环境检测器 - 无需人工干预，基于雷达信号自动判断室内/室外
 *
 * 使用三路信号融合：
 *   1. 水平激光扫描开放度  (/scan/left, /scan/right)
 *      室内墙壁多 → 大量射线短程返回 → 开放度低
 *      室外开阔   → 大量射线打到 range_max → 开放度高
 *   2. 倾斜雷达开放度      (/scan/left_tilt, /scan/right_tilt)
 *      室内天花板 → 向上射线短程返回 → 倾斜开放度低
 *      室外无顶   → 向上射线打到 range_max → 倾斜开放度高
 *   3. 双侧墙体对称性      (DecisionMaker 回传侧向距离)
 *      室内走廊   → 两侧同时有墙且对称
 *
 * @version 1.1
 * @date 2025-11-01
 * @copyright Copyright (c) 2025 Freego Technology Co., Ltd.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>

/** 环境类型枚举 */
enum class EnvironmentType { OUTDOOR, INDOOR };

/**
 * @class EnvironmentDetector
 * @brief 自动室内/室外环境检测器
 *
 * 作为独立 ROS2 节点运行，订阅四路激光雷达话题，融合三路信号。
 * 通过非对称滞后状态机防止频繁切换：进室内 6s，出室内 12s。
 * 切换完成后通过回调通知上层切换参数配置，且回调在锁外执行无死锁风险。
 */
class EnvironmentDetector : public rclcpp::Node {
public:
    using OnChangedCb = std::function<void(EnvironmentType)>;

    /** @brief 构造函数，初始化四路 LaserScan 订阅与滑动窗口 */
    EnvironmentDetector();

    /**
     * @brief 由 DecisionMaker 每帧回传侧向最小距离（base_link 坐标系, 单位 m）
     * @param left_dist  左侧最近障碍物距离
     * @param right_dist 右侧最近障碍物距离
     */
    void updateLateralDistances(double left_dist, double right_dist);

    /** @brief 设置环境切换回调，切换完成后在锁外调用，无死锁风险 */
    void setOnEnvironmentChanged(OnChangedCb cb) { on_changed_ = std::move(cb); }

    /** @brief 获取当前环境类型 */
    EnvironmentType currentEnvironment() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return current_env_;
    }

private:
    // ─── 订阅回调 ───────────────────────────────────────────────
    void onLeftScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void onRightScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void onLeftTilt(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void onRightTilt(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

    // ─── 信号计算（无锁，纯函数） ───────────────────────────────
    /**
     * @brief 计算一帧 LaserScan 的开放度
     *
     * 开放度 = (无效射线数 + 打到 range_max 的射线数) / 总射线数
     * - 值接近 0：射线大多在短距离返回（有墙/有天花板）→ 室内特征
     * - 值接近 1：射线大多无返回或打到最远处（开阔/无顶）→ 室外特征
     */
    static double computeOpenness(const sensor_msgs::msg::LaserScan& scan);

    // ─── 内部核心逻辑（需在 mutex_ 保护下调用） ─────────────────
    /**
     * @brief 融合三路信号评分，驱动滞后状态机
     * @return 若环境发生切换，返回新环境类型；否则返回 std::nullopt
     * @note 调用方负责在返回后、锁外调用 on_changed_，避免死锁
     */
    std::optional<EnvironmentType> evaluate();

    // ─── 滑动窗口辅助 ───────────────────────────────────────────
    template<typename T>
    static void pushWindow(std::deque<T>& w, T val, std::size_t max_sz) {
        w.push_back(std::move(val));
        if (w.size() > max_sz) w.pop_front();
    }
    static double windowMean(const std::deque<double>& w);
    static double windowTrueRatio(const std::deque<bool>& w);

    // ─── 判定参数 ───────────────────────────────────────────────
    /** 滑动窗口大小（30 帧 ≈ 3s @ 10Hz） */
    static constexpr std::size_t WIN_SIZE = 30;

    /** 室内判定得分阈值（score >= 此值持续确认时间后切换） */
    static constexpr double INDOOR_SCORE_HIGH = 0.60;
    /** 室外判定得分阈值（score <= 此值持续确认时间后切换） */
    static constexpr double INDOOR_SCORE_LOW  = 0.25;

    /** 切入室内确认时间（快：安全优先） */
    static constexpr double ENTER_INDOOR_S  = 6.0;
    /** 切入室外确认时间（慢：避免误切） */
    static constexpr double ENTER_OUTDOOR_S = 12.0;

    // 信号权重（之和 = 1.0）
    static constexpr double W_OPEN = 0.35; ///< 水平扫描开放度权重
    static constexpr double W_CEIL = 0.45; ///< 倾斜雷达开放度权重（天花板信号更可靠）
    static constexpr double W_WALL = 0.20; ///< 双侧墙体权重

    // 水平扫描开放度阈值（室内开放度低，室外高）
    static constexpr double OPEN_INDOOR_MAX  = 0.25; ///< < 0.25 → 室内
    static constexpr double OPEN_OUTDOOR_MIN = 0.55; ///< > 0.55 → 室外

    // 倾斜雷达开放度阈值
    // 室内：向上射线打到天花板 → 开放度低 (< 0.15)
    // 室外：向上射线无遮挡 → 开放度高 (> 0.40)
    static constexpr double TILT_INDOOR_MAX  = 0.15; ///< < 0.15 → 有天花板(室内)
    static constexpr double TILT_OUTDOOR_MIN = 0.40; ///< > 0.40 → 无天花板(室外)

    // 双侧墙判定
    static constexpr double WALL_DIST_THRESH = 1.8;  ///< 两侧均 < 此距离(m) 视为有墙
    static constexpr double WALL_SYM_RATIO   = 0.5;  ///< 左右距离比 > 此值视为对称
    static constexpr double WALL_WIN_RATIO   = 0.60; ///< 滑窗内双侧墙帧占比阈值

    // ─── 状态 ────────────────────────────────────────────────────
    EnvironmentType current_env_{EnvironmentType::OUTDOOR};
    bool            transition_armed_{false};
    EnvironmentType armed_target_{EnvironmentType::OUTDOOR};
    rclcpp::Time    arm_time_;

    // ─── 滑动窗口 ────────────────────────────────────────────────
    std::deque<double> open_win_;  ///< 水平扫描开放度 [0,1]，低=室内
    std::deque<double> ceil_win_;  ///< 倾斜雷达开放度 [0,1]，低=有天花板=室内
    std::deque<bool>   wall_win_;  ///< 双侧墙对称检测布尔序列

    // ─── 订阅（均为 LaserScan，无需 TF/PCL） ────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr left_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr right_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr left_tilt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr right_tilt_sub_;

    OnChangedCb on_changed_;
    mutable std::mutex mutex_;
};
