/**
 * @file virtual_guide_rail_controller.h
 * @brief 虚拟导轨主控制器 — 状态机 + 子模块组合
 *
 * 线程安全：所有公开方法均通过 mutex_ 保护内部状态。
 * 子模块（WallDetector / CenterLineCalculator / PathTracker）为无状态类，
 * 不需要自身的锁。
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <vector>
#include <string>

#include "wall_detector.h"
#include "center_line_calculator.h"
#include "path_tracker.h"

namespace scene_node {

/// 虚拟导轨工作状态
enum class GuideRailState {
    DISABLED,    ///< 模式未激活
    WALL_DETECT, ///< 正在检测和确认墙线
    TRACKING,    ///< 双墙拟合成功，导轨跟踪中
    FALLBACK,    ///< 拟合质量差，退化为简单 P 控制
    STANDBY,     ///< 已激活但当前无通道结构
    EXITING      ///< 退出过渡（角速度淡出）
};

/// 聚合所有子模块参数
struct GuideRailParams {
    WallDetectorParams  wall_detector;
    CenterLineParams    center_line;
    PathTrackerParams   path_tracker;

    // 状态机管理
    int    confirm_frames         = 3;    ///< 进入 TRACKING 所需连续有效帧数
    double hold_time              = 0.5;  ///< 墙线消失后保持状态时间 (s)
    int    fit_failure_exit_frames = 5;   ///< 连续拟合失败帧数 → 退回 STANDBY

    // 弯道处理
    double bend_threshold          = 0.3; ///< 弯道判定阈值
    double bend_speed_reduction_max = 0.5; ///< 弯道最大减速比例 [0, 1]

    // 安全
    double max_linear_speed          = 0.5;  ///< 最大前进速度 (m/s)
    double lateral_emergency_brake   = 0.03; ///< 侧向紧急制动距离 (m)
    double absolute_min_clearance    = 0.02; ///< 绝对最小间隙 (m)
    double radar_timeout_threshold   = 0.1;  ///< 雷达超时阈值 (s)

    // 退化 P 控制（FALLBACK）
    double fallback_kp          = 1.5;  ///< P 增益
    double fallback_deadzone    = 0.03; ///< 死区 (m)
    double fallback_max_angular = 0.3;  ///< 最大角速度 (rad/s)
};

class VirtualGuideRailController {
public:
    explicit VirtualGuideRailController(rclcpp::Logger logger);
    ~VirtualGuideRailController() = default;

    // ─── 核心接口 ───────────────────────────────────────────────

    /// 注入新一帧聚类数据，触发内部状态更新
    /// @param left_clusters  左侧聚类列表
    /// @param right_clusters 右侧聚类列表
    /// @param forward_min_distance 前向最近障碍物距离（用于紧急制动判断）
    void updateClusters(
        const std::vector<ClusterInfo>& left_clusters,
        const std::vector<ClusterInfo>& right_clusters,
        double forward_min_distance);

    /// 计算角速度修正量（rad/s）
    /// @param user_linear_x 用户前进速度 (m/s)
    double computeAngularCorrection(double user_linear_x);

    /// 获取弯道自适应推荐最大线速度 (m/s)
    double getRecommendedMaxSpeed() const;

    /// 获取当前状态
    GuideRailState getState() const;

    // ─── 模式控制 ───────────────────────────────────────────────

    void activate();
    void deactivate();

    /// 检查是否需要紧急退出（侧向过近 / 摇杆后拉等）
    /// @return true 表示需要退出导轨模式
    bool checkEmergencyExit(double user_linear_x) const;

    // ─── 调试数据获取（用于 RViz 可视化）───────────────────────

    WallLine       getLeftWall()       const;
    WallLine       getRightWall()      const;
    CenterPath     getCenterPath()     const;
    TrackingResult getTrackingResult() const;

    // ─── 参数 ───────────────────────────────────────────────────

    void setParams(const GuideRailParams& params);
    GuideRailParams getParams() const;

private:
    // ─── 状态机 ─────────────────────────────────────────────────

    void updateStateMachine();

    // ─── 控制计算 ────────────────────────────────────────────────

    /// 退化 P 控制（FALLBACK 状态使用）
    double computeFallbackCorrection(double user_linear_x) const;

    /// 弯道减速因子 [0, 1]
    double computeBendSpeedFactor() const;

    // ─── 成员变量 ────────────────────────────────────────────────

    rclcpp::Logger    logger_;
    mutable std::mutex mutex_;

    GuideRailState  state_ = GuideRailState::DISABLED;
    GuideRailParams params_;

    // 子模块（无状态）
    WallDetector          wall_detector_;
    CenterLineCalculator  center_line_calc_;
    PathTracker           path_tracker_;

    // 当前帧计算结果
    WallLine       current_left_wall_;
    WallLine       current_right_wall_;
    CenterPath     current_center_path_;
    TrackingResult current_tracking_;

    // FALLBACK 退化数据（左右最近距离）
    double fallback_d_left_  = 1e6;
    double fallback_d_right_ = 1e6;

    // 前向最近距离（紧急制动判断）
    double forward_min_distance_ = 1e6;

    // 状态管理计数器
    int            consecutive_valid_frames_       = 0;
    int            consecutive_fit_failure_frames_ = 0;
    int            wall_detect_timeout_frames_     = 0; ///< WALL_DETECT 无墙超时计数（与 valid 分离）
    rclcpp::Time   last_valid_time_;

    // 退出淡出
    rclcpp::Time   exit_start_time_;
    double         exit_angular_at_start_ = 0.0;
};

} // namespace scene_node
