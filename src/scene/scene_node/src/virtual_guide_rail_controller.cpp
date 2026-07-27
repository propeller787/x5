/**
 * @file virtual_guide_rail_controller.cpp
 * @brief 虚拟导轨主控制器实现
 */
#include "virtual_guide_rail_controller.h"

#include <algorithm>
#include <cmath>

namespace scene_node {

// ─────────────────────── 构造 / 参数 ───────────────────────

VirtualGuideRailController::VirtualGuideRailController(rclcpp::Logger logger)
    : logger_(logger)
    , wall_detector_(logger)
{}

void VirtualGuideRailController::setParams(const GuideRailParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;
    wall_detector_.setParams(params.wall_detector);
    center_line_calc_.setParams(params.center_line);
    path_tracker_.setParams(params.path_tracker);
}

GuideRailParams VirtualGuideRailController::getParams() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return params_;
}

// ─────────────────────── 模式控制 ───────────────────────────

void VirtualGuideRailController::activate() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == GuideRailState::DISABLED || state_ == GuideRailState::EXITING) {
        state_                         = GuideRailState::WALL_DETECT;
        consecutive_valid_frames_       = 0;
        consecutive_fit_failure_frames_ = 0;
        current_tracking_               = TrackingResult{};
        RCLCPP_INFO(logger_, "[VirtualGuideRail] 模式激活 → WALL_DETECT");
    }
}

void VirtualGuideRailController::deactivate() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != GuideRailState::DISABLED) {
        // 记录退出起始角速度，用于淡出
        exit_angular_at_start_ = current_tracking_.angular_correction;
        exit_start_time_       = rclcpp::Clock().now();
        state_                 = GuideRailState::EXITING;
        RCLCPP_INFO(logger_, "[VirtualGuideRail] 模式停用 → EXITING");
    }
}

bool VirtualGuideRailController::checkEmergencyExit(double user_linear_x) const {
    std::lock_guard<std::mutex> lock(mutex_);
    // 用户后拉摇杆
    if (user_linear_x < -0.01) {
        return true;
    }
    // 侧向过近（使用 lateral_emergency_brake 而非 absolute_min_clearance，更安全）
    if (fallback_d_left_  < params_.lateral_emergency_brake ||
        fallback_d_right_ < params_.lateral_emergency_brake) {
        return true;
    }
    return false;
}

GuideRailState VirtualGuideRailController::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

// ─────────────────────── 数据注入 ───────────────────────────

void VirtualGuideRailController::updateClusters(
    const std::vector<ClusterInfo>& left_clusters,
    const std::vector<ClusterInfo>& right_clusters,
    double forward_min_distance)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ == GuideRailState::DISABLED ||
        state_ == GuideRailState::EXITING) {
        return;
    }

    forward_min_distance_ = forward_min_distance;

    // 更新 FALLBACK 退化所需的左右最近距离（使用内侧边界，而非聚类中心）
    if (!left_clusters.empty()) {
        double min_d = 1e6;
        for (const auto& c : left_clusters) {
            // 左侧墙壁在正 Y 方向，内侧边界是 min_y
            if (c.center_x >= 0.0) {
                min_d = std::min(min_d, std::abs(c.min_y));
            }
        }
        fallback_d_left_ = min_d;
    } else {
        fallback_d_left_ = 1e6;
    }

    if (!right_clusters.empty()) {
        double min_d = 1e6;
        for (const auto& c : right_clusters) {
            // 右侧墙壁在负 Y 方向，内侧边界是 max_y
            if (c.center_x >= 0.0) {
                min_d = std::min(min_d, std::abs(c.max_y));
            }
        }
        fallback_d_right_ = min_d;
    } else {
        fallback_d_right_ = 1e6;
    }

    // 墙线拟合
    current_left_wall_  = wall_detector_.detectWall(left_clusters,  true);
    current_right_wall_ = wall_detector_.detectWall(right_clusters, false);

    // 中心线计算
    if (current_left_wall_.is_valid && current_right_wall_.is_valid) {
        current_center_path_ = center_line_calc_.calculate(
            current_left_wall_, current_right_wall_);
    } else {
        current_center_path_ = CenterPath{};
    }

    // 更新状态机
    updateStateMachine();
}

// ─────────────────────── 状态机 ─────────────────────────────

void VirtualGuideRailController::updateStateMachine() {
    // 注意：此函数在 mutex_ 已持有的情况下调用

    bool dual_wall_valid = current_left_wall_.is_valid &&
                           current_right_wall_.is_valid &&
                           current_center_path_.is_valid;

    bool fit_quality_ok =
        current_left_wall_.overall_r_squared  >= params_.wall_detector.r_squared_threshold &&
        current_right_wall_.overall_r_squared >= params_.wall_detector.r_squared_threshold;

    switch (state_) {

    case GuideRailState::WALL_DETECT:
        if (dual_wall_valid && fit_quality_ok) {
            consecutive_valid_frames_++;
            consecutive_fit_failure_frames_ = 0;
            wall_detect_timeout_frames_ = 0;
            if (consecutive_valid_frames_ >= params_.confirm_frames) {
                state_ = GuideRailState::TRACKING;
                RCLCPP_INFO(logger_, "[VirtualGuideRail] WALL_DETECT → TRACKING (连续%d帧有效)",
                            params_.confirm_frames);
            }
        } else if (dual_wall_valid) {
            // 有双墙但质量差 → FALLBACK
            consecutive_valid_frames_ = 0;
            wall_detect_timeout_frames_ = 0;
            state_ = GuideRailState::FALLBACK;
            RCLCPP_DEBUG(logger_, "[VirtualGuideRail] WALL_DETECT → FALLBACK (R²不足)");
        } else {
            // 无有效双墙：用独立计数器计超时
            consecutive_valid_frames_ = 0;
            wall_detect_timeout_frames_++;
            if (wall_detect_timeout_frames_ > 30) {
                state_ = GuideRailState::STANDBY;
                wall_detect_timeout_frames_ = 0;
                RCLCPP_INFO(logger_, "[VirtualGuideRail] WALL_DETECT → STANDBY (超时)");
            }
        }
        break;

    case GuideRailState::TRACKING:
        if (dual_wall_valid && fit_quality_ok) {
            consecutive_fit_failure_frames_ = 0;
            last_valid_time_ = rclcpp::Clock().now();
        } else {
            consecutive_fit_failure_frames_++;
            if (consecutive_fit_failure_frames_ >= params_.fit_failure_exit_frames) {
                state_ = GuideRailState::STANDBY;
                consecutive_fit_failure_frames_ = 0;
                consecutive_valid_frames_ = 0;
                RCLCPP_INFO(logger_, "[VirtualGuideRail] TRACKING → STANDBY (连续%d帧拟合失败)",
                            params_.fit_failure_exit_frames);
            } else if (dual_wall_valid) {
                state_ = GuideRailState::FALLBACK;
                RCLCPP_DEBUG(logger_, "[VirtualGuideRail] TRACKING → FALLBACK");
            }
        }
        break;

    case GuideRailState::FALLBACK:
        if (dual_wall_valid && fit_quality_ok) {
            consecutive_valid_frames_++;
            consecutive_fit_failure_frames_ = 0;
            if (consecutive_valid_frames_ >= params_.confirm_frames) {
                state_ = GuideRailState::TRACKING;
                RCLCPP_INFO(logger_, "[VirtualGuideRail] FALLBACK → TRACKING");
            }
        } else if (!dual_wall_valid) {
            consecutive_fit_failure_frames_++;
            consecutive_valid_frames_ = 0;
            if (consecutive_fit_failure_frames_ >= params_.fit_failure_exit_frames) {
                state_ = GuideRailState::STANDBY;
                consecutive_fit_failure_frames_ = 0;
                RCLCPP_INFO(logger_, "[VirtualGuideRail] FALLBACK → STANDBY");
            }
        }
        break;

    case GuideRailState::STANDBY:
        if (dual_wall_valid) {
            // 有结构了，重新检测
            state_ = GuideRailState::WALL_DETECT;
            consecutive_valid_frames_ = 0;
            RCLCPP_DEBUG(logger_, "[VirtualGuideRail] STANDBY → WALL_DETECT");
        }
        break;

    default:
        break;
    }
}

// ─────────────────────── 控制计算 ───────────────────────────

double VirtualGuideRailController::computeAngularCorrection(
    double user_linear_x)
{
    std::lock_guard<std::mutex> lock(mutex_);

    switch (state_) {
    case GuideRailState::DISABLED:
    case GuideRailState::WALL_DETECT:
    case GuideRailState::STANDBY:
        return 0.0;

    case GuideRailState::EXITING: {
        double elapsed = (rclcpp::Clock().now() - exit_start_time_).seconds();
        double fade    = std::max(0.0, 1.0 - elapsed / 0.5);
        if (fade <= 0.0) {
            // 淡出结束，回到 DISABLED，允许重新激活
            state_ = GuideRailState::DISABLED;
            return 0.0;
        }
        return exit_angular_at_start_ * fade;
    }

    case GuideRailState::FALLBACK:
        return computeFallbackCorrection(user_linear_x);

    case GuideRailState::TRACKING: {
        if (!current_center_path_.is_valid) {
            return computeFallbackCorrection(user_linear_x);
        }
        auto result = path_tracker_.track(
            current_center_path_,
            user_linear_x,
            current_center_path_.bend_index);

        // 缓存跟踪结果（用于 EXITING 淡出和调试）
        current_tracking_ = result;

        return result.is_valid
            ? result.angular_correction
            : computeFallbackCorrection(user_linear_x);
    }

    default:
        return 0.0;
    }
}

double VirtualGuideRailController::computeFallbackCorrection(
    double user_linear_x) const
{
    double offset = fallback_d_left_ - fallback_d_right_;

    if (std::abs(offset) < params_.fallback_deadzone) {
        return 0.0;
    }

    double correction = -params_.fallback_kp * offset;

    // 速度耦合：低速时减弱修正
    double speed_factor = std::clamp(
        std::abs(user_linear_x) / 0.3, 0.0, 1.0);
    correction *= speed_factor;

    return std::clamp(correction,
                      -params_.fallback_max_angular,
                       params_.fallback_max_angular);
}

double VirtualGuideRailController::getRecommendedMaxSpeed() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ == GuideRailState::TRACKING && current_center_path_.is_valid) {
        return params_.max_linear_speed * computeBendSpeedFactor();
    }
    return params_.max_linear_speed;
}

double VirtualGuideRailController::computeBendSpeedFactor() const {
    if (!current_center_path_.is_valid) return 1.0;

    double bend = current_center_path_.bend_index;
    if (bend < params_.bend_threshold) return 1.0;

    double reduction = std::clamp(bend / 1.0, 0.0, params_.bend_speed_reduction_max);
    return 1.0 - reduction;
}

// ─────────────────────── 调试数据 ───────────────────────────

WallLine VirtualGuideRailController::getLeftWall() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_left_wall_;
}

WallLine VirtualGuideRailController::getRightWall() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_right_wall_;
}

CenterPath VirtualGuideRailController::getCenterPath() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_center_path_;
}

TrackingResult VirtualGuideRailController::getTrackingResult() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_tracking_;
}

} // namespace scene_node
