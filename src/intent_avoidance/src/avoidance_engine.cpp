#include "intent_avoidance/avoidance_engine.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace intent_avoidance {

AvoidanceEngine::AvoidanceEngine(const Config& config)
    : config_(config)
{
}

AvoidanceDecision AvoidanceEngine::decide(
    const FreeSectorList& free_sectors,
    const NarrowPassageInfo& narrow_info,
    const UserIntent& intent,
    const SpeedInfo& speed)
{
    AvoidanceDecision decision;

    if (!config_.enable_intent_avoidance) {
        decision.strategy = AvoidanceDecision::Strategy::DISABLED;
        return decision;
    }

    // ============ 情况C：无任何可通行空间 → 强制刹车 ============
    if (free_sectors.empty()) {
        decision.strategy = AvoidanceDecision::Strategy::FORCE_BRAKE;
        decision.target_linear_x = 0.0;
        decision.target_angular_z = 0.0;
        decision.braking = true;
        return decision;
    }

    // TTC 硬停检测
    if (intent.has_forward_intent && speed.linear_x > 0.01) {
        // 找正前方最近的障碍距离
        double min_forward_dist = std::numeric_limits<double>::max();
        for (const auto& fs : free_sectors) {
            if (fs.min_clearance < min_forward_dist) {
                min_forward_dist = fs.min_clearance;
            }
        }
        // 如果有阻塞扇区横跨正前方
        for (const auto& fs : free_sectors) {
            if (fs.angle_start <= 0.0 && fs.angle_end >= 0.0) {
                min_forward_dist = std::min(min_forward_dist, fs.min_clearance);
            }
        }

        if (min_forward_dist < config_.emergency_brake_distance) {
            decision.strategy = AvoidanceDecision::Strategy::FORCE_BRAKE;
            decision.target_linear_x = 0.0;
            decision.target_angular_z = 0.0;
            decision.braking = true;
            return decision;
        }

        double ttc = min_forward_dist / speed.linear_x;
        if (ttc < config_.emergency_brake_ttc) {
            decision.strategy = AvoidanceDecision::Strategy::FORCE_BRAKE;
            decision.target_linear_x = 0.0;
            decision.target_angular_z = 0.0;
            decision.braking = true;
            return decision;
        }
    }

    // ============ 窄道优先：居中引导 ============
    if (narrow_info.is_narrow) {
        decision.strategy = AvoidanceDecision::Strategy::NARROW_GUIDE;
        // 引导角速度：偏移越大纠正越强，负偏移(偏右)时左转补偿
        double correction_gain = config_.max_angular_guidance / (config_.emergency_brake_distance * 2.0);
        double target_wz = -narrow_info.center_offset * correction_gain;  // center_offset +偏左，需右转(负wz)
        decision.guidance_angle_rad = (target_wz > 0) ? 0.3 : -0.3;  // 近似方向
        decision.target_angular_z = applyDirectionLock(target_wz, intent);
        decision.target_linear_x = intent.has_forward_intent
            ? speed.linear_x * config_.speed_reduction_factor
            : 0.0;
        return decision;
    }

    // ============ 情况A：用户方向可行 ============
    if (isIntentFeasible(intent.angle_rad, free_sectors)) {
        decision.strategy = AvoidanceDecision::Strategy::DIRECT_PASS;
        // 用户方向可行，保留其转向意图
        // 由上游根据摇杆x/y映射angular.z，这里仅限幅
        double steering_strength = static_cast<double>(intent.rocker_y) / 100.0;
        decision.target_angular_z = steering_strength * config_.max_angular_guidance;
        decision.target_angular_z = std::clamp(decision.target_angular_z,
            -config_.max_angular_guidance, config_.max_angular_guidance);
        decision.guidance_angle_rad = intent.angle_rad;

        // 角度偏越大越减速
        double reduction = std::cos(intent.angle_rad);
        reduction = std::max(reduction, config_.speed_reduction_factor);
        decision.target_linear_x = intent.has_forward_intent ? speed.linear_x * reduction : 0.0;
        return decision;
    }

    // ============ 情况B：不可行但存在可通行空间 ============
    double guidance_angle;

    if (intent.has_steering_intent) {
        // B1: 有转向意图 → 投影到最近可行方向
        decision.strategy = AvoidanceDecision::Strategy::PROJECT_NEAREST;
        guidance_angle = findNearestFreeAngle(intent.angle_rad, free_sectors);
    } else {
        // B2: 无转向意图 → 选最宽方向
        decision.strategy = AvoidanceDecision::Strategy::SELECT_BEST;
        guidance_angle = findWidestFreeAngle(free_sectors);
    }

    decision.guidance_angle_rad = guidance_angle;

    // 应用方向锁定防抖
    double locked_angle = applyDirectionLock(guidance_angle, intent);

    // 角度误差 → 角速度 (P控制)
    double angle_error = locked_angle;  // base_link前向为0
    double wz = angle_error * 1.5;  // gain，将角误差转换为角速度
    decision.target_angular_z = std::clamp(wz, -config_.max_angular_guidance, config_.max_angular_guidance);

    // 避障时降速
    decision.target_linear_x = intent.has_forward_intent
        ? speed.linear_x * config_.speed_reduction_factor
        : 0.0;

    return decision;
}

bool AvoidanceEngine::isIntentFeasible(double intended_angle, const FreeSectorList& free_sectors)
{
    for (const auto& fs : free_sectors) {
        if (intended_angle >= fs.angle_start && intended_angle <= fs.angle_end) {
            return true;
        }
    }
    return false;
}

double AvoidanceEngine::findNearestFreeAngle(double intended_angle, const FreeSectorList& free_sectors)
{
    double best = 0.0;
    double min_diff = std::numeric_limits<double>::max();

    for (const auto& fs : free_sectors) {
        double diff = std::abs(intended_angle - fs.angle_center);
        if (diff < min_diff) {
            min_diff = diff;
            best = fs.angle_center;
        }
    }

    return best;
}

double AvoidanceEngine::findWidestFreeAngle(const FreeSectorList& free_sectors)
{
    double best = 0.0;
    double max_width = -1.0;

    for (const auto& fs : free_sectors) {
        if (fs.width > max_width) {
            max_width = fs.width;
            best = fs.angle_center;
        }
    }

    return best;
}

double AvoidanceEngine::applyDirectionLock(double guidance_angle, const UserIntent& intent)
{
    // 解锁条件检查
    bool should_unlock = false;

    if (!lock_.active) {
        should_unlock = true;
    } else if (lock_.remaining_frames <= 0) {
        should_unlock = true;  // 锁定期满
    } else if (std::abs(intent.angle_rad - lock_.locked_angle_rad) > config_.direction_unlock_threshold_rad) {
        should_unlock = true;  // 用户大幅改变意图
    }

    if (should_unlock) {
        lock_.locked_angle_rad = guidance_angle;
        lock_.remaining_frames = config_.direction_lock_frames;
        lock_.active = true;
        return guidance_angle;
    }

    // 保持锁定
    lock_.remaining_frames--;
    return lock_.locked_angle_rad;
}

}  // namespace intent_avoidance
