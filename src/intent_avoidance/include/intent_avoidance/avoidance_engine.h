#pragma once

#include "intent_avoidance/types.h"

namespace intent_avoidance {

// ABC策略决策 + 防抖方向锁定
class AvoidanceEngine {
public:
    struct Config {
        bool enable_intent_avoidance = true;
        int direction_lock_frames = 8;
        double direction_unlock_threshold_rad = 0.524;  // ≈30°
        double speed_reduction_factor = 0.6;
        double max_angular_guidance = 0.5;              // rad/s
        double emergency_brake_distance = 0.3;          // m
        double emergency_brake_ttc = 0.5;               // s
    };

    explicit AvoidanceEngine(const Config& config);

    // 核心决策
    AvoidanceDecision decide(
        const FreeSectorList& free_sectors,
        const NarrowPassageInfo& narrow_info,
        const UserIntent& intent,
        const SpeedInfo& speed);

private:
    // 判断用户方向是否在可通行扇区内
    static bool isIntentFeasible(double intended_angle, const FreeSectorList& free_sectors);

    // 找到角度最近的可行扇区中心
    static double findNearestFreeAngle(double intended_angle, const FreeSectorList& free_sectors);

    // 找到最宽的可行扇区中心
    static double findWidestFreeAngle(const FreeSectorList& free_sectors);

    // 更新方向锁定状态
    double applyDirectionLock(double guidance_angle, const UserIntent& intent);

    Config config_;
    DirectionLock lock_;
};

}  // namespace intent_avoidance
