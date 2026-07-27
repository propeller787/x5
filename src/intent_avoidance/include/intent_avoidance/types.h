#pragma once

#include <vector>
#include <cstdint>

namespace intent_avoidance {

// 雷达安装参数（静态标定值）
struct RadarMount {
    double x;      // 雷达在base_link下的X偏移(m)，前向为正
    double y;      // 雷达在base_link下的Y偏移(m)，左向为正
    double yaw;    // 雷达偏航角(rad)
};

// 阻塞扇区：雷达扫描中检测到障碍物的一段连续角度范围
struct ObstacleSector {
    double angle_start;   // 阻塞起始角(rad)，base_link坐标系
    double angle_end;     // 阻塞结束角(rad)
    double min_distance;  // 扇区内最近障碍距离(m)
};

using ObstacleSectorList = std::vector<ObstacleSector>;

// 可通行扇区：阻塞扇区的补集
struct FreeSector {
    double angle_center;  // 扇区中心角(rad)
    double angle_start;   // 起始角(rad)
    double angle_end;     // 结束角(rad)
    double width;         // 扇区宽度(rad)
    double min_clearance; // 扇区内最小通行距离(m)
};

using FreeSectorList = std::vector<FreeSector>;

// 窄道检测信息
struct NarrowPassageInfo {
    bool is_narrow = false;
    double passage_width = 0.0;     // 通道宽度(m)
    double center_offset = 0.0;     // 中心线偏移(m)，正值偏左
    double left_boundary_dist = 0.0;  // 左边界距离(m)
    double right_boundary_dist = 0.0; // 右边界距离(m)
};

// 用户意图
struct UserIntent {
    double angle_deg = 0.0;        // 意图方向角(deg)，atan2(y, x)，右正左负
    double angle_rad = 0.0;        // 意图方向角(rad)
    double strength = 0.0;         // 意图强度 [0, 1] = sqrt(x²+y²)/100
    bool has_forward_intent = false;   // rocker_x > deadzone，有前推意图
    bool has_steering_intent = false;  // |rocker_y| > deadzone，有转向意图
    bool is_neutral = true;            // 摇杆回中
    int16_t rocker_x = 0;
    int16_t rocker_y = 0;
};

// 方向锁定（防抖）
struct DirectionLock {
    double locked_angle_rad = 0.0;  // 锁定的引导方向(rad)
    int remaining_frames = 0;       // 剩余锁定帧数
    bool active = false;
};

// 避障决策结果
struct AvoidanceDecision {
    enum class Strategy {
        DIRECT_PASS,       // 情况A：用户方向可行，直接执行
        PROJECT_NEAREST,   // 情况B1：不可行但有意图，投影最近方向
        SELECT_BEST,       // 情况B2：不可行且无意图，选最佳方向
        FORCE_BRAKE,       // 情况C：无空间，强制刹车
        NARROW_GUIDE,      // 窄道居中引导
        DISABLED           // 模块禁用，直通
    };

    Strategy strategy = Strategy::DISABLED;
    double target_linear_x = 0.0;   // 目标线速度(m/s)
    double target_angular_z = 0.0;  // 目标角速度(rad/s)
    double guidance_angle_rad = 0.0; // 引导方向角(rad)
    bool braking = false;
};

// 车速信息
struct SpeedInfo {
    double linear_x = 0.0;   // 实际线速度(m/s)
    double angular_z = 0.0;  // 实际角速度(rad/s)
};

}  // namespace intent_avoidance
