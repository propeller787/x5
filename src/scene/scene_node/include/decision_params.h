/**
 * @file decision_params.h
 * @brief 决策器可调参数结构体（与 decision_maker_params.yaml / ROS 参数名一致）
 */
#pragma once

#include <cstddef>

/** 决策参数（默认值与 ROS declare_parameter 缺省一致） */
struct DecisionParams {
    double radar_frequency = 13.0; // 雷达频率 (Hz)
    double prediction_time = 0.1; // 预测时间 (s)

    // 车体几何（base_link，与 URDF 碰撞一致；由 decision_maker_params.yaml / ros2 param 配置）
    double robot_length = 0.88; // 前后方向尺寸 (m)
    double robot_width = 0.70; // 左右方向尺寸 (m)
    double robot_height = 0.8; // 上下方向尺寸 (m)
    double robot_safety_margin_front = 0.10;
    double robot_safety_margin_back = 0.20;
    double robot_safety_margin_side = 0.03;

    // 前向碰撞距离阈值（由外到内）
    double warning_zone = 3.0; // 预警区距离 (m)
    double deceleration_zone = 2.2; // 减速区距离 (m)
    double emergency_brake_distance = 0.35; // 制动区距离 (m)
    double fixed_stopping_distance_forward = 0.25; // 固定停车距离 (m)，小于此距离该方向速度=0，不判断TTC

    // TTC (Time-To-Collision) 阈值 (s) - 基于碰撞时间的干预，速度自适应
    double warning_ttc = 2.0; // 预警区 TTC (s)
    double deceleration_ttc = 1.05; // 减速区 TTC (s)
    double emergency_brake_ttc = 0.5; // 制动区 TTC 基准值 (s)，高速时动态增大
    double emergency_brake_ttc_decel = 2.2; // 减速度 (m/s²)
    double emergency_brake_ttc_base = 0.09; // 扫描+反应+执行延迟 (s)
    double emergency_brake_ttc_margin = 0.0; // 安全裕度 (s)
    /** 接近速度大于该值 (m/s) 时使用 deceleration_ttc_high_speed / emergency_brake_ttc_high_speed；默认 7 km/h = 7/3.6；<=0 表示始终用低速剖面 */
    double forward_ttc_high_speed_threshold_m_s = 7.0 / 3.6;
    /** 高速剖面：须 > emergency_brake_ttc_high_speed，且通常 >= deceleration_ttc 以更保守 */
    double deceleration_ttc_high_speed = 1.05;
    double emergency_brake_ttc_high_speed = 0.5;
    double min_approach_velocity_for_ttc = 0.05; // 低于此速度时使用距离阈值 (m/s)
    double foot_distance_to_radar = 0.32; // 脚踏距离雷达距离 (m)
    double radar_distance_to_vehicle_rear = 0.22; // 后向雷达距离车辆后缘距离 (m)

    // 前向制动时摇杆 Y 死区锁角速：总开关；关闭则忽略 forward_brake_joystick_y_deadzone
    bool enable_forward_brake_joystick_y_deadzone = true;
    // |rocker_y| 小于该阈值（与 Joystick.msg 同量纲）则角速度置 0；<=0 时即使开关开启也不生效
    double forward_brake_joystick_y_deadzone = 50.0;
    // 前向制动死区锁角速时：Tracking 轮速线速度须 < 该值 (m/s) 才置零角速度，避免仍滑行时误锁
    double forward_brake_chassis_speed_threshold = 0.05;
    // 前向制动触发后该时间窗内（ms），若输出线速度低于阈值则强制置零
    double forward_brake_zero_speed_window_ms = 100.0;
    // 前向制动触发后低速清零阈值 (km/h)
    double forward_brake_zero_speed_threshold_kmh = 0.5;

    // 侧向碰撞距离阈值（由外到内）
    double lateral_dominance_ratio = 0.35;
    double warning_zone_lateral = 2.0; // 预警区距离 (m)
    double deceleration_zone_lateral = 0.8; // 减速区距离 (m)
    double emergency_brake_distance_lateral = 0.15; // 制动区距离 (m)
    double fixed_stopping_distance_lateral = 0.12; // 固定停车距离 (m)
    double warning_ttc_lateral = 1.5; // 侧向预警区 TTC (s)
    double deceleration_ttc_lateral = 0.8; // 侧向减速区 TTC (s)
    double emergency_brake_ttc_lateral = 0.4; // 侧向制动区 TTC (s)

    // 后向碰撞距离阈值（由外到内）
    double warning_zone_backward = 2.0; // 预警区距离 (m)
    double deceleration_zone_backward = 1.9; // 减速区距离 (m)
    double emergency_brake_distance_backward = 0.2; // 制动区距离 (m)
    double fixed_stopping_distance_backward = 0.08; // 固定停车距离 (m)
    double warning_ttc_backward = 1.5; // 后向预警区 TTC (s)
    double deceleration_ttc_backward = 0.7; // 后向减速区 TTC (s)
    double emergency_brake_ttc_backward = 0.3; // 后向制动区 TTC (s)
    double warning_distance_backward = 0.5; // 后向报警距离 (m)
    // 后向平顺制动参数
    double backward_brake_decel_max_m_s2 = 0.8; // 倒车减速斜率上限 (m/s^2)
    double backward_recover_accel_max_m_s2 = 0.5; // 倒车恢复斜率上限 (m/s^2)
    double backward_emergency_creep_speed = 0.04; // 急停确认窗口内保留爬行速度 (m/s)
    double backward_emergency_hold_ms = 120.0; // 急停置零确认窗口 (ms)

    double min_speed_limit = 0.056; // 最小速度限制 (m/s)
    double max_speed_limit = 2.222; // 最大速度限制 (m/s)
    double max_speed_limit_backward = 2.222 * 0.5; // 后向最大速度限制 (m/s)
    double absolute_max_linear_speed = 2.222; // 绝对最大线速度 (m/s)
    double absolute_max_angular_speed = 3.0; // 绝对最大角速度 (rad/s)

    double state_hold_time = 0.3; // 状态保持时间 (s)
    double speed_recovery_time = 1.5; // 速度恢复时间 (s)

    double radar_timeout = 0.05; // 雷达数据超时时间 (s)
    size_t max_history_size = 10; // 最大历史数据大小

    bool enable_forward_collision = true;
    bool enable_lateral_collision = true;
    bool enable_backward_collision = true;

    // 避让意图检测（轮速反馈 ω/v；与 Tracking 同源）衰减前向制动
    bool enable_evasion_detection = true;              // 总开关
    double evasion_angular_threshold = 0.15;           // |omega_fb| 达到此值起算避让 (rad/s)
    double evasion_angular_full = 0.5;                 // |omega_fb| 满避让角速度 (rad/s)
    double evasion_min_factor = 0.2;                   // 满避让时衰减因子下限 [0,1]，越小前向限速越接近 max_speed_limit

    // 轮速驱动的障碍物位置预测（运动补偿）
    bool enable_obstacle_prediction = true;            // 总开关
    double obstacle_prediction_max_age = 0.5;          // 累计补偿时间上限 (s)
    double obstacle_prediction_drift_ratio = 0.5;      // 距离偏差比例阈值
    double obstacle_prediction_min_valid_x = -1.0;     // 最小有效 x 坐标 (m)

    // 输出平滑 RATE_LIMITED（/nav/control 前）：舒适模式 1=TFT，每周期变化上限与线加速度 (m/s²)
    double smooth_comfort_max_linear_delta = 0.05;
    double smooth_comfort_max_angular_delta = 0.05;
    double smooth_comfort_max_linear_accel_m_s2 = 0.05;
    // 运动模式 2=TFT
    double smooth_sport_max_linear_delta = 0.05;
    double smooth_sport_max_angular_delta = 0.05;
    double smooth_sport_max_linear_accel_m_s2 = 0.05;
};
