#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace shared_control_assist {

// 三路水平雷达在程序内部的固定槽位。
// 注意：这里不是雷达 frame_id，而是节点内部数组 scan_points_/scan_stamps_
// 的索引约定。所有按雷达保存的健康状态、点云缓存、TF 失败时间都使用
// 这个枚举做下标，避免用裸数字 0/1/2 造成左右/后方混淆。
enum class LidarSlot : std::size_t {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR = 2,
};

// 辅助系统当前模式。
// MANUAL_ONLY:
//   关闭自动辅助，保留人工输入为主。当前实现仍可保留安全裁剪。
// OBSTACLE_ASSIST:
//   普通局部避障辅助，只限速/停车，不进行窄门居中控制。
// NARROW_DOOR_ASSIST:
//   窄门辅助，融合用户摇杆和门中心线误差，主要修正角速度。
// RECOVERY_ASSIST:
//   预留的恢复/脱困模式，当前 MVP 暂未实现专门策略。
// FAULT:
//   传感器/TF/安全条件异常，输出应趋向零速度。
enum class AssistMode : std::uint8_t {
    MANUAL_ONLY = 0,
    OBSTACLE_ASSIST = 1,
    NARROW_DOOR_ASSIST = 2,
    RECOVERY_ASSIST = 3,
    FAULT = 4,
};

// 窄门通过状态机。
// 状态含义与 docs/方案.md 保持一致：
// IDLE      : 未进入窄门流程，只做普通避障。
// SEARCH    : 看到疑似门洞，等待候选稳定；此阶段不做强自动转向。
// APPROACH  : 门洞稳定，低速向门洞中心接近。
// ALIGN     : 到达对齐距离，增强角速度修正，尝试降低横向误差。
// ENTER     : 对齐完成，车头进入门洞。
// PASSING   : 车身穿过门洞，重点保护左右间隙和后部。
// EXIT      : 尾部离开门洞，逐步释放窄门辅助。
// COMPLETE  : 通过完成，短暂停留后回到 IDLE。
// ABORT     : 用户松杆/反向/安全停车/候选丢失等异常退出。
enum class DoorState : std::uint8_t {
    IDLE = 0,
    SEARCH = 1,
    APPROACH = 2,
    ALIGN = 3,
    ENTER = 4,
    PASSING = 5,
    EXIT = 6,
    COMPLETE = 7,
    ABORT = 8,
};

// 节点运行参数集合。
//
// 设计意图：
// - 所有可调参数都集中在一个结构体里，LocalPerception、DoorDetector 等
//   算法类只持有 const 引用，不直接访问 ROS 参数服务。
// - 这样每个算法类更容易单独阅读和后续单元测试。
// - 字段默认值应与 config/shared_control_assist.yaml 保持一致；若未加载
//   YAML，节点仍应以保守参数运行。
//
// 单位约定：
// - 距离、尺寸、速度相关字段默认使用 m、m/s、rad/s。
// - 角度阈值中带 _deg 的字段使用 degree。
// - 时间阈值使用 second。
struct AssistParams {
    // ---------------- Topic 配置 ----------------
    // 摇杆输入。用户意图始终来自该 topic，辅助控制只在此基础上修正/限速。
    std::string input_control_topic = "/handle/control";
    // 最终输出到底盘控制链路的 Twist。
    std::string output_control_topic = "/nav/control";
    // 车辆实际速度，用于估计 ENTER/PASSING/EXIT 中已经前进的距离。
    std::string wheel_speed_topic = "/wheel/twist";
    // 三路水平雷达输入。每个 LaserScan 需要通过 TF 转到 base_frame。
    std::string left_scan_topic = "/scan/left";
    std::string right_scan_topic = "/scan/right";
    std::string back_scan_topic = "/scan/back";
    // 辅助开关。false 时进入 MANUAL_ONLY。
    std::string assist_enable_topic = "/assist/enable";
    // RViz/调试输出 topic。
    std::string assist_points_topic = "/assist/points";
    std::string assist_local_grid_topic = "/assist/local_grid";
    std::string assist_state_topic = "/assist/state";
    std::string debug_markers_topic = "/assist/debug/markers";
    std::string debug_reason_topic = "/assist/debug/reason";
    std::string debug_door_score_topic = "/assist/debug/door_score";
    // 所有局部几何计算所在坐标系。约定 x 向前，y 向左，z 向上。
    std::string base_frame = "base_link";

    // ---------------- 频率与超时 ----------------
    // 主控制循环频率，也是 /nav/control 的发布频率。
    double control_rate = 50.0;
    // 调试点云、栅格、Marker、状态文本发布频率。
    double debug_publish_rate = 10.0;
    // 摇杆输入超过该时间未更新时，认为用户松杆/链路异常，输出零速度。
    double command_timeout = 0.20;
    // 单路雷达数据超过该时间未更新即视为该路不新鲜。
    double lidar_timeout = 0.30;
    // 某一路雷达到 base_frame 的 TF 连续失败超过该时间后触发故障保护。
    double tf_fail_timeout = 0.50;
    // 非零控制输出要求的新鲜雷达数量。三水平雷达方案默认 3 路都在线。
    int min_fresh_lidars = 3;
    // 是否发布调试 topic。现场 CPU/带宽紧张时可关闭。
    bool publish_debug_topics = true;

    // ---------------- 车体几何 ----------------
    // vehicle_width 用于左右安全距离和门洞可通行宽度判断。
    double vehicle_length = 0.96;
    double vehicle_width = 0.68;
    // base_frame 下车体前/后边缘位置。常见设定是 base_link 在车体中心，
    // 则 front_edge 为正，rear_edge 为负。
    double front_edge = 0.48;
    double rear_edge = -0.48;
    // 过滤车体自身回波时额外扩出的边界，避免雷达扫到脚踏/车架被当障碍。
    double footprint_padding = 0.03;

    // ---------------- 雷达点过滤 ----------------
    // LaserScan 距离有效范围。NaN/Inf 以及超出该范围的点都会被丢弃。
    double scan_min_range = 0.05;
    double scan_max_range = 4.00;
    // 采样步长。1 表示不降采样；2 表示每隔一个点取一次。
    int scan_sample_step = 1;

    // ---------------- 局部栅格 ----------------
    // 栅格仅用于局部调试/可视化，不承担全局地图职责。
    double grid_resolution = 0.03;
    // base_frame 下局部栅格范围。x 前后、y 左右。
    double grid_x_min = -1.20;
    double grid_x_max = 2.20;
    double grid_y_min = -1.20;
    double grid_y_max = 1.20;
    // 栅格障碍膨胀半径，便于 RViz 中观察安全余量。
    double grid_inflation = 0.08;

    // ---------------- 安全距离 ----------------
    // stop_distance 内触发停车；slow_distance 内按线性比例降速。
    double front_stop_distance = 0.18;
    double front_slow_distance = 0.60;
    double rear_stop_distance = 0.16;
    double rear_slow_distance = 0.45;
    double side_stop_distance = 0.05;
    double side_slow_distance = 0.18;
    // 触发停车至少需要的点数，降低单点噪声误触发概率。
    int min_valid_points_for_stop = 3;
    // 软件急停后保持零速度的时间，避免刚离开阈值就立刻恢复导致抖动。
    double emergency_stop_hold_time = 0.30;

    // ---------------- 门洞检测 ----------------
    // 只在前方 ROI 内找门洞，避免侧后方障碍被误判为门边界。
    double door_roi_x_min = 0.25;
    double door_roi_x_max = 2.00;
    double door_roi_y_min = -0.90;
    double door_roi_y_max = 0.90;
    // 门宽约束。实际通行判断还会结合 vehicle_width + 2 * required_side_margin。
    double min_door_width = 0.72;
    double max_door_width = 1.20;
    // 车体两侧希望保留的最低安全余量。
    double required_side_margin = 0.05;
    // 最大自由间隙检测的 y 向离散精度和 x 向切片设置。
    double door_bin_resolution = 0.03;
    double door_x_sample_step = 0.08;
    double door_x_window = 0.08;
    // 门候选稳定性判断。连续稳定帧数越大，误触发越少，但响应更慢。
    int door_stable_frame_count = 4;
    // enter: 允许进入窄门自动辅助的置信度；keep: 已在流程中维持候选的最低置信度。
    double door_confidence_enter = 0.65;
    double door_confidence_keep = 0.45;
    // 门候选短时丢失时保持上一候选的时间，用于处理雷达瞬时遮挡/稀疏。
    double candidate_lost_timeout = 0.40;

    // ---------------- 窄门状态机 ----------------
    // 用户前推超过该速度才自动搜索/进入窄门辅助。
    double user_forward_threshold = 0.03;
    // 用户明显大角速度或反向操作时退出窄门辅助，体现人工优先。
    double user_cancel_angular_threshold = 0.60;
    // 状态机距离阈值。当前 MVP 主要使用 align_distance/enter_distance。
    double approach_distance = 1.20;
    double align_distance = 0.80;
    double enter_distance = 0.35;
    // 进入 ENTER 状态需要满足的横向/角度误差。
    double lateral_error_enter = 0.04;
    double angle_error_enter_deg = 5.0;
    // COMPLETE/ABORT 状态保持时间，之后回到 IDLE。
    double complete_hold_time = 0.50;

    // ---------------- 控制器限幅与增益 ----------------
    // MANUAL_ONLY 下的最大速度。通常略大于辅助模式。
    double manual_v_max = 0.45;
    double manual_w_max = 1.00;
    // 普通障碍辅助模式速度上限。
    double obstacle_assist_v_max = 0.35;
    double obstacle_assist_w_max = 0.80;
    // 窄门辅助速度上限。ALIGN/ENTER 会使用更低的 door_align_v_max。
    double door_v_max = 0.15;
    double door_align_v_max = 0.08;
    double door_w_max = 0.45;
    // 角速度自动修正增益：横向误差、角度误差、左右间隙差。
    double k_lateral = 1.20;
    double k_angle = 1.00;
    double k_gap = 0.80;
    // 自动辅助角速度与用户角速度的混合比例。越大越偏自动修正。
    double blend_ratio_approach = 0.50;
    double blend_ratio_align = 0.80;
    double blend_ratio_passing = 0.70;
    // 输出加速度限制，避免低速载人设备出现明显顿挫。
    double max_accel_linear = 0.25;
    double max_accel_angular = 0.80;
};

// base_frame 下的二维点。x 向前，y 向左。
// 当前算法只使用水平雷达 2D 几何，z 被过滤/忽略。
struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

// 局部障碍距离结果。
// 这些字段是安全裁剪的输入：前进看 front_*，倒车看 rear_*，
// 原地/低速转向看 left/right 与四个角点距离。
struct ObstacleDistance {
    // 前/后/左/右方向最近障碍距离。无有效障碍时为 infinity。
    double front_min = std::numeric_limits<double>::infinity();
    double rear_min = std::numeric_limits<double>::infinity();
    double left_min = std::numeric_limits<double>::infinity();
    double right_min = std::numeric_limits<double>::infinity();
    // 四个角区域最近距离，用于限制“前进+转向”或“倒车+转向”的刮碰风险。
    double front_left_min = std::numeric_limits<double>::infinity();
    double front_right_min = std::numeric_limits<double>::infinity();
    double rear_left_min = std::numeric_limits<double>::infinity();
    double rear_right_min = std::numeric_limits<double>::infinity();
    // 通道/门洞左右间隙估计，主要用于窄门居中控制和调试显示。
    double left_clearance = std::numeric_limits<double>::infinity();
    double right_clearance = std::numeric_limits<double>::infinity();
    double min_clearance = std::numeric_limits<double>::infinity();
    // 各区域有效点数。停车判断会要求达到 min_valid_points_for_stop。
    int front_points = 0;
    int rear_points = 0;
    int left_points = 0;
    int right_points = 0;
    // 是否触发对应停车区。side_stop 表示左侧或右侧任一侧过近。
    bool front_stop = false;
    bool rear_stop = false;
    bool side_stop = false;
    // 本帧是否有任意有效点。false 时安全层会更保守。
    bool valid = false;
};

// 门洞候选。
// MVP 中 source=3 表示最大自由间隙，source=2 表示平行墙 fallback。
// 当前尚未做真实线段拟合，因此 yaw/angle_error 暂时保持 0。
struct DoorCandidate {
    bool detected = false;
    double confidence = 0.0;
    // 门中心在 base_frame 下的位置。center_y > 0 表示门中心偏左。
    double center_x = 0.0;
    double center_y = 0.0;
    // 门方向相对 base_link x 轴的角度。MVP 未拟合门方向时为 0。
    double yaw = 0.0;
    // 门洞宽度与距离。distance 当前等价于 center_x。
    double width = 0.0;
    double distance = 0.0;
    // 左右边界点在 base_frame 下的位置。左边界 y 为正，右边界 y 为负。
    double left_boundary_x = 0.0;
    double left_boundary_y = 0.0;
    double right_boundary_x = 0.0;
    double right_boundary_y = 0.0;
    // 车体半宽之外的左右剩余间隙。
    double left_clearance = 0.0;
    double right_clearance = 0.0;
    // 控制误差。lateral_error=center_y；angle_error 当前 MVP 为 0。
    double angle_error = 0.0;
    double lateral_error = 0.0;
    // source/reason 使用 docs/方案.md 中的枚举约定。
    std::uint8_t source = 0;
    std::uint8_t reason = 0;
    // 与上一候选连续稳定的帧数。
    int stable_count = 0;
};

}  // namespace shared_control_assist
