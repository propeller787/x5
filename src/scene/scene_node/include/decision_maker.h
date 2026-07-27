/**
 * @file decision_maker.h
 * @author likang (likang@freego.com)
 * @brief 决策器 - 基于雷达数据的速度干预决策
 * @version 0.1
 * @date 2025-10-16
 *
 * @copyright Copyright (c) 2025
 */
#pragma once

#include "clustering.h"
#include "robot_dimensions.h"
#include "smooth_filter.h"
#include "tracking.h"
#include "vehicle_interface.h"
#include <interface/msg/auxiliary_alarm.hpp>
#include <interface/msg/collision_warning.hpp>
#include <interface/msg/joystick.hpp>
#include "../../../common/utils/topic.h"
#include "../../../common/utils/warning_type.h"
#include <chrono>
#include <deque>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <interface/msg/car_info.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <atomic>
#include <string>
#include <vector>

#include "decision_params.h"
#include "evasion_detector.h"
#include "obstacle_predictor.h"
#include "risk_assessment.h"

// 声明struct 记录每个雷达前向 左侧 右侧最近点信息
struct RadarNearestPoint {
    std::string radar_name; // 雷达名称
    double forward_distance; // 最近前向距离
    double left_distance; // 最近左侧距离
    double right_distance; // 最近右侧距离
};

// 速度限制结构体
struct SpeedLimit {
    double max_linear_speed; // 最大线速度 (m/s)
    double max_angular_speed; // 最大角速度 (rad/s)
    double distance_threshold; // 距离阈值 (m)
};

// 最小前向 后向 侧向的安全距离结构体
struct MinDirectionalDistance {
    double min_forward_distance; // 最小前向安全距离 (m)
    double min_backward_distance; // 最小后向安全距离 (m)
    double min_lateral_left_distance; // 最小左侧安全距离 (m)
    double min_lateral_right_distance; // 最小右侧安全距离 (m)
};

// 方向性状态管理结构体（用于解决多方向干预的状态冲突）
struct DirectionalState {
    bool has_last_detection = false; // 是否有有效的历史检测
    double last_detected_distance = 0.0; // 最后一次检测到的有效距离
    rclcpp::Time last_detection_time; // 最后一次检测到障碍物的时间
    bool is_recovering = false; // 是否正在恢复中
    double current_speed_limit = 0.0; // 当前速度限制
    double target_speed_limit = 0.0; // 目标速度限制
    double recovery_start_speed = 0.0; // 恢复起始速度（用于线性插值）
    rclcpp::Time recovery_start_time; // 恢复开始时间
};

/** 环境配置预设 */
enum class EnvironmentProfile { INDOOR, OUTDOOR };

class DecisionMaker : public rclcpp::Node {
public:
    /** @brief 构造决策器节点，初始化参数、滤波器、订阅/发布与各方向状态 */
    DecisionMaker();
    /** @brief 析构决策器，释放资源 */
    ~DecisionMaker();

    /**
     * @brief 设置控制输出回调函数
     * @param callback 当决策器产生最终控制命令时被调用，参数为 geometry_msgs::msg::Twist
     */
    void setControlOutputCallback(std::function<void(const geometry_msgs::msg::Twist&)> callback);

    /**
     * @brief 处理指定雷达的聚类结果并更新内部聚类映射与最近点信息
     * @param clusters 该雷达当前帧的聚类列表
     * @param source_name 雷达源名称（如 left/right/left_tilt/right_tilt/back）
     */
    void processClusters(const std::vector<ClusterInfo>& clusters, const std::string& source_name);

    /**
     * @brief 更新决策器内部缓存的车辆状态（线速度、角速度、轮速、时间戳）
     * @param vehicle_state 当前车辆状态
     */
    void updateVehicleState(const VehicleState& vehicle_state);

    /**
     * @brief 缓存摇杆期望控制（由 VehicleInterface 高频回调写入，不触发重决策）
     * @param handle_control 原始摇杆控制命令（Twist）
     */
    void updateDesiredTwist(const geometry_msgs::msg::Twist& handle_control);

    /**
     * @brief 兼容入口：等价于 updateDesiredTwist
     * @param handle_control 原始摇杆控制命令（Twist）
     */
    void processHandleControl(const geometry_msgs::msg::Twist& handle_control);

    /** @brief 启用/禁用定时决策输出（透传模式时应为 false，避免重复发布） */
    void setDecisionOutputEnabled(bool enabled);

    /** @brief 由定时器调用：读取缓存期望控制并执行避障决策 */
    void runDecisionTick();

    /**
     * @brief 设置跟踪器引用，用于目标预测与数据缺失时的补帧
     * @param tracking 跟踪器共享指针
     */
    void setTracking(std::shared_ptr<Tracking> tracking);

    /**
     * @brief 注入障碍物位置预测器，用于获取轮速补偿后的聚类数据
     */
    void setObstaclePredictor(std::shared_ptr<ObstaclePredictor> predictor);

    /**
     * @brief 注入操作者避让意图检测器
     */
    void setEvasionDetector(std::shared_ptr<EvasionDetector> detector);

    /**
     * @brief 获取指定雷达的聚类目标列表
     * @param radar_name 雷达源名称
     * @return 该雷达对应的聚类列表，若不存在则返回空向量
     */
    std::vector<ClusterInfo> getRadarClusters(const std::string& radar_name) const;

    /**
     * @brief 清除指定雷达的聚类数据（用于完全更新或重置）
     * @param radar_name 雷达源名称
     */
    void clearRadarClusters(const std::string& radar_name);

    /** @brief 启用或禁用前向碰撞检测与速度干预 */
    void setForwardCollisionEnabled(bool enabled);
    /** @brief 启用或禁用侧向（左/右）碰撞检测与速度干预 */
    void setLateralCollisionEnabled(bool enabled);
    /** @brief 启用或禁用后向碰撞检测与速度干预 */
    void setBackwardCollisionEnabled(bool enabled);
    /** @brief 返回前向碰撞检测是否启用 */
    bool isForwardCollisionEnabled() const;
    /** @brief 返回侧向碰撞检测是否启用 */
    bool isLateralCollisionEnabled() const;
    /** @brief 返回后向碰撞检测是否启用 */
    bool isBackwardCollisionEnabled() const;

    /**
     * @brief 批量配置前向、侧向、后向碰撞检测开关
     * @param forward_enabled 是否启用前向
     * @param lateral_enabled 是否启用侧向
     * @param backward_enabled 是否启用后向
     */
    void configureCollisionDetection(bool forward_enabled, bool lateral_enabled, bool backward_enabled);

    /**
     * @brief 获取当前碰撞检测配置状态的字符串描述（用于日志或调试）
     * @return 描述各方向开关状态的字符串
     */
    std::string getCollisionDetectionStatus() const;

    /**
     * @brief 复位避障相关的瞬态：制动保持、平滑滤波器、各向 DirectionalState、前向制动时间窗。
     *        在切换安全功能时应调用（见 SceneNode::handleAuxiliaryService）。
     */
    void resetBrakeHoldState();
    /** @brief 设置当前是否处于巡航模式（用于屏蔽巡航下前向制动保持逻辑） */
    void setCruiseModeActive(bool active);

    /**
     * @brief 一键应用室内/室外参数预设，线程安全
     * @param profile 环境预设枚举值
     */
    void applyProfile(EnvironmentProfile profile);

    /**
     * @brief 设置侧向距离回调，每帧 calculateMinDirectionalDistance 后触发
     * @param cb 回调函数，参数为 (left_dist_m, right_dist_m)
     */
    void setLateralDistanceCallback(std::function<void(double, double)> cb) {
        lateral_distance_callback_ = std::move(cb);
    }

    /**
     * @brief 设置决策参数（预警区/减速区/制动区、速度限制、保持与恢复时间等）
     * @param params 决策参数结构体
     */
    void setDecisionParams(const DecisionParams& params);
    /** @brief 获取当前决策参数副本 */
    DecisionParams getDecisionParams() const;

    /**
     * @brief 获取决策器当前状态信息（速度限制、最近距离、恢复标志、雷达与聚类数量等）
     * @return DecisionStatus 结构体
     */
    struct DecisionStatus {
        double current_speed_limit; // 当前速度限制
        double last_detected_distance; // 最近检测距离
        bool is_recovering; // 是否正在速度恢复
        size_t active_radar_count; // 活跃雷达数量
        size_t total_cluster_count; // 总聚类数量
    };
    /** @brief 获取当前决策器状态（速度限制、最近距离、恢复标志、雷达与聚类数量） */
    DecisionStatus getStatus() const;

    /**
     * @brief 发布安全报警（前向/侧向等方向）
     * @param direction_name 方向名称标识
     * @param warning_type 报警类型枚举
     */
    void publishSafetyAlarm(const std::string& direction_name, WarningType warning_type);

    /**
     * @brief 发布后向安全报警（支持多级报警等级）
     * @param direction_name 方向名称标识
     * @param alarm_level 报警等级
     */
    void publishSafetyAlarmForBackward(const std::string& direction_name, int alarm_level);

    /**
     * @brief 发布碰撞警告消息（CollisionWarning）
     * @param warning_msg 已填充的碰撞警告消息
     */
    void publishCollisionWarning(interface::msg::CollisionWarning& warning_msg);

    /** @brief 获取当前缓存的摇杆位置消息（Joystick） */
    interface::msg::Joystick getCurrentJoystickMsg() const;

private:
    /** @brief 将 decision_params_ 中 robot_* 同步到 robot_dimensions_ */
    void syncRobotDimensionsFromParams();
    void declareSweptAvoidanceParams();
    void updateSweptAvoidanceParamsFromRos();
    scene_node::avoidance::SafetyFilterResult evaluateSweptAvoidance(
        const geometry_msgs::msg::Twist& handle_control,
        const rclcpp::Time& now);
    geometry_msgs::msg::Twist smoothDecisionOutput(const geometry_msgs::msg::Twist& control);
    void logSweptAvoidanceShadow(
        const scene_node::avoidance::SafetyFilterResult& result,
        const geometry_msgs::msg::Twist& output_control);

    // =========== 机器人尺寸/区域配置接口（对内） ===========
    /** @brief 设置机器人主体尺寸，用于碰撞区域与距离计算 */
    void setRobotDimensions(const RobotDimensions::BaseDimensions& dimensions);
    /** @brief 获取当前机器人主体尺寸 */
    RobotDimensions::BaseDimensions getRobotDimensions() const;
    /** @brief 设置碰撞检测区域（前/后/左/右等） */
    void setCollisionZones(const RobotDimensions::CollisionZones& zones);
    /** @brief 获取当前碰撞检测区域配置 */
    RobotDimensions::CollisionZones getCollisionZones() const;

    /**
     * @brief 计算聚类列表中到本车最近点的距离
     * @param clusters 聚类列表
     * @return 最近距离 (m)，若无有效聚类则返回较大值
     */
    double calculateNearestDistance(const std::vector<ClusterInfo>& clusters);

    /**
     * @brief 根据前向障碍物距离与TTC计算速度限制（线速度/角速度/距离阈值）
     * @param distance 最近前向距离 (m)
     * @param approach_velocity 接近速度 (m/s)，用于计算 TTC = distance/velocity
     * @return SpeedLimit 结构体
     */
    SpeedLimit calculateSpeedLimit(double distance, double approach_velocity);
    /**
     * @brief 根据后向障碍物距离与TTC计算后向速度限制
     * @param distance 最近后向距离 (m)
     * @param approach_velocity 接近速度 (m/s)
     * @return SpeedLimit 结构体
     */
    SpeedLimit calculateSpeedLimitForBackward(double distance, double approach_velocity);
    /**
     * @brief 根据侧向障碍物距离与TTC计算侧向速度限制
     * @param distance 最近侧向距离 (m)
     * @param approach_velocity 接近速度 (m/s)，侧向为 |角速度| * 半宽
     * @return SpeedLimit 结构体
     */
    SpeedLimit calculateSpeedLimitForLateral(double distance, double approach_velocity);

    /**
     * @brief 计算前/后/左/右四个方向的最小安全距离，并更新 min_directional_distance_ 及雷达来源、最近点 Z 等
     * @param clusters 当前参与计算的聚类（可为多雷达融合结果）
     * @param cluster_radar_map 可选，聚类索引到雷达名称的映射
     */
    void calculateMinDirectionalDistance(const std::vector<ClusterInfo>& clusters, 
                                         const std::map<size_t, std::string>& cluster_radar_map = {});

    /**
     * @brief 将前向速度限制应用到原始控制命令，返回限制后的 Twist
     * @param original_control 原始控制命令
     * @param limit 速度限制
     * @return 应用限制后的控制命令
     */
    geometry_msgs::msg::Twist applySpeedLimit(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit);
    /** @brief 将后向速度限制应用到原始控制命令 */
    geometry_msgs::msg::Twist applySpeedLimitForBackward(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit);
    /** @brief 将左侧速度限制应用到原始控制命令 */
    geometry_msgs::msg::Twist applySpeedLimitForLeft(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit);
    /** @brief 将右侧速度限制应用到原始控制命令 */
    geometry_msgs::msg::Twist applySpeedLimitForRight(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit);

    /** @brief 将 DecisionParams 中的预测器参数同步到 ObstaclePredictor */
    void syncObstaclePredictorParams();
    /** @brief 将 DecisionParams 中的避让检测参数同步到 EvasionDetector */
    void syncEvasionDetectorParams();

    /** @brief 根据当前车辆速度与雷达频率预测车辆未来位置（供目标预测使用） */
    void predictVehiclePosition();

    /** @brief 基于车辆运动学在雷达数据缺失时预测目标位置，用于补帧 */
    void predictTargetPositionsBasedOnVehicleMotion();

    /**
     * @brief 基于轮速反馈预测目标在 time_delta 后的位置，将 last_clusters 变换到预测时刻
     * @param last_clusters 上一时刻聚类
     * @param time_delta 时间间隔 (s)
     * @param vehicle_state 当前车辆状态（含轮速）
     * @return 预测后的聚类列表（车辆运动补偿）
     */
    std::vector<ClusterInfo> predictTargetsBasedOnWheelSpeed(const std::vector<ClusterInfo>& last_clusters, double time_delta,
        const VehicleState& vehicle_state);

    /** @brief 碰撞检测方向枚举，用于区分前/侧/后及倾斜雷达 */
    enum class CollisionDirection {
        FORWARD,     ///< 前向碰撞
        LEFT,        ///< 左侧碰撞
        RIGHT,       ///< 右侧碰撞
        LEFT_TILT,   ///< 左倾斜碰撞
        RIGHT_TILT,  ///< 右倾斜碰撞
        BACKWARD     ///< 后向碰撞
    };

    // ==================== 各方向速度干预入口 ====================

    /**
     * @brief 应用前向速度干预：状态保持、速度恢复、分级限速，并可能根据负平面/侧向条件调整前向距离
     * @param output_control 输入为当前控制，输出为施加前向干预后的控制（可被多方向串联修改）
     */
    void applySpeedForwardIntervention(geometry_msgs::msg::Twist& output_control);
    /** @brief 应用后向速度干预，根据后向最小距离限制线速度与角速度 */
    void applySpeedBackwardIntervention(geometry_msgs::msg::Twist& output_control);
    /** @brief 应用左侧速度干预，根据左侧最小距离限制线速度与角速度 */
    void applySpeedLeftIntervention(geometry_msgs::msg::Twist& output_control);
    /** @brief 应用右侧速度干预，根据右侧最小距离限制线速度与角速度 */
    void applySpeedRightIntervention(geometry_msgs::msg::Twist& output_control);

    // ==================== 统一干预逻辑辅助函数 ====================

    /**
     * @brief 统一的状态保持逻辑：障碍物短暂消失时在 state_hold_time 内维持上次检测结果，避免振荡
     * @param min_distance 当前最小距离
     * @param warning_zone 预警区距离阈值
     * @param state 该方向的状态（上次距离、时间、恢复标志等）
     * @param direction_name 方向名称（日志用）
     * @param current_time 当前时间
     * @return 用于后续限速计算的有效最小距离（可能为保持值）
     */
    double applyStateHoldLogic(
        double min_distance,
        double warning_zone,
        DirectionalState& state,
        const std::string& direction_name,
        rclcpp::Time current_time);

    /**
     * @brief 统一的速度恢复逻辑：障碍物消失后在一段时间内将速度限制平滑恢复到 max_speed_limit
     * @param state 该方向的状态
     * @param max_speed_limit 最大允许速度
     * @param speed_recovery_time 恢复时长 (s)
     * @param direction_name 方向名称（日志用）
     * @param current_time 当前时间
     * @param should_start_recovery 是否在本帧启动恢复
     * @return 本帧应用的速度限制值
     */
    double applySpeedRecoveryLogic(
        DirectionalState& state,
        double max_speed_limit,
        double speed_recovery_time,
        const std::string& direction_name,
        rclcpp::Time current_time,
        bool should_start_recovery = false);

    /**
     * @brief 统一的单方向干预应用：状态保持 + 恢复 + 按TTC/距离算限速并应用到 output_control
     * @param min_distance 该方向最小距离
     * @param warning_zone 预警区距离（低速回退用）
     * @param emergency_brake_distance 制动区距离（低速回退用）
     * @param warning_ttc 预警区 TTC (s)，用于判断是否需要干预
     * @param approach_velocity 接近速度 (m/s)，用于 TTC 计算
     * @param state 该方向的 DirectionalState
     * @param calc_limit_func 根据 (distance, approach_velocity) 计算 SpeedLimit 的函数
     * @param apply_limit_func 将 SpeedLimit 应用到 Twist 的函数
     * @param direction_name 方向名称
     * @param output_control 输入/输出控制命令
     * @param use_predicted 是否使用预测距离（雷达缺失时）
     */
    void applyDirectionalInterventionInternal(
        double min_distance,
        double warning_zone,
        double emergency_brake_distance,
        double warning_ttc,
        double approach_velocity,
        DirectionalState& state,
        std::function<SpeedLimit(double, double)> calc_limit_func,
        std::function<geometry_msgs::msg::Twist(const geometry_msgs::msg::Twist&, const SpeedLimit&)> apply_limit_func,
        const std::string& direction_name,
        geometry_msgs::msg::Twist& output_control,
        bool use_predicted = false);

    /** @brief 车辆信息（CarInfo）订阅回调，用于获取档位等信息 */
    void carInfoCallback(const interface::msg::CarInfo::SharedPtr msg);

    /**
     * @brief 根据本帧聚类结果更新该雷达的前/左/右最近点信息，写入 radar_nearest_point_map_
     * @param clusters 该雷达的聚类列表
     * @param radar_name 雷达源名称
     */
    void updateRadarNearestPoint(const std::vector<ClusterInfo>& clusters, const std::string& radar_name);

    /** @brief 摇杆（Joystick）订阅回调，更新 joystick_msg_ 供外部查询 */
    void joystickCallback(const interface::msg::Joystick::SharedPtr msg);
    /** @brief TFT 车辆模式 /vlago/tft/car/mode：1 舒适 2 运动，用于平滑速率限制 */
    void tftCarModeCallback(const std_msgs::msg::Int32::SharedPtr msg);

    /** @brief 根据前向速度限制判断前向预警场景（含地面以上/负平面），更新前向预警消息并可能发布 */
    void judgeForwardWarningScene(const SpeedLimit& limit);
    /** @brief 根据后向速度限制判断后向预警场景（地面以上），更新后向预警消息并可能发布 */
    void judgeBackwardWarningScene(const SpeedLimit& limit);
    /** @brief 根据左侧速度限制判断左侧预警场景（地面以上/负平面），更新左侧预警消息并可能发布 */
    void judgeLeftLateralWarningScene(const SpeedLimit& limit);
    /** @brief 根据右侧速度限制判断右侧预警场景（地面以上/负平面），更新右侧预警消息并可能发布 */
    void judgeRightLateralWarningScene(const SpeedLimit& limit);

    /**
     * @brief 根据当前各方向状态更新碰撞警告消息的预警等级等字段（可合并多方向）
     * @param warning_msg 待更新的 CollisionWarning 消息
     */
    void updateWarningStatus(interface::msg::CollisionWarning& warning_msg, int16_t previous_level);

    /** 按 tft_car_mode_ 与 decision_params_ 更新 smooth_filter_ 的 RATE_LIMITED 上限 */
    void updateSmoothFilterCommandLimits();

    /**
     * @brief 发布指定方向的干预状态（制动/减速/恢复）
     * @param direction_name 方向名称："前向"、"后向"、"左侧"、"右侧"
     * @param level 干预等级：BRAKE=制动, DECELERATION=减速, NONE=恢复/正常区
     * @param effective_distance 有效距离 (m)，用于消息或日志
     */
    void publishInterventionState(const std::string& direction_name, WarningLevel level, double effective_distance);

private:
    /** 基于轮速反馈：本周期相对上周期是否出现真实减速/已刹停，用于决定是否发布干预等级 */
    bool shouldPublishPhysicalIntervention(const std::string& direction_name, WarningLevel level) const;

    MinDirectionalDistance min_directional_distance_;
    // 控制输出回调函数
    std::function<void(const geometry_msgs::msg::Twist&)> control_output_callback_;
    /** 侧向距离回调，用于环境检测器采样 */
    std::function<void(double, double)> lateral_distance_callback_;

    // 车辆状态
    mutable std::mutex state_mutex_;
    VehicleState current_vehicle_state_;

    // 平滑滤波器
    std::unique_ptr<SmoothFilter> smooth_filter_;
    // 卡尔曼滤波器
    std::unique_ptr<SmoothFilter> kalman_filter_;
    // 跟踪器引用
    std::shared_ptr<Tracking> tracking_;
    std::shared_ptr<ObstaclePredictor> obstacle_predictor_;
    std::shared_ptr<EvasionDetector> evasion_detector_;

    // 预测目标历史数据
    std::deque<std::pair<std::vector<ClusterInfo>, rclcpp::Time>> cluster_history_;

    DecisionParams decision_params_;
    scene_node::avoidance::RiskAssessment swept_risk_assessment_;
    scene_node::avoidance::RiskAssessmentParams swept_avoidance_params_;
    std::string avoidance_engine_{"legacy"};
    std::string last_avoidance_engine_{"legacy"};
    bool avoidance_shadow_log_{true};
    double swept_footprint_back_extent_{0.25};

    // 最近一次检测到的最近距离
    double last_nearest_distance_;

    // 状态保持相关（已废弃，保留用于兼容性，实际使用各方向独立状态）
    double last_detected_distance_; // 最后一次检测到的有效距离（已废弃）
    rclcpp::Time last_detection_time_; // 最后一次检测到障碍物的时间（已废弃）
    bool has_last_detection_; // 是否有有效的历史检测（已废弃）

    // 速度恢复相关（已废弃，保留用于兼容性，实际使用各方向独立状态）
    double current_speed_limit_; // 当前速度限制（已废弃）
    double target_speed_limit_; // 目标速度限制（已废弃）
    rclcpp::Time recovery_start_time_; // 恢复开始时间（已废弃）
    bool is_recovering_; // 是否正在恢复中（已废弃）

    // 各方向独立的状态管理（解决多方向干预冲突）
    DirectionalState forward_state_; // 前向状态
    DirectionalState backward_state_; // 后向状态
    DirectionalState left_state_; // 左侧状态
    DirectionalState right_state_; // 右侧状态

    // 雷达数据更新时间映射（雷达名称 -> 最后更新时间）
    std::map<std::string, rclcpp::Time> radar_last_update_time_map_;

    // 摇杆期望控制（高频写入）与决策输出控制（定时器读取）
    geometry_msgs::msg::Twist desired_handle_control_;
    geometry_msgs::msg::Twist current_handle_control_;
    bool has_handle_control_{false};
    std::atomic<bool> decision_output_enabled_{false};
    rclcpp::TimerBase::SharedPtr decision_timer_;

    // 雷达聚类目标映射（雷达名称 -> 聚类目标列表）
    std::map<std::string, std::vector<ClusterInfo>> radar_cluster_map_;

    // =========== 新增：机器人尺寸与碰撞区域配置（对内存储） ===========
    RobotDimensions::BaseDimensions robot_dimensions_ {};
    RobotDimensions::CollisionZones collision_zones_ {};

    // =========== 前向最小距离的雷达来源和最近点信息 ===========
    std::string forward_min_distance_radar_source_; // 提供最小前向距离的雷达名称
    double forward_min_distance_nearest_z_; // 最近点的 Z 值
    double tilt_radar_distance_adjustment_ = 0.1; // tilt雷达前向距离调整值（可配置）
    
    // =========== 左侧最小距离的雷达来源和最近点信息 ===========
    std::string left_min_distance_radar_source_; // 提供最小左侧距离的雷达名称
    double left_min_distance_nearest_z_; // 最近点的 Z 值
    
    // =========== 右侧最小距离的雷达来源和最近点信息 ===========
    std::string right_min_distance_radar_source_; // 提供最小右侧距离的雷达名称
    double right_min_distance_nearest_z_; // 最近点的 Z 值
    
    // =========== 侧向tilt雷达距离调整值 ===========
    double lateral_tilt_radar_distance_adjustment_ = 0.2; // tilt雷达侧向距离调整值（可配置）
    double forward_tilt_radar_distance_adjustment_when_lateral_triggered_ = 0.4; // 当侧向满足条件时，前向距离调整值（可配置）

    rclcpp::Publisher<interface::msg::AuxiliaryAlarm>::SharedPtr auxiliary_alarm_publisher_;
    rclcpp::Publisher<interface::msg::AuxiliaryAlarm>::SharedPtr auxiliary_alarm_publisher_back_;
    rclcpp::Publisher<interface::msg::CollisionWarning>::SharedPtr collision_warning_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr narrow_end_publisher_;
    // 获取档位信息
    rclcpp::Subscription<interface::msg::CarInfo>::SharedPtr car_info_subscription_;

    // 获取摇杆位置
    rclcpp::Subscription<interface::msg::Joystick>::SharedPtr joystick_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tft_car_mode_subscription_;
    // 摇杆位置
    interface::msg::Joystick joystick_msg_;
    /** TFT 车辆模式缓存：1 舒适 2 运动（tftCarModeTopic） */
    std::atomic<int32_t> tft_car_mode_ { 1 };

    // 每个雷达的最近点信息
    std::map<std::string, RadarNearestPoint> radar_nearest_point_map_;
    // 后向距离最近状态，用于判断是否需要进行后向速度限制
    bool is_backward_distance_nearest_ = false;
    //前向障碍是否为负平面
    bool is_forward_obstacle_negative_plane_ = false;
    //侧向障碍是否为负平面
    bool is_lateral_obstacle_negative_plane_ = false;
    // 减速区 速度抑制 原子变量
    std::atomic<bool> is_deceleration_zone_speed_suppressed_ = false;
    // 刹停区 速度抑制 原子变量
    std::atomic<bool> is_emergency_brake_zone_speed_suppressed_ = false;

    // 前向制动保持：TTC紧急制动后，障碍物未移动(≤2cm)且摇杆未回中期间，跳过前向干预直接置零
    std::atomic<bool> forward_brake_hold_{false};
    double forward_brake_hold_distance_ = 0.0;
    // 当前是否处于巡航模式（仅用于控制前向制动保持逻辑是否生效）
    std::atomic<bool> cruise_mode_active_{false};
    // 后向制动保持：与上对称，倒车刹停后障碍物未动(≤2cm) 且 仍有后退量(handle/反馈) 时持续置零
    std::atomic<bool> backward_brake_hold_{false};
    double backward_brake_hold_distance_ = 0.0;

    // 前向预警
    interface::msg::CollisionWarning forward_warning_msg_ {};
    // 后向预警
    interface::msg::CollisionWarning backward_warning_msg_ {};
    // 左侧预警
    interface::msg::CollisionWarning left_warning_msg_ {};
    // 右侧预警
    interface::msg::CollisionWarning right_warning_msg_ {};
    // 负平面前向预警
    interface::msg::CollisionWarning negative_plane_forward_warning_msg_ {};
    // 负平面左侧预警
    interface::msg::CollisionWarning negative_plane_left_warning_msg_ {};
    // 负平面右侧预警
    interface::msg::CollisionWarning negative_plane_right_warning_msg_ {};

    // 同一方向同一级别上次发布时间，用于 1 秒内最多发送一次
    std::map<std::string, rclcpp::Time> last_intervention_publish_time_;

    // processHandleControl 每周期开头采样轮速：与上一周期对比判断真实减速/制动
    double fb_linear_prev_{0.0};
    double fb_linear_last_{0.0};
    double fb_angular_prev_{0.0};
    double fb_angular_last_{0.0};
    int speed_feedback_sample_count_{0};
    bool speed_feedback_ready_{false};
    // 最近一次前向制动触发时间（用于制动后短时间低速清零）
    rclcpp::Time last_forward_brake_time_;
    // 后向平顺限速内部状态
    rclcpp::Time last_backward_limit_time_;
    rclcpp::Time backward_emergency_start_time_;
    double last_backward_limited_linear_x_ = 0.0;
    bool backward_emergency_active_ = false;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr decision_params_cb_handle_;
};
