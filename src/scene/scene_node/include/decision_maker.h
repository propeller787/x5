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
#include <vector>

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
    rclcpp::Time recovery_start_time; // 恢复开始时间
};

// 决策参数
struct DecisionParams {
    double radar_frequency = 10.0; // 雷达频率 (Hz)
    double prediction_time = 0.1; // 预测时间 (s)

    // 前向碰撞距离阈值（由外到内）
    double warning_zone = 3.0; // 预警区距离 (m)
    double deceleration_zone = 1.9; // 减速区距离 (m)
    double emergency_brake_distance = 0.20; // 制动区距离 (m)
    // 脚踏距离雷达距离
    double foot_distance_to_radar = 0.2; // 脚踏距离雷达距离 (m)
    // 后向雷达距离车辆后缘距离
    double radar_distance_to_vehicle_rear = 0.2; // 后向雷达距离车辆后缘距离 (m)

    // 侧向碰撞距离阈值（由外到内）
    double warning_zone_lateral = 2.0; // 预警区距离 (m)
    double deceleration_zone_lateral = 0.8; // 减速区距离 (m)
    double emergency_brake_distance_lateral = 0.15; // 制动区距离 (m)

    // 后向碰撞距离阈值（由外到内）
    double warning_zone_backward = 2.0; // 预警区距离 (m)
    double deceleration_zone_backward = 1.9; // 减速区距离 (m)
    double emergency_brake_distance_backward = 0.2; // 制动区距离 (m)
    // 后向报警距离
    double warning_distance_backward = 0.5; // 报警距离 (m)
    // 速度限制（干预控制）
    double min_speed_limit = 0.056; // 最小速度限制 (m/s) = 0.2km/h
    double max_speed_limit = 1.67; // 最大速度限制 (m/s) = 6km/h

    double max_speed_limit_backward = 1.67 * 0.5;//后向最大速度限制 (m/s) = 3km/h
    // 绝对安全限制（硬限制，防止任何情况下超速）
    double absolute_max_linear_speed = 2.0; // 绝对最大线速度 (m/s)
    double absolute_max_angular_speed = 1.0; // 绝对最大角速度 (rad/s)

    // 状态保持与平滑恢复
    double state_hold_time = 0.3; // 状态保持时间 (s) - 避免振荡
    double speed_recovery_time = 1.5; // 速度恢复时间 (s) - 平滑恢复

    double radar_timeout = 0.05; // 雷达数据超时时间 (s)
    size_t max_history_size = 10; // 最大历史数据大小

    // 碰撞检测开关配置
    bool enable_forward_collision = true; // 启用前向碰撞检测
    bool enable_lateral_collision = true; // 启用侧向碰撞检测
    bool enable_backward_collision = true; // 启用后向碰撞检测
};

class DecisionMaker : public rclcpp::Node {
public:
    DecisionMaker();
    ~DecisionMaker();

    // 设置控制输出回调函数
    void setControlOutputCallback(std::function<void(const geometry_msgs::msg::Twist&)> callback);

    // 处理聚类结果并做出决策
    void processClusters(const std::vector<ClusterInfo>& clusters, const std::string& source_name);

    // 更新车辆状态
    void updateVehicleState(const VehicleState& vehicle_state);

    // 处理摇杆控制输入
    void processHandleControl(const geometry_msgs::msg::Twist& handle_control);

    // 设置跟踪器引用
    void setTracking(std::shared_ptr<Tracking> tracking);

    // 获取指定雷达的聚类目标
    std::vector<ClusterInfo> getRadarClusters(const std::string& radar_name) const;

    // 清除指定雷达的聚类数据（用于完全更新）
    void clearRadarClusters(const std::string& radar_name);

    // 碰撞检测开关控制接口
    void setForwardCollisionEnabled(bool enabled);
    void setLateralCollisionEnabled(bool enabled);
    void setBackwardCollisionEnabled(bool enabled);
    bool isForwardCollisionEnabled() const;
    bool isLateralCollisionEnabled() const;
    bool isBackwardCollisionEnabled() const;

    // 批量配置碰撞检测开关
    void configureCollisionDetection(bool forward_enabled, bool lateral_enabled, bool backward_enabled);

    // 获取当前碰撞检测配置状态
    std::string getCollisionDetectionStatus() const;

    // 参数配置接口
    void setDecisionParams(const DecisionParams& params);
    DecisionParams getDecisionParams() const;

    // 获取决策器状态信息
    struct DecisionStatus {
        double current_speed_limit; // 当前速度限制
        double last_detected_distance; // 最近检测距离
        bool is_recovering; // 是否正在速度恢复
        size_t active_radar_count; // 活跃雷达数量
        size_t total_cluster_count; // 总聚类数量
    };
    DecisionStatus getStatus() const;

    // 发布安全报警
    void publishSafetyAlarm(const std::string& direction_name, WarningType warning_type);

    // 发布后向安全报警
    void publishSafetyAlarmForBackward(const std::string& direction_name, int alarm_level);

    // 获取当前摇杆位置
    interface::msg::Joystick getCurrentJoystickMsg() const;
    
private:
    // =========== 新增：机器人尺寸/区域配置接口（对内） ===========
    // 设置机器人主体尺寸
    void setRobotDimensions(const RobotDimensions::BaseDimensions& dimensions);
    // 获取机器人主体尺寸
    RobotDimensions::BaseDimensions getRobotDimensions() const;
    // 设置碰撞检测区域
    void setCollisionZones(const RobotDimensions::CollisionZones& zones);
    // 获取碰撞检测区域
    RobotDimensions::CollisionZones getCollisionZones() const;

    // 计算最近聚类点的距离
    double calculateNearestDistance(const std::vector<ClusterInfo>& clusters);

    // 根据距离计算速度限制
    SpeedLimit calculateSpeedLimit(double distance);
    // 根据距离计算后向速度限制
    SpeedLimit calculateSpeedLimitForBackward(double distance);
    // 根据距离计算侧向速度限制
    SpeedLimit calculateSpeedLimitForLateral(double distance);

    // 计算最小安全距离
    void calculateMinDirectionalDistance(const std::vector<ClusterInfo>& clusters, 
                                         const std::map<size_t, std::string>& cluster_radar_map = {});

    // 应用速度限制
    geometry_msgs::msg::Twist applySpeedLimit(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit);
    // 应用后向速度限制
    geometry_msgs::msg::Twist applySpeedLimitForBackward(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit);
    // 应用左侧速度限制
    geometry_msgs::msg::Twist applySpeedLimitForLeft(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit);
    // 应用右侧速度限制
    geometry_msgs::msg::Twist applySpeedLimitForRight(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit);

    // 预测车辆未来位置（基于当前速度和雷达频率）
    void predictVehiclePosition();

    // 基于车辆运动预测目标位置（雷达数据缺失时使用）
    void predictTargetPositionsBasedOnVehicleMotion();

    // 基于轮速反馈预测目标位置
    std::vector<ClusterInfo> predictTargetsBasedOnWheelSpeed(const std::vector<ClusterInfo>& last_clusters, double time_delta,
        const VehicleState& vehicle_state);

    // 碰撞检测策略
    enum class CollisionDirection {
        FORWARD, // 前向碰撞
        LEFT, // 左侧碰撞
        RIGHT, // 右侧碰撞
        LEFT_TILT, // 左倾斜碰撞
        RIGHT_TILT, // 右倾斜碰撞
        BACKWARD // 后向碰撞
    };

    // ==================== 辅助函数 ====================

    // 应用速度前向干预（修改为接受输出控制引用，支持多方向同时干预）
    void applySpeedForwardIntervention(geometry_msgs::msg::Twist& output_control);
    // 应用速度后向干预
    void applySpeedBackwardIntervention(geometry_msgs::msg::Twist& output_control);
    // 应用速度左侧干预
    void applySpeedLeftIntervention(geometry_msgs::msg::Twist& output_control);
    // 应用速度右侧干预
    void applySpeedRightIntervention(geometry_msgs::msg::Twist& output_control);

    // ==================== 统一的干预逻辑辅助函数 ====================
    // 统一的状态保持逻辑
    double applyStateHoldLogic(
        double min_distance,
        double warning_zone,
        DirectionalState& state,
        const std::string& direction_name,
        rclcpp::Time current_time);

    // 统一的速度恢复逻辑
    double applySpeedRecoveryLogic(
        DirectionalState& state,
        double max_speed_limit,
        double speed_recovery_time,
        const std::string& direction_name,
        rclcpp::Time current_time,
        bool should_start_recovery = false);

    // 统一的干预应用函数（内部辅助函数）
    void applyDirectionalInterventionInternal(
        double min_distance,
        double warning_zone,
        double emergency_brake_distance,
        DirectionalState& state,
        std::function<SpeedLimit(double)> calc_limit_func,
        std::function<geometry_msgs::msg::Twist(const geometry_msgs::msg::Twist&, const SpeedLimit&)> apply_limit_func,
        const std::string& direction_name,
        geometry_msgs::msg::Twist& output_control,
        bool use_predicted = false);

    // 获取档位信息
    void carInfoCallback(const interface::msg::CarInfo::SharedPtr msg);

    // 更新每个雷达的最近点信息
    void updateRadarNearestPoint(const std::vector<ClusterInfo>& clusters, const std::string& radar_name);

    // 获取摇杆位置
    void joystickCallback(const interface::msg::Joystick::SharedPtr msg);

    // 判断前向预警场景(地面以上 负平面)
    void judgeForwardWarningScene(const SpeedLimit& limit);
    // 判断后向预警场景(地面以上)
    void judgeBackwardWarningScene(const SpeedLimit& limit);
    // 判断侧向预警场景(地面以上 负平面)
    void judgeLateralWarningScene(const SpeedLimit& limit);
private:
    MinDirectionalDistance min_directional_distance_;
    // 控制输出回调函数
    std::function<void(const geometry_msgs::msg::Twist&)> control_output_callback_;

    // 车辆状态
    mutable std::mutex state_mutex_;
    VehicleState current_vehicle_state_;

    // 平滑滤波器
    std::unique_ptr<SmoothFilter> smooth_filter_;
    // 卡尔曼滤波器
    std::unique_ptr<SmoothFilter> kalman_filter_;
    // 跟踪器引用
    std::shared_ptr<Tracking> tracking_;

    // 预测目标历史数据
    std::deque<std::pair<std::vector<ClusterInfo>, rclcpp::Time>> cluster_history_;

    DecisionParams decision_params_;

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

    // 当前摇杆控制命令
    geometry_msgs::msg::Twist current_handle_control_;
    bool has_handle_control_;

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

    // 获取档位信息
    rclcpp::Subscription<interface::msg::CarInfo>::SharedPtr car_info_subscription_;

    // 获取摇杆位置
    rclcpp::Subscription<interface::msg::Joystick>::SharedPtr joystick_subscription_;
    // 摇杆位置
    interface::msg::Joystick joystick_msg_;

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
};