/**
 * @file decision_maker.cpp
 * @author likang (likang@freego.com)
 * @brief 决策器实现 - 基于雷达数据的速度干预决策
 * @version 0.1
 * @date 2025-10-16
 *
 * @copyright Copyright (c) 2025
 *
 * 本文件实现了决策器的核心功能，包括：
 * - 雷达聚类数据处理和碰撞检测
 * - 基于距离的速度干预决策
 * - 摇杆控制命令的处理和平滑输出
 * - 目标位置预测和跟踪
 */

#include "decision_maker.h"
#include "decision_maker_params_io.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace {
/** 两位小数取整；若原值 >0 但取整后为 0（亚厘米），保留原值，避免误判为贴脸 0 m */
double round_distance_keep_subcentimeter(double v)
{
    const double r = std::round(v * 100.0) / 100.0;
    return (v > 1e-9 && std::abs(r) < 1e-9) ? v : r;
}
} // namespace

// ==================== 构造函数和初始化 ====================

DecisionMaker::DecisionMaker()
    : rclcpp::Node("decision_maker")
{
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 初始化决策器", __LINE__);

    decision_maker_params_io::declare_and_load_decision_params(*this, decision_params_);

    // 初始化参数
    last_nearest_distance_ = decision_params_.warning_zone;
    has_handle_control_ = false;
    last_forward_brake_time_ = this->now();
    last_backward_limit_time_ = this->now();
    backward_emergency_start_time_ = this->now();
    last_backward_limited_linear_x_ = 0.0;
    backward_emergency_active_ = false;

    // 初始化状态保持相关（已废弃，保留用于兼容性）
    last_detected_distance_ = decision_params_.warning_zone;
    last_detection_time_ = this->now();
    has_last_detection_ = false;

    // 初始化速度恢复相关（已废弃，保留用于兼容性）
    current_speed_limit_ = decision_params_.max_speed_limit;
    target_speed_limit_ = decision_params_.max_speed_limit;
    recovery_start_time_ = this->now();
    is_recovering_ = false;

    // 初始化各方向独立状态
    auto now_time = this->now();
    forward_state_.current_speed_limit = decision_params_.max_speed_limit;
    forward_state_.target_speed_limit = decision_params_.max_speed_limit;
    forward_state_.recovery_start_time = now_time;
    forward_state_.last_detection_time = now_time;

    backward_state_.current_speed_limit = decision_params_.max_speed_limit;
    backward_state_.target_speed_limit = decision_params_.max_speed_limit;
    backward_state_.recovery_start_time = now_time;
    backward_state_.last_detection_time = now_time;

    left_state_.current_speed_limit = decision_params_.max_speed_limit;
    left_state_.target_speed_limit = decision_params_.max_speed_limit;
    left_state_.recovery_start_time = now_time;
    left_state_.last_detection_time = now_time;

    right_state_.current_speed_limit = decision_params_.max_speed_limit;
    right_state_.target_speed_limit = decision_params_.max_speed_limit;
    right_state_.recovery_start_time = now_time;
    right_state_.last_detection_time = now_time;

    // 车体尺寸：由 decision_params_（YAML / ros2 param 的 robot_*）同步，见 syncRobotDimensionsFromParams
    syncRobotDimensionsFromParams();
    collision_zones_ = RobotDimensions::getCollisionZones();
    declareSweptAvoidanceParams();

    // 初始化平滑滤波器
    smooth_filter_ = std::make_unique<SmoothFilter>();
    smooth_filter_->setFilterType(FilterType::RATE_LIMITED);
    smooth_filter_->setAlpha(0.3); // 指数平滑系数
    updateSmoothFilterCommandLimits();

    // 初始化卡尔曼滤波器
    kalman_filter_ = std::make_unique<SmoothFilter>();
    kalman_filter_->setFilterType(FilterType::KALMAN_DECELERATION);

    // 初始化前向最小距离的雷达来源和最近点信息
    forward_min_distance_radar_source_.clear();
    forward_min_distance_nearest_z_ = 0.0;

    // 初始化左侧最小距离的雷达来源和最近点信息
    left_min_distance_radar_source_.clear();
    left_min_distance_nearest_z_ = 0.0;

    // 初始化右侧最小距离的雷达来源和最近点信息
    right_min_distance_radar_source_.clear();
    right_min_distance_nearest_z_ = 0.0;

    // 初始化报警发布器
    auxiliary_alarm_publisher_ = this->create_publisher<interface::msg::AuxiliaryAlarm>(
        auxiliaryAlarmTopic, 1);
    auxiliary_alarm_publisher_back_ = this->create_publisher<interface::msg::AuxiliaryAlarm>(
        auxiliaryAlarmBackTopic, 1);
    collision_warning_publisher_ = this->create_publisher<interface::msg::CollisionWarning>(
        collisionWarningTopic, 1);
    narrow_end_publisher_ = this->create_publisher<std_msgs::msg::Empty>("/narrow/end", 10);
    
    // 初始化预警消息的 warning_type 字段
    forward_warning_msg_.warning_type = static_cast<int16_t>(WarningType::FORWARD);
    backward_warning_msg_.warning_type = static_cast<int16_t>(WarningType::BACKWARD);
    left_warning_msg_.warning_type = static_cast<int16_t>(WarningType::LEFT);
    right_warning_msg_.warning_type = static_cast<int16_t>(WarningType::RIGHT);
    negative_plane_forward_warning_msg_.warning_type = static_cast<int16_t>(WarningType::NEGATIVE_PLANE_FORWARD);
    negative_plane_left_warning_msg_.warning_type = static_cast<int16_t>(WarningType::NEGATIVE_PLANE_LEFT);
    negative_plane_right_warning_msg_.warning_type = static_cast<int16_t>(WarningType::NEGATIVE_PLANE_RIGHT);
    
    // 初始化预警消息的 warning_level 字段为 NONE
    forward_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::NONE);
    backward_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::NONE);
    left_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::NONE);
    right_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::NONE);
    negative_plane_forward_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::NONE);
    negative_plane_left_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::NONE);
    negative_plane_right_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::NONE);
    
    // 初始化摇杆订阅器
    joystick_subscription_ = this->create_subscription<interface::msg::Joystick>(
        joystickTopic, 1, std::bind(&DecisionMaker::joystickCallback, this, std::placeholders::_1));
    tft_car_mode_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        tftCarModeTopic, 1, std::bind(&DecisionMaker::tftCarModeCallback, this, std::placeholders::_1));
    // 初始化档位订阅器
    car_info_subscription_ = this->create_subscription<interface::msg::CarInfo>(
        carInfoTopic, 1, std::bind(&DecisionMaker::carInfoCallback, this, std::placeholders::_1));

    decision_params_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto result = decision_maker_params_io::apply_ros_parameter_updates(decision_params_, parameters);
            if (result.successful) {
                RCLCPP_INFO(this->get_logger(), "决策参数已通过 ROS 动态更新");
                syncRobotDimensionsFromParams();
                syncObstaclePredictorParams();
                syncEvasionDetectorParams();
            }
            return result;
        });

    // 阶段 A：决策与摇杆解耦，默认定时 20Hz（可用 decision_rate_hz 覆盖）
    this->declare_parameter<double>("decision_rate_hz", 20.0);
    const double decision_rate_hz = std::max(1.0, this->get_parameter("decision_rate_hz").as_double());
    const auto decision_period = std::chrono::duration<double>(1.0 / decision_rate_hz);
    decision_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(decision_period),
        [this]() { this->runDecisionTick(); });
    RCLCPP_INFO(this->get_logger(), "[DecisionMaker] 决策定时器已启动: %.1f Hz", decision_rate_hz);

    RCLCPP_DEBUG(this->get_logger(), "[L%d] 报警发布器初始化完成", __LINE__);

    RCLCPP_DEBUG(this->get_logger(), "[L%d] 决策器初始化完成 - 预警区:%.1fm, 减速区:%.1fm, 制动区:%.1fm", __LINE__, decision_params_.warning_zone,
        decision_params_.deceleration_zone, decision_params_.emergency_brake_distance);
}

DecisionMaker::~DecisionMaker() { }

void DecisionMaker::declareSweptAvoidanceParams()
{
    this->declare_parameter<std::string>("avoidance_engine", avoidance_engine_);
    this->declare_parameter<bool>("avoidance_shadow_log", avoidance_shadow_log_);
    this->declare_parameter<double>("rollout_horizon_s", swept_avoidance_params_.rollout.horizon_s);
    this->declare_parameter<double>("rollout_dt_s", swept_avoidance_params_.rollout.dt_s);
    this->declare_parameter<int>("rollout_speed_samples", swept_avoidance_params_.rollout.speed_samples);
    this->declare_parameter<int>("rollout_angular_samples", swept_avoidance_params_.rollout.angular_samples);
    this->declare_parameter<double>("swept_radar_timeout_s", swept_avoidance_params_.obstacle.radar_timeout_s);
    this->declare_parameter<int>("swept_max_obstacles", static_cast<int>(swept_avoidance_params_.obstacle.max_obstacles));
    this->declare_parameter<double>("footprint_back_extent_m", swept_footprint_back_extent_);
    this->declare_parameter<double>("footprint_front_margin", swept_avoidance_params_.margins.front);
    this->declare_parameter<double>("footprint_back_margin", swept_avoidance_params_.margins.back);
    this->declare_parameter<double>("footprint_side_margin", swept_avoidance_params_.margins.side);
    this->declare_parameter<double>("footprint_obstacle_radius_default", swept_avoidance_params_.obstacle.default_radius);
    this->declare_parameter<double>("comfort_clearance", swept_avoidance_params_.risk.comfort_clearance_m);
    this->declare_parameter<double>("swept_warning_ttc_s", swept_avoidance_params_.risk.warning_ttc_s);
    this->declare_parameter<double>("swept_limiting_ttc_s", swept_avoidance_params_.risk.limiting_ttc_s);
    this->declare_parameter<double>("swept_emergency_ttc_s", swept_avoidance_params_.risk.emergency_ttc_s);
    this->declare_parameter<double>("swept_hard_stop_distance_m", swept_avoidance_params_.risk.hard_stop_distance_m);
    this->declare_parameter<double>("safety_filter_angular_weight", swept_avoidance_params_.filter.angular_weight);
    updateSweptAvoidanceParamsFromRos();
}

void DecisionMaker::updateSweptAvoidanceParamsFromRos()
{
    avoidance_engine_ = this->get_parameter("avoidance_engine").as_string();
    avoidance_shadow_log_ = this->get_parameter("avoidance_shadow_log").as_bool();

    swept_avoidance_params_.rollout.horizon_s = this->get_parameter("rollout_horizon_s").as_double();
    swept_avoidance_params_.rollout.dt_s = this->get_parameter("rollout_dt_s").as_double();
    swept_avoidance_params_.rollout.speed_samples = static_cast<int>(this->get_parameter("rollout_speed_samples").as_int());
    swept_avoidance_params_.rollout.angular_samples = static_cast<int>(this->get_parameter("rollout_angular_samples").as_int());

    swept_avoidance_params_.obstacle.radar_timeout_s = this->get_parameter("swept_radar_timeout_s").as_double();
    swept_avoidance_params_.obstacle.max_obstacles = static_cast<std::size_t>(
        std::max<int64_t>(1, this->get_parameter("swept_max_obstacles").as_int()));
    swept_avoidance_params_.obstacle.default_radius = this->get_parameter("footprint_obstacle_radius_default").as_double();

    swept_footprint_back_extent_ = this->get_parameter("footprint_back_extent_m").as_double();
    swept_avoidance_params_.margins.front = this->get_parameter("footprint_front_margin").as_double();
    swept_avoidance_params_.margins.back = this->get_parameter("footprint_back_margin").as_double();
    swept_avoidance_params_.margins.side = this->get_parameter("footprint_side_margin").as_double();

    swept_avoidance_params_.risk.comfort_clearance_m = this->get_parameter("comfort_clearance").as_double();
    swept_avoidance_params_.risk.warning_ttc_s = this->get_parameter("swept_warning_ttc_s").as_double();
    swept_avoidance_params_.risk.limiting_ttc_s = this->get_parameter("swept_limiting_ttc_s").as_double();
    swept_avoidance_params_.risk.emergency_ttc_s = this->get_parameter("swept_emergency_ttc_s").as_double();
    swept_avoidance_params_.risk.hard_stop_distance_m = this->get_parameter("swept_hard_stop_distance_m").as_double();
    swept_avoidance_params_.filter.angular_weight = this->get_parameter("safety_filter_angular_weight").as_double();
    swept_avoidance_params_.filter.emergency_ttc_s = swept_avoidance_params_.risk.emergency_ttc_s;
}

scene_node::avoidance::SafetyFilterResult DecisionMaker::evaluateSweptAvoidance(
    const geometry_msgs::msg::Twist& handle_control,
    const rclcpp::Time& now)
{
    std::map<std::string, std::vector<ClusterInfo>> radar_clusters;
    std::map<std::string, rclcpp::Time> radar_times;
    VehicleState vehicle_state;
    RobotDimensions::BaseDimensions robot_dimensions;
    DecisionParams decision_params_snapshot;
    double feedback_linear = 0.0;
    double feedback_angular = 0.0;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        radar_clusters = radar_cluster_map_;
        radar_times = radar_last_update_time_map_;
        vehicle_state = current_vehicle_state_;
        robot_dimensions = robot_dimensions_;
        decision_params_snapshot = decision_params_;
        feedback_linear = tracking_ ? tracking_->getWheelSpeedLinearVelocity() : current_vehicle_state_.linear_velocity;
        feedback_angular = tracking_ ? tracking_->getWheelSpeedAngularVelocity() : current_vehicle_state_.angular_velocity;
    }

    auto params = swept_avoidance_params_;
    params.obstacle.radar_timeout_s = std::max(params.obstacle.radar_timeout_s, decision_params_snapshot.radar_timeout);
    params.footprint.front = robot_dimensions.length;
    params.footprint.back = std::max(0.0, swept_footprint_back_extent_);
    params.footprint.half_width = robot_dimensions.width * 0.5;

    const scene_node::avoidance::MotionCommand desired{
        handle_control.linear.x,
        handle_control.angular.z
    };
    const scene_node::avoidance::VehicleFeedback feedback{
        feedback_linear,
        feedback_angular,
        vehicle_state.timestamp
    };

    return swept_risk_assessment_.evaluate(
        desired, feedback, radar_clusters, radar_times, now, params);
}

geometry_msgs::msg::Twist DecisionMaker::smoothDecisionOutput(const geometry_msgs::msg::Twist& control)
{
    geometry_msgs::msg::Twist filtered_control;
    updateSmoothFilterCommandLimits();

    const double kEmergencyStopThreshold = 0.02;
    if (std::abs(control.linear.x) <= kEmergencyStopThreshold
        && std::abs(control.angular.z) <= kEmergencyStopThreshold) {
        filtered_control = smooth_filter_->filter(control);
        const double kBrakeDecay = 0.3;
        if (std::abs(filtered_control.linear.x) > kEmergencyStopThreshold) {
            filtered_control.linear.x *= kBrakeDecay;
        } else {
            filtered_control.linear.x = 0.0;
        }
        if (std::abs(filtered_control.angular.z) > kEmergencyStopThreshold) {
            filtered_control.angular.z *= kBrakeDecay;
        } else {
            filtered_control.angular.z = 0.0;
        }
    } else {
        filtered_control = smooth_filter_->filter(control);
    }

    if (std::abs(filtered_control.angular.z) > decision_params_.absolute_max_angular_speed) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[L%d] å·¦ä¾§-è§’é€Ÿåº¦è¶…é™ï¼%.3f rad/s -> %.3f rad/s",
            __LINE__,
            filtered_control.angular.z,
            std::copysign(decision_params_.absolute_max_angular_speed, filtered_control.angular.z));
        filtered_control.angular.z = std::copysign(decision_params_.absolute_max_angular_speed, filtered_control.angular.z);
    }

    return filtered_control;
}

void DecisionMaker::logSweptAvoidanceShadow(
    const scene_node::avoidance::SafetyFilterResult& result,
    const geometry_msgs::msg::Twist& output_control)
{
    if (!avoidance_shadow_log_) {
        return;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[SweptAvoidance][%s] level=%s emergency=%s ttc=%.3f clearance=%.3f "
        "selected=(%.3f, %.3f) output=(%.3f, %.3f) obstacles=%zu trajectories=%zu source=%s collision=(%.2f, %.2f)",
        avoidance_engine_ == "swept_footprint" ? "takeover" : "shadow",
        scene_node::avoidance::riskLevelName(result.risk.level),
        result.emergency_brake ? "true" : "false",
        result.risk.time_to_collision,
        result.risk.min_distance,
        result.selected_command.linear_x,
        result.selected_command.angular_z,
        output_control.linear.x,
        output_control.angular.z,
        result.obstacle_count,
        result.trajectory_count,
        result.risk.source.c_str(),
        result.risk.collision_x,
        result.risk.collision_y);
}

// ==================== 公共接口函数 ====================

void DecisionMaker::setControlOutputCallback(std::function<void(const geometry_msgs::msg::Twist&)> callback) { control_output_callback_ = callback; }

void DecisionMaker::processClusters(const std::vector<ClusterInfo>& clusters, const std::string& source_name)
{
    // 输入验证
    if (source_name.empty()) {
        RCLCPP_WARN(this->get_logger(), "[L%d] 雷达源名称为空，忽略该聚类数据", __LINE__);
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);

    // 更新雷达聚类目标映射（不做上下帧融合，直接完全更新）
    radar_cluster_map_[source_name] = clusters;

    updateRadarNearestPoint(clusters, source_name);
    // 更新该雷达的数据时间戳
    radar_last_update_time_map_[source_name] = this->now();

    RCLCPP_DEBUG(this->get_logger(), "[L%d] %s雷达聚类目标完全更新: %zu个聚类目标", __LINE__, source_name.c_str(), clusters.size());
}

void DecisionMaker::updateVehicleState(const VehicleState& vehicle_state)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_vehicle_state_ = vehicle_state;
}

/**
 * @brief 应用前向速度干预
 *
 * 本函数是前向碰撞检测的核心干预逻辑，主要功能包括：
 * 1. 状态保持机制：当障碍物短暂消失时，在保持时间内维持之前的状态，避免控制振荡
 * 2. 速度平滑恢复：障碍物消失后，平滑地将速度限制恢复到最大值，避免突然加速
 * 3. 分级速度限制：根据障碍物距离，将前方区域分为制动区、减速区、预警区和正常区，实施不同的速度限制策略
 * 4. 速度限制应用：将计算得到的速度限制应用到摇杆控制命令上，确保安全行驶
 *
 * 工作流程：
 * - 获取前向最小安全距离（由 calculateMinDirectionalDistance 计算得出）
 * - 判断当前是否检测到障碍物（距离是否小于预警区阈值）
 * - 如果障碍物消失，进入状态保持或速度恢复流程
 * - 如果检测到障碍物，根据距离计算并应用相应的速度限制
 * - 最终更新 current_handle_control_，供后续平滑滤波使用
 */
void DecisionMaker::applySpeedForwardIntervention(geometry_msgs::msg::Twist& output_control)
{
    double min_distance = min_directional_distance_.min_forward_distance;
    bool use_predicted = false;

    // 检查左侧或右侧是否满足条件（由 left_tilt 或 right_tilt 提供且 Z 值为 0）
    bool lateral_condition_met = false;
    if ((left_min_distance_radar_source_ == "left_tilt" || left_min_distance_radar_source_ == "right_tilt") 
        && std::fabs(left_min_distance_nearest_z_) <= 0.01 && min_directional_distance_.min_lateral_left_distance < 0.40) {
        lateral_condition_met = true;
    }
    if ((right_min_distance_radar_source_ == "left_tilt" || right_min_distance_radar_source_ == "right_tilt") 
        && std::fabs(right_min_distance_nearest_z_) <= 0.01 && min_directional_distance_.min_lateral_right_distance < 0.40) {
        lateral_condition_met = true;
    }

    // 检查是否需要调整安全距离
    // 条件：前向最小安全距离是由 left_tilt 或 right_tilt 提供的，且最近点Z值在-0.05到0.05之间
    if ((forward_min_distance_radar_source_ == "left_tilt" || forward_min_distance_radar_source_ == "right_tilt") 
        && (forward_min_distance_nearest_z_ <= 0.01 || forward_min_distance_nearest_z_ >= 0.20)) {
        is_forward_obstacle_negative_plane_ = true;
        // 如果侧向满足条件，使用更大的调整值（0.2），否则使用默认值（0.1）
        double adjustment_value = lateral_condition_met ? min_distance : tilt_radar_distance_adjustment_;
        min_distance = std::max(0.0, min_distance - adjustment_value);
        RCLCPP_DEBUG(this->get_logger(), "[L%d] [前向-tilt调整] 雷达来源: %s, 最近点Z: %.3f, 侧向触发: %s, 原始距离: %.3f m, 调整后距离: %.3f m (减去 %.3f m)",
            __LINE__, forward_min_distance_radar_source_.c_str(), forward_min_distance_nearest_z_,
            lateral_condition_met ? "是" : "否",
            min_directional_distance_.min_forward_distance, min_distance, adjustment_value);
    } else {
        is_forward_obstacle_negative_plane_ = false;
    }
    double approach_velocity = 0.0;
    double feedback_angular_z = 0.0;
    double feedback_linear_x = 0.0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        // 使用指令速度与实测速度的较大值，确保 TTC 计算不低估（实测可能延迟或为0）
        if (current_handle_control_.linear.x > 0.0) {
            approach_velocity = std::max(current_vehicle_state_.linear_velocity, current_handle_control_.linear.x);
        }
        // 避让意图：与 applySpeedLimit 底盘门限一致，使用轮速反馈（Tracking 与 /wheel/twist 同源）
        if (tracking_) {
            feedback_angular_z = tracking_->getWheelSpeedAngularVelocity();
            feedback_linear_x = tracking_->getWheelSpeedLinearVelocity();
        } else {
            feedback_angular_z = current_vehicle_state_.angular_velocity;
            feedback_linear_x = current_vehicle_state_.linear_velocity;
        }
    }

    applyDirectionalInterventionInternal(
        min_distance,
        decision_params_.warning_zone,
        decision_params_.emergency_brake_distance,
        decision_params_.warning_ttc,
        approach_velocity,
        forward_state_,
        [this, feedback_angular_z, feedback_linear_x](double dist, double vel) {
            SpeedLimit limit = this->calculateSpeedLimit(dist, vel);
            // 轮速反馈推断避让运动，衰减前向限速；与 calculateSpeedLimit 使用同一有效距离 dist（含状态保持后）
            if (decision_params_.enable_evasion_detection && evasion_detector_
                && limit.max_linear_speed < decision_params_.max_speed_limit) {
                double factor = evasion_detector_->computeEvasionFactor(feedback_angular_z, feedback_linear_x, dist);
                if (factor < 1.0) {
                    double relaxed = evasion_detector_->applyEvasionRelax(
                        limit.max_linear_speed, decision_params_.max_speed_limit, factor);
                    RCLCPP_INFO(this->get_logger(),
                        "[L%d][前向-避让衰减] omega_fb=%.3f v_fb=%.3f factor=%.3f 限速 %.3f -> %.3f m/s",
                        __LINE__, feedback_angular_z, feedback_linear_x, factor, limit.max_linear_speed, relaxed);
                    limit.max_linear_speed = relaxed;
                }
            }
            return limit;
        },
        [this](const geometry_msgs::msg::Twist& ctrl, const SpeedLimit& limit) {
            return this->applySpeedLimit(ctrl, limit);
        },
        "前侧",
        output_control,
        use_predicted);
}

void DecisionMaker::applySpeedBackwardIntervention(geometry_msgs::msg::Twist& output_control)
{
    double min_distance = min_directional_distance_.min_backward_distance;
    bool use_predicted = false;

    double approach_velocity = 0.0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (current_handle_control_.linear.x < 0.0) {
            approach_velocity = std::max(std::abs(current_vehicle_state_.linear_velocity), std::abs(current_handle_control_.linear.x));
        }
    }
    applyDirectionalInterventionInternal(
        min_distance,
        decision_params_.warning_zone_backward,
        decision_params_.emergency_brake_distance_backward,
        decision_params_.warning_ttc_backward,
        approach_velocity,
        backward_state_,
        [this](double dist, double vel) { return this->calculateSpeedLimitForBackward(dist, vel); },
        [this](const geometry_msgs::msg::Twist& ctrl, const SpeedLimit& limit) {
            return this->applySpeedLimitForBackward(ctrl, limit);
        },
        "后侧",
        output_control,
        use_predicted);
}

void DecisionMaker::applySpeedLeftIntervention(geometry_msgs::msg::Twist& output_control)
{
    double min_distance = min_directional_distance_.min_lateral_left_distance;
    bool use_predicted = false;

    // 检查是否需要调整安全距离
    // 条件：左侧最小安全距离是由 left_tilt 或 right_tilt 提供的，且最近点Z值为0
    if ((left_min_distance_radar_source_ == "left_tilt" || left_min_distance_radar_source_ == "right_tilt") 
        && std::abs(left_min_distance_nearest_z_) < 1e-6) { // Z值为0（使用小阈值判断）
        min_distance = std::max(0.0, min_distance - lateral_tilt_radar_distance_adjustment_);
        RCLCPP_DEBUG(this->get_logger(), "[L%d] [左侧-tilt调整] 雷达来源: %s, 最近点Z: %.3f, 原始距离: %.3f m, 调整后距离: %.3f m (减去 %.3f m)",
            __LINE__, left_min_distance_radar_source_.c_str(), left_min_distance_nearest_z_,
            min_directional_distance_.min_lateral_left_distance, min_distance, lateral_tilt_radar_distance_adjustment_);
    }

    double approach_velocity = 0.0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        double half_width = robot_dimensions_.width * 0.5;
        // Bug3 修复：取命令角速度与轮速反馈的较大值，快速转向命令刚下达时不滞后
        if (current_handle_control_.angular.z > 0.0) {
            double cmd_vel = std::abs(current_handle_control_.angular.z) * half_width;
            double fb_vel = std::abs(current_vehicle_state_.angular_velocity) * half_width;
            approach_velocity = std::max(cmd_vel, fb_vel);
        }
    }
    applyDirectionalInterventionInternal(
        min_distance,
        decision_params_.warning_zone_lateral,
        decision_params_.emergency_brake_distance_lateral,
        decision_params_.warning_ttc_lateral,
        approach_velocity,
        left_state_,
        [this](double dist, double vel) { return this->calculateSpeedLimitForLateral(dist, vel); },
        [this](const geometry_msgs::msg::Twist& ctrl, const SpeedLimit& limit) {
            return this->applySpeedLimitForLeft(ctrl, limit);
        },
        "左侧",
        output_control,
        use_predicted);
}

void DecisionMaker::applySpeedRightIntervention(geometry_msgs::msg::Twist& output_control)
{
    double min_distance = min_directional_distance_.min_lateral_right_distance;
    bool use_predicted = false;

    // 检查是否需要调整安全距离
    // 条件：右侧最小安全距离是由 left_tilt 或 right_tilt 提供的，且最近点Z值为0
    if ((right_min_distance_radar_source_ == "left_tilt" || right_min_distance_radar_source_ == "right_tilt")
         && std::abs(right_min_distance_nearest_z_) < 1e-6) { // Z值为0（使用小阈值判断）
        min_distance = std::max(0.0, min_distance - lateral_tilt_radar_distance_adjustment_);
        RCLCPP_DEBUG(this->get_logger(), "[L%d] [右侧-tilt调整] 雷达来源: %s, 最近点Z: %.3f, 原始距离: %.3f m, 调整后距离: %.3f m (减去 %.3f m)",
            __LINE__, right_min_distance_radar_source_.c_str(), right_min_distance_nearest_z_,
            min_directional_distance_.min_lateral_right_distance, min_distance, lateral_tilt_radar_distance_adjustment_);
    }

    double approach_velocity = 0.0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        double half_width = robot_dimensions_.width * 0.5;
        // Bug3 修复：取命令角速度与轮速反馈的较大值
        if (current_handle_control_.angular.z < 0.0) {
            double cmd_vel = std::abs(current_handle_control_.angular.z) * half_width;
            double fb_vel = std::abs(current_vehicle_state_.angular_velocity) * half_width;
            approach_velocity = std::max(cmd_vel, fb_vel);
        }
    }
    applyDirectionalInterventionInternal(
        min_distance,
        decision_params_.warning_zone_lateral,
        decision_params_.emergency_brake_distance_lateral,
        decision_params_.warning_ttc_lateral,
        approach_velocity,
        right_state_,
        [this](double dist, double vel) { return this->calculateSpeedLimitForLateral(dist, vel); },
        [this](const geometry_msgs::msg::Twist& ctrl, const SpeedLimit& limit) {
            return this->applySpeedLimitForRight(ctrl, limit);
        },
        "右侧",
        output_control,
        use_predicted);
}

// ==================== 统一的干预逻辑辅助函数 ====================

double DecisionMaker::applyStateHoldLogic(
    double min_distance,
    double warning_zone,
    DirectionalState& state,
    const std::string& direction_name,
    rclcpp::Time current_time)
{
    double effective_distance = min_distance;

    if (min_distance >= warning_zone) {
        // 当前没有障碍物
        if (state.has_last_detection) {
            double time_since_last = (current_time - state.last_detection_time).seconds();

            if (time_since_last <= decision_params_.state_hold_time) {
                // 在保持时间内，使用上一次检测到的距离
                effective_distance = state.last_detected_distance;
                RCLCPP_DEBUG(this->get_logger(), "[L%d][%s-状态保持] 障碍物消失 %.3f 秒，保持上次距离 %.3f m",
                    __LINE__, direction_name.c_str(), time_since_last, effective_distance);
            } else {
                // 超过保持时间，清除检测标志
                state.has_last_detection = false;
            }
        }
    } else {
        // 检测到障碍物，更新状态
        state.has_last_detection = true;
        state.last_detected_distance = min_distance;
        state.last_detection_time = current_time;

        // 取消恢复状态
        if (state.is_recovering) {
            state.is_recovering = false;
            RCLCPP_DEBUG(this->get_logger(), "[L%d][%s-速度恢复] 检测到障碍物，取消恢复", __LINE__, direction_name.c_str());
        }
    }

    return effective_distance;
}

double DecisionMaker::applySpeedRecoveryLogic(
    DirectionalState& state,
    double max_speed_limit,
    double speed_recovery_time,
    const std::string& direction_name,
    rclcpp::Time current_time,
    bool should_start_recovery)
{
    double final_speed_limit = max_speed_limit;

    if (state.is_recovering) {
        double recovery_elapsed = (current_time - state.recovery_start_time).seconds();
        double recovery_ratio = std::min(1.0, recovery_elapsed / speed_recovery_time);

        // 真正的线性插值：从恢复起始速度到目标速度
        state.current_speed_limit = state.recovery_start_speed
            + (state.target_speed_limit - state.recovery_start_speed) * recovery_ratio;

        final_speed_limit = state.current_speed_limit;

        RCLCPP_DEBUG(this->get_logger(), "[L%d][%s-速度恢复] 恢复进度 %.1f%%, 当前限速 %.3f m/s (%.1f km/h)",
            __LINE__, direction_name.c_str(), recovery_ratio * 100.0, state.current_speed_limit, state.current_speed_limit * 3.6);

        if (recovery_ratio >= 1.0) {
            state.is_recovering = false;
            state.current_speed_limit = max_speed_limit;
            RCLCPP_DEBUG(this->get_logger(), "[L%d][%s-速度恢复] 恢复完成，当前限速 %.3f m/s (%.1f km/h)",
                __LINE__, direction_name.c_str(), state.current_speed_limit, state.current_speed_limit * 3.6);
        }
    } else if (should_start_recovery) {
        if (state.current_speed_limit < max_speed_limit) {
            state.is_recovering = true;
            state.recovery_start_time = current_time;
            state.recovery_start_speed = state.current_speed_limit;
            state.target_speed_limit = max_speed_limit;
            RCLCPP_DEBUG(this->get_logger(), "[L%d][%s-速度恢复] 开始恢复，从 %.3f m/s 恢复到 %.3f m/s，用时 %.1f 秒",
                __LINE__, direction_name.c_str(), state.current_speed_limit, state.target_speed_limit, speed_recovery_time);
        }
    }

    return final_speed_limit;
}

void DecisionMaker::applyDirectionalInterventionInternal(
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
    bool use_predicted)
{
    auto current_time = this->now();

    // 步骤1：应用状态保持逻辑，得到有效距离
    double effective_distance = applyStateHoldLogic(min_distance, warning_zone, state, direction_name, current_time);
    RCLCPP_DEBUG(this->get_logger(), "[L%d] [%s] 应用状态保持逻辑，得到有效距离 %.3f m", __LINE__, direction_name.c_str(), effective_distance);

    // TTC + 距离双重模式：TTC 用于高速自适应，距离用于低速兜底
    const double v_eps = 0.01;
    const bool use_ttc = (approach_velocity >= decision_params_.min_approach_velocity_for_ttc);
    double effective_ttc = (approach_velocity > v_eps) ? (effective_distance / approach_velocity) : 999.0;

    bool need_intervention = false;
    if (use_ttc) {
        // TTC 触发 或 距离低于制动距离时均需干预（低速距离兜底）
        need_intervention = (effective_ttc < warning_ttc)
                         || (effective_distance < emergency_brake_distance);
    } else {
        need_intervention = (effective_distance < warning_zone);
    }

    double final_speed_limit = decision_params_.max_speed_limit;
    bool should_start_recovery = false;

    if (need_intervention) {
        SpeedLimit speed_limit = calc_limit_func(effective_distance, approach_velocity);
        RCLCPP_DEBUG(this->get_logger(), "[L%d] [%s] 计算速度限制(TTC=%.3fs)，得到限速 线速度: %.3f m/s, 角速度: %.3f rad/s",
            __LINE__, direction_name.c_str(), effective_ttc, speed_limit.max_linear_speed, speed_limit.max_angular_speed);
        if (speed_limit.max_linear_speed < std::numeric_limits<double>::max() || speed_limit.max_angular_speed < std::numeric_limits<double>::max()) {
            output_control = apply_limit_func(output_control, speed_limit);

            bool in_emergency = (speed_limit.max_linear_speed <= 0.01) || (speed_limit.max_angular_speed <= 0.01);
            if (in_emergency) {
                RCLCPP_WARN(this->get_logger(), "[L%d]%s[%s-制动区] 紧急制动！距离 %.3f m, TTC %.3f s",
                    __LINE__, use_predicted ? "[预测]" : "[实时]", direction_name.c_str(), effective_distance, effective_ttc);
                if (shouldPublishPhysicalIntervention(direction_name, WarningLevel::BRAKE)) {
                    publishInterventionState(direction_name, WarningLevel::BRAKE, effective_distance);
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "[L%d][%s-制动区] 决策已限幅，轮速尚未体现减速/刹停，跳过干预等级发布",
                        __LINE__, direction_name.c_str());
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "[L%d]%s[%s-减速区] 平滑减速，距离 %.3f m, TTC %.3f s",
                    __LINE__, use_predicted ? "[预测]" : "[实时]", direction_name.c_str(), effective_distance, effective_ttc);
                if (shouldPublishPhysicalIntervention(direction_name, WarningLevel::DECELERATION)) {
                    publishInterventionState(direction_name, WarningLevel::DECELERATION, effective_distance);
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "[L%d][%s-减速区] 决策已限幅，轮速尚未体现减速，跳过干预等级发布",
                        __LINE__, direction_name.c_str());
                }
            }
        }
    } else {
        state.current_speed_limit = decision_params_.max_speed_limit;
        should_start_recovery = !state.has_last_detection;
        RCLCPP_DEBUG(this->get_logger(), "[L%d]%s[%s-正常区] 无需干预，距离 %.3f m, TTC %.3f s",
            __LINE__, use_predicted ? "[预测]" : "[实时]", direction_name.c_str(), effective_distance, effective_ttc);
    }

    final_speed_limit = applySpeedRecoveryLogic(
        state, decision_params_.max_speed_limit, decision_params_.speed_recovery_time,
        direction_name, current_time, should_start_recovery);

    if (state.is_recovering) {
        // Bug5 修复：侧向恢复使用 absolute_max_angular_speed 作为目标，而非 max_speed_limit（线速度）
        bool is_lateral = (direction_name == "左侧" || direction_name == "右侧");
        double recovery_ratio = (decision_params_.max_speed_limit > 1e-6)
            ? (final_speed_limit / decision_params_.max_speed_limit) : 1.0;
        SpeedLimit recovery_limit;
        recovery_limit.max_linear_speed = is_lateral ? std::numeric_limits<double>::max() : final_speed_limit;
        recovery_limit.max_angular_speed = is_lateral
            ? (decision_params_.absolute_max_angular_speed * recovery_ratio)
            : (final_speed_limit * 0.5);
        recovery_limit.distance_threshold = 0.0;
        output_control = apply_limit_func(output_control, recovery_limit);
    }
}

// ==================== 摇杆缓存与定时决策 ====================

void DecisionMaker::updateDesiredTwist(const geometry_msgs::msg::Twist& handle_control)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    desired_handle_control_ = handle_control;
    has_handle_control_ = true;
}

void DecisionMaker::processHandleControl(const geometry_msgs::msg::Twist& handle_control)
{
    updateDesiredTwist(handle_control);
}

void DecisionMaker::setDecisionOutputEnabled(bool enabled)
{
    decision_output_enabled_.store(enabled, std::memory_order_release);
}

void DecisionMaker::runDecisionTick()
{
    if (!decision_output_enabled_.load(std::memory_order_acquire)) {
        return;
    }

    geometry_msgs::msg::Twist handle_control;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!has_handle_control_) {
            return;
        }
        handle_control = desired_handle_control_;
        current_handle_control_ = handle_control;
    }

    // 本周期轮速反馈采样（与上一 control 周期对比，供干预等级发布前判断真实减速/制动）
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const double v_fb = tracking_ ? tracking_->getWheelSpeedLinearVelocity() : current_vehicle_state_.linear_velocity;
        const double w_fb = tracking_ ? tracking_->getWheelSpeedAngularVelocity() : current_vehicle_state_.angular_velocity;
        fb_linear_prev_ = fb_linear_last_;
        fb_linear_last_ = v_fb;
        fb_angular_prev_ = fb_angular_last_;
        fb_angular_last_ = w_fb;
        ++speed_feedback_sample_count_;
        if (speed_feedback_sample_count_ >= 2) {
            speed_feedback_ready_ = true;
        }
    }

    // 步骤2：检查回调函数
    if (!control_output_callback_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[L%d] 控制输出回调函数未设置，无法输出控制命令", __LINE__);
        return;
    }

    // 步骤4：收集前向相关的所有雷达数据
    auto current_time = this->now();
    updateSweptAvoidanceParamsFromRos();
    if (avoidance_engine_ != last_avoidance_engine_) {
        resetBrakeHoldState();
        last_avoidance_engine_ = avoidance_engine_;
    }

    const bool swept_shadow_enabled = avoidance_engine_ == "shadow";
    const bool swept_takeover_enabled = avoidance_engine_ == "swept_footprint";
    if (avoidance_engine_ != "legacy" && !swept_shadow_enabled && !swept_takeover_enabled) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "[SweptAvoidance] unknown avoidance_engine='%s', falling back to legacy",
            avoidance_engine_.c_str());
    }

    scene_node::avoidance::SafetyFilterResult swept_result;
    const bool has_swept_result = swept_shadow_enabled || swept_takeover_enabled;
    if (has_swept_result) {
        swept_result = evaluateSweptAvoidance(handle_control, current_time);
    }

    if (swept_takeover_enabled) {
        geometry_msgs::msg::Twist output_control = handle_control;
        output_control.linear.x = swept_result.selected_command.linear_x;
        output_control.angular.z = swept_result.selected_command.angular_z;
        current_handle_control_ = output_control;

        forward_brake_hold_.store(false, std::memory_order_relaxed);
        backward_brake_hold_.store(false, std::memory_order_relaxed);

        const auto filtered_control = smoothDecisionOutput(output_control);
        logSweptAvoidanceShadow(swept_result, filtered_control);
        control_output_callback_(filtered_control);
        return;
    }
    bool has_valid_data = false;

    std::vector<ClusterInfo> merged_forward_clusters;
    std::map<size_t, std::string> cluster_radar_map; // 聚类索引到雷达名称的映射

    // 锁定状态以读取雷达数据
    // 优先从 ObstaclePredictor 获取轮速补偿后的聚类，否则回退到原始 radar_cluster_map_
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // 决定数据源：有预测器且开启时从预测器读取，否则从原始映射读取
        const bool use_predictor = obstacle_predictor_ && decision_params_.enable_obstacle_prediction;
        auto predicted_all = use_predictor ? obstacle_predictor_->getAllPredictedClusters()
                                           : std::map<std::string, std::vector<ClusterInfo>>{};

        const auto& cluster_source = use_predictor ? predicted_all : radar_cluster_map_;

        // 阶段 A：分通道合并 — 前向主决策仅 left/right；侧向含 tilt；后向仅倒车时启用 back
        const bool reversing = handle_control.linear.x < -0.01;
        for (const auto& radar_pair : cluster_source) {
            const std::string& radar_name = radar_pair.first;
            const std::vector<ClusterInfo>& radar_clusters = radar_pair.second;

            const bool is_lateral_relevant =
                (radar_name == "left" || radar_name == "right"
                 || radar_name == "left_tilt" || radar_name == "right_tilt");
            const bool is_backward_relevant = (radar_name == "back" && reversing);

            if (!is_lateral_relevant && !is_backward_relevant) {
                RCLCPP_DEBUG(this->get_logger(), "[L%d] 雷达 %s 本周期不参与距离融合", __LINE__, radar_name.c_str());
                continue;
            }

            // 时效性判断
            double t_diff = decision_params_.radar_timeout + 1.0; // 默认超时
            if (use_predictor) {
                // 预测器已内部管理时效性，直接查询雷达帧年龄
                t_diff = obstacle_predictor_->getRadarDataAge(radar_name, current_time);
            } else {
                auto t_it = radar_last_update_time_map_.find(radar_name);
                if (t_it != radar_last_update_time_map_.end()) {
                    t_diff = (current_time - t_it->second).seconds();
                }
            }

            // 数据有效性和超时补偿逻辑
            std::vector<ClusterInfo> valid_clusters;

            if (use_predictor) {
                // 预测器模式：预测器内部已做超时丢弃(max_prediction_age)，
                // 此处仅做宽松的上限检查
                if (t_diff <= decision_params_.obstacle_prediction_max_age && !radar_clusters.empty()) {
                    valid_clusters = radar_clusters;
                    has_valid_data = true;
                    RCLCPP_DEBUG(this->get_logger(), "[L%d] 雷达 %s 预测器数据 (age=%.3f s)，聚类数: %zu",
                                 __LINE__, radar_name.c_str(), t_diff, radar_clusters.size());
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "[L%d] 雷达 %s 预测器数据过期 (age=%.3f s) 或为空，跳过",
                                 __LINE__, radar_name.c_str(), t_diff);
                    continue;
                }
            } else {
                // 原始模式（无预测器或预测器关闭）
                if (t_diff <= decision_params_.radar_timeout) {
                    valid_clusters = radar_clusters;
                    has_valid_data = true;
                    RCLCPP_DEBUG(this->get_logger(), "[L%d] 雷达 %s 数据新鲜 (%.3f s)，聚类数: %zu", __LINE__, radar_name.c_str(), t_diff, radar_clusters.size());
                } else if (t_diff <= decision_params_.radar_timeout * 3.0 && !radar_clusters.empty()) {
                    valid_clusters = radar_clusters;
                    has_valid_data = true;
                    RCLCPP_DEBUG(this->get_logger(), "[L%d] 雷达 %s 数据超时 (%.3f s)，使用原始数据，聚类数: %zu", __LINE__, radar_name.c_str(), t_diff, valid_clusters.size());
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "[L%d] 雷达 %s 数据严重超时 (%.3f s) 或为空，跳过", __LINE__, radar_name.c_str(), t_diff);
                    continue;
                }
            }

            // 记录每个聚类对应的雷达名称
            size_t start_index = merged_forward_clusters.size();
            for (size_t j = 0; j < valid_clusters.size(); ++j) {
                cluster_radar_map[start_index + j] = radar_name;
            }

            // 合并有效聚类数据
            merged_forward_clusters.insert(merged_forward_clusters.end(), valid_clusters.begin(), valid_clusters.end());
        }
    }

    // 步骤5：计算前向最小安全距离
    if (has_valid_data && !merged_forward_clusters.empty()) {
        calculateMinDirectionalDistance(merged_forward_clusters, cluster_radar_map);
    } else {
        // 无雷达数据时，用“远大于预警区”的哨兵值表示“无障碍物”，
        // 避免 warning_zone 原值在高速下被 TTC 判定为减速（例如 2.0m / 2.25m·s = 0.889s < warning_ttc）。
        const double kNoObstacleDistance = 1e6;
        MinDirectionalDistance distances { kNoObstacleDistance,
            kNoObstacleDistance,
            kNoObstacleDistance,
            kNoObstacleDistance };
        min_directional_distance_ = distances;
        forward_min_distance_radar_source_.clear();
        forward_min_distance_nearest_z_ = 0.0;
        left_min_distance_radar_source_.clear();
        left_min_distance_nearest_z_ = 0.0;
        right_min_distance_radar_source_.clear();
        right_min_distance_nearest_z_ = 0.0;
    }

    // 回传侧向距离给环境检测器（锁外调用，避免与 env_detector 锁产生死锁）
    if (lateral_distance_callback_) {
        lateral_distance_callback_(
            min_directional_distance_.min_lateral_left_distance,
            min_directional_distance_.min_lateral_right_distance);
    }

    // 步骤6：平滑速度干预并输出
    // 为了保证平稳性，即使没有障碍物也要经过平滑滤波
    // 支持多方向同时干预（例如：前进+左转）
    geometry_msgs::msg::Twist output_control = current_handle_control_;
    //WarningType last_warning_type = WarningType::NONE;

    // 摇杆回中 / 无前进量 → 解除前向制动保持
    if (handle_control.linear.x <= 0.01) {
        if (forward_brake_hold_.load(std::memory_order_relaxed)) {
            RCLCPP_INFO(this->get_logger(), "[L%d][前向-制动保持] 摇杆回中(%.3f)，解除制动保持",
                __LINE__, handle_control.linear.x);
        }
        forward_brake_hold_.store(false, std::memory_order_relaxed);
    }
    // 无后退量（与上对称）→ 解除后向制动保持
    if (handle_control.linear.x >= -0.01) {
        if (backward_brake_hold_.load(std::memory_order_relaxed)) {
            RCLCPP_INFO(this->get_logger(), "[L%d][后向-制动保持] 无后退量(%.3f)，解除制动保持",
                __LINE__, handle_control.linear.x);
        }
        backward_brake_hold_.store(false, std::memory_order_relaxed);
    }

    // 根据控制命令方向，结合碰撞检测开关，应用多个方向的干预
    constexpr double kNoObstacleDistance = 1e6; // 无障碍物哨兵值
    if (decision_params_.enable_forward_collision && current_handle_control_.linear.x > 0.0) {
        if (cruise_mode_active_.load(std::memory_order_acquire)) {
            // 巡航模式下不启用前向“制动保持”，避免障碍移除后因保持状态导致速度恢复困难
            forward_brake_hold_.store(false, std::memory_order_relaxed);
            applySpeedForwardIntervention(output_control);
        } else {
        double current_forward_dist = min_directional_distance_.min_forward_distance;

        if (forward_brake_hold_.load(std::memory_order_relaxed)) {
            // 读到哨兵值(1e6)或已不在预警区，立即退出保持
            if (current_forward_dist >= kNoObstacleDistance
                || forward_brake_hold_distance_ >= kNoObstacleDistance
                || current_forward_dist >= decision_params_.warning_zone) {
                RCLCPP_INFO(this->get_logger(),
                    "[L%d][前向-制动保持] 退出保持(当前=%.3f, 记录=%.3f, 预警区=%.3f)",
                    __LINE__, current_forward_dist, forward_brake_hold_distance_, decision_params_.warning_zone);
                forward_brake_hold_.store(false, std::memory_order_relaxed);
                applySpeedForwardIntervention(output_control);
            }
            // 障碍物距离变化 <= 2cm → 保持制动，跳过前向干预计算
            else if (std::abs(current_forward_dist - forward_brake_hold_distance_) <= 0.02) {
                output_control.linear.x = 0.0;

                // 维护 forward_state_：保持检测状态与速度限制一致，避免解除后恢复逻辑异常
                auto current_time = this->now();
                forward_state_.has_last_detection = true;
                forward_state_.last_detected_distance = current_forward_dist;
                forward_state_.last_detection_time = current_time;
                forward_state_.current_speed_limit = 0.0;
                if (forward_state_.is_recovering) {
                    forward_state_.is_recovering = false;
                }

                // 保持碰撞预警发布，确保 UI 持续显示制动状态
                //publishInterventionState("前侧", WarningLevel::BRAKE, current_forward_dist);

                RCLCPP_DEBUG(this->get_logger(),
                    "[L%d][前向-制动保持] 障碍物未移动(当前%.3fm, 记录%.3fm, Δ%.3fm)，摇杆%.3f，保持制动",
                    __LINE__, current_forward_dist, forward_brake_hold_distance_,
                    std::abs(current_forward_dist - forward_brake_hold_distance_),
                    handle_control.linear.x);
            } else {
                // 障碍物已明显移动(>2cm)，解除保持，恢复正常干预
                RCLCPP_INFO(this->get_logger(),
                    "[L%d][前向-制动保持] 障碍物已移动(当前%.3fm, 记录%.3fm, Δ%.3fm)，解除制动保持",
                    __LINE__, current_forward_dist, forward_brake_hold_distance_,
                    std::abs(current_forward_dist - forward_brake_hold_distance_));
                forward_brake_hold_.store(false, std::memory_order_relaxed);
                applySpeedForwardIntervention(output_control);
            }
        } else {
            applySpeedForwardIntervention(output_control);

            // 紧急制动判定：仅在确实有障碍物(距离在预警区内且非哨兵)时允许进入保持
            if (output_control.linear.x <= 0.01 && handle_control.linear.x > 0.01
                && current_forward_dist < decision_params_.warning_zone
                && current_forward_dist < kNoObstacleDistance) {
                forward_brake_hold_.store(true, std::memory_order_relaxed);
                forward_brake_hold_distance_ = current_forward_dist;
                RCLCPP_INFO(this->get_logger(),
                    "[L%d][前向-制动保持] 触发制动保持，障碍物距离 %.3f m，摇杆 %.3f",
                    __LINE__, forward_brake_hold_distance_, handle_control.linear.x);
            }
        }
        }
    }
    if (decision_params_.enable_backward_collision && current_handle_control_.linear.x < 0.0) {
        double current_backward_dist = min_directional_distance_.min_backward_distance;
        if (backward_brake_hold_.load(std::memory_order_relaxed)) {
            // 读到哨兵值(1e6)或已不在预警区，立即退出保持
            if (current_backward_dist >= kNoObstacleDistance
                || backward_brake_hold_distance_ >= kNoObstacleDistance
                || current_backward_dist >= decision_params_.warning_zone_backward) {
                RCLCPP_INFO(this->get_logger(),
                    "[L%d][后向-制动保持] 退出保持(当前=%.3f, 记录=%.3f, 预警区=%.3f)",
                    __LINE__, current_backward_dist, backward_brake_hold_distance_, decision_params_.warning_zone_backward);
                backward_brake_hold_.store(false, std::memory_order_relaxed);
                applySpeedBackwardIntervention(output_control);
            } else if (std::abs(current_backward_dist - backward_brake_hold_distance_) <= 0.02) {
                output_control.linear.x = 0.0;
                {
                    const auto t = this->now();
                    backward_state_.has_last_detection = true;
                    backward_state_.last_detected_distance = current_backward_dist;
                    backward_state_.last_detection_time = t;
                    backward_state_.current_speed_limit = 0.0;
                    if (backward_state_.is_recovering) {
                        backward_state_.is_recovering = false;
                    }
                }
                RCLCPP_DEBUG(this->get_logger(),
                    "[L%d][后向-制动保持] 障碍物未移动(当前%.3fm, 记录%.3fm, Δ%.3fm)，线速度保持为 0, handle=%.3f",
                    __LINE__, current_backward_dist, backward_brake_hold_distance_,
                    std::abs(current_backward_dist - backward_brake_hold_distance_),
                    handle_control.linear.x);
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "[L%d][后向-制动保持] 障碍物已移动(当前%.3fm, 记录%.3fm, Δ%.3fm)，解除制动保持",
                    __LINE__, current_backward_dist, backward_brake_hold_distance_,
                    std::abs(current_backward_dist - backward_brake_hold_distance_));
                backward_brake_hold_.store(false, std::memory_order_relaxed);
                applySpeedBackwardIntervention(output_control);
            }
        } else {
            applySpeedBackwardIntervention(output_control);
            // 仅在确实有障碍物(距离在预警区内且非哨兵)时允许进入保持
            if (output_control.linear.x >= -0.01 && handle_control.linear.x < -0.01
                && current_backward_dist < decision_params_.warning_zone_backward
                && current_backward_dist < kNoObstacleDistance) {
                backward_brake_hold_.store(true, std::memory_order_relaxed);
                backward_brake_hold_distance_ = current_backward_dist;
                RCLCPP_INFO(this->get_logger(),
                    "[L%d][后向-制动保持] 触发，障碍物距离 %.3f m，handle %.3f",
                    __LINE__, backward_brake_hold_distance_, handle_control.linear.x);
            }
        }
    }
    if (decision_params_.enable_lateral_collision) {
        if (is_backward_distance_nearest_ && std::abs(joystick_msg_.rocker_angle) > 140.0 && joystick_msg_.rocker_x < -50) {
            // 后退紧急状态下跳过侧向干预
        } else {
            // Bug2 修复：即使 angular.z==0（直行），侧向距离极近时也需干预（窄道防刮蹭）
            bool left_emergency = (min_directional_distance_.min_lateral_left_distance
                                   <= decision_params_.emergency_brake_distance_lateral);
            bool right_emergency = (min_directional_distance_.min_lateral_right_distance
                                    <= decision_params_.emergency_brake_distance_lateral);
            if (current_handle_control_.angular.z > 0.0 || (current_handle_control_.angular.z != 0.0 && left_emergency)) {
                applySpeedLeftIntervention(output_control);
            }
            if (current_handle_control_.angular.z < 0.0 || (current_handle_control_.angular.z != 0.0 && right_emergency)) {
                applySpeedRightIntervention(output_control);
            }
        }
    }
    
    // 更新当前控制命令
    current_handle_control_ = output_control;

    // 紧急制动处理：使用快速衰减而非瞬断，减少急停突兀感
    geometry_msgs::msg::Twist filtered_control;
    updateSmoothFilterCommandLimits();
    const double kEmergencyStopThreshold = 0.02;
    if (std::abs(current_handle_control_.linear.x) <= kEmergencyStopThreshold
        && std::abs(current_handle_control_.angular.z) <= kEmergencyStopThreshold) {
        // 快速衰减：每帧保留 30% 的上一帧速度，比瞬断平滑
        filtered_control = smooth_filter_->filter(current_handle_control_);
        const double kBrakeDecay = 0.3;
        if (std::abs(filtered_control.linear.x) > kEmergencyStopThreshold) {
            filtered_control.linear.x *= kBrakeDecay;
        } else {
            filtered_control.linear.x = 0.0;
        }
        if (std::abs(filtered_control.angular.z) > kEmergencyStopThreshold) {
            filtered_control.angular.z *= kBrakeDecay;
        } else {
            filtered_control.angular.z = 0.0;
        }
    } else {
        filtered_control = smooth_filter_->filter(current_handle_control_);
    }

    if (std::abs(filtered_control.angular.z) > decision_params_.absolute_max_angular_speed) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[L%d] 左侧-角速度超限！%.3f rad/s -> %.3f rad/s", __LINE__, filtered_control.angular.z,
            std::copysign(decision_params_.absolute_max_angular_speed, filtered_control.angular.z));
        filtered_control.angular.z = std::copysign(decision_params_.absolute_max_angular_speed, filtered_control.angular.z);
    }

    // 输出控制命令
    if (has_swept_result) {
        logSweptAvoidanceShadow(swept_result, filtered_control);
    }

    if (control_output_callback_) {
        control_output_callback_(filtered_control);
    }
}

void DecisionMaker::setTracking(std::shared_ptr<Tracking> tracking)
{
    tracking_ = tracking;
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 设置跟踪器引用", __LINE__);
}

void DecisionMaker::setObstaclePredictor(std::shared_ptr<ObstaclePredictor> predictor)
{
    obstacle_predictor_ = predictor;
    syncObstaclePredictorParams();
    RCLCPP_INFO(this->get_logger(), "[L%d] 设置障碍物位置预测器引用", __LINE__);
}

void DecisionMaker::setEvasionDetector(std::shared_ptr<EvasionDetector> detector)
{
    evasion_detector_ = detector;
    syncEvasionDetectorParams();
    RCLCPP_INFO(this->get_logger(), "[L%d] 设置操作者避让意图检测器引用", __LINE__);
}

void DecisionMaker::syncObstaclePredictorParams()
{
    if (!obstacle_predictor_) return;
    ObstaclePredictorParams pp;
    pp.enable = decision_params_.enable_obstacle_prediction;
    pp.max_prediction_age = decision_params_.obstacle_prediction_max_age;
    pp.max_distance_drift_ratio = decision_params_.obstacle_prediction_drift_ratio;
    pp.min_valid_x = decision_params_.obstacle_prediction_min_valid_x;
    obstacle_predictor_->updateParams(pp);
}

void DecisionMaker::syncEvasionDetectorParams()
{
    if (!evasion_detector_) return;
    EvasionDetectorParams ep;
    ep.enable = decision_params_.enable_evasion_detection;
    ep.angular_threshold = decision_params_.evasion_angular_threshold;
    ep.angular_full_evasion = decision_params_.evasion_angular_full;
    ep.min_factor = decision_params_.evasion_min_factor;
    ep.safe_distance_override = decision_params_.fixed_stopping_distance_forward;
    evasion_detector_->updateParams(ep);
}

std::vector<ClusterInfo> DecisionMaker::getRadarClusters(const std::string& radar_name) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    auto it = radar_cluster_map_.find(radar_name);
    if (it != radar_cluster_map_.end()) {
        return it->second;
    }

    // 如果找不到对应的雷达数据，返回空列表
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 未找到雷达 %s 的聚类数据", __LINE__, radar_name.c_str());
    return std::vector<ClusterInfo>();
}

void DecisionMaker::clearRadarClusters(const std::string& radar_name)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    auto it = radar_cluster_map_.find(radar_name);
    if (it != radar_cluster_map_.end()) {
        radar_cluster_map_.erase(it);
        RCLCPP_DEBUG(this->get_logger(), "[L%d] 清除雷达 %s 的聚类数据", __LINE__, radar_name.c_str());
    }

    // 同时清除该雷达的更新时间
    auto time_it = radar_last_update_time_map_.find(radar_name);
    if (time_it != radar_last_update_time_map_.end()) {
        radar_last_update_time_map_.erase(time_it);
        RCLCPP_DEBUG(this->get_logger(), "[L%d] 清除雷达 %s 的更新时间", __LINE__, radar_name.c_str());
    }
}

// ==================== 碰撞检测开关控制接口 ====================

void DecisionMaker::applyProfile(EnvironmentProfile profile)
{
    // 重置各方向恢复状态，避免切换后旧限速残留
    auto reset_state = [](DirectionalState& s, double max_v) {
        s.is_recovering = false;
        s.current_speed_limit = max_v;
        s.target_speed_limit  = max_v;
    };

    std::lock_guard<std::mutex> lock(state_mutex_);

    switch (profile) {
    case EnvironmentProfile::INDOOR:
        decision_params_.warning_zone              = 2.0;
        decision_params_.deceleration_zone         = 1.0;
        decision_params_.emergency_brake_distance  = 0.15;  // 前向刹停后障碍物距车前 ~15cm
        decision_params_.fixed_stopping_distance_forward = 0.13; // 必须 < emergency_brake_distance
        decision_params_.max_speed_limit           = 2.22;  // 8 km/h（室内可达 8 km/h，配合 TTC 自适应）
        decision_params_.state_hold_time           = 0.8;   // 0.5→0.8s 减少障碍物闪烁导致的震荡
        decision_params_.speed_recovery_time       = 2.0;
        decision_params_.enable_lateral_collision  = true;
        decision_params_.enable_backward_collision = true;
        reset_state(forward_state_,  decision_params_.max_speed_limit);
        reset_state(backward_state_, decision_params_.max_speed_limit);
        reset_state(left_state_,     decision_params_.max_speed_limit);
        reset_state(right_state_,    decision_params_.max_speed_limit);
        RCLCPP_INFO(this->get_logger(),
            "[DecisionMaker] 切换到【室内】配置 | "
            "限速 %.2f m/s (%.1f km/h) | 预警区 %.1fm | 保持时间 %.1fs | 侧向/后向碰撞 已启用",
            decision_params_.max_speed_limit,
            decision_params_.max_speed_limit * 3.6,
            decision_params_.warning_zone,
            decision_params_.state_hold_time);
        break;

    case EnvironmentProfile::OUTDOOR:
        decision_params_.warning_zone              = 3.0;   // 8 km/h 时 TTC=2s 对应 4.4m，预警区适当增大
        decision_params_.deceleration_zone         = 2.2;   // 配合 8 km/h 制动距离
        decision_params_.emergency_brake_distance  = 0.15;  // 前向刹停后障碍物距车前 ~15cm
        decision_params_.fixed_stopping_distance_forward = 0.13; // 必须 < emergency_brake_distance
        decision_params_.max_speed_limit           = 2.222; // 8 km/h
        decision_params_.state_hold_time           = 0.3;
        decision_params_.speed_recovery_time       = 1.5;
        decision_params_.enable_lateral_collision  = false;
        decision_params_.enable_backward_collision = false;
        reset_state(forward_state_,  decision_params_.max_speed_limit);
        reset_state(backward_state_, decision_params_.max_speed_limit);
        reset_state(left_state_,     decision_params_.max_speed_limit);
        reset_state(right_state_,    decision_params_.max_speed_limit);
        RCLCPP_INFO(this->get_logger(),
            "[DecisionMaker] 切换到【室外】配置 | "
            "限速 %.2f m/s | 预警区 %.1fm | 侧向/后向碰撞 已禁用",
            decision_params_.max_speed_limit,
            decision_params_.warning_zone);
        break;
    }
}

void DecisionMaker::setForwardCollisionEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    decision_params_.enable_forward_collision = enabled;
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 前向碰撞检测 %s", __LINE__, enabled ? "启用" : "禁用");
}

void DecisionMaker::setLateralCollisionEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    decision_params_.enable_lateral_collision = enabled;
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 侧向碰撞检测 %s", __LINE__, enabled ? "启用" : "禁用");
}

void DecisionMaker::setBackwardCollisionEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    decision_params_.enable_backward_collision = enabled;
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 后向碰撞检测 %s", __LINE__, enabled ? "启用" : "禁用");
}

bool DecisionMaker::isForwardCollisionEnabled() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return decision_params_.enable_forward_collision;
}

bool DecisionMaker::isLateralCollisionEnabled() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return decision_params_.enable_lateral_collision;
}

bool DecisionMaker::isBackwardCollisionEnabled() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return decision_params_.enable_backward_collision;
}

void DecisionMaker::configureCollisionDetection(bool forward_enabled, bool lateral_enabled, bool backward_enabled)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    decision_params_.enable_forward_collision = forward_enabled;
    decision_params_.enable_lateral_collision = lateral_enabled;
    decision_params_.enable_backward_collision = backward_enabled;

    RCLCPP_DEBUG(this->get_logger(), "[L%d] 批量配置碰撞检测: 前向=%s, 侧向=%s, 后向=%s", __LINE__, forward_enabled ? "启用" : "禁用", lateral_enabled ? "启用" : "禁用",
        backward_enabled ? "启用" : "禁用");
}

std::string DecisionMaker::getCollisionDetectionStatus() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    std::string status = "碰撞检测状态: ";
    status += "前向=" + std::string(decision_params_.enable_forward_collision ? "启用" : "禁用") + ", ";
    status += "侧向=" + std::string(decision_params_.enable_lateral_collision ? "启用" : "禁用") + ", ";
    status += "后向=" + std::string(decision_params_.enable_backward_collision ? "启用" : "禁用");

    return status;
}

void DecisionMaker::resetBrakeHoldState()
{
    forward_brake_hold_.store(false, std::memory_order_relaxed);
    backward_brake_hold_.store(false, std::memory_order_relaxed);
}

void DecisionMaker::setCruiseModeActive(bool active)
{
    cruise_mode_active_.store(active, std::memory_order_release);
    if (active) {
        // 进入巡航时立刻清掉前向制动保持，确保本周期开始就按“无保持”逻辑运行
        forward_brake_hold_.store(false, std::memory_order_relaxed);
    }
}

// ==================== 参数配置接口 ====================

void DecisionMaker::setDecisionParams(const DecisionParams& params)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    decision_params_ = params;
    syncRobotDimensionsFromParams();
    syncObstaclePredictorParams();
    syncEvasionDetectorParams();
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 决策参数已更新", __LINE__);
}

DecisionParams DecisionMaker::getDecisionParams() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return decision_params_;
}

DecisionMaker::DecisionStatus DecisionMaker::getStatus() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    DecisionStatus status;
    status.current_speed_limit = current_speed_limit_;
    status.last_detected_distance = last_detected_distance_;
    status.is_recovering = is_recovering_;
    status.active_radar_count = radar_cluster_map_.size();

    // 计算总聚类数量
    size_t total_clusters = 0;
    for (const auto& radar_pair : radar_cluster_map_) {
        total_clusters += radar_pair.second.size();
    }
    status.total_cluster_count = total_clusters;

    return status;
}

// ==================== 距离计算和速度限制 ====================

double DecisionMaker::calculateNearestDistance(const std::vector<ClusterInfo>& clusters)
{
    double min_distance = std::numeric_limits<double>::max();

    for (const auto& cluster : clusters) {
        // 计算聚类中心到原点的距离
        double distance = std::sqrt(cluster.center_x * cluster.center_x + cluster.center_y * cluster.center_y);
        min_distance = std::min(min_distance, distance);

        // 也可以计算最近点的距离（更保守的方法）
        // 增加空指针检查
        if (cluster.points && !cluster.points->points.empty()) {
            for (const auto& point : cluster.points->points) {
                double point_distance = std::sqrt(point.x * point.x + point.y * point.y);
                min_distance = std::min(min_distance, point_distance);
            }
        }
    }

    return min_distance;
}

SpeedLimit DecisionMaker::calculateSpeedLimit(double distance, double approach_velocity)
{
    SpeedLimit limit;
    limit.distance_threshold = distance;

    // 固定停车距离：小于此距离该方向速度=0，不判断TTC
    if (distance <= decision_params_.fixed_stopping_distance_forward) {
        limit.max_linear_speed = 0.0;
        limit.max_angular_speed = 0.0;
        std_msgs::msg::Empty narrow_end_msg;
        narrow_end_publisher_->publish(narrow_end_msg);
        RCLCPP_WARN(this->get_logger(), "[L%d][前向-固定停车] 距离 %.3f m <= %.3f m，强制停车", __LINE__, distance, decision_params_.fixed_stopping_distance_forward);
        return limit;
    }

    const double v_eps = 0.01;
    const bool use_ttc = (approach_velocity >= decision_params_.min_approach_velocity_for_ttc);
    double ttc = (approach_velocity > v_eps) ? (distance / approach_velocity) : 999.0;

    // 接近速度高于阈值（默认 7 km/h）时换用高速剖面的 deceleration / emergency 基准
    const double hs_th = decision_params_.forward_ttc_high_speed_threshold_m_s;
    const bool use_high_speed_ttc = (hs_th > 0.0 && approach_velocity > hs_th);
    const double decel_ttc_sel = use_high_speed_ttc ? decision_params_.deceleration_ttc_high_speed : decision_params_.deceleration_ttc;
    const double emergency_brake_base_sel = use_high_speed_ttc ? decision_params_.emergency_brake_ttc_high_speed : decision_params_.emergency_brake_ttc;

    // 速度相关紧急制动 TTC：高速时需要更早制动（制动距离更长）
    // effective_ttc = base + v/(2a) + margin，确保有足够时间刹停
    double effective_emergency_ttc = emergency_brake_base_sel;
    if (approach_velocity > v_eps && decision_params_.emergency_brake_ttc_decel > 1e-6) {
        double speed_dependent_ttc = decision_params_.emergency_brake_ttc_base
            + approach_velocity / (2.0 * decision_params_.emergency_brake_ttc_decel)
            + decision_params_.emergency_brake_ttc_margin;
        effective_emergency_ttc = std::max(emergency_brake_base_sel, speed_dependent_ttc);
    }

    // TTC 模式：基于碰撞时间，速度自适应，叠加距离兜底确保低速安全
    if (use_ttc) {
        // 优先检查距离兜底：即使 TTC 充裕，距离过近仍需制动/减速
        if (distance <= decision_params_.emergency_brake_distance) {
            limit.max_linear_speed = 0.0;
            limit.max_angular_speed = 0.0;
            RCLCPP_WARN(this->get_logger(), "[L%d][前向-距离兜底制动] 距离 %.3f m <= %.3f m，强制制动 (速度 %.1f km/h)",
                __LINE__, distance, decision_params_.emergency_brake_distance, approach_velocity * 3.6);
            return limit;
        }
        if (distance <= decision_params_.deceleration_zone) {
            double dist_denom = decision_params_.deceleration_zone - decision_params_.emergency_brake_distance;
            double dist_ratio = (dist_denom > 1e-6) ? std::clamp((distance - decision_params_.emergency_brake_distance) / dist_denom, 0.0, 1.0) : 0.0;
            double dist_limit = decision_params_.min_speed_limit + std::sqrt(dist_ratio) * (decision_params_.max_speed_limit - decision_params_.min_speed_limit);
            // TTC 也可能给出更严格的限制，取两者最小值
            double ttc_limit = std::numeric_limits<double>::max();
            if (ttc <= effective_emergency_ttc) {
                ttc_limit = 0.0;
            } else if (ttc <= decel_ttc_sel) {
                double ttc_denom = decel_ttc_sel - effective_emergency_ttc;
                double ttc_ratio = (ttc_denom > 1e-6) ? std::clamp((ttc - effective_emergency_ttc) / ttc_denom, 0.0, 1.0) : 0.0;
                ttc_limit = decision_params_.min_speed_limit + std::sqrt(ttc_ratio) * (decision_params_.max_speed_limit - decision_params_.min_speed_limit);
            }
            limit.max_linear_speed = std::min(dist_limit, ttc_limit);
            limit.max_angular_speed = std::numeric_limits<double>::max();
            RCLCPP_INFO(this->get_logger(), "[L%d][前向-TTC+距离减速] TTC %.3f s, 距离 %.3f m, 限速: %.3f m/s (%.1f km/h)",
                __LINE__, ttc, distance, limit.max_linear_speed, limit.max_linear_speed * 3.6);
            return limit;
        }
        // 纯 TTC 判定（距离在减速区之外）
        if (ttc <= effective_emergency_ttc) {
            limit.max_linear_speed = 0.0;
            limit.max_angular_speed = 0.0;
            RCLCPP_WARN(this->get_logger(), "[L%d][前向-TTC制动区] 紧急制动！TTC: %.3f s <= %.3f s, 距离: %.3f m, 速度: %.1f km/h",
                __LINE__, ttc, effective_emergency_ttc, distance, approach_velocity * 3.6);
            return limit;
        }
        if (ttc <= decel_ttc_sel) {
            double denominator = decel_ttc_sel - effective_emergency_ttc;
            if (denominator <= 1e-6) {
                limit.max_linear_speed = 0.0;
                limit.max_angular_speed = std::numeric_limits<double>::max();
                return limit;
            }
            double ratio = (ttc - effective_emergency_ttc) / denominator;
            ratio = std::max(0.0, std::min(1.0, ratio));
            double smooth_ratio = std::sqrt(ratio);
            limit.max_linear_speed = decision_params_.min_speed_limit + smooth_ratio * (decision_params_.max_speed_limit - decision_params_.min_speed_limit);
            limit.max_angular_speed = std::numeric_limits<double>::max();
            RCLCPP_INFO(this->get_logger(), "[L%d][前向-TTC减速区] TTC %.3f s, 距离 %.3f m, 限速: %.3f m/s (%.1f km/h)",
                __LINE__, ttc, distance, limit.max_linear_speed, limit.max_linear_speed * 3.6);
            return limit;
        }
        if (ttc <= decision_params_.warning_ttc) {
            limit.max_linear_speed = std::numeric_limits<double>::max();
            limit.max_angular_speed = std::numeric_limits<double>::max();
            RCLCPP_DEBUG(this->get_logger(), "[L%d][前向-TTC预警区] TTC %.3f s, 不限速", __LINE__, ttc);
            return limit;
        }
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();
        RCLCPP_DEBUG(this->get_logger(), "[L%d][前向-TTC正常区] TTC %.3f s > %.3f s，不限速", __LINE__, ttc, decision_params_.warning_ttc);
        return limit;
    }

    // 距离模式（低速回退）：approach_velocity 低于阈值时使用
    if (distance <= decision_params_.emergency_brake_distance) {
        limit.max_linear_speed = 0.0;
        limit.max_angular_speed = 0.0;
        RCLCPP_WARN(this->get_logger(), "[L%d][前向-制动区] 紧急制动！距离: %.3f m <= %.3f m", __LINE__, distance, decision_params_.emergency_brake_distance);
        return limit;
    }
    if (distance <= decision_params_.deceleration_zone) {
        double denominator = decision_params_.deceleration_zone - decision_params_.emergency_brake_distance;
        if (denominator <= 1e-6) {
            limit.max_linear_speed = 0.0;
            limit.max_angular_speed = std::numeric_limits<double>::max();
            return limit;
        }
        double ratio = (distance - decision_params_.emergency_brake_distance) / denominator;
        ratio = std::max(0.0, std::min(1.0, ratio));
        double smooth_ratio = std::sqrt(ratio);
        limit.max_linear_speed = decision_params_.min_speed_limit + smooth_ratio * (decision_params_.max_speed_limit - decision_params_.min_speed_limit);
        limit.max_angular_speed = std::numeric_limits<double>::max();
        RCLCPP_INFO(this->get_logger(), "[L%d][前向-减速区] 距离 %.3f m, 限速: %.3f m/s (%.1f km/h)", __LINE__, distance, limit.max_linear_speed, limit.max_linear_speed * 3.6);
    } else if (distance <= decision_params_.warning_zone) {
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();
        RCLCPP_DEBUG(this->get_logger(), "[L%d][前向-预警区] 距离 %.3f m, 不限速", __LINE__, distance);
    } else {
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();
        RCLCPP_DEBUG(this->get_logger(), "[L%d][前向-正常区] 距离 %.3f m > %.3f m，不限速", __LINE__, distance, decision_params_.warning_zone);
    }
    return limit;
}

geometry_msgs::msg::Twist DecisionMaker::applySpeedLimit(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit)
{
    geometry_msgs::msg::Twist limited_control = original_control;
    bool need_kalman_deceleration = false;
    const bool forward_brake_active = (limit.max_linear_speed <= 0.01) && (original_control.linear.x > 0.0);
    // ==================== 线速度限制 ====================
    // 只限制前进方向的速度（x轴正方向），后退不受前向碰撞限制
    if (original_control.linear.x > 0.0) {
        // 前进方向需要限制
        if (original_control.linear.x > limit.max_linear_speed) {
            need_kalman_deceleration = true;
            limited_control.linear.x = limit.max_linear_speed;
            //judgeForwardWarningScene(limit);
            RCLCPP_INFO(this->get_logger(), "[L%d] 前进速度限制: %.3f m/s -> %.3f m/s", __LINE__, original_control.linear.x, limited_control.linear.x);
        }
    } else {
        // 后退方向保持不变（前向碰撞不影响后退）
        limited_control.linear.x = original_control.linear.x;
    }

    // ==================== 角速度限制 ====================
    // 前向避障只限制线速度，角速度使用原车角速度（不限制）
    // 保持原车角速度不变
    limited_control.angular.z = original_control.angular.z;

    // 保持其他控制分量不变（侧向移动、垂直运动等）
    limited_control.linear.y = original_control.linear.y;
    limited_control.linear.z = original_control.linear.z;
    limited_control.angular.x = original_control.angular.x;
    limited_control.angular.y = original_control.angular.y;
    if (need_kalman_deceleration) {
        limited_control = kalman_filter_->filter(limited_control);
        if (limited_control.linear.x > -0.01 && limited_control.linear.x < 0.01) {
            limited_control.linear.x = 0.0;
        }
    }

    // 前向制动后短时间内，将低速残余线速度归零，避免刹停末端仍有微小爬行
    if (forward_brake_active) {
        last_forward_brake_time_ = this->now();
    }
    if (decision_params_.forward_brake_zero_speed_window_ms > 0.0 && decision_params_.forward_brake_zero_speed_threshold_kmh > 0.0) {
        const double zero_speed_threshold_m_s = decision_params_.forward_brake_zero_speed_threshold_kmh / 3.6;
        const double elapsed_ms = (this->now() - last_forward_brake_time_).seconds() * 1000.0;
        if (elapsed_ms <= decision_params_.forward_brake_zero_speed_window_ms
            && std::abs(limited_control.linear.x) < zero_speed_threshold_m_s) {
            limited_control.linear.x = 0.0;
        }
    }

    // 前向制动且摇杆 Y 在死区内：禁止转向（与 /sensor/joystick 的 rocker_y 一致）
    // 轮速反馈线速度须 < forward_brake_chassis_speed_threshold（见 decision_maker_params.yaml）
    const double dz = decision_params_.forward_brake_joystick_y_deadzone;
    const double chassis_spd_th = decision_params_.forward_brake_chassis_speed_threshold;
    if (decision_params_.enable_forward_brake_joystick_y_deadzone && dz > 0.0 && limit.max_linear_speed <= 0.01
        && original_control.linear.x > 0.0) {
        int16_t rocker_y = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            rocker_y = joystick_msg_.rocker_y;
        }
        if (tracking_ && static_cast<double>(std::abs(static_cast<int>(rocker_y))) < dz
            && tracking_->getWheelSpeedLinearVelocity() < chassis_spd_th) {
            limited_control.angular.z = 0.0;
        }
    }

    return limited_control;
}

// ==================== 侧向速度限制计算和应用 ====================

SpeedLimit DecisionMaker::calculateSpeedLimitForBackward(double distance, double approach_velocity)
{
    SpeedLimit limit;
    limit.distance_threshold = distance;
    is_backward_distance_nearest_ = false;

    // 固定停车距离：小于此距离该方向速度=0，不判断TTC
    if (distance <= decision_params_.fixed_stopping_distance_backward) {
        limit.max_linear_speed = 0.0;
        limit.max_angular_speed = 0.0;
        is_backward_distance_nearest_ = true;
        RCLCPP_WARN(this->get_logger(), "[L%d][后向-固定停车] 距离 %.3f m <= %.3f m，强制停车", __LINE__, distance, decision_params_.fixed_stopping_distance_backward);
        return limit;
    }

    if (distance >= decision_params_.emergency_brake_distance_backward
        && distance <= decision_params_.warning_distance_backward) {
        publishSafetyAlarmForBackward("后向", 2);
    }

    const double v_eps = 0.01;
    const bool use_ttc = (approach_velocity >= decision_params_.min_approach_velocity_for_ttc);
    double ttc = (approach_velocity > v_eps) ? (distance / approach_velocity) : 999.0;

    if (use_ttc) {
        if (ttc <= decision_params_.emergency_brake_ttc_backward) {
            limit.max_linear_speed = 0.0;
            limit.max_angular_speed = 0.0;
            is_backward_distance_nearest_ = true;
            return limit;
        }
        if (ttc <= decision_params_.deceleration_ttc_backward) {
            double denominator = decision_params_.deceleration_ttc_backward - decision_params_.emergency_brake_ttc_backward;
            if (denominator <= 1e-6) {
                limit.max_linear_speed = 0.0;
                limit.max_angular_speed = 0.0;
                return limit;
            }
            double ratio = (ttc - decision_params_.emergency_brake_ttc_backward) / denominator;
            ratio = std::max(0.0, std::min(1.0, ratio));
            double t = ratio;
            double smooth_ratio = (1.0 - t) * (1.0 - t) * (1.0 - t) * 0.0 + 3.0 * (1.0 - t) * (1.0 - t) * t * 0.2 + 3.0 * (1.0 - t) * t * t * 0.8 + t * t * t * 1.0;
            limit.max_linear_speed = decision_params_.min_speed_limit + smooth_ratio * (decision_params_.max_speed_limit_backward - decision_params_.min_speed_limit);
            double angular_ratio = std::sqrt(smooth_ratio);
            limit.max_angular_speed = limit.max_linear_speed * (0.2 + 0.3 * angular_ratio);
            limit.max_angular_speed = std::max(limit.max_angular_speed, 0.05);
            RCLCPP_DEBUG(this->get_logger(), "[L%d][后向-TTC减速区] TTC %.3f s, 限速: %.3f m/s", __LINE__, ttc, limit.max_linear_speed);
            return limit;
        }
        if (ttc <= decision_params_.warning_ttc_backward) {
            limit.max_linear_speed = std::numeric_limits<double>::max();
            limit.max_angular_speed = std::numeric_limits<double>::max();
            return limit;
        }
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();
        return limit;
    }

    if (distance <= decision_params_.emergency_brake_distance_backward) {
        limit.max_linear_speed = 0.0;
        limit.max_angular_speed = 0.0;
        is_backward_distance_nearest_ = true;
        return limit;
    }
    if (distance <= decision_params_.deceleration_zone_backward) {
        double denominator = decision_params_.deceleration_zone_backward - decision_params_.emergency_brake_distance_backward;
        if (denominator <= 1e-6) {
            limit.max_linear_speed = 0.0;
            limit.max_angular_speed = 0.0;
            return limit;
        }
        double ratio = (distance - decision_params_.emergency_brake_distance_backward) / denominator;
        ratio = std::max(0.0, std::min(1.0, ratio));
        double t = ratio;
        double smooth_ratio = (1.0 - t) * (1.0 - t) * (1.0 - t) * 0.0 + 3.0 * (1.0 - t) * (1.0 - t) * t * 0.2 + 3.0 * (1.0 - t) * t * t * 0.8 + t * t * t * 1.0;
        limit.max_linear_speed = decision_params_.min_speed_limit + smooth_ratio * (decision_params_.max_speed_limit_backward - decision_params_.min_speed_limit);
        double angular_ratio = std::sqrt(smooth_ratio);
        limit.max_angular_speed = limit.max_linear_speed * (0.2 + 0.3 * angular_ratio);
        limit.max_angular_speed = std::max(limit.max_angular_speed, 0.05);
        RCLCPP_DEBUG(this->get_logger(), "[L%d][后向-减速区] 距离 %.3f m, 限速: %.3f m/s", __LINE__, distance, limit.max_linear_speed);
    } else if (distance <= decision_params_.warning_zone_backward) {
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();
    } else {
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();
    }
    return limit;
}

SpeedLimit DecisionMaker::calculateSpeedLimitForLateral(double distance, double approach_velocity)
{
    SpeedLimit limit;
    limit.distance_threshold = distance;

    // 固定停车距离：小于此距离该方向速度=0，不判断TTC
    if (distance <= decision_params_.fixed_stopping_distance_lateral) {
        limit.max_linear_speed = 0.0;
        limit.max_angular_speed = 0.0;
        RCLCPP_WARN(this->get_logger(), "[L%d][侧向-固定停车] 距离 %.3f m <= %.3f m，强制停车", __LINE__, distance, decision_params_.fixed_stopping_distance_lateral);
        return limit;
    }

    const double v_eps = 0.01;
    const bool use_ttc = (approach_velocity >= decision_params_.min_approach_velocity_for_ttc);
    double ttc = (approach_velocity > v_eps) ? (distance / approach_velocity) : 999.0;

    if (use_ttc) {
        if (ttc <= decision_params_.emergency_brake_ttc_lateral) {
            limit.max_linear_speed = 0.0;
            limit.max_angular_speed = 0.0;
            RCLCPP_WARN(this->get_logger(), "[L%d][侧向-TTC制动区] 紧急制动！TTC: %.3f s, 距离: %.3f m", __LINE__, ttc, distance);
            return limit;
        }
        if (ttc <= decision_params_.deceleration_ttc_lateral) {
            double denominator = decision_params_.deceleration_ttc_lateral - decision_params_.emergency_brake_ttc_lateral;
            if (denominator <= 1e-6) {
                // Bug7 修复：配置异常时保留最低角速度而非直接急停
                limit.max_linear_speed = std::numeric_limits<double>::max();
                limit.max_angular_speed = 0.05;
                RCLCPP_WARN(this->get_logger(), "[L%d][侧向-TTC] deceleration_ttc_lateral ≈ emergency_brake_ttc_lateral，配置异常，使用最低角速度", __LINE__);
                return limit;
            }
            double ratio = (ttc - decision_params_.emergency_brake_ttc_lateral) / denominator;
            ratio = std::max(0.0, std::min(1.0, ratio));
            double smooth_ratio = std::sqrt(ratio);
            limit.max_linear_speed = std::numeric_limits<double>::max();
            limit.max_angular_speed = std::max(decision_params_.absolute_max_angular_speed * smooth_ratio, 0.05);
            RCLCPP_DEBUG(this->get_logger(), "[L%d][侧向-TTC减速区] TTC %.3f s, 角速度限速: %.3f rad/s", __LINE__, ttc, limit.max_angular_speed);
            return limit;
        }
        if (ttc <= decision_params_.warning_ttc_lateral) {
            limit.max_linear_speed = std::numeric_limits<double>::max();
            limit.max_angular_speed = std::numeric_limits<double>::max();
            return limit;
        }
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();
        return limit;
    }

    if (distance <= decision_params_.emergency_brake_distance_lateral) {
        limit.max_linear_speed = 0.0;
        limit.max_angular_speed = 0.0;
        RCLCPP_WARN(this->get_logger(), "[L%d][侧向-制动区] 紧急制动！距离: %.3f m", __LINE__, distance);
        return limit;
    }
    if (distance <= decision_params_.deceleration_zone_lateral) {
        double denominator = decision_params_.deceleration_zone_lateral - decision_params_.emergency_brake_distance_lateral;
        if (denominator <= 1e-6) {
            // Bug7 修复：配置异常时保留最低角速度而非直接急停
            limit.max_linear_speed = std::numeric_limits<double>::max();
            limit.max_angular_speed = 0.05;
            RCLCPP_WARN(this->get_logger(), "[L%d][侧向-距离] deceleration_zone ≈ emergency_brake_distance，配置异常，使用最低角速度", __LINE__);
            return limit;
        }
        double ratio = (distance - decision_params_.emergency_brake_distance_lateral) / denominator;
        ratio = std::max(0.0, std::min(1.0, ratio));
        double smooth_ratio = std::sqrt(ratio);
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::max(decision_params_.absolute_max_angular_speed * smooth_ratio, 0.05);
        RCLCPP_DEBUG(this->get_logger(), "[L%d][侧向-减速区] 距离 %.3f m, 角速度限速: %.3f rad/s", __LINE__, distance, limit.max_angular_speed);
    } else if (distance <= decision_params_.warning_zone_lateral) {
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();
    } else {
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();
    }
    return limit;
}

geometry_msgs::msg::Twist DecisionMaker::applySpeedLimitForBackward(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit)
{
    geometry_msgs::msg::Twist limited_control = original_control;
    const auto now_time = this->now();
    double dt = (now_time - last_backward_limit_time_).seconds();
    if (dt <= 1e-4 || dt > 0.2) {
        dt = 0.02; // 回调周期兜底，避免大时间跳变导致限速突变
    }
    last_backward_limit_time_ = now_time;

    // ==================== 线速度限制 ====================
    // 只限制后退方向的速度（x轴负方向），前进不受后向碰撞限制
    if (original_control.linear.x < 0.0) {
        // 目标后退速度：先按后向全局上限，再按本次避障限速
        double desired_backward_speed = std::min(std::abs(original_control.linear.x), decision_params_.max_speed_limit_backward);
        if (limit.max_linear_speed < std::numeric_limits<double>::max()) {
            desired_backward_speed = std::min(desired_backward_speed, std::max(0.0, limit.max_linear_speed));
        }

        // 急停两段式：先降到爬行速度，确认窗口后再锁停
        const bool emergency_requested = (limit.max_linear_speed <= 1e-6);
        if (emergency_requested) {
            if (!backward_emergency_active_) {
                backward_emergency_active_ = true;
                backward_emergency_start_time_ = now_time;
            }
            const double emergency_elapsed_ms = (now_time - backward_emergency_start_time_).seconds() * 1000.0;
            if (emergency_elapsed_ms < decision_params_.backward_emergency_hold_ms) {
                desired_backward_speed = std::min(desired_backward_speed, decision_params_.backward_emergency_creep_speed);
            } else {
                desired_backward_speed = 0.0;
            }
        } else {
            backward_emergency_active_ = false;
        }

        // 倒车斜率限制：减速与恢复分别按不同加速度上限，避免“台阶式”变化
        const double last_speed = (last_backward_limited_linear_x_ < 0.0) ? std::abs(last_backward_limited_linear_x_) : 0.0;
        double next_speed = desired_backward_speed;
        if (desired_backward_speed < last_speed) {
            const double max_down_step = decision_params_.backward_brake_decel_max_m_s2 * dt;
            next_speed = std::max(desired_backward_speed, last_speed - max_down_step);
        } else {
            const double max_up_step = decision_params_.backward_recover_accel_max_m_s2 * dt;
            next_speed = std::min(desired_backward_speed, last_speed + max_up_step);
        }

        limited_control.linear.x = -next_speed;
        last_backward_limited_linear_x_ = limited_control.linear.x;

        if (std::abs(original_control.linear.x - limited_control.linear.x) > 1e-6) {
            RCLCPP_DEBUG(this->get_logger(), "[L%d] 后退速度限制(平顺): %.3f m/s -> %.3f m/s", __LINE__, original_control.linear.x, limited_control.linear.x);
        }
    } else {
        // 前进方向保持不变（后向碰撞不影响前进）
        limited_control.linear.x = original_control.linear.x;
        last_backward_limited_linear_x_ = limited_control.linear.x;
        backward_emergency_active_ = false;
    }

    // ==================== 角速度限制 ====================
    // 当后退速度受限时，角速度也应该相应受限以保证安全转向
    if (std::abs(original_control.angular.z) > limit.max_angular_speed) {
        // 保持转向方向，只限制幅度
        limited_control.angular.z = std::copysign(limit.max_angular_speed, original_control.angular.z);
        RCLCPP_DEBUG(this->get_logger(), "[L%d] 后向-角速度限制: %.3f rad/s -> %.3f rad/s", __LINE__, original_control.angular.z, limited_control.angular.z);
    }

    // ==================== 后向碰撞特殊处理 ====================
    // 如果后方距离非常近（急刹区域），同时有大的转向指令，
    // 可能需要完全停止而不是继续转向以避免碰撞
    if (limit.max_linear_speed == 0.0 && std::abs(original_control.angular.z) > 0.1) {
        // 急刹状态下，如果有显著的转向意图，考虑保留小量的转向能力以便脱困
        // 但线速度必须为0
        limited_control.angular.z = std::copysign(0.1, original_control.angular.z);
        RCLCPP_DEBUG(this->get_logger(), "[L%d] 后向-急刹状态：线速度=0，保留小量转向能力 %.3f rad/s", __LINE__, limited_control.angular.z);
    }

    // 保持其他控制分量不变
    limited_control.linear.y = original_control.linear.y;
    limited_control.linear.z = original_control.linear.z;
    limited_control.angular.x = original_control.angular.x;
    limited_control.angular.y = original_control.angular.y;

    return limited_control;
}

geometry_msgs::msg::Twist DecisionMaker::applySpeedLimitForLeft(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit)
{
    geometry_msgs::msg::Twist limited_control = original_control;

    // ==================== 线速度限制 ====================
    // Bug8 修复：固定停车距离内（max_linear_speed=0）限制线速度，防止极近距离刮蹭
    // if (limit.max_linear_speed < std::numeric_limits<double>::max()) {
    //     if (std::abs(original_control.linear.x) > limit.max_linear_speed) {
    //         limited_control.linear.x = std::copysign(limit.max_linear_speed, original_control.linear.x);
    //         RCLCPP_DEBUG(this->get_logger(), "[L%d] 左侧-线速度限制: %.3f -> %.3f m/s", __LINE__, original_control.linear.x, limited_control.linear.x);
    //     }
    // }

    // ==================== 角速度限制 ====================
    // 左侧碰撞主要限制左转（正角速度）
    if (original_control.angular.z > 0.0) {
        // 左转方向需要限制
        if (original_control.angular.z > limit.max_angular_speed) {
            limited_control.angular.z = limit.max_angular_speed;
            RCLCPP_DEBUG(this->get_logger(), "[L%d] 左侧-左转角速度限制: %.3f rad/s -> %.3f rad/s", __LINE__, original_control.angular.z, limited_control.angular.z);
        }
    } else {
        // 右转方向不受左侧碰撞限制
        limited_control.angular.z = original_control.angular.z;
    }

    // ==================== 左侧碰撞特殊处理 ====================
    // 如果左侧距离非常近（急刹区域），完全禁止左转
    if (limit.max_angular_speed == 0.0 && original_control.angular.z > 0.0) {
        limited_control.angular.z = 0.0;
        RCLCPP_DEBUG(this->get_logger(), "[L%d] 左侧-急刹状态：禁止左转", __LINE__);
    }

    // 保持其他控制分量不变
    limited_control.linear.y = original_control.linear.y;
    limited_control.linear.z = original_control.linear.z;
    limited_control.angular.x = original_control.angular.x;
    limited_control.angular.y = original_control.angular.y;

    return limited_control;
}

geometry_msgs::msg::Twist DecisionMaker::applySpeedLimitForRight(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit)
{
    geometry_msgs::msg::Twist limited_control = original_control;

    // ==================== 线速度限制 ====================
    // Bug8 修复：固定停车距离内（max_linear_speed=0）限制线速度，防止极近距离刮蹭
    // if (limit.max_linear_speed < std::numeric_limits<double>::max()) {
    //     if (std::abs(original_control.linear.x) > limit.max_linear_speed) {
    //         limited_control.linear.x = std::copysign(limit.max_linear_speed, original_control.linear.x);
    //         RCLCPP_DEBUG(this->get_logger(), "[L%d] 右侧-线速度限制: %.3f -> %.3f m/s", __LINE__, original_control.linear.x, limited_control.linear.x);
    //     }
    // }

    // ==================== 角速度限制 ====================
    // 右侧碰撞主要限制右转（负角速度）
    if (original_control.angular.z < 0.0) {
        // 右转方向需要限制
        if (std::abs(original_control.angular.z) > limit.max_angular_speed) {
            limited_control.angular.z = -limit.max_angular_speed; // 保持负方向
            RCLCPP_DEBUG(this->get_logger(), "[L%d] 右侧-右转角速度限制: %.3f rad/s -> %.3f rad/s", __LINE__, original_control.angular.z, limited_control.angular.z);
        }
    } else {
        // 左转方向不受右侧碰撞限制
        limited_control.angular.z = original_control.angular.z;
    }

    // ==================== 右侧碰撞特殊处理 ====================
    // 如果右侧距离非常近（急刹区域），完全禁止右转
    if (limit.max_angular_speed == 0.0 && original_control.angular.z < 0.0) {
        limited_control.angular.z = 0.0;
        RCLCPP_DEBUG(this->get_logger(), "[L%d] 右侧-急刹状态：禁止右转", __LINE__);
    }

    // 保持其他控制分量不变
    limited_control.linear.y = original_control.linear.y;
    limited_control.linear.z = original_control.linear.z;
    limited_control.angular.x = original_control.angular.x;
    limited_control.angular.y = original_control.angular.y;
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 右侧-限制后角速度: %.3f rad/s", __LINE__, limited_control.angular.z);
    return limited_control;
}

// ==================== 目标位置预测 ====================

void DecisionMaker::predictVehiclePosition()
{
    // 基于当前车辆状态预测未来位置
    // 这里可以实现更复杂的预测算法，比如考虑加速度、转向等
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 简单的位置预测（基于当前速度）
    // 实际应用中可能需要更复杂的运动模型
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 预测车辆位置 - 当前速度: %.3f m/s, 角速度: %.3f rad/s", __LINE__, current_vehicle_state_.linear_velocity,
        current_vehicle_state_.angular_velocity);
}

void DecisionMaker::predictTargetPositionsBasedOnVehicleMotion()
{
    // 当雷达数据超时时，基于车辆运动预测目标位置
    if (cluster_history_.empty()) {
        return;
    }

    auto current_time = this->now();

    // 查找所有雷达中最大的超时时间（使用最旧的雷达更新时间）
    double max_time_since_last_update = 0.0;
    if (!radar_last_update_time_map_.empty()) {
        for (const auto& radar_pair : radar_last_update_time_map_) {
            auto time_since_last_update = (current_time - radar_pair.second).seconds();
            max_time_since_last_update = std::max(max_time_since_last_update, time_since_last_update);
        }
    }

    if (max_time_since_last_update > decision_params_.radar_timeout) {
        // 获取最后一次的聚类数据
        auto last_cluster_data = cluster_history_.back();
        std::vector<ClusterInfo> last_clusters = last_cluster_data.first;

        // 基于轮速反馈预测目标位置
        std::vector<ClusterInfo> predicted_clusters = predictTargetsBasedOnWheelSpeed(last_clusters, max_time_since_last_update, current_vehicle_state_);

        // 从预测的目标中计算最近距离
        if (!predicted_clusters.empty()) {
            double min_distance = calculateNearestDistance(predicted_clusters);
            last_nearest_distance_ = min_distance;
            RCLCPP_DEBUG(this->get_logger(), "[L%d] 基于轮速预测最近距离: %.3f m, 最大超时时间: %.3f s", __LINE__, min_distance, max_time_since_last_update);
        }

        RCLCPP_DEBUG(this->get_logger(), "[L%d] 雷达数据超时，基于轮速反馈预测目标位置，最大超时时间: %.3f s", __LINE__, max_time_since_last_update);
    }
}

std::vector<ClusterInfo> DecisionMaker::predictTargetsBasedOnWheelSpeed(const std::vector<ClusterInfo>& last_clusters, double time_delta,
    const VehicleState& vehicle_state)
{
    std::vector<ClusterInfo> predicted_clusters;

    // 安全检查：如果时间间隔过大（超过0.5秒），认为预测不可靠，返回空
    if (time_delta > 0.5) {
        RCLCPP_WARN(this->get_logger(), "[L%d] 目标预测：时间间隔 %.3f s 过大，预测不可靠，返回空列表", __LINE__, time_delta);
        return predicted_clusters;
    }

    // 如果输入聚类为空，直接返回
    if (last_clusters.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "[L%d] 目标预测：输入聚类为空", __LINE__);
        return predicted_clusters;
    }

    // 计算车辆运动参数
    double linear_velocity = vehicle_state.linear_velocity;
    double angular_velocity = vehicle_state.angular_velocity;

    // 对于前向碰撞检测，如果车辆在后退，预测的意义不大
    if (linear_velocity < -0.01) {
        RCLCPP_DEBUG(this->get_logger(), "[L%d] 目标预测：车辆正在后退(%.3f m/s)，前向预测不适用，返回原始数据", __LINE__, linear_velocity);
        return last_clusters; // 返回原始数据而不是预测数据
    }

    // 计算车辆在时间间隔内的位移（在车辆坐标系中）
    // 假设目标相对地面静止，车辆运动导致目标在车辆坐标系中的位置发生变化
    double vehicle_dx = -linear_velocity * time_delta; // 车辆前进，目标相对后退
    double vehicle_dy = 0.0; // 假设车辆主要沿x轴运动
    double vehicle_dtheta = -angular_velocity * time_delta; // 车辆左转，目标相对右转

    RCLCPP_DEBUG(this->get_logger(), "[L%d] 目标预测: 车辆线速度=%.3f m/s, 角速度=%.3f rad/s, 时间间隔=%.3f s", __LINE__, linear_velocity, angular_velocity, time_delta);
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 车辆相对位移: dx=%.3f m, dy=%.3f m, dtheta=%.3f rad", __LINE__, vehicle_dx, vehicle_dy, vehicle_dtheta);

    // 预计算三角函数值
    double cos_theta = std::cos(vehicle_dtheta);
    double sin_theta = std::sin(vehicle_dtheta);

    for (const auto& cluster : last_clusters) {
        ClusterInfo predicted_cluster = cluster;

        // 目标静止，只考虑车辆运动对目标观测位置的影响
        // 使用车辆运动学模型进行预测

        // 目标在车辆坐标系中的新位置
        // 车辆前进dx，目标相对后退dx
        // 车辆左转dtheta，目标相对右转dtheta
        double new_x = cluster.center_x * cos_theta - cluster.center_y * sin_theta + vehicle_dx;
        double new_y = cluster.center_x * sin_theta + cluster.center_y * cos_theta + vehicle_dy;

        // 安全检查：如果预测位置过于异常（超过合理范围），跳过该聚类
        double predicted_distance = std::sqrt(new_x * new_x + new_y * new_y);
        double original_distance = std::sqrt(cluster.center_x * cluster.center_x + cluster.center_y * cluster.center_y);

        // 如果预测距离与原始距离差异过大（超过50%），认为预测不可靠
        if (std::abs(predicted_distance - original_distance) > original_distance * 0.5 + 1.0) {
            RCLCPP_DEBUG(this->get_logger(), "[L%d] 目标预测：预测位置异常，原始距离=%.3f m, 预测距离=%.3f m，跳过", __LINE__, original_distance, predicted_distance);
            continue; // 跳过异常预测
        }

        // 如果目标预测位置移到了车后方（x < -1.0），也认为不合理
        if (new_x < -1.0) {
            RCLCPP_DEBUG(this->get_logger(), "[L%d] 目标预测：目标移至车后方(x=%.3f m)，跳过", __LINE__, new_x);
            continue;
        }

        // 更新聚类中心位置
        predicted_cluster.center_x = new_x;
        predicted_cluster.center_y = new_y;

        // 更新边界框（简化处理，假设目标尺寸不变）
        double dx_offset = new_x - cluster.center_x;
        double dy_offset = new_y - cluster.center_y;
        predicted_cluster.min_x += dx_offset;
        predicted_cluster.max_x += dx_offset;
        predicted_cluster.min_y += dy_offset;
        predicted_cluster.max_y += dy_offset;

        // 更新点云数据
        if (predicted_cluster.points && !predicted_cluster.points->points.empty()) {
            for (auto& point : predicted_cluster.points->points) {
                double point_new_x = point.x * cos_theta - point.y * sin_theta + vehicle_dx;
                double point_new_y = point.x * sin_theta + point.y * cos_theta + vehicle_dy;
                point.x = point_new_x;
                point.y = point_new_y;
            }
        }

        predicted_clusters.push_back(predicted_cluster);

        RCLCPP_DEBUG(this->get_logger(), "[L%d] 目标预测: 原位置(%.3f, %.3f) -> 预测位置(%.3f, %.3f), 距离变化: %.3f m -> %.3f m", __LINE__, cluster.center_x, cluster.center_y,
            new_x, new_y, original_distance, predicted_distance);
    }

    RCLCPP_DEBUG(this->get_logger(), "[L%d] 目标预测完成: 原始聚类数=%zu, 有效预测数=%zu", __LINE__, last_clusters.size(), predicted_clusters.size());

    return predicted_clusters;
}

// ==================== 机器人尺寸/区域配置接口 ====================
void DecisionMaker::syncRobotDimensionsFromParams()
{
    robot_dimensions_.length = decision_params_.robot_length;
    robot_dimensions_.width = decision_params_.robot_width;
    robot_dimensions_.height = decision_params_.robot_height;
    robot_dimensions_.safety_margin_front = decision_params_.robot_safety_margin_front;
    robot_dimensions_.safety_margin_back = decision_params_.robot_safety_margin_back;
    robot_dimensions_.safety_margin_side = decision_params_.robot_safety_margin_side;
    RCLCPP_DEBUG(this->get_logger(),
        "[L%d] 车体几何: 长=%.3f 宽=%.3f 高=%.3f 裕量前/后/侧=%.3f/%.3f/%.3f",
        __LINE__, robot_dimensions_.length, robot_dimensions_.width, robot_dimensions_.height,
        robot_dimensions_.safety_margin_front, robot_dimensions_.safety_margin_back, robot_dimensions_.safety_margin_side);
}

void DecisionMaker::setRobotDimensions(const RobotDimensions::BaseDimensions& dimensions)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    robot_dimensions_ = dimensions;
    decision_params_.robot_length = dimensions.length;
    decision_params_.robot_width = dimensions.width;
    decision_params_.robot_height = dimensions.height;
    decision_params_.robot_safety_margin_front = dimensions.safety_margin_front;
    decision_params_.robot_safety_margin_back = dimensions.safety_margin_back;
    decision_params_.robot_safety_margin_side = dimensions.safety_margin_side;
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 设置机器人尺寸: 长=%.3f, 宽=%.3f, 高=%.3f", __LINE__, dimensions.length, dimensions.width, dimensions.height);
}

RobotDimensions::BaseDimensions DecisionMaker::getRobotDimensions() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return robot_dimensions_;
}

void DecisionMaker::setCollisionZones(const RobotDimensions::CollisionZones& zones)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    collision_zones_ = zones;
}

RobotDimensions::CollisionZones DecisionMaker::getCollisionZones() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return collision_zones_;
}

void DecisionMaker::calculateMinDirectionalDistance(const std::vector<ClusterInfo>& clusters,
    const std::map<size_t, std::string>& cluster_radar_map)
{
    // 使用“远大于预警区”的哨兵值作为无障碍物默认；若用 warning_zone 作为初值，
    // 当没有任何聚类落入前向走廊时会被 TTC 误判为减速（高速下 TTC = warning_zone / v < warning_ttc）。
    const double kNoObstacleDistance = 1e6;
    MinDirectionalDistance distances { kNoObstacleDistance,
        kNoObstacleDistance,
        kNoObstacleDistance,
        kNoObstacleDistance };

    double half_wid = robot_dimensions_.width * 0.5; // 0.35
    double safety_side = robot_dimensions_.safety_margin_side;

    // 初始化前向最小距离的雷达来源和最近点Z值
    forward_min_distance_radar_source_.clear();
    forward_min_distance_nearest_z_ = 0.0;

    // 初始化左侧最小距离的雷达来源和最近点Z值
    left_min_distance_radar_source_.clear();
    left_min_distance_nearest_z_ = 0.0;

    // 初始化右侧最小距离的雷达来源和最近点Z值
    right_min_distance_radar_source_.clear();
    right_min_distance_nearest_z_ = 0.0;

    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        double backward_distance = std::numeric_limits<double>::max();
        double forward_distance = std::numeric_limits<double>::max();
        double left_distance = std::numeric_limits<double>::max();
        double right_distance = std::numeric_limits<double>::max();
        const bool contributes_forward_main =
            (cluster.source_name != "left_tilt" && cluster.source_name != "right_tilt"
             && cluster.source_name != "back");
        if (contributes_forward_main && std::abs(cluster.center_y) <= (half_wid + safety_side)) {
            if (cluster.center_x >= robot_dimensions_.length) {
                // 距离前缘的前向投影距离，>0 表示前方剩余距离
                forward_distance = (cluster.center_x - robot_dimensions_.length);
            } else if (cluster.center_x > robot_dimensions_.length - decision_params_.foot_distance_to_radar) {
                forward_distance = 0.0;
            }
        }
        // 侧向检测范围：覆盖车身全长（含后半侧面），x ∈ (-safety_margin_back, length + foot_distance)
        if (cluster.center_x > -safety_side && cluster.center_x < robot_dimensions_.length + decision_params_.foot_distance_to_radar) {
            // 用 center_y 和边界框综合判断侧向距离
            // center_y 是 |y| 最小的点（最靠近车体），但可能 <= half_wid
            // 此时用边界框 max_y/min_y 判断聚类是否在车侧伸出
            double y_abs = std::abs(cluster.center_y);
            double y_protrude = 0.0;
            bool has_lateral_threat = false;
            int lateral_side = 0; // 1=left, -1=right

            if (y_abs > half_wid) {
                // 正常情况：center 点在车体外
                y_protrude = y_abs - half_wid;
                has_lateral_threat = true;
                lateral_side = (cluster.center_y > 0.0) ? 1 : -1;
            } else {
                // center 在车体投影内，但边界框伸出车侧：用车缘到框边的真实伸出量（米），
                // 勿写死 0 —— 否则静止时噪声/微伸出也会被当成 0 m，误触固定停车。
                if (cluster.max_y > half_wid) {
                    y_protrude = std::max(0.0, cluster.max_y - half_wid);
                    has_lateral_threat = true;
                    lateral_side = 1;
                } else if (cluster.min_y < -half_wid) {
                    y_protrude = std::max(0.0, -half_wid - cluster.min_y);
                    has_lateral_threat = true;
                    lateral_side = -1;
                }
            }

            if (has_lateral_threat) {
                // 侧向主导判定：
                // - 在车身范围内（center_x < length）的障碍物直接视为侧向威胁
                // - 超出车身前缘附近的聚类须满足 lateral_dominance_ratio 以区分前向斜向障碍
                bool is_beside_body = (cluster.center_x < robot_dimensions_.length);
                bool lateral_dominant = (y_protrude >= decision_params_.lateral_dominance_ratio * std::max(cluster.center_x, 0.01));
                if (is_beside_body || lateral_dominant) {
                    if (lateral_side > 0) {
                        left_distance = y_protrude;
                    } else {
                        right_distance = y_protrude;
                    }
                }
            }
        }
        // 后向距离
        if (cluster.center_x < 0.0 && cluster.source_name == "back" && std::abs(cluster.center_x) < decision_params_.warning_zone_backward) {
            backward_distance = std::abs(cluster.center_x) - decision_params_.radar_distance_to_vehicle_rear;
        }
        // 如果这个聚类提供了更小的前向距离，记录雷达来源和最近点的Z值
        if (forward_distance < distances.min_forward_distance) {
            distances.min_forward_distance = std::max(0.0, forward_distance);
            // 只取小数点后两位
            distances.min_forward_distance = std::round(distances.min_forward_distance * 100) / 100;
            // 查找该聚类对应的雷达名称
            auto radar_it = cluster_radar_map.find(i);
            if (radar_it != cluster_radar_map.end()) {
                forward_min_distance_radar_source_ = radar_it->second;
                // 斜雷达提供的前向距离 减去10cm输出
                // if(forward_min_distance_radar_source_ == "left_tilt" || forward_min_distance_radar_source_ == "right_tilt") {
                //     if(cluster.max_z > 0.10) {
                //         distances.min_forward_distance = std::max(0.0, distances.min_forward_distance - 0.10);
                //     }
                // }
                // 计算最近点的Z值
                double nearest_z = 0.0;
                if (cluster.points && !cluster.points->points.empty()) {
                    double min_distance_sq = std::numeric_limits<double>::max();
                    for (const auto& point : cluster.points->points) {
                        double point_distance_sq = point.x * point.x + point.y * point.y;
                        if (point_distance_sq < min_distance_sq) {
                            min_distance_sq = point_distance_sq;
                            nearest_z = point.z;
                        }
                    }
                }
                forward_min_distance_nearest_z_ = nearest_z;
            }
        }

        if (backward_distance < distances.min_backward_distance) {
            // 使用cluster 中最小的x值来赋值
            distances.min_backward_distance = std::max(0.0, backward_distance);
            // 只取小数点后两位
            distances.min_backward_distance = std::round(distances.min_backward_distance * 100) / 100;
        }

        // 如果这个聚类提供了更小的左侧距离，记录雷达来源和最近点的Z值
        if (left_distance < distances.min_lateral_left_distance) {
            distances.min_lateral_left_distance = std::max(0.0, left_distance);
            distances.min_lateral_left_distance = round_distance_keep_subcentimeter(distances.min_lateral_left_distance);

            // 查找该聚类对应的雷达名称
            auto radar_it = cluster_radar_map.find(i);
            if (radar_it != cluster_radar_map.end()) {
                left_min_distance_radar_source_ = radar_it->second;
                // 计算最近点的Z值：对于左侧，找到y坐标最大的点（最靠近车辆左侧边界）
                double nearest_z = 0.0;
                if (cluster.points && !cluster.points->points.empty()) {
                    if (cluster.zero_z_point_count > 0) {
                        nearest_z = 0.0;
                    } else {
                        nearest_z = cluster.min_z;
                    }
                }
                left_min_distance_nearest_z_ = nearest_z;
            }
        }

        // 如果这个聚类提供了更小的右侧距离，记录雷达来源和最近点的Z值
        if (right_distance < distances.min_lateral_right_distance) {
            distances.min_lateral_right_distance = std::max(0.0, right_distance);
            distances.min_lateral_right_distance = round_distance_keep_subcentimeter(distances.min_lateral_right_distance);

            // 查找该聚类对应的雷达名称
            auto radar_it = cluster_radar_map.find(i);
            if (radar_it != cluster_radar_map.end()) {
                right_min_distance_radar_source_ = radar_it->second;

                // 计算最近点的Z值：对于右侧，找到y坐标最小的点（最靠近车辆右侧边界）
                double nearest_z = 0.0;
                if (cluster.points && !cluster.points->points.empty()) {
                    if (cluster.zero_z_point_count > 0) {
                        nearest_z = 0.0;
                    } else {
                        nearest_z = cluster.min_z;
                    }
                }
                right_min_distance_nearest_z_ = nearest_z;
            }
        }
    }

    // 如果前向的最小距离
    // RCLCPP_INFO(this->get_logger(), "[L%d] min_directional_distance: 前向=%.3f(来源:%s,Z:%.3f), 后向=%.3f, 左侧=%.3f(来源:%s,Z:%.3f), 右侧=%.3f(来源:%s,Z:%.3f)",
    //     __LINE__,
    //     distances.min_forward_distance, forward_min_distance_radar_source_.c_str(), forward_min_distance_nearest_z_,
    //     distances.min_backward_distance,
    //     distances.min_lateral_left_distance, left_min_distance_radar_source_.c_str(), left_min_distance_nearest_z_,
    //     distances.min_lateral_right_distance, right_min_distance_radar_source_.c_str(), right_min_distance_nearest_z_);
    min_directional_distance_ = distances;
}

void DecisionMaker::publishSafetyAlarm(const std::string& direction_name, WarningType warning_type)
{
    interface::msg::AuxiliaryAlarm alarm_msg;
    alarm_msg.header.stamp = this->now();
    alarm_msg.header.frame_id = "base_link";
    alarm_msg.alarm_level = static_cast<int32_t>(warning_type);
    auxiliary_alarm_publisher_->publish(alarm_msg);
}
void DecisionMaker::publishCollisionWarning(interface::msg::CollisionWarning& warning_msg)
{
    warning_msg.header.stamp = this->now();
    warning_msg.header.frame_id = "base_link";
    collision_warning_publisher_->publish(warning_msg);
}

bool DecisionMaker::shouldPublishPhysicalIntervention(const std::string& direction_name, WarningLevel level) const
{
    constexpr double kStoppedMps = 0.06;
    constexpr double kMinSpeedForDecel = 0.05;
    constexpr double kSpeedDropMps = 0.03;

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!speed_feedback_ready_) {
        return false;
    }

    const double v = tracking_ ? tracking_->getWheelSpeedLinearVelocity() : current_vehicle_state_.linear_velocity;
    const double w = tracking_ ? tracking_->getWheelSpeedAngularVelocity() : current_vehicle_state_.angular_velocity;
    const double v_prev = fb_linear_prev_;
    const double w_prev = fb_angular_prev_;
    const double half_w = robot_dimensions_.width * 0.5;

    const bool is_forward = (direction_name == "前向" || direction_name == "前侧");
    const bool is_backward = (direction_name == "后向" || direction_name == "后侧");
    const bool is_left = (direction_name == "左侧");
    const bool is_right = (direction_name == "右侧");

    auto physical_slowdown = [&](double now, double prev) -> bool {
        if (level == WarningLevel::BRAKE) {
            return (now <= kStoppedMps) || (prev > kMinSpeedForDecel && now < prev - kSpeedDropMps);
        }
        if (now <= kMinSpeedForDecel) {
            return false;
        }
        return prev > kMinSpeedForDecel && now < prev - kSpeedDropMps;
    };

    if (is_forward) {
        if (v <= 0.0) {
            return false;
        }
        return physical_slowdown(v, v_prev);
    }
    if (is_backward) {
        const double av = std::abs(v);
        const double av_prev = std::abs(v_prev);
        if (v >= -0.01) {
            return false;
        }
        return physical_slowdown(av, av_prev);
    }
    if (is_left || is_right) {
        const double eq = std::abs(w) * half_w;
        const double eq_prev = std::abs(w_prev) * half_w;
        if (eq <= 1e-6) {
            return false;
        }
        return physical_slowdown(eq, eq_prev);
    }
    return false;
}

void DecisionMaker::publishInterventionState(const std::string& direction_name, WarningLevel level, double effective_distance)
{
    // 同一方向同一级别 1 秒最多发送一次
    const double kMinPublishIntervalSec = 1.0;
    std::string key = direction_name + "_" + std::to_string(static_cast<int>(level));
    rclcpp::Time now = this->now();
    auto it = last_intervention_publish_time_.find(key);
    if (it != last_intervention_publish_time_.end()) {
        double elapsed = (now - it->second).seconds();
        if (elapsed < kMinPublishIntervalSec) {
            return;
        }
    }
    last_intervention_publish_time_[key] = now;

    // 方向名称 -> warning_type: 1 地上前向 2 地上左向 3 地上右向 4 地上后向
    int16_t warning_type = 0;
    if (direction_name == "前向" || direction_name == "前侧") {
        warning_type = static_cast<int16_t>(WarningType::FORWARD);
    } else if (direction_name == "后向" || direction_name == "后侧") {
        warning_type = static_cast<int16_t>(WarningType::BACKWARD);
    } else if (direction_name == "左侧") {
        warning_type = static_cast<int16_t>(WarningType::LEFT);
    } else if (direction_name == "右侧") {
        warning_type = static_cast<int16_t>(WarningType::RIGHT);
    } else {
        RCLCPP_DEBUG(this->get_logger(), "[L%d] publishInterventionState 未知方向: %s", __LINE__, direction_name.c_str());
        return;
    }
    if(direction_name == "前向") {
        std_msgs::msg::Empty narrow_end_msg;
        narrow_end_publisher_->publish(narrow_end_msg);
    }

    interface::msg::CollisionWarning msg;
    msg.header.stamp = now;
    msg.header.frame_id = "base_link";
    msg.warning_type = warning_type;
    msg.warning_level = static_cast<int16_t>(level);
    collision_warning_publisher_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 发布干预状态: 方向=%s, 等级=%d, 距离=%.3f m",
        __LINE__, direction_name.c_str(), static_cast<int>(level), effective_distance);
}

void DecisionMaker::publishSafetyAlarmForBackward(const std::string& direction_name, int alarm_level)
{
    interface::msg::AuxiliaryAlarm alarm_msg;
    alarm_msg.header.stamp = this->now();
    alarm_msg.header.frame_id = "base_link";
    alarm_msg.alarm_level = alarm_level;
    auxiliary_alarm_publisher_back_->publish(alarm_msg);
}

void DecisionMaker::carInfoCallback(const interface::msg::CarInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 档位上限：按 CarInfo.gear + mode_type（1 舒适 2 运动）与 scene_node 巡航限速表一致。
    // 运动 4 档未在产品需求中写明时取 8 km/h，与 absolute_max_linear_speed 对齐。
    // 注意：不再用 base_speed * ratio，避免与舒适/运动分档 km/h 不一致。
    auto kmh_to_ms = [](double kmh) { return kmh / 3.6; };
    const int gear = static_cast<int>(msg->gear);
    const bool sport = (msg->mode_type == 2);
    double gear_max_speed = kmh_to_ms(1.5);
    switch (gear) {
    case 1:
        gear_max_speed = sport ? kmh_to_ms(2.0) : kmh_to_ms(1.5);
        break;
    case 2:
        gear_max_speed = kmh_to_ms(3.0);
        break;
    case 3:
        gear_max_speed = sport ? kmh_to_ms(6.0) : kmh_to_ms(4.5);
        break;
    case 4:
        gear_max_speed = sport ? kmh_to_ms(8.0) : kmh_to_ms(6.0);
        break;
    default:
        gear_max_speed = sport ? kmh_to_ms(2.0) : kmh_to_ms(1.5);
        break;
    }

    decision_params_.max_speed_limit = std::min(gear_max_speed, decision_params_.absolute_max_linear_speed);
    decision_params_.max_speed_limit_backward = gear_max_speed * 0.5;

    // 同步重置各方向恢复状态，避免旧限速残留
    // 关键：target_speed_limit 必须无条件跟随新的 max_speed_limit，
    // 否则升档后若处于恢复态或有残留检测，target 仍停留在旧档位上限（如 1.67 m/s = 6 km/h），
    // 造成 4 档最高速被前一次 3 档限速卡住、并持续报减速。
    auto reset = [&](DirectionalState& s) {
        s.target_speed_limit = decision_params_.max_speed_limit;
        if (!s.has_last_detection && !s.is_recovering) {
            s.current_speed_limit = decision_params_.max_speed_limit;
        } else {
            // 若旧档位上限大于新档位，clamp 当前限速，避免瞬时超限
            s.current_speed_limit = std::min(s.current_speed_limit, decision_params_.max_speed_limit);
        }
    };
    reset(forward_state_);
    reset(backward_state_);
    reset(left_state_);
    reset(right_state_);
}

void DecisionMaker::updateRadarNearestPoint(const std::vector<ClusterInfo>& clusters, const std::string& radar_name)
{
    double half_wid = robot_dimensions_.width * 0.5; // 0.32
    double safety_side = robot_dimensions_.safety_margin_side;
    double forward_distance_min = std::numeric_limits<double>::max();
    double left_distance_min = std::numeric_limits<double>::max();
    double right_distance_min = std::numeric_limits<double>::max();
    const bool contributes_forward_main =
        (radar_name != "left_tilt" && radar_name != "right_tilt" && radar_name != "back");
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        double forward_distance = std::numeric_limits<double>::max();
        double left_distance = std::numeric_limits<double>::max();
        double right_distance = std::numeric_limits<double>::max();
        if (contributes_forward_main && std::abs(cluster.center_y) <= (half_wid + safety_side)) {
            if (cluster.center_x >= robot_dimensions_.length) {
                // 距离前缘的前向投影距离，>0 表示前方剩余距离
                forward_distance = (cluster.center_x - robot_dimensions_.length);
            } else if (cluster.center_x > robot_dimensions_.length - decision_params_.foot_distance_to_radar) {
                forward_distance = 0.0;
            }
        }
        if (cluster.center_x > 0.0 && cluster.center_x < robot_dimensions_.length + decision_params_.foot_distance_to_radar) {
            double y_abs = std::abs(cluster.center_y);
            if (y_abs > half_wid) {
                double y_protrude = y_abs - half_wid;
                if (y_protrude >= decision_params_.lateral_dominance_ratio * cluster.center_x) {
                    if (cluster.center_y > 0.0) {
                        left_distance = y_protrude;
                    } else {
                        right_distance = y_protrude;
                    }
                }
            }
        }
        forward_distance_min = std::min(forward_distance_min, forward_distance);
        left_distance_min = std::min(left_distance_min, left_distance);
        right_distance_min = std::min(right_distance_min, right_distance);
    }
    // 每帧完全替换该雷达的最近点信息，避免旧值累积
    RadarNearestPoint nearest_point;
    nearest_point.radar_name = radar_name;
    nearest_point.forward_distance = forward_distance_min;
    nearest_point.left_distance = left_distance_min;
    nearest_point.right_distance = right_distance_min;
    radar_nearest_point_map_[radar_name] = nearest_point;
    RCLCPP_DEBUG(this->get_logger(), "[L%d] radar_nearest_point: %s, forward_distance: %.3f",
        __LINE__, radar_name.c_str(), forward_distance_min);
    // RCLCPP_WARN(this->get_logger(), "[L%d] radar_nearest_point: %s, left_distance: %.3f",
    //     __LINE__, radar_name.c_str(), nearest_point.left_distance);
    // RCLCPP_WARN(this->get_logger(), "[L%d] radar_nearest_point: %s, right_distance: %.3f",
    //     __LINE__, radar_name.c_str(), nearest_point.right_distance);
}

void DecisionMaker::joystickCallback(const interface::msg::Joystick::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    joystick_msg_ = *msg;
}

void DecisionMaker::tftCarModeCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (!msg) {
        return;
    }
    const int32_t v = msg->data;
    tft_car_mode_.store(v == 2 ? 2 : 1, std::memory_order_relaxed);
}

void DecisionMaker::updateSmoothFilterCommandLimits()
{
    SmoothCommandLimits lim;
    const int32_t mode = tft_car_mode_.load(std::memory_order_relaxed);
    if (mode == 2) {
        lim.max_linear_delta = decision_params_.smooth_sport_max_linear_delta;
        lim.max_angular_delta = decision_params_.smooth_sport_max_angular_delta;
        lim.max_linear_accel_m_s2 = decision_params_.smooth_sport_max_linear_accel_m_s2;
    } else {
        lim.max_linear_delta = decision_params_.smooth_comfort_max_linear_delta;
        lim.max_angular_delta = decision_params_.smooth_comfort_max_angular_delta;
        lim.max_linear_accel_m_s2 = decision_params_.smooth_comfort_max_linear_accel_m_s2;
    }
    smooth_filter_->setCommandLimits(lim);
}

interface::msg::Joystick DecisionMaker::getCurrentJoystickMsg() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return joystick_msg_;
}

void DecisionMaker::judgeForwardWarningScene(const SpeedLimit& limit)
{
    bool is_negative_plane = false;
    // 区分是否为负平面
    if(is_forward_obstacle_negative_plane_ && forward_min_distance_nearest_z_ <= 0.01) {
        is_negative_plane = true;
    }else if((forward_min_distance_radar_source_ == "left" || forward_min_distance_radar_source_ == "right") 
           && (forward_min_distance_nearest_z_ <= 0.01)) {
        is_negative_plane = true;
    }else{
        is_negative_plane = false;
    }
    if(limit.max_linear_speed < 0.01){
        if (is_negative_plane) {
            int16_t old_level = negative_plane_forward_warning_msg_.warning_level;
            negative_plane_forward_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::BRAKE);
            updateWarningStatus(negative_plane_forward_warning_msg_, old_level);
        } else {
            int16_t old_level = forward_warning_msg_.warning_level;
            forward_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::BRAKE);
            updateWarningStatus(forward_warning_msg_, old_level);
        }
        is_emergency_brake_zone_speed_suppressed_ = true;
        is_deceleration_zone_speed_suppressed_ = false;
    }else{
        if (is_negative_plane) {
            int16_t old_level = negative_plane_forward_warning_msg_.warning_level;
            negative_plane_forward_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::DECELERATION);
            updateWarningStatus(negative_plane_forward_warning_msg_, old_level);
        } else {
            int16_t old_level = forward_warning_msg_.warning_level;
            forward_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::DECELERATION);
            updateWarningStatus(forward_warning_msg_, old_level);
        }
        is_deceleration_zone_speed_suppressed_ = true;
        is_emergency_brake_zone_speed_suppressed_ = false;
    }
}

void DecisionMaker::judgeBackwardWarningScene(const SpeedLimit& limit)
{
    if (limit.max_linear_speed < 0.01) {
        int16_t old_level = backward_warning_msg_.warning_level;
        backward_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::BRAKE);
        updateWarningStatus(backward_warning_msg_, old_level);
        is_emergency_brake_zone_speed_suppressed_ = true;
        is_deceleration_zone_speed_suppressed_ = false;
    } else {
        int16_t old_level = backward_warning_msg_.warning_level;
        backward_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::DECELERATION);
        updateWarningStatus(backward_warning_msg_, old_level);
        is_deceleration_zone_speed_suppressed_ = true;
        is_emergency_brake_zone_speed_suppressed_ = false;
    }
}

void DecisionMaker::judgeLeftLateralWarningScene(const SpeedLimit& limit)
{
    if (limit.max_angular_speed < 0.01) {
        if(is_lateral_obstacle_negative_plane_ && left_min_distance_nearest_z_ <= 0.01){
            int16_t old_level = negative_plane_left_warning_msg_.warning_level;
            negative_plane_left_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::BRAKE);
            updateWarningStatus(negative_plane_left_warning_msg_, old_level);
        }else{
            int16_t old_level = left_warning_msg_.warning_level;
            left_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::BRAKE);
            updateWarningStatus(left_warning_msg_, old_level);
        }
        is_emergency_brake_zone_speed_suppressed_ = true;
        is_deceleration_zone_speed_suppressed_ = false;
    }else {
        if(is_lateral_obstacle_negative_plane_ && left_min_distance_nearest_z_ <= 0.01){
            int16_t old_level = negative_plane_left_warning_msg_.warning_level;
            negative_plane_left_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::DECELERATION);
            updateWarningStatus(negative_plane_left_warning_msg_, old_level);
        }else{
            int16_t old_level = left_warning_msg_.warning_level;
            left_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::DECELERATION);
            updateWarningStatus(left_warning_msg_, old_level);
        }
        is_deceleration_zone_speed_suppressed_ = true;
        is_emergency_brake_zone_speed_suppressed_ = false;
    }
}

void DecisionMaker::judgeRightLateralWarningScene(const SpeedLimit& limit)
{
    if (limit.max_angular_speed < 0.01) {
        if(is_lateral_obstacle_negative_plane_ && right_min_distance_nearest_z_ <= 0.01){
            int16_t old_level = negative_plane_right_warning_msg_.warning_level;
            negative_plane_right_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::BRAKE);
            updateWarningStatus(negative_plane_right_warning_msg_, old_level);
        }else{
            int16_t old_level = right_warning_msg_.warning_level;
            right_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::BRAKE);
            updateWarningStatus(right_warning_msg_, old_level);
        }
        is_emergency_brake_zone_speed_suppressed_ = true;
        is_deceleration_zone_speed_suppressed_ = false;
    }else {
        if(is_lateral_obstacle_negative_plane_ && right_min_distance_nearest_z_ <= 0.01){
            int16_t old_level = negative_plane_right_warning_msg_.warning_level;
            negative_plane_right_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::DECELERATION);
            updateWarningStatus(negative_plane_right_warning_msg_, old_level);
        }else{
            int16_t old_level = right_warning_msg_.warning_level;
            right_warning_msg_.warning_level = static_cast<int16_t>(WarningLevel::DECELERATION);
            updateWarningStatus(right_warning_msg_, old_level);
        }
        is_deceleration_zone_speed_suppressed_ = true;
        is_emergency_brake_zone_speed_suppressed_ = false;
    }
}

void DecisionMaker::updateWarningStatus(interface::msg::CollisionWarning& warning_msg, int16_t previous_level)
{
    // warning_msg.warning_level 已由调用方设置为新等级
    int16_t new_level = warning_msg.warning_level;

    // 如果之前有预警（减速或刹停），且等级发生变化，发布驶离预警
    if (previous_level != new_level) {
        if (previous_level == static_cast<int16_t>(WarningLevel::DECELERATION)) {
            warning_msg.warning_level = static_cast<int16_t>(WarningLevel::LEAVE_DECELERATION_ZONE);
            publishCollisionWarning(warning_msg);
        } else if (previous_level == static_cast<int16_t>(WarningLevel::BRAKE)) {
            warning_msg.warning_level = static_cast<int16_t>(WarningLevel::LEAVE_BRAKE_ZONE);
            publishCollisionWarning(warning_msg);
        }
    }

    // 发布当前预警等级
    warning_msg.warning_level = new_level;
    publishCollisionWarning(warning_msg);
}
