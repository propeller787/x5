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
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

// ==================== 构造函数和初始化 ====================

DecisionMaker::DecisionMaker()
    : rclcpp::Node("decision_maker")
{
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 初始化决策器", __LINE__);

    // 初始化参数
    last_nearest_distance_ = decision_params_.warning_zone;
    has_handle_control_ = false;

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

    // 初始化机器人主体尺寸与碰撞区域（默认来源于URDF碰撞几何体尺寸与合理区域）
    robot_dimensions_ = RobotDimensions::getBaseDimensions();
    collision_zones_ = RobotDimensions::getCollisionZones();

    // 初始化平滑滤波器
    smooth_filter_ = std::make_unique<SmoothFilter>();
    smooth_filter_->setFilterType(FilterType::RATE_LIMITED);
    smooth_filter_->setAlpha(0.3); // 指数平滑系数

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

    // 初始化档位订阅器
    car_info_subscription_ = this->create_subscription<interface::msg::CarInfo>(
        carInfoTopic, 1, std::bind(&DecisionMaker::carInfoCallback, this, std::placeholders::_1));

    RCLCPP_DEBUG(this->get_logger(), "[L%d] 报警发布器初始化完成", __LINE__);

    RCLCPP_DEBUG(this->get_logger(), "[L%d] 决策器初始化完成 - 预警区:%.1fm, 减速区:%.1fm, 制动区:%.1fm", __LINE__, decision_params_.warning_zone,
        decision_params_.deceleration_zone, decision_params_.emergency_brake_distance);
}

DecisionMaker::~DecisionMaker() { }

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

    applyDirectionalInterventionInternal(
        min_distance,
        decision_params_.warning_zone,
        decision_params_.emergency_brake_distance,
        forward_state_,
        [this](double dist) { return this->calculateSpeedLimit(dist); },
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

    applyDirectionalInterventionInternal(
        min_distance,
        decision_params_.warning_zone_backward,
        decision_params_.emergency_brake_distance_backward,
        backward_state_,
        [this](double dist) { return this->calculateSpeedLimitForBackward(dist); },
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

    applyDirectionalInterventionInternal(
        min_distance,
        decision_params_.warning_zone_lateral,
        decision_params_.emergency_brake_distance_lateral,
        left_state_,
        [this](double dist) { return this->calculateSpeedLimitForLateral(dist); },
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

    applyDirectionalInterventionInternal(
        min_distance,
        decision_params_.warning_zone_lateral,
        decision_params_.emergency_brake_distance_lateral,
        right_state_,
        [this](double dist) { return this->calculateSpeedLimitForLateral(dist); },
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

        // 线性插值恢复速度
        state.current_speed_limit = state.current_speed_limit + (state.target_speed_limit - state.current_speed_limit) * recovery_ratio;

        final_speed_limit = state.current_speed_limit;

        RCLCPP_DEBUG(this->get_logger(), "[L%d][%s-速度恢复] 恢复进度 %.1f%%, 当前限速 %.3f m/s (%.1f km/h)",
            __LINE__, direction_name.c_str(), recovery_ratio * 100.0, state.current_speed_limit, state.current_speed_limit * 3.6);

        // 恢复完成
        if (recovery_ratio >= 1.0) {
            state.is_recovering = false;
            state.current_speed_limit = max_speed_limit;
            RCLCPP_DEBUG(this->get_logger(), "[L%d][%s-速度恢复] 恢复完成，当前限速 %.3f m/s (%.1f km/h)",
                __LINE__, direction_name.c_str(), state.current_speed_limit, state.current_speed_limit * 3.6);
        }
    } else if (should_start_recovery) {
        // 如果需要启动恢复，检查条件并启动
        if (state.current_speed_limit < max_speed_limit) {
            state.is_recovering = true;
            state.recovery_start_time = current_time;
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
    DirectionalState& state,
    std::function<SpeedLimit(double)> calc_limit_func,
    std::function<geometry_msgs::msg::Twist(const geometry_msgs::msg::Twist&, const SpeedLimit&)> apply_limit_func,
    const std::string& direction_name,
    geometry_msgs::msg::Twist& output_control,
    bool use_predicted)
{
    auto current_time = this->now();

    // 步骤1：应用状态保持逻辑，得到有效距离
    double effective_distance = applyStateHoldLogic(min_distance, warning_zone, state, direction_name, current_time);
    RCLCPP_DEBUG(this->get_logger(), "[L%d] [%s] 应用状态保持逻辑，得到有效距离 %.3f m", __LINE__, direction_name.c_str(), effective_distance);
    // 步骤2：根据有效距离计算速度限制
    double final_speed_limit = decision_params_.max_speed_limit;
    bool should_start_recovery = false;

    if (effective_distance < warning_zone) {
        // 有效距离在预警区内，需要干预
        SpeedLimit speed_limit = calc_limit_func(effective_distance);
        RCLCPP_DEBUG(this->get_logger(), "[L%d] [%s] 计算速度限制，得到限速 线速度: %.3f m/s, 角速度: %.3f rad/s",
            __LINE__, direction_name.c_str(), speed_limit.max_linear_speed, speed_limit.max_angular_speed);
        // 更新当前速度限制
        // 对于侧向碰撞，max_linear_speed 总是 max()，但 max_angular_speed 可能有限制
        // 因此需要同时检查线速度和角速度是否有限制
        if (speed_limit.max_linear_speed < std::numeric_limits<double>::max() || speed_limit.max_angular_speed < std::numeric_limits<double>::max()) {
            RCLCPP_DEBUG(this->get_logger(), "[L%d] [%s] 应用速度限制 - 线速度: %.3f m/s, 角速度: %.3f rad/s",
                __LINE__, direction_name.c_str(), state.current_speed_limit, state.current_speed_limit);
            output_control = apply_limit_func(output_control, speed_limit);

            // 输出干预信息
            if (effective_distance <= emergency_brake_distance) {
                RCLCPP_WARN(this->get_logger(), "[L%d]%s[%s-制动区] 紧急制动！距离 %.3f m",
                    __LINE__, use_predicted ? "[预测]" : "[实时]", direction_name.c_str(), effective_distance);
            } else {
                RCLCPP_DEBUG(this->get_logger(), "[L%d]%s[%s-减速区] 平滑减速，距离 %.3f m",
                    __LINE__, use_predicted ? "[预测]" : "[实时]", direction_name.c_str(), effective_distance);
            }
        }
    } else {
        // 预警区或正常区，不限制速度，可能需要启动恢复
        state.current_speed_limit = decision_params_.max_speed_limit;
        should_start_recovery = !state.has_last_detection; // 如果障碍物已消失，可能需要恢复
        RCLCPP_DEBUG(this->get_logger(), "[L%d]%s[%s-正常区] 无需干预，距离 %.3f m",
            __LINE__, use_predicted ? "[预测]" : "[实时]", direction_name.c_str(), effective_distance);
    }

    // 步骤3：应用速度恢复逻辑（包括启动恢复和恢复过程）
    final_speed_limit = applySpeedRecoveryLogic(
        state, decision_params_.max_speed_limit, decision_params_.speed_recovery_time,
        direction_name, current_time, should_start_recovery);

    // 步骤4：如果正在恢复中，需要应用恢复的速度限制
    if (state.is_recovering) {
        SpeedLimit recovery_limit;
        recovery_limit.max_linear_speed = final_speed_limit;
        recovery_limit.max_angular_speed = final_speed_limit * 0.5;
        recovery_limit.distance_threshold = 0.0;
        output_control = apply_limit_func(output_control, recovery_limit);
    }
}

// ==================== 摇杆控制处理主函数 ====================

void DecisionMaker::processHandleControl(const geometry_msgs::msg::Twist& handle_control)
{
    // 步骤1：保存摇杆控制命令
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_handle_control_ = handle_control;
        has_handle_control_ = true;
    }

    // 步骤2：检查回调函数
    if (!control_output_callback_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[L%d] 控制输出回调函数未设置，无法输出控制命令", __LINE__);
        return;
    }

    // 步骤4：收集前向相关的所有雷达数据
    auto current_time = this->now();
    double min_forward_distance = decision_params_.warning_zone;
    bool use_predicted = false;
    bool has_valid_data = false;

    std::vector<ClusterInfo> merged_forward_clusters;
    std::map<size_t, std::string> cluster_radar_map; // 聚类索引到雷达名称的映射

    // 锁定状态以读取雷达数据
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // 收集所有雷达的前向数据（主要关注 left, right, left_tilt, right_tilt 雷达）
        for (const auto& radar_pair : radar_cluster_map_) {
            const std::string& radar_name = radar_pair.first;
            const std::vector<ClusterInfo>& radar_clusters = radar_pair.second;

            // 检查该雷达是否与前向检测相关
            // 前向碰撞主要关注：left, right, left_tilt, right_tilt
            bool is_forward_relevant = (radar_name == "left" || radar_name == "right" || radar_name == "left_tilt" || radar_name == "right_tilt" || radar_name == "back");

            if (!is_forward_relevant) {
                RCLCPP_DEBUG(this->get_logger(), "[L%d] 雷达 %s 不参与前向碰撞检测", __LINE__, radar_name.c_str());
                continue;
            }

            // 时效性判断
            double t_diff = decision_params_.radar_timeout + 1.0; // 默认超时
            auto t_it = radar_last_update_time_map_.find(radar_name);
            if (t_it != radar_last_update_time_map_.end()) {
                t_diff = (current_time - t_it->second).seconds();
            }

            // 数据有效性和超时补偿逻辑
            std::vector<ClusterInfo> valid_clusters;

            if (t_diff <= decision_params_.radar_timeout) {
                // 数据新鲜，直接使用
                valid_clusters = radar_clusters;
                has_valid_data = true;
                RCLCPP_DEBUG(this->get_logger(), "[L%d] 雷达 %s 数据新鲜 (%.3f s)，聚类数: %zu", __LINE__, radar_name.c_str(), t_diff, radar_clusters.size());
            } else if (t_diff <= decision_params_.radar_timeout * 3.0 && !radar_clusters.empty()) {
                // 数据轻微超时（不超过3倍超时阈值），基于车辆运动进行预测补偿
                valid_clusters = radar_clusters; // predictTargetsBasedOnWheelSpeed(radar_clusters, t_diff, current_vehicle_state_);
                use_predicted = true;
                has_valid_data = true;
                RCLCPP_DEBUG(this->get_logger(), "[L%d] 雷达 %s 数据超时 (%.3f s)，使用预测补偿，预测聚类数: %zu", __LINE__, radar_name.c_str(), t_diff, valid_clusters.size());
            } else {
                // 数据严重超时或为空，跳过该雷达
                RCLCPP_DEBUG(this->get_logger(), "[L%d] 雷达 %s 数据严重超时 (%.3f s) 或为空，跳过", __LINE__, radar_name.c_str(), t_diff);
                continue;
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
        MinDirectionalDistance distances { decision_params_.warning_zone,
            decision_params_.warning_zone_backward,
            decision_params_.warning_zone_lateral,
            decision_params_.warning_zone_lateral };
        min_directional_distance_ = distances;
        forward_min_distance_radar_source_.clear();
        forward_min_distance_nearest_z_ = 0.0;
        left_min_distance_radar_source_.clear();
        left_min_distance_nearest_z_ = 0.0;
        right_min_distance_radar_source_.clear();
        right_min_distance_nearest_z_ = 0.0;
    }

    // 步骤6：平滑速度干预并输出
    // 为了保证平稳性，即使没有障碍物也要经过平滑滤波
    // 支持多方向同时干预（例如：前进+左转）
    geometry_msgs::msg::Twist output_control = current_handle_control_;

    // 根据控制命令方向，同时应用多个方向的干预（使用独立的if语句，而非if-else链）
    if (current_handle_control_.linear.x > 0.0) {
        applySpeedForwardIntervention(output_control);
    }
    if (current_handle_control_.linear.x < 0.0) {
        applySpeedBackwardIntervention(output_control);
    }
    if (is_backward_distance_nearest_ && std::abs(joystick_msg_.rocker_angle) > 140.0 && joystick_msg_.rocker_x < -50) {

    } else {
        if (current_handle_control_.angular.z > 0.0) {
            applySpeedLeftIntervention(output_control);
        }
        if (current_handle_control_.angular.z < 0.0) {
            applySpeedRightIntervention(output_control);
        }
    }

    // if (current_handle_control_.angular.z > 0.0) {
    //    applySpeedLeftIntervention(output_control);
    // }
    // if (current_handle_control_.angular.z < 0.0) {
    //    applySpeedRightIntervention(output_control);
    // }

    // 更新当前控制命令
    current_handle_control_ = output_control;
    // 应用平滑滤波（保证行驶平稳性）
    geometry_msgs::msg::Twist filtered_control = smooth_filter_->filter(current_handle_control_);
    // geometry_msgs::msg::Twist filtered_control = current_handle_control_;
    //  最终输出前的安全检查（使用决策参数中的配置）
    if (std::abs(filtered_control.linear.x) > decision_params_.absolute_max_linear_speed) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[L%d] 左侧-线速度超限！%.3f m/s -> %.3f m/s", __LINE__, filtered_control.linear.x,
            std::copysign(decision_params_.absolute_max_linear_speed, filtered_control.linear.x));
        filtered_control.linear.x = std::copysign(decision_params_.absolute_max_linear_speed, filtered_control.linear.x);
    }

    if (std::abs(filtered_control.angular.z) > decision_params_.absolute_max_angular_speed) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[L%d] 左侧-角速度超限！%.3f rad/s -> %.3f rad/s", __LINE__, filtered_control.angular.z,
            std::copysign(decision_params_.absolute_max_angular_speed, filtered_control.angular.z));
        filtered_control.angular.z = std::copysign(decision_params_.absolute_max_angular_speed, filtered_control.angular.z);
    }

    // 输出控制命令
    if (control_output_callback_) {
        control_output_callback_(filtered_control);
    }
}

void DecisionMaker::setTracking(std::shared_ptr<Tracking> tracking)
{
    tracking_ = tracking;
    RCLCPP_DEBUG(this->get_logger(), "[L%d] 设置跟踪器引用", __LINE__);
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

// ==================== 参数配置接口 ====================

void DecisionMaker::setDecisionParams(const DecisionParams& params)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    decision_params_ = params;
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

SpeedLimit DecisionMaker::calculateSpeedLimit(double distance)
{
    SpeedLimit limit;
    limit.distance_threshold = distance;
    bool need_kalman_deceleration = false;
    // 制动区：距离小于等于0.3m，紧急制动
    // 前向避障只限制线速度，角速度使用原车角速度（不限制）
    if (distance <= decision_params_.emergency_brake_distance) {
        limit.max_linear_speed = 0.0;
        limit.max_angular_speed = 0.0;
        RCLCPP_WARN(this->get_logger(), "[L%d][制动区] 紧急制动！距离: %.3f m <= %.3f m", __LINE__, distance, decision_params_.emergency_brake_distance);
        return limit;
    }
    // 减速区：0.3m到0.8m之间进行平滑减速
    if (distance <= decision_params_.deceleration_zone) {
        // 计算归一化比例 [0, 1]
        // 0.3m -> 0, 0.8m -> 1
        double denominator = decision_params_.deceleration_zone - decision_params_.emergency_brake_distance;

        // 防止除零或分母过小导致的数值溢出
        if (denominator <= 1e-6) {
            RCLCPP_WARN(this->get_logger(), "[L%d][前向-减速区] 参数配置错误：deceleration_zone (%.3f) 与 emergency_brake_distance (%.3f) 差值过小，使用制动区逻辑",
                __LINE__, decision_params_.deceleration_zone, decision_params_.emergency_brake_distance);
            limit.max_linear_speed = 0.0;
            limit.max_angular_speed = std::numeric_limits<double>::max(); // 不限制角速度，使用原车角速度
            return limit;
        }

        double ratio = (distance - decision_params_.emergency_brake_distance) / denominator;

        // 确保ratio在[0,1]范围内
        ratio = std::max(0.0, std::min(1.0, ratio));
        // 使用平方根函数进行平滑插值，使得低速区域变化更缓慢
        double smooth_ratio = std::sqrt(ratio);
        // 使用立方根函数
        // double smooth_ratio = std::cbrt(ratio);
        // 线性插值：从最小速度到最大速度
        // 0.3m处最大速度为0.056m/s (0.2km/h)
        // 0.8m处最大速度为1.67m/s (6km/h)
        limit.max_linear_speed = decision_params_.min_speed_limit + smooth_ratio * (decision_params_.max_speed_limit - decision_params_.min_speed_limit);
        limit.max_angular_speed = std::numeric_limits<double>::max();
        RCLCPP_DEBUG(this->get_logger(), "[L%d][减速区] 距离 %.3f m, 比例 %.3f, 平滑比例 %.3f, 限速: %.3f m/s (%.1f km/h), 角速度不限制（使用原车角速度）", __LINE__, distance, ratio,
            smooth_ratio, limit.max_linear_speed, limit.max_linear_speed * 3.6);
    }
    // 预警区：0.8m到1.5m之间不限制速度，但进入状态保持
    else if (distance <= decision_params_.warning_zone) {
        // 预警区内不限制速度，但记录状态以便状态保持
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();

        RCLCPP_DEBUG(this->get_logger(), "[L%d][预警区] 距离 %.3f m, 不限速", __LINE__, distance);
    }
    // 正常区：距离大于1.5m，完全不限制
    else {
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();

        RCLCPP_DEBUG(this->get_logger(), "[L%d][正常区] 距离 %.3f m > %.3f m，不限速", __LINE__, distance, decision_params_.warning_zone);
    }
    return limit;
}

geometry_msgs::msg::Twist DecisionMaker::applySpeedLimit(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit)
{
    geometry_msgs::msg::Twist limited_control = original_control;
    bool need_kalman_deceleration = false;
    // ==================== 线速度限制 ====================
    // 只限制前进方向的速度（x轴正方向），后退不受前向碰撞限制
    if (original_control.linear.x > 0.0) {
        // 前进方向需要限制
        if (original_control.linear.x > limit.max_linear_speed) {
            need_kalman_deceleration = true;
            limited_control.linear.x = limit.max_linear_speed;
            judgeForwardWarningScene(limit);
            RCLCPP_DEBUG(this->get_logger(), "[L%d] 前进速度限制: %.3f m/s -> %.3f m/s", __LINE__, original_control.linear.x, limited_control.linear.x);
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
    return limited_control;
}

// ==================== 侧向速度限制计算和应用 ====================

SpeedLimit DecisionMaker::calculateSpeedLimitForBackward(double distance)
{
    SpeedLimit limit;
    limit.distance_threshold = distance;
    is_backward_distance_nearest_ = false;
    // 根据距离发布安全报警
    if (distance >= decision_params_.emergency_brake_distance_backward
        && distance <= decision_params_.warning_distance_backward) {
        publishSafetyAlarmForBackward("后向", 2);
    }
    // 制动区：距离小于等于后向制动距离，紧急制动
    if (distance <= decision_params_.emergency_brake_distance_backward) {
        limit.max_linear_speed = 0.0;
        limit.max_angular_speed = 0.0;
        is_backward_distance_nearest_ = true;
        RCLCPP_WARN(this->get_logger(), "[L%d][后向-制动区] 紧急制动！距离: %.3f m <= %.3f m", __LINE__, distance, decision_params_.emergency_brake_distance_backward);
        return limit;
    }

    // 减速区：在后向减速区之间进行平滑减速
    if (distance <= decision_params_.deceleration_zone_backward) {
        // 计算归一化比例 [0, 1]
        double denominator = decision_params_.deceleration_zone_backward - decision_params_.emergency_brake_distance_backward;

        // 防止除零或分母过小导致的数值溢出
        if (denominator <= 1e-6) {
            RCLCPP_WARN(this->get_logger(), "[L%d][后向-减速区] 参数配置错误：deceleration_zone_backward (%.3f) 与 emergency_brake_distance_backward (%.3f) 差值过小，使用制动区逻辑",
                __LINE__, decision_params_.deceleration_zone_backward, decision_params_.emergency_brake_distance_backward);
            limit.max_linear_speed = 0.0;
            limit.max_angular_speed = 0.0;
            return limit;
        }

        double ratio = (distance - decision_params_.emergency_brake_distance_backward) / denominator;

        // 确保ratio在[0,1]范围内
        ratio = std::max(0.0, std::min(1.0, ratio));
        double t = ratio;
        double smooth_ratio = (1.0 - t) * (1.0 - t) * (1.0 - t) * 0.0 + 3.0 * (1.0 - t) * (1.0 - t) * t * 0.2 + 3.0 * (1.0 - t) * t * t * 0.8 + t * t * t * 1.0;
        // 使用平方根函数进行平滑插值
        // double smooth_ratio = std::sqrt(ratio);
        // 使用立方根函数进行平滑插值
        // double smooth_ratio = std::cbrt(ratio);
        // 5次 Smoothstep
        // double smooth_ratio =  ratio * ratio * (ratio * (ratio * 6.0 - 15.0) + 10.0);
        // 线性插值：从最小速度到最大速度
        limit.max_linear_speed = decision_params_.min_speed_limit + smooth_ratio * (decision_params_.max_speed_limit - decision_params_.min_speed_limit);

        // 角速度限制：基于线速度限制计算
        // 使用平方根函数进行平滑插值
        double angular_ratio = std::sqrt(smooth_ratio);
        limit.max_angular_speed = limit.max_linear_speed * (0.2 + 0.3 * angular_ratio);

        // 确保最小角速度限制
        const double MIN_ANGULAR_SPEED = 0.05; // rad/s
        limit.max_angular_speed = std::max(limit.max_angular_speed, MIN_ANGULAR_SPEED);

        RCLCPP_DEBUG(this->get_logger(), "[L%d][后向-减速区] 距离 %.3f m, 比例 %.3f, 平滑比例 %.3f, 限速: %.3f m/s (%.1f km/h), 角速度: %.3f rad/s",
            __LINE__, distance, ratio, smooth_ratio, limit.max_linear_speed, limit.max_linear_speed * 3.6, limit.max_angular_speed);
    }
    // 预警区：不限制速度，但进入状态保持
    else if (distance <= decision_params_.warning_zone_backward) {
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();

        RCLCPP_DEBUG(this->get_logger(), "[L%d][后向-预警区] 距离 %.3f m, 不限速", __LINE__, distance);
    }
    // 正常区：距离大于预警区，完全不限制
    else {
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();

        RCLCPP_DEBUG(this->get_logger(), "[L%d][后向-正常区] 距离 %.3f m > %.3f m，不限速", __LINE__, distance, decision_params_.warning_zone_backward);
    }

    return limit;
}

SpeedLimit DecisionMaker::calculateSpeedLimitForLateral(double distance)
{
    SpeedLimit limit;
    limit.distance_threshold = distance;

    // 制动区：距离小于等于侧向制动距离，紧急制动
    if (distance <= decision_params_.emergency_brake_distance_lateral) {
        // limit.max_linear_speed = std::numeric_limits<double>::max(); // 侧向碰撞不影响线速度
        limit.max_linear_speed = 0.0;
        limit.max_angular_speed = 0.0; // 但限制角速度
        RCLCPP_WARN(this->get_logger(), "[L%d][侧向-制动区] 紧急制动！距离: %.3f m <= %.3f m", __LINE__, distance, decision_params_.emergency_brake_distance_lateral);
        return limit;
    }

    // 减速区：在侧向减速区之间进行平滑减速
    if (distance <= decision_params_.deceleration_zone_lateral) {
        // 计算归一化比例 [0, 1]
        double denominator = decision_params_.deceleration_zone_lateral - decision_params_.emergency_brake_distance_lateral;

        // 防止除零或分母过小导致的数值溢出
        if (denominator <= 1e-6) {
            RCLCPP_WARN(this->get_logger(), "[L%d][侧向-减速区] 参数配置错误：deceleration_zone_lateral (%.3f) 与 emergency_brake_distance_lateral (%.3f) 差值过小，使用制动区逻辑",
                __LINE__, decision_params_.deceleration_zone_lateral, decision_params_.emergency_brake_distance_lateral);
            limit.max_linear_speed = std::numeric_limits<double>::max();
            limit.max_angular_speed = 0.0;
            return limit;
        }

        double ratio = (distance - decision_params_.emergency_brake_distance_lateral) / denominator;

        // 确保ratio在[0,1]范围内
        ratio = std::max(0.0, std::min(1.0, ratio));

        // 使用平方根函数进行平滑插值
        double smooth_ratio = std::sqrt(ratio);

        // 侧向碰撞主要限制角速度，线速度不受影响
        limit.max_linear_speed = std::numeric_limits<double>::max();

        // 角速度限制：基于最大角速度限制计算
        double max_angular = decision_params_.absolute_max_angular_speed;
        limit.max_angular_speed = max_angular * smooth_ratio;

        // 确保最小角速度限制
        const double MIN_ANGULAR_SPEED = 0.05; // rad/s
        limit.max_angular_speed = std::max(limit.max_angular_speed, MIN_ANGULAR_SPEED);
        limit.max_angular_speed = std::max(limit.max_angular_speed, MIN_ANGULAR_SPEED);

        RCLCPP_DEBUG(this->get_logger(), "[L%d][侧向-减速区] 距离 %.3f m, 比例 %.3f, 平滑比例 %.3f, 角速度限速: %.3f rad/s",
            __LINE__, distance, ratio, smooth_ratio, limit.max_angular_speed);
    }
    // 预警区：不限制速度，但进入状态保持
    else if (distance <= decision_params_.warning_zone_lateral) {
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();

        RCLCPP_DEBUG(this->get_logger(), "[L%d][侧向-预警区] 距离 %.3f m, 不限速", __LINE__, distance);
    }
    // 正常区：距离大于预警区，完全不限制
    else {
        limit.max_linear_speed = std::numeric_limits<double>::max();
        limit.max_angular_speed = std::numeric_limits<double>::max();

        RCLCPP_DEBUG(this->get_logger(), "[L%d][侧向-正常区] 距离 %.3f m > %.3f m，不限速", __LINE__, distance, decision_params_.warning_zone_lateral);
    }

    return limit;
}

geometry_msgs::msg::Twist DecisionMaker::applySpeedLimitForBackward(const geometry_msgs::msg::Twist& original_control, const SpeedLimit& limit)
{
    geometry_msgs::msg::Twist limited_control = original_control;

    // ==================== 线速度限制 ====================
    // 只限制后退方向的速度（x轴负方向），前进不受后向碰撞限制
    if (original_control.linear.x < 0.0) {
        // 后退方向需要限制
        if (std::abs(original_control.linear.x) > limit.max_linear_speed) {
            limited_control.linear.x = -limit.max_linear_speed; // 保持负方向
            judgeBackwardWarningScene(limit);
            RCLCPP_DEBUG(this->get_logger(), "[L%d] 后退速度限制: %.3f m/s -> %.3f m/s", __LINE__, original_control.linear.x, limited_control.linear.x);
        }
    } else {
        // 前进方向保持不变（后向碰撞不影响前进）
        limited_control.linear.x = original_control.linear.x;
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
    // 侧向碰撞不影响线速度
    limited_control.linear.x = original_control.linear.x;

    // ==================== 角速度限制 ====================
    // 左侧碰撞主要限制左转（正角速度）
    if (original_control.angular.z > 0.0) {
        // 左转方向需要限制
        if (original_control.angular.z > limit.max_angular_speed) {
            limited_control.angular.z = limit.max_angular_speed;
            judgeLateralWarningScene(limit);
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
    // 侧向碰撞不影响线速度
    limited_control.linear.x = original_control.linear.x;

    // ==================== 角速度限制 ====================
    // 右侧碰撞主要限制右转（负角速度）
    if (original_control.angular.z < 0.0) {
        // 右转方向需要限制
        if (std::abs(original_control.angular.z) > limit.max_angular_speed) {
            limited_control.angular.z = -limit.max_angular_speed; // 保持负方向
            judgeLateralWarningScene(limit);
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
void DecisionMaker::setRobotDimensions(const RobotDimensions::BaseDimensions& dimensions)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    robot_dimensions_ = dimensions;
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
    MinDirectionalDistance distances { decision_params_.warning_zone,
        decision_params_.warning_zone_backward,
        decision_params_.warning_zone_lateral,
        decision_params_.warning_zone_lateral };

    double half_wid = robot_dimensions_.width * 0.5; // 0.32
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
        if (std::abs(cluster.center_y) <= (half_wid + safety_side)) {
            if (cluster.center_x >= robot_dimensions_.length) {
                // 距离前缘的前向投影距离，>0 表示前方剩余距离
                forward_distance = (cluster.center_x - robot_dimensions_.length);
            } else if (cluster.center_x > robot_dimensions_.length - decision_params_.foot_distance_to_radar) {
                forward_distance = 0.0;
            }
        }
        if (cluster.center_x > 0.0 && cluster.center_x < robot_dimensions_.length + decision_params_.foot_distance_to_radar) {
            if (cluster.center_y > half_wid) {
                left_distance = std::abs(cluster.center_y) - half_wid;
            }
            if (cluster.center_y < -half_wid) {
                right_distance = std::abs(cluster.center_y) - half_wid;
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
    RCLCPP_INFO(this->get_logger(), "[L%d] min_directional_distance: 前向=%.3f(来源:%s,Z:%.3f), 后向=%.3f, 左侧=%.3f(来源:%s,Z:%.3f), 右侧=%.3f(来源:%s,Z:%.3f)",
        __LINE__,
        distances.min_forward_distance, forward_min_distance_radar_source_.c_str(), forward_min_distance_nearest_z_,
        distances.min_backward_distance,
        distances.min_lateral_left_distance, left_min_distance_radar_source_.c_str(), left_min_distance_nearest_z_,
        distances.min_lateral_right_distance, right_min_distance_radar_source_.c_str(), right_min_distance_nearest_z_);
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
    switch (msg->gear) {
    case 1:
        decision_params_.max_speed_limit = 1.67 * 0.4;
        decision_params_.max_speed_limit_backward = 1.67 * 0.5 * 0.4;
        decision_params_.emergency_brake_distance = 0.1;
        break;
    case 2:
        decision_params_.max_speed_limit = 1.67 * 0.7;
        decision_params_.max_speed_limit_backward = 1.67 * 0.5 * 0.7;
        decision_params_.emergency_brake_distance = 0.15;
        break;
    case 3:
        decision_params_.max_speed_limit = 1.67;
        decision_params_.max_speed_limit_backward = 1.67 * 0.5;
        decision_params_.emergency_brake_distance = 0.2;
        break;
    }
}

void DecisionMaker::updateRadarNearestPoint(const std::vector<ClusterInfo>& clusters, const std::string& radar_name)
{
    double half_wid = robot_dimensions_.width * 0.5; // 0.32
    double safety_side = robot_dimensions_.safety_margin_side;
    double forward_distance_min = std::numeric_limits<double>::max();
    double left_distance_min = std::numeric_limits<double>::max();
    double right_distance_min = std::numeric_limits<double>::max();
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        double forward_distance = std::numeric_limits<double>::max();
        double left_distance = std::numeric_limits<double>::max();
        double right_distance = std::numeric_limits<double>::max();
        if (std::abs(cluster.center_y) <= (half_wid + safety_side)) {
            if (cluster.center_x >= robot_dimensions_.length) {
                // 距离前缘的前向投影距离，>0 表示前方剩余距离
                forward_distance = (cluster.center_x - robot_dimensions_.length);
            } else if (cluster.center_x > robot_dimensions_.length - decision_params_.foot_distance_to_radar) {
                forward_distance = 0.0;
            }
        }
        if (cluster.center_x > 0.0 && cluster.center_x < robot_dimensions_.length + decision_params_.foot_distance_to_radar) {
            if (cluster.center_y > half_wid) {
                left_distance = std::abs(cluster.center_y) - half_wid;
            }
            if (cluster.center_y < -half_wid) {
                right_distance = std::abs(cluster.center_y) - half_wid;
            }
        }
        forward_distance_min = std::min(forward_distance_min, forward_distance);
        left_distance_min = std::min(left_distance_min, left_distance);
        right_distance_min = std::min(right_distance_min, right_distance);
    }
    // 判断是否存在该雷达的元素
    if (radar_nearest_point_map_.find(radar_name) == radar_nearest_point_map_.end()) {
        RadarNearestPoint nearest_point;
        nearest_point.radar_name = radar_name;
        nearest_point.forward_distance = forward_distance_min;
        nearest_point.left_distance = left_distance_min;
        nearest_point.right_distance = right_distance_min;
        radar_nearest_point_map_[radar_name] = nearest_point;
    } else {
        RadarNearestPoint& nearest_point = radar_nearest_point_map_[radar_name];
        nearest_point.forward_distance = std::min(nearest_point.forward_distance, forward_distance_min);
        nearest_point.left_distance = std::min(nearest_point.left_distance, left_distance_min);
        nearest_point.right_distance = std::min(nearest_point.right_distance, right_distance_min);
    }
    RCLCPP_DEBUG(this->get_logger(), "[L%d] radar_nearest_point: %s, forward_distance: %.3f",
        __LINE__, radar_name.c_str(), forward_distance_min);
    // RCLCPP_WARN(this->get_logger(), "[L%d] radar_nearest_point: %s, left_distance: %.3f",
    //     __LINE__, radar_name.c_str(), nearest_point.left_distance);
    // RCLCPP_WARN(this->get_logger(), "[L%d] radar_nearest_point: %s, right_distance: %.3f",
    //     __LINE__, radar_name.c_str(), nearest_point.right_distance);
}

void DecisionMaker::joystickCallback(const interface::msg::Joystick::SharedPtr msg)
{
    joystick_msg_ = *msg;
}

interface::msg::Joystick DecisionMaker::getCurrentJoystickMsg() const
{
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
            publishSafetyAlarm("前向负平面刹停区", WarningType::FORWARD_NEGATIVE_PLANE_BRAKE_WARNING);
        } else {
            publishSafetyAlarm("前向地面以上刹停区", WarningType::FORWARD_BRAKE_WARNING);
        }
        is_emergency_brake_zone_speed_suppressed_ = true;
        is_deceleration_zone_speed_suppressed_ = false;
    }else{
        if (is_negative_plane) {
            publishSafetyAlarm("前向负平面减速区", WarningType::FORWARD_NEGATIVE_PLANE_WARNING);
        } else {
            publishSafetyAlarm("前向地面以上减速区", WarningType::FORWARD_WARNING);
        }
        is_deceleration_zone_speed_suppressed_ = true;
        is_emergency_brake_zone_speed_suppressed_ = false;
    }
}

void DecisionMaker::judgeBackwardWarningScene(const SpeedLimit& limit)
{
    if (limit.max_linear_speed < 0.01) {
        publishSafetyAlarm("后向刹停区", WarningType::BACKWARD_BRAKE_WARNING);
        is_emergency_brake_zone_speed_suppressed_ = true;
        is_deceleration_zone_speed_suppressed_ = false;
    } else {
        publishSafetyAlarm("后向减速区", WarningType::BACKWARD_WARNING);
        is_deceleration_zone_speed_suppressed_ = true;
        is_emergency_brake_zone_speed_suppressed_ = false;
    }
}

void DecisionMaker::judgeLateralWarningScene(const SpeedLimit& limit)
{
    if (limit.max_angular_speed < 0.01) {
        if(left_min_distance_radar_source_ == "left" && left_min_distance_nearest_z_ <= 0.01){
            publishSafetyAlarm("左侧侧向刹停区", WarningType::LATERAL_NEGATIVE_PLANE_BRAKE_WARNING);
        }else if(right_min_distance_radar_source_ == "right" && right_min_distance_nearest_z_ <= 0.01){
            publishSafetyAlarm("右侧侧向刹停区", WarningType::LATERAL_NEGATIVE_PLANE_BRAKE_WARNING);
        }else{
            publishSafetyAlarm("侧向减速区", WarningType::LATERAL_BRAKE_WARNING);
        }
        is_emergency_brake_zone_speed_suppressed_ = true;
        is_deceleration_zone_speed_suppressed_ = false;
    } else {
        if(left_min_distance_radar_source_ == "left" && left_min_distance_nearest_z_ <= 0.01){
            publishSafetyAlarm("左侧侧向减速区", WarningType::LATERAL_NEGATIVE_PLANE_WARNING);
        }else if(right_min_distance_radar_source_ == "right" && right_min_distance_nearest_z_ <= 0.01){
            publishSafetyAlarm("右侧侧向减速区", WarningType::LATERAL_NEGATIVE_PLANE_WARNING);
        }else{
            publishSafetyAlarm("侧向减速区", WarningType::LATERAL_WARNING);
        }
        is_deceleration_zone_speed_suppressed_ = true;
        is_emergency_brake_zone_speed_suppressed_ = false;
    }
}