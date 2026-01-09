/**
 * @file scene_node.h
 * @author likang (likang@freego.com)
 * @brief 场景节点类定义 - 雷达数据处理和车辆控制干预系统
 * 
 * 系统架构说明：
 * 本节点采用模块化设计，通过回调机制连接各个功能组件。
 * 
 * 数据流向：
 * 1. LidarDriver     - 接收多雷达数据，进行坐标转换和ROI过滤
 * 2. Clustering      - 对点云进行聚类，识别潜在障碍物
 * 3. Tracking        - 跟踪检测到的目标，预测未来位置
 * 4. DecisionMaker   - 基于障碍物信息进行碰撞检测和速度干预决策
 * 5. VehicleInterface- 获取车辆状态和摇杆控制输入
 * 
 * @version 0.1
 * @date 2025-10-16
 * @copyright Copyright (c) 2025 Freego Technology Co., Ltd.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <atomic>
#include <mutex>

#include "lidar_driver.h"
#include "clustering.h"
#include "decision_maker.h"
#include "vehicle_interface.h"
#include "tracking.h"
#include "../../../common/utils/topic.h"
#include <interface/srv/scene_controls.hpp>
#include <interface/srv/key_value_int.hpp>
#include <interface/msg/auxiliary_alarm.hpp>
#include <chrono>

// ==================== 常量定义 ====================
namespace {
    constexpr const char* NODE_NAME = "scene__node";
    constexpr const char* LOG_PREFIX = "[SceneAuxTest]";
}

/**
 * @class SceneNode
 * @brief 场景辅助测试主控制节点
 * 
 * 该类作为整个系统的核心，负责：
 * 1. 创建和管理所有功能组件
 * 2. 建立组件间的数据流连接
 * 3. 处理各组件的回调事件
 * 4. 发布最终的控制命令
 * 
 * 设计模式：观察者模式 (通过回调函数实现组件间解耦)
 */
class SceneNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数 - 初始化节点及所有组件
     * @throws std::runtime_error 如果组件初始化失败
     */
    SceneNode();
    
    /**
     * @brief 析构函数 - 清理资源
     */
    ~SceneNode();

    // ==================== 公共访问接口 ====================
    
    /**
     * @brief 获取车辆接口组件 (用于加入executor)
     * @return 车辆接口的共享指针
     */
    std::shared_ptr<VehicleInterface> getVehicleInterface() const {
        return vehicle_interface_;
    }
    
    /**
     * @brief 获取雷达驱动器组件 (用于加入executor)
     * @return 雷达驱动器的共享指针
     */
    std::shared_ptr<LidarDriver> getLidarDriver() const {
        return lidar_driver_;
    }
    
    /**
     * @brief 获取决策制定器组件 (用于加入executor)
     * @return 决策制定器的共享指针
     */
    std::shared_ptr<DecisionMaker> getDecisionMaker() const {
        return decision_maker_;
    }
    
    /**
     * @brief 获取聚类处理器组件 (用于加入executor)
     * @return 聚类处理器的共享指针
     */
    std::shared_ptr<Clustering> getClustering() const {
        return clustering_;
    }

private:
    // ==================== 初始化方法 ====================
    
    /**
     * @brief 初始化所有功能组件
     * 
     * 创建顺序说明：
     * 1. LidarDriver - 数据源组件，最先创建
     * 2. Clustering - 数据处理组件
     * 3. Tracking - 目标跟踪组件
     * 4. DecisionMaker - 决策控制组件
     * 5. VehicleInterface - 车辆状态管理组件
     * 6. ControlPublisher - 输出组件，最后创建
     * 
     * @throws std::runtime_error 如果任何组件创建失败
     */
    void initializeComponents();
    
    /**
     * @brief 建立组件间的回调连接
     * 
     * 数据流连接说明：
     * LidarDriver -> Clustering -> Tracking -> DecisionMaker -> ControlPublisher
     *                                ↑            ↑
     *                                └────────────┴─── VehicleInterface
     * 
     * 回调链说明：
     * 1. LidarDriver接收雷达数据 → onPointCloudReceived
     * 2. Clustering聚类处理 → onClustersDetected
     * 3. Tracking更新目标跟踪 → DecisionMaker处理
     * 4. DecisionMaker生成控制命令 → onControlCommandGenerated
     * 5. VehicleInterface更新车辆状态 → onVehicleStateChanged
     */
    void setupConnections();
    
    // ==================== 数据处理回调函数 ====================
    
    /**
     * @brief 雷达点云数据接收回调
     * 
     * 数据流: LidarDriver → onPointCloudReceived → Clustering
     * 
     * @param cloud 接收到的点云数据 (已经过ROI过滤)
     * @param source_name 雷达源名称 (left/right/left_tilt/right_tilt/back)
     */
    void onPointCloudReceived(const PointCloud::Ptr& cloud, const std::string& source_name);
    
    /**
     * @brief 聚类结果检测回调
     * 
     * 数据流: Clustering → onClustersDetected → Tracking & DecisionMaker
     * 
     * @param clusters 检测到的聚类列表 (每个聚类代表一个潜在障碍物)
     * @param source_name 雷达源名称
     */
    void onClustersDetected(const std::vector<ClusterInfo>& clusters, const std::string& source_name);
    
    /**
     * @brief 控制命令生成回调
     * 
     * 数据流: DecisionMaker → onControlCommandGenerated → ControlPublisher
     * 
     * @param control_cmd 经过速度干预的控制命令
     */
    void onControlCommandGenerated(const geometry_msgs::msg::Twist& control_cmd);
    
    /**
     * @brief 车辆状态变化回调
     * 
     * 数据流: VehicleInterface → onVehicleStateChanged → DecisionMaker/ControlPublisher
     * 
     * @param vehicle_state 当前车辆状态 (速度、轮速等)
     */
    void onVehicleStateChanged(const VehicleState& vehicle_state);
    
    // ==================== 服务回调函数 ====================
    
    /**
     * @brief 安全功能服务回调
     * @param request 服务请求 (start: 0关闭, 1开启)
     * @param response 服务响应
     */
    void handleAuxiliaryService(
        const std::shared_ptr<interface::srv::SceneControls::Request> request,
        std::shared_ptr<interface::srv::SceneControls::Response> response);
    
    /**
     * @brief 标准功能服务回调
     * @param request 服务请求 (start: 0关闭, 1开启)
     * @param response 服务响应
     */
    void handleStandardService(
        const std::shared_ptr<interface::srv::SceneControls::Request> request,
        std::shared_ptr<interface::srv::SceneControls::Response> response);
    
    /**
     * @brief 巡航功能服务回调
     * @param request 服务请求 (start: 0关闭, 1开启)
     * @param response 服务响应
     */
    void handleCruiseService(
        const std::shared_ptr<interface::srv::SceneControls::Request> request,
        std::shared_ptr<interface::srv::SceneControls::Response> response);

    /**
     * @brief 巡航模式关闭，用于检测到后退速度时，自动关闭巡航模式
     */
    void handleCruiseClose();
    
    /**
     * @brief 检查巡航模式速度超时并处理
     * 
     * 当开启巡航功能后，8秒内没有收到大于0的速度则退出巡航模式
     * 
     * @param vehicle_state 当前车辆状态
     * @param need_close_cruise 输出参数，如果为true，表示需要调用handleCruiseClose()
     * @return true 如果巡航模式继续运行，false 如果巡航模式已关闭
     * @note 此函数假设调用者已持有mode_mutex_锁，不会再次获取锁
     */
    bool checkCruiseSpeedTimeout(const VehicleState& vehicle_state, bool& need_close_cruise);
    
    /**
     * @brief 检查巡航模式最终控制命令超时并处理
     * 
     * 在AI巡航过程中，如果最终控制命令的线速度和角速度均为0，
     * 则触发8秒超时逻辑
     * 
     * @param control_cmd 最终控制命令
     * @return true 如果巡航模式继续运行，false 如果巡航模式已关闭
     */
    bool checkCruiseControlCommandTimeout(const geometry_msgs::msg::Twist& control_cmd);
    
private:
    // ==================== 成员变量 ====================
    
    // 核心功能组件
    std::shared_ptr<LidarDriver> lidar_driver_;           ///< 雷达驱动器 - 多雷达数据接收和预处理
    std::shared_ptr<Clustering> clustering_;              ///< 聚类处理器 - 点云聚类和障碍物识别
    std::shared_ptr<Tracking> tracking_;                  ///< 目标跟踪器 - 多目标跟踪和位置预测
    std::shared_ptr<DecisionMaker> decision_maker_;       ///< 决策制定器 - 碰撞检测和速度干预
    std::shared_ptr<VehicleInterface> vehicle_interface_; ///< 车辆接口 - 车辆状态管理和控制接收
    
    // ROS2通信组件
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_publisher_; ///< 控制命令发布器
    rclcpp::Publisher<interface::msg::AuxiliaryAlarm>::SharedPtr auxiliary_alarm_pub_; ///< 安全报警发布器
    
    // 场景控制服务
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr scene_auxiliary_service_;  ///< 安全功能服务
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr scene_standard_service_;   ///< 标准功能服务
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr scene_cruise_service_;     ///< 巡航功能服务
    // 设置状态
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr cruise_close_client_;
    // 模式状态标志 (线程安全)
    mutable std::mutex mode_mutex_;                       ///< 模式状态互斥锁
    std::atomic<bool> auxiliary_mode_enabled_{true};     ///< 安全模式是否启用
    std::atomic<bool> standard_mode_enabled_{false};      ///< 标准模式是否启用
    std::atomic<bool> cruise_mode_enabled_{false};        ///< 巡航模式是否启用
    double cruise_max_linear_speed_{0.0};                 ///< 巡航模式记录的最大线速度 (m/s)
    double cruise_original_linear_speed_{0.0};            ///< 巡航模式原始线速度（角速度过大之前的值）(m/s)
    double cruise_angular_speed_threshold_{0.5};          ///< 角速度阈值，超过此值认为角速度过大 (rad/s)
    double cruise_speed_recovery_rate_{0.05};             ///< 速度恢复速率 (m/s per cycle)
    rclcpp::Time last_alarm_time_{rclcpp::Time(0)};       ///< 上次报警时间，用于限制报警频率
    rclcpp::Time cruise_last_speed_time_{rclcpp::Time(0)}; ///< 巡航模式最后收到大于0速度的时间戳
    const double cruise_speed_timeout_{8.0};               ///< 巡航模式速度超时时间（秒），8秒内没有收到大于0的速度则退出
    bool cruise_speed_timeout_enabled_{false};             ///< 是否启用8秒超时检查（只有刚开启巡航模式后生效，有了有效速度则关闭）
};

// ==================== 信号处理 ====================

/**
 * @brief 信号处理函数 - 优雅关闭节点
 * 
 * @param signal 接收到的信号类型 (SIGINT, SIGTERM等)
 * @param node 场景节点指针
 */
void signalHandler(int signal, std::shared_ptr<SceneNode> node);

