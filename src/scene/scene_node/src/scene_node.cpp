/**
 * @file scene_node.cpp
 * @author likang (likang@freego.com)
 * @brief 场景节点类实现 - 雷达数据处理和车辆控制干预系统
 * 
 * @version 0.1
 * @date 2025-10-16
 * @copyright Copyright (c) 2025 Freego Technology Co., Ltd.
 */

#include "scene_node.h"
#include <signal.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace {
/** km/h → m/s（与 CarInfo.speedkmh 等单位一致） */
constexpr double kmhToMs(double kmh) { return kmh / 3.6; }

/**
 * 按 CarInfo 档位与驾驶模式计算最大线速度 (m/s)。
 * mode_type：1 舒适 2 运动（见 CarInfo.msg）。
 * 运动模式 4 档需求未写明具体 km/h，此处取 8 km/h，与 decision 的 absolute_max 上限一致。
 */
double gearMaxLinearSpeedMs(int gear, int mode_type) {
    const bool sport = (mode_type == 2);
    switch (gear) {
        case 1:
            return sport ? kmhToMs(2.0) : kmhToMs(1.5);
        case 2:
            return kmhToMs(3.0);
        case 3:
            return sport ? kmhToMs(6.0) : kmhToMs(4.5);
        case 4:
            return sport ? kmhToMs(8.0) : kmhToMs(6.0);
        default:
            return sport ? kmhToMs(2.0) : kmhToMs(1.5);
    }
}
}  // namespace

// ==================== 构造函数和析构函数 ====================

SceneNode::SceneNode() : rclcpp::Node(NODE_NAME) {
    RCLCPP_INFO(this->get_logger(), "%s 开始初始化场景辅助测试节点", LOG_PREFIX);
    
    try {
        // 初始化各个功能组件
        initializeComponents();
        
        // 建立组件间的回调连接
        setupConnections();
        
        RCLCPP_INFO(this->get_logger(), "%s 节点初始化成功！系统准备就绪", LOG_PREFIX);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "%s 节点初始化失败: %s", LOG_PREFIX, e.what());
        throw;
    }
}

SceneNode::~SceneNode() {
    RCLCPP_INFO(this->get_logger(), "%s 场景辅助测试节点正在关闭...", LOG_PREFIX);
}

// ==================== 初始化方法 ====================

void SceneNode::initializeComponents() {
    RCLCPP_INFO(this->get_logger(), "%s 开始创建各功能组件...", LOG_PREFIX);
    
    // 创建雷达驱动器 - 负责多雷达数据接收和预处理
    lidar_driver_ = std::make_shared<LidarDriver>();
    if (!lidar_driver_) {
        throw std::runtime_error("雷达驱动器创建失败");
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ 雷达驱动器 (LidarDriver) 创建成功");
    
    // 创建聚类处理器 - 负责点云聚类和障碍物识别
    clustering_ = std::make_shared<Clustering>();
    if (!clustering_) {
        throw std::runtime_error("聚类处理器创建失败");
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ 聚类处理器 (Clustering) 创建成功");
    
    // 创建目标跟踪器 - 负责多目标跟踪和位置预测
    tracking_ = std::make_shared<Tracking>();
    if (!tracking_) {
        throw std::runtime_error("目标跟踪器创建失败");
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ 目标跟踪器 (Tracking) 创建成功");
    
    // 创建决策制定器 - 负责碰撞检测和速度干预决策
    decision_maker_ = std::make_shared<DecisionMaker>();
    if (!decision_maker_) {
        throw std::runtime_error("决策制定器创建失败");
    }
    decision_maker_->setCruiseModeActive(false);
    RCLCPP_INFO(this->get_logger(), "  ✓ 决策制定器 (DecisionMaker) 创建成功");
    
    // 创建车辆接口 - 负责车辆状态管理和摇杆控制接收
    vehicle_interface_ = std::make_shared<VehicleInterface>();
    if (!vehicle_interface_) {
        throw std::runtime_error("车辆接口创建失败");
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ 车辆接口 (VehicleInterface) 创建成功");

    // 创建障碍物位置预测器 - 轮速驱动的实时运动补偿
    obstacle_predictor_ = std::make_shared<ObstaclePredictor>(this->get_logger());
    if (!obstacle_predictor_) {
        throw std::runtime_error("障碍物位置预测器创建失败");
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ 障碍物位置预测器 (ObstaclePredictor) 创建成功");

    // 创建操作者避让意图检测器 - 转向时衰减前向制动
    evasion_detector_ = std::make_shared<EvasionDetector>(this->get_logger());
    if (!evasion_detector_) {
        throw std::runtime_error("避让意图检测器创建失败");
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ 避让意图检测器 (EvasionDetector) 创建成功");

    // 创建控制命令发布器 - 发布经过干预的控制命令
    control_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        nav2ControlTopic, 10);
    if (!control_publisher_) {
        throw std::runtime_error("控制命令发布器创建失败");
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ 控制命令发布器创建成功 (话题: %s)", nav2ControlTopic);
    
    // 创建安全功能状态发布器 - 发布安全功能开启/关闭状态
    auxiliary_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        auxiliaryStatusTopic, 10);
    if (!auxiliary_status_pub_) {
        throw std::runtime_error("安全功能状态发布器创建失败");
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ 安全功能状态发布器创建成功 (话题: %s)", auxiliaryStatusTopic);

    // 启动时恢复上次安全预警状态（失败时保留默认开启）
    loadAuxiliaryModeState();
    updateDecisionOutputEnabled();
    
    // 创建巡航结束原因发布器 (1=没有前进速度 2=前向速度因障碍物后 3=巡航激活成功 4=巡航开启成功)
    cruise_end_reason_pub_ = this->create_publisher<std_msgs::msg::Int32>(cursorAlarmTopic, 10);
    if (!cruise_end_reason_pub_) {
        throw std::runtime_error("巡航结束原因发布器创建失败");
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ 巡航结束原因发布器创建成功 (话题: %s)", cursorAlarmTopic);

    // 定时 1 秒发布一次安全功能状态
    using namespace std::chrono_literals;
    auxiliary_status_timer_ = this->create_wall_timer(1000ms, [this]() {
        std_msgs::msg::Bool status_msg;
        status_msg.data = auxiliary_mode_enabled_.load(std::memory_order_acquire);
        auxiliary_status_pub_->publish(status_msg);
    });
    // 初始化后立即发布一次当前状态，避免等待首个1秒周期
    {
        std_msgs::msg::Bool status_msg;
        status_msg.data = auxiliary_mode_enabled_.load(std::memory_order_acquire);
        auxiliary_status_pub_->publish(status_msg);
    }
    
    // 创建场景控制服务
    scene_auxiliary_service_ = this->create_service<interface::srv::SceneControls>(
        sceneAuxiliaryService,
        std::bind(&SceneNode::handleAuxiliaryService, this, 
                 std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "  ✓ 安全功能服务创建成功 (服务: %s)", sceneAuxiliaryService);
    
    scene_standard_service_ = this->create_service<interface::srv::SceneControls>(
        sceneStandardService,
        std::bind(&SceneNode::handleStandardService, this, 
                 std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "  ✓ 标准功能服务创建成功 (服务: %s)", sceneStandardService);
    
    scene_cruise_service_ = this->create_service<interface::srv::SceneControls>(
        sceneCruiseService,
        std::bind(&SceneNode::handleCruiseService, this, 
                 std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "  ✓ 巡航功能服务创建成功 (服务: %s)", sceneCruiseService);
    cruise_close_client_ = this->create_client<interface::srv::SceneControls>(std::string(manageCruiseService));
    
    // 创建车辆信息订阅器 - 用于监听档位变化
    car_info_subscription_ = this->create_subscription<interface::msg::CarInfo>(
        carInfoTopic, 10,
        std::bind(&SceneNode::onCarInfoReceived, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "  ✓ 车辆信息订阅器创建成功 (话题: %s)", carInfoTopic);

    scene_service_mode_subscription_ = this->create_subscription<std_msgs::msg::String>(
        sceneServiceModeTopic, 10,
        std::bind(&SceneNode::onSceneServiceMode, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "  ✓ 场景模式订阅器创建成功 (话题: %s)", sceneServiceModeTopic);
    
    // 创建环境检测器 - 自动判断室内/室外并切换参数配置
    env_detector_ = std::make_shared<EnvironmentDetector>();
    if (!env_detector_) {
        throw std::runtime_error("环境检测器创建失败");
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ 环境检测器 (EnvironmentDetector) 创建成功");

    // // 创建虚拟导轨控制器
    // guide_rail_controller_ = std::make_shared<scene_node::VirtualGuideRailController>(
    //     this->get_logger());
    // if (!guide_rail_controller_) {
    //     throw std::runtime_error("虚拟导轨控制器创建失败");
    // }
    // RCLCPP_INFO(this->get_logger(), "  ✓ 虚拟导轨控制器 (VirtualGuideRailController) 创建成功");

    // // 创建过门居中服务（复用 /scene/narrow 端点）
    // scene_door_centering_service_ = this->create_service<interface::srv::SceneControls>(
    //     sceneNarrowService,
    //     std::bind(&SceneNode::handleDoorCenteringService, this,
    //              std::placeholders::_1, std::placeholders::_2));
    // RCLCPP_INFO(this->get_logger(), "  ✓ 过门居中服务创建成功 (服务: %s)", sceneNarrowService);

    // // 创建虚拟导轨调试发布器
    // guide_rail_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    //     "/scene/guide_rail/center_path", 5);
    // guide_rail_lookahead_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    //     "/scene/guide_rail/lookahead_point", 5);
    // RCLCPP_INFO(this->get_logger(), "  ✓ 虚拟导轨调试发布器创建成功");

    RCLCPP_INFO(this->get_logger(), "%s 所有组件创建完成 (%zu个组件)",
                LOG_PREFIX, size_t(9));
}

void SceneNode::setupConnections() {
    RCLCPP_INFO(this->get_logger(), "%s 开始建立组件间连接...", LOG_PREFIX);
    
    // 1. 设置雷达数据回调 (LidarDriver → SceneNode)
    RCLCPP_INFO(this->get_logger(), "  → 连接: LidarDriver → Clustering");
    lidar_driver_->setPointCloudCallback(
        [this](const PointCloud::Ptr& cloud, const std::string& source_name) {
            this->onPointCloudReceived(cloud, source_name);
        });
    
    // 2. 设置聚类结果回调 (Clustering → SceneNode)
    RCLCPP_INFO(this->get_logger(), "  → 连接: Clustering → Tracking/DecisionMaker");
    clustering_->setClusterCallback(
        [this](const std::vector<ClusterInfo>& clusters, const std::string& source_name) {
            this->onClustersDetected(clusters, source_name);
        });
    
    // 3. 设置决策器控制输出回调 (DecisionMaker → SceneNode)
    RCLCPP_INFO(this->get_logger(), "  → 连接: DecisionMaker → ControlPublisher");
    decision_maker_->setControlOutputCallback(
        [this](const geometry_msgs::msg::Twist& control_cmd) {
            this->onControlCommandGenerated(control_cmd);
        });
    
    // 4. 设置车辆状态变化回调 (VehicleInterface → SceneNode)
    //    注意: 只有摇杆控制数据会触发此回调
    RCLCPP_INFO(this->get_logger(), "  → 连接: VehicleInterface(摇杆控制) → DecisionMaker");
    vehicle_interface_->setStateChangeCallback(
        [this](const VehicleState& vehicle_state) {
            this->onVehicleStateChanged(vehicle_state);
        });
    
    // 4.5. 设置轮速反馈回调 (VehicleInterface → Tracking + ObstaclePredictor)
    RCLCPP_INFO(this->get_logger(), "  → 连接: VehicleInterface(轮速反馈) → Tracking + ObstaclePredictor");
    vehicle_interface_->setWheelSpeedCallback(
        [this](double linear_speed, double angular_speed) {
            tracking_->updateWheelSpeed(linear_speed, angular_speed);
            obstacle_predictor_->onWheelSpeedUpdate(linear_speed, angular_speed, this->now());
        });
    
    // 5. 将跟踪器和预测器注入到决策器 (依赖注入模式)
    RCLCPP_INFO(this->get_logger(), "  → 注入: Tracking → DecisionMaker");
    decision_maker_->setTracking(tracking_);
    RCLCPP_INFO(this->get_logger(), "  → 注入: ObstaclePredictor → DecisionMaker");
    decision_maker_->setObstaclePredictor(obstacle_predictor_);
    RCLCPP_INFO(this->get_logger(), "  → 注入: EvasionDetector → DecisionMaker");
    decision_maker_->setEvasionDetector(evasion_detector_);
    
    // 6. 环境检测器回调连接
    RCLCPP_INFO(this->get_logger(), "  → 连接: DecisionMaker(侧向距离) → EnvironmentDetector");
    decision_maker_->setLateralDistanceCallback(
        [this](double left, double right) {
            env_detector_->updateLateralDistances(left, right);
        });

    RCLCPP_INFO(this->get_logger(), "  → 连接: EnvironmentDetector → DecisionMaker(applyProfile)");
    env_detector_->setOnEnvironmentChanged(
        [this](EnvironmentType env) {
            decision_maker_->applyProfile(
                env == EnvironmentType::INDOOR
                    ? EnvironmentProfile::INDOOR
                    : EnvironmentProfile::OUTDOOR);
        });

    RCLCPP_INFO(this->get_logger(), "%s 组件连接建立完成 (8条数据流)", LOG_PREFIX);
}

// ==================== 数据处理回调函数 ====================

void SceneNode::onPointCloudReceived(const PointCloud::Ptr& cloud, const std::string& source_name) {
    // 数据有效性检查
    if (!cloud || cloud->points.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "%s 收到空点云数据 (来源: %s)", LOG_PREFIX, source_name.c_str());
        return;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "%s 收到 %s 雷达点云: %zu 个点", 
                 LOG_PREFIX, source_name.c_str(), cloud->points.size());
    
    // 传递给聚类处理器进行障碍物识别
    clustering_->clusterPointCloud(cloud, source_name);
}

void SceneNode::onClustersDetected(const std::vector<ClusterInfo>& clusters, const std::string& source_name) {
    RCLCPP_DEBUG(this->get_logger(), "%s %s 雷达检测到 %zu 个障碍物聚类", 
                 LOG_PREFIX, source_name.c_str(), clusters.size());
    
    // 传递给决策器 - 进行碰撞检测和速度干预决策
    decision_maker_->processClusters(clusters, source_name);

    // 同步给障碍物预测器 - 存入新鲜聚类并重置累计补偿
    obstacle_predictor_->updateRadarClusters(clusters, source_name, this->now());

    // // 虚拟导轨：缓存左右聚类，凑齐双侧后注入控制器
    // if (door_centering_enabled_.load(std::memory_order_acquire)) {
    //     if (source_name == "left" || source_name == "left_tilt") {
    //         cached_left_clusters_ = clusters;
    //     } else if (source_name == "right" || source_name == "right_tilt") {
    //         cached_right_clusters_ = clusters;
    //     }

    //     // 每次聚类到达都更新（即使另一侧为空，控制器内部可容忍）
    //     auto status = decision_maker_->getStatus();
    //     guide_rail_controller_->updateClusters(
    //         cached_left_clusters_,
    //         cached_right_clusters_,
    //         status.last_detected_distance);

    //     publishGuideRailDebugData();
    // }
}

void SceneNode::onControlCommandGenerated(const geometry_msgs::msg::Twist& control_cmd) {
    // 在巡航模式下，检查最终控制命令是否超时（预警速度为0后支持8秒则关闭巡航）
    if (cruise_mode_enabled_.load(std::memory_order_acquire)) {
        // 注意：checkCruiseControlCommandTimeout 内部会管理锁，避免死锁
        if (!checkCruiseControlCommandTimeout(control_cmd)) {
            // 巡航模式已关闭，不发布控制命令
            RCLCPP_DEBUG(this->get_logger(), "%s 巡航模式已关闭，跳过控制命令发布", LOG_PREFIX);
            return;
        }
    }
    // 发布最终控制命令
    control_publisher_->publish(control_cmd);
    
    RCLCPP_DEBUG(this->get_logger(), "%s 发布控制命令: 线速度=%.3f m/s, 角速度=%.3f rad/s",
                 LOG_PREFIX, control_cmd.linear.x, control_cmd.angular.z);
}

void SceneNode::onVehicleStateChanged(const VehicleState& vehicle_state) {
    // 将车辆状态转换为摇杆控制命令格式（在锁外创建，减少锁持有时间）
    geometry_msgs::msg::Twist handle_control;
    handle_control.linear.x = vehicle_state.linear_velocity;
    handle_control.linear.y = 0.0;
    handle_control.linear.z = 0.0;
    handle_control.angular.x = 0.0;
    handle_control.angular.y = 0.0;
    handle_control.angular.z = vehicle_state.angular_velocity;
    
    // 快速路径：使用原子变量快速检查模式，避免不必要的锁获取
    bool auxiliary_mode     = auxiliary_mode_enabled_.load(std::memory_order_acquire);
    bool standard_mode      = standard_mode_enabled_.load(std::memory_order_acquire);
    bool cruise_mode        = cruise_mode_enabled_.load(std::memory_order_acquire);
    // bool door_centering     = door_centering_enabled_.load(std::memory_order_acquire);

    // // 过门居中 / 虚拟导轨模式：接管方向控制，用户只控制前进/停止
    // if (door_centering) {
    //     // 后拉摇杆或侧向过近时退出
    //     if (guide_rail_controller_->checkEmergencyExit(handle_control.linear.x)) {
    //         door_centering_enabled_.store(false, std::memory_order_release);
    //         guide_rail_controller_->deactivate();
    //         cached_left_clusters_.clear();
    //         cached_right_clusters_.clear();
    //         RCLCPP_INFO(this->get_logger(), "%s 虚拟导轨：紧急退出，发送停车指令", LOG_PREFIX);
    //         // 紧急退出必须发送停车指令，防止车辆以上次速度继续行驶
    //         geometry_msgs::msg::Twist stop_cmd;
    //         stop_cmd.linear.x  = 0.0;
    //         stop_cmd.angular.z = 0.0;
    //         decision_maker_->updateVehicleState(vehicle_state);
    //         decision_maker_->processHandleControl(stop_cmd);
    //     } else {
    //         // 限速（弯道自适应）
    //         double recommended = guide_rail_controller_->getRecommendedMaxSpeed();
    //         handle_control.linear.x = std::min(handle_control.linear.x, recommended);
    //         // 角速度由程序全权控制
    //         handle_control.angular.z = guide_rail_controller_->computeAngularCorrection(
    //             handle_control.linear.x);

    //         RCLCPP_DEBUG(this->get_logger(),
    //             "%s 虚拟导轨: state=%d linear=%.3f angular=%.3f",
    //             LOG_PREFIX,
    //             static_cast<int>(guide_rail_controller_->getState()),
    //             handle_control.linear.x, handle_control.angular.z);

    //         // 经 DecisionMaker 前向碰撞检测后发布
    //         decision_maker_->updateVehicleState(vehicle_state);
    //         decision_maker_->processHandleControl(handle_control);
    //     }
    //     return;
    // }
    
    // if (auxiliary_mode) {
    //     // 安全模式: 使用执行器 (通过决策器处理)
    //     // 不需要锁，因为只读取原子变量
    //     decision_maker_->updateVehicleState(vehicle_state);
    //     decision_maker_->processHandleControl(handle_control);
    //     return;
    // }
    if (standard_mode) {
        // 标准模式: 安全关闭时透传；开启时由 DecisionMaker 定时器输出
        if (auxiliary_mode) {
            decision_maker_->updateVehicleState(vehicle_state);
            decision_maker_->updateDesiredTwist(handle_control);
        } else {
            control_publisher_->publish(handle_control);
        }
        return;
    } else if (cruise_mode) {
        // 巡航模式需要访问共享变量，需要锁保护
        std::unique_lock<std::mutex> lock(mode_mutex_);
        
        // 双重检查：在获取锁后再次确认巡航模式状态
        if (!cruise_mode_enabled_.load(std::memory_order_relaxed)) {
            return;
        }
        // 巡航模式: 记录最大线速度，后续一直用该速度发送，角速度不修改
        // 检测到后退速度时，自动关闭巡航模式
        interface::msg::Joystick joystick_msg = decision_maker_->getCurrentJoystickMsg();
        if (std::abs(joystick_msg.rocker_angle) > JOYSTICK_ANGLE_THRESHOLD && joystick_msg.rocker_x < JOYSTICK_X_THRESHOLD) {
            // 修改共享变量（当前已持有锁）
            cruise_mode_enabled_.store(false, std::memory_order_release);
            decision_maker_->setCruiseModeActive(false);
            // cruise_max_linear_speed_ = 0.0;
            // cruise_original_linear_speed_ = 0.0;
            cruise_last_speed_time_ = rclcpp::Time(0);
            cruise_speed_timeout_enabled_ = false;
            cruise_forward_speed_reported_ = false;
            // 注意：需要提前释放锁，因为 handleCruiseClose() 可能触发服务回调
            // 而服务回调会尝试获取同一个锁，导致死锁
            lock.unlock();  // 手动释放锁
            handleCruiseClose();
            RCLCPP_INFO(this->get_logger(), "%s 检测到后退速度，巡航模式已自动关闭", LOG_PREFIX);
            return;
        }
        // 检查巡航模式速度超时，如果超时则退出巡航模式
        // 注意：checkCruiseSpeedTimeout 假设调用者已持有锁，不会再次获取锁
        bool need_close_cruise = false;
        if (!checkCruiseSpeedTimeout(vehicle_state, need_close_cruise)) {
            if (need_close_cruise) {
                // 需要关闭巡航模式，先释放锁再调用可能阻塞的服务
                lock.unlock();
                publishCruiseEndReason(1);  // 1=没有前进速度 巡航结束
                handleCruiseClose();
                RCLCPP_INFO(this->get_logger(), "%s 8秒内未收到大于0的速度，巡航模式已自动关闭", LOG_PREFIX);
            }
            return;  // 巡航模式已关闭，直接返回
        }
        // 只记录前进速度（正数），忽略后退速度（负数）
        // 当用户通过摇杆增加速度时，更新原始线速度（不受角速度影响的目标速度）
        // 但不超过当前档位的最大速度限制
        if (handle_control.linear.x > 0.0 && handle_control.linear.x > cruise_original_linear_speed_) {
            // 限制速度不超过当前档位的最大速度
            double limited_speed = std::min(handle_control.linear.x, current_gear_max_speed_);
            cruise_original_linear_speed_ = limited_speed;
            // 如果当前没有因为角速度过大而降低速度，也更新当前使用的线速度
            if (cruise_max_linear_speed_ >= cruise_original_linear_speed_ || cruise_max_linear_speed_ == 0.0) {
                cruise_max_linear_speed_ = limited_speed;
            }
            // 如果用户尝试加速超过档位限制，清除降速目标（因为已经达到或超过目标）
            if (limited_speed >= current_gear_max_speed_) {
                cruise_target_speed_ = 0.0;
            }
            RCLCPP_INFO(this->get_logger(), "%s 巡航模式: 更新原始线速度为 %.3f m/s (档位限制: %.3f m/s)", 
                       LOG_PREFIX, limited_speed, current_gear_max_speed_);
        }
        
        // 获取当前角速度的绝对值
        double abs_angular_velocity = std::abs(handle_control.angular.z);
        // 计算实际使用的线速度（从当前使用的线速度开始）
        double actual_linear_speed = cruise_max_linear_speed_;
        
        // 如果原始线速度未设置，使用当前最大线速度作为原始值
        if (cruise_original_linear_speed_ <= 0.0 && cruise_max_linear_speed_ > 0.0) {
            cruise_original_linear_speed_ = cruise_max_linear_speed_;
        }
        
        // 检查是否需要平滑变速（档位变化导致的目标速度）
        rclcpp::Time current_time = this->now();
        double dt = (cruise_last_update_time_.seconds() > 0.0)
            ? (current_time - cruise_last_update_time_).seconds()
            : 0.02;  // 首次使用默认 20ms
        dt = std::min(dt, 0.1);  // 限制 dt 避免跳变

        if (cruise_target_speed_ > 0.0 && actual_linear_speed > cruise_target_speed_) {
            // 档位降低：基于时间的平滑降速
            double decel_amount = cruise_speed_deceleration_rate_ * dt;
            double speed_gap = actual_linear_speed - cruise_target_speed_;
            actual_linear_speed -= std::min(decel_amount, speed_gap);
            actual_linear_speed = std::max(actual_linear_speed, cruise_target_speed_);

            if (std::abs(actual_linear_speed - cruise_target_speed_) < 0.01) {
                actual_linear_speed = cruise_target_speed_;
                cruise_max_linear_speed_ = cruise_target_speed_;
                cruise_original_linear_speed_ = cruise_target_speed_;
                cruise_target_speed_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "%s 巡航模式: 已平滑降低到新档位速度 %.3f m/s (%.1f km/h)",
                           LOG_PREFIX, actual_linear_speed, actual_linear_speed * 3.6);
            } else {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "%s 巡航模式: 平滑降速中 %.3f -> %.3f m/s (目标: %.3f m/s)",
                    LOG_PREFIX, cruise_max_linear_speed_, actual_linear_speed, cruise_target_speed_);
            }
        }
        cruise_last_update_time_ = current_time;
        
        // 当角速度过大时，降低线速度防止倾倒
        if (abs_angular_velocity > cruise_angular_speed_threshold_) {
            // 根据角速度大小按比例降低线速度
            // 角速度越大，线速度降低越多
            double angular_ratio = cruise_angular_speed_threshold_ / abs_angular_velocity;
            // 使用平方关系，使速度降低更明显
            angular_ratio = angular_ratio * angular_ratio;
            actual_linear_speed = cruise_original_linear_speed_ * angular_ratio;
            if (cruise_target_speed_ > 0.0) {
                actual_linear_speed = std::min(actual_linear_speed, cruise_target_speed_);
            }
            actual_linear_speed = std::max(0.0, actual_linear_speed);
            
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "%s 巡航模式: 角速度过大 (%.3f rad/s > %.3f rad/s)，降低线速度: %.3f -> %.3f m/s",
                LOG_PREFIX, abs_angular_velocity, cruise_angular_speed_threshold_,
                cruise_original_linear_speed_, actual_linear_speed);
        } else if (abs_angular_velocity > 0.0 && cruise_original_linear_speed_ > 0.0
                   && !(cruise_target_speed_ > 0.0 && actual_linear_speed > cruise_target_speed_)) {
            // 角速度恢复中：档位变速时恢复目标为 cruise_target_speed_，否则为 cruise_original_linear_speed_
            double recovery_target = (cruise_target_speed_ > 0.0) ? cruise_target_speed_ : cruise_original_linear_speed_;
            if (actual_linear_speed < recovery_target) {
                actual_linear_speed += cruise_speed_recovery_rate_;
                actual_linear_speed = std::min(actual_linear_speed, recovery_target);
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "%s 巡航模式: 角速度恢复中，线速度: %.3f -> %.3f m/s (目标: %.3f m/s)",
                    LOG_PREFIX, cruise_max_linear_speed_, actual_linear_speed, recovery_target);
            }
        } else if (abs_angular_velocity == 0.0 && cruise_original_linear_speed_ > 0.0
                   && !(cruise_target_speed_ > 0.0 && actual_linear_speed > cruise_target_speed_)) {
            // 角速度已恢复为零：档位变速时目标为 cruise_target_speed_
            double recovery_target = (cruise_target_speed_ > 0.0) ? cruise_target_speed_ : cruise_original_linear_speed_;
            if (actual_linear_speed < recovery_target) {
                actual_linear_speed = recovery_target;
                RCLCPP_DEBUG(this->get_logger(),
                    "%s 巡航模式: 角速度已恢复为零，线速度已恢复: %.3f m/s",
                    LOG_PREFIX, actual_linear_speed);
            }
        }
        
        // 更新当前最大线速度（用于下次比较）
        cruise_max_linear_speed_ = actual_linear_speed;
        
        // 使用计算后的线速度，角速度使用当前摇杆值
        // 优化：在锁外创建控制命令，减少锁持有时间
        geometry_msgs::msg::Twist cruise_control;
        cruise_control.linear.x = (actual_linear_speed > 0.0) ? actual_linear_speed : 0.0;
        cruise_control.linear.y = 0.0;
        cruise_control.linear.z = 0.0;
        cruise_control.angular.x = 0.0;
        cruise_control.angular.y = 0.0;
        cruise_control.angular.z = handle_control.angular.z;  // 角速度不修改
        
        // 提前释放锁，避免在决策器处理期间持有锁
        lock.unlock();
        if (auxiliary_mode) {
            decision_maker_->updateVehicleState(vehicle_state);
            decision_maker_->updateDesiredTwist(cruise_control);
        } else {
            control_publisher_->publish(cruise_control);
        }
    } else if (helpcar_mode_enabled_.load(std::memory_order_acquire)) {
        // 推行模式(helpcar)：仅摇杆透传至 /nav/control（与 scene_services mode 字符串同步）
        decision_maker_->updateVehicleState(vehicle_state);
        control_publisher_->publish(handle_control);
    }
}

// ==================== 服务回调函数 ====================

void SceneNode::handleAuxiliaryService(
    const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response) {

    // 记录当前状态，用于判断是否发生变化
    bool old_status = auxiliary_mode_enabled_.load(std::memory_order_acquire);
    bool new_status = (request->start == 1);
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        const bool cruise_active = cruise_mode_enabled_.load(std::memory_order_relaxed);
        if (new_status) {
            auxiliary_mode_enabled_.store(true, std::memory_order_release);
            // 巡航进行中切换安全预警时，不应清空巡航速度状态，否则会出现“巡航未退出但速度归零”。
            // if (!cruise_active) {
            //     cruise_max_linear_speed_ = 0.0;
            //     cruise_original_linear_speed_ = 0.0;
            // }
            RCLCPP_INFO(this->get_logger(), "%s 安全功能已开启 (使用执行器)", LOG_PREFIX);
            response->code = 0;
            response->message = "安全功能已开启";
        } else {
            auxiliary_mode_enabled_.store(false, std::memory_order_release);
            RCLCPP_INFO(this->get_logger(), "%s 安全功能已关闭", LOG_PREFIX);
            response->code = 0;
            response->message = "安全功能已关闭";
        }
    }

    updateDecisionOutputEnabled();

    // 只在状态改变时发布
    if (old_status != new_status) {
        // 避免避障/安全模式开关后仍保留「前/后向制动保持」，导致车速锁 0 且需后拉摇杆才恢复
        decision_maker_->resetBrakeHoldState();
        std_msgs::msg::Bool status_msg;
        status_msg.data = new_status;
        auxiliary_status_pub_->publish(status_msg);
        RCLCPP_INFO(this->get_logger(), "%s 发布安全功能状态: %s", 
                    LOG_PREFIX, new_status ? "开启" : "关闭");
    }
}

bool SceneNode::loadAuxiliaryModeState() {
    try {
        std::ifstream in(user_config_file_path_);
        if (!in.is_open()) {
            RCLCPP_INFO(this->get_logger(),
                        "%s 未找到统一配置文件，使用默认状态: %s (文件: %s)",
                        LOG_PREFIX,
                        auxiliary_mode_enabled_.load(std::memory_order_acquire) ? "开启" : "关闭",
                        user_config_file_path_.c_str());
            return false;
        }

        std::string line;
        bool found = false;
        bool enabled = auxiliary_mode_enabled_.load(std::memory_order_acquire);
        while (std::getline(in, line)) {
            // 兼容 "safety_warning_enabled: 1" 与带前导空格的 YAML 行
            if (line.find("safety_warning_enabled:") == std::string::npos) {
                continue;
            }
            auto pos = line.find(':');
            if (pos == std::string::npos) {
                continue;
            }
            std::string value_part = line.substr(pos + 1);
            std::stringstream ss(value_part);
            int parsed = 1;
            if (ss >> parsed) {
                enabled = (parsed != 0);
                found = true;
                break;
            }
        }

        if (!found) {
            RCLCPP_WARN(this->get_logger(),
                        "%s 统一配置中未找到 safety_warning_enabled，使用默认状态: %s",
                        LOG_PREFIX, enabled ? "开启" : "关闭");
            return false;
        }

        auxiliary_mode_enabled_.store(enabled, std::memory_order_release);
        RCLCPP_INFO(this->get_logger(),
                    "%s 已从统一配置恢复安全功能状态: %s (文件: %s)",
                    LOG_PREFIX, enabled ? "开启" : "关闭", user_config_file_path_.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
                    "%s 读取统一配置失败，使用默认状态: %s, error=%s",
                    LOG_PREFIX,
                    auxiliary_mode_enabled_.load(std::memory_order_acquire) ? "开启" : "关闭",
                    e.what());
        return false;
    }
}

void SceneNode::handleStandardService(
    const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response) {
    
    std::lock_guard<std::mutex> lock(mode_mutex_);
    if (request->start == 1) {
        standard_mode_enabled_.store(true, std::memory_order_release);
        cruise_max_linear_speed_ = 0.0;  // 重置巡航最大速度
        cruise_original_linear_speed_ = 0.0;  // 重置原始线速度
        RCLCPP_INFO(this->get_logger(), "%s 标准功能已开启 (关闭执行器，直接透传)", LOG_PREFIX);
        response->code = 0;
        response->message = "标准功能已开启";
    } else {
        standard_mode_enabled_.store(false, std::memory_order_release);
        RCLCPP_INFO(this->get_logger(), "%s 标准功能已关闭", LOG_PREFIX);
        response->code = 0;
        response->message = "标准功能已关闭";
    }
    updateDecisionOutputEnabled();
}

void SceneNode::handleCruiseService(
    const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response) {
    
    std::lock_guard<std::mutex> lock(mode_mutex_);
    
    if (request->start == 1) {
        cruise_mode_enabled_.store(true, std::memory_order_release);
        decision_maker_->setCruiseModeActive(true);
        cruise_max_linear_speed_ = 0.0;  // 重置，等待接收摇杆数据
        cruise_original_linear_speed_ = 0.0;  // 重置原始线速度
        cruise_last_speed_time_ = this->now();  // 记录开启巡航的时间
        cruise_speed_timeout_enabled_ = true;  // 启用8秒超时检查（只有刚开启巡航模式后生效）
        cruise_forward_speed_reported_ = false;
        publishCruiseProcessState(3);  // 3=巡航激活成功
        RCLCPP_INFO(this->get_logger(), "%s 巡航功能已开启 (启用执行器，记录最大线速度)", LOG_PREFIX);
        response->code = 0;
        response->message = "巡航功能已开启";
    } else {
        cruise_mode_enabled_.store(false, std::memory_order_release);
        decision_maker_->setCruiseModeActive(false);
        cruise_max_linear_speed_ = 0.0;  // 重置巡航最大速度
        cruise_original_linear_speed_ = 0.0;  // 重置原始线速度
        cruise_last_speed_time_ = rclcpp::Time(0);  // 重置最后收到速度的时间
        cruise_speed_timeout_enabled_ = false;  // 关闭8秒超时检查
        cruise_forward_speed_reported_ = false;
        RCLCPP_INFO(this->get_logger(), "%s 巡航功能已关闭", LOG_PREFIX);
        response->code = 0;
        response->message = "巡航功能已关闭";
    }
    updateDecisionOutputEnabled();
}

void SceneNode::updateDecisionOutputEnabled()
{
    if (!decision_maker_) {
        return;
    }
    const bool enabled =
        auxiliary_mode_enabled_.load(std::memory_order_acquire)
        && (standard_mode_enabled_.load(std::memory_order_acquire)
            || cruise_mode_enabled_.load(std::memory_order_acquire))
        && !helpcar_mode_enabled_.load(std::memory_order_acquire);
    decision_maker_->setDecisionOutputEnabled(enabled);
}

// ==================== 过门居中 / 虚拟导轨 ====================

void SceneNode::handleDoorCenteringService(
    const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response)
{
#if !SCENE_NODE_BUILD_GUIDE_RAIL
    response->code = -1;
    response->message = "虚拟导轨未编译(SCENE_NODE_BUILD_GUIDE_RAIL=OFF)";
    return;
#else
    if (request->start == 1) {
        // 与 cruise / standard 模式互斥
        if (cruise_mode_enabled_.load(std::memory_order_acquire) ||
            standard_mode_enabled_.load(std::memory_order_acquire)) {
            RCLCPP_WARN(this->get_logger(),
                "%s 虚拟导轨：无法激活，当前巡航/标准模式已开启", LOG_PREFIX);
            response->code    = -1;
            response->message = "与当前模式冲突";
            return;
        }
        cached_left_clusters_.clear();
        cached_right_clusters_.clear();
        door_centering_enabled_.store(true, std::memory_order_release);
        guide_rail_controller_->activate();
        RCLCPP_INFO(this->get_logger(), "%s 虚拟导轨已激活", LOG_PREFIX);
        response->code    = 0;
        response->message = "虚拟导轨已激活";
    } else {
        door_centering_enabled_.store(false, std::memory_order_release);
        guide_rail_controller_->deactivate();
        cached_left_clusters_.clear();
        cached_right_clusters_.clear();
        RCLCPP_INFO(this->get_logger(), "%s 虚拟导轨已关闭", LOG_PREFIX);
        response->code    = 0;
        response->message = "虚拟导轨已关闭";
    }
#endif
}

void SceneNode::publishGuideRailDebugData()
{
#if !SCENE_NODE_BUILD_GUIDE_RAIL
    return;
#else
    // 中心线路径（仅在有订阅者时序列化，零开销）
    if (guide_rail_path_pub_->get_subscription_count() > 0) {
        auto center_path = guide_rail_controller_->getCenterPath();
        if (center_path.is_valid) {
            nav_msgs::msg::Path path_msg;
            path_msg.header.frame_id = "base_link";
            path_msg.header.stamp    = this->now();
            path_msg.poses.reserve(center_path.points.size());
            for (const auto& cp : center_path.points) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header        = path_msg.header;
                pose.pose.position.x = cp.x;
                pose.pose.position.y = cp.y;
                pose.pose.position.z = 0.0;
                // heading → 四元数（绕 z 轴旋转）
                pose.pose.orientation.w = std::cos(cp.heading / 2.0);
                pose.pose.orientation.z = std::sin(cp.heading / 2.0);
                path_msg.poses.push_back(pose);
            }
            guide_rail_path_pub_->publish(path_msg);
        }
    }

    // 前瞻点
    if (guide_rail_lookahead_pub_->get_subscription_count() > 0) {
        auto tracking = guide_rail_controller_->getTrackingResult();
        if (tracking.is_valid) {
            geometry_msgs::msg::PointStamped pt;
            pt.header.frame_id = "base_link";
            pt.header.stamp    = this->now();
            pt.point.x         = tracking.lookahead_x;
            pt.point.y         = tracking.lookahead_y;
            pt.point.z         = 0.0;
            guide_rail_lookahead_pub_->publish(pt);
        }
    }
#endif
}

// ==================== 信号处理 ====================

void signalHandler(int signal, std::shared_ptr<SceneNode> node) {
    if (node) {
        RCLCPP_INFO(node->get_logger(), "%s 收到信号 %d，正在优雅关闭节点...", 
                    LOG_PREFIX, signal);
        rclcpp::shutdown();
    }
}

void SceneNode::publishCruiseEndReason(int reason) {
    if (cruise_end_reason_pub_) {
        std_msgs::msg::Int32 msg;
        msg.data = reason;
        cruise_end_reason_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "%s 发布巡航结束原因: %d (1=无前进速度 2=障碍物降速)", LOG_PREFIX, reason);
    }
}

void SceneNode::publishCruiseProcessState(int status) {
    if (cruise_end_reason_pub_) {
        std_msgs::msg::Int32 msg;
        msg.data = status;
        cruise_end_reason_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "%s 发布巡航过程状态: %d (3=激活成功 4=开启成功)", LOG_PREFIX, status);
    }
}

void SceneNode::handleCruiseClose() {
    auto request = std::make_shared<interface::srv::SceneControls::Request>();
    request->start = 0;
    cruise_close_client_->async_send_request(request,
        [this](rclcpp::Client<interface::srv::SceneControls>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->code == 0) {
                    RCLCPP_INFO(this->get_logger(), "%s 巡航模式已自动关闭", LOG_PREFIX);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "%s 巡航模式自动关闭失败: code=%d", LOG_PREFIX, response->code);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "%s 巡航模式关闭服务调用异常: %s", LOG_PREFIX, e.what());
            }
        });
}

bool SceneNode::checkCruiseSpeedTimeout(const VehicleState& vehicle_state, bool& need_close_cruise) {
    // 如果8秒超时检查已关闭（已收到有效速度），直接返回true
    // 注意：在巡航模式运行过程中，预警速度的超时检查由 checkCruiseControlCommandTimeout 处理
    // 注意：此函数假设调用者已持有mode_mutex_锁，不会再次获取锁
    need_close_cruise = false;
    
    if (!cruise_speed_timeout_enabled_) {
        return true;  // 巡航模式继续运行
    }
    rclcpp::Time current_time = this->now();
    if (vehicle_state.linear_velocity > 0.0 || vehicle_state.angular_velocity != 0.0) {
        // 收到有效速度，关闭8秒超时检查逻辑（只有刚开启巡航模式后生效）
        cruise_speed_timeout_enabled_ = false;
        cruise_last_speed_time_ = current_time;
        if (!cruise_forward_speed_reported_ && vehicle_state.linear_velocity > 0.0) {
            publishCruiseProcessState(4);  // 4=检测到前向速度，巡航开启成功
            cruise_forward_speed_reported_ = true;
        }
        RCLCPP_INFO(this->get_logger(), "%s 检测到有效速度，关闭8秒超时检查（刚开启巡航模式时的检查）", LOG_PREFIX);
        return true;  // 巡航模式继续运行
    } else {
        // 如果 cruise_last_speed_time_ 是无效时间（0），则设置为当前时间（刚开启巡航模式的情况）
        if (cruise_last_speed_time_.seconds() == 0.0) {
            cruise_last_speed_time_ = current_time;
            return true;  // 刚开启，还未超时
        }
        
            // 检查是否超过8秒没有收到大于0的速度（只有刚开启巡航模式后生效）
            double time_since_last_speed = (current_time - cruise_last_speed_time_).seconds();
            if (time_since_last_speed >= cruise_speed_timeout_) {
                // 超时，关闭巡航模式
                // 注意：此函数假设调用者已持有锁，直接修改共享变量
                cruise_mode_enabled_.store(false, std::memory_order_release);
                decision_maker_->setCruiseModeActive(false);
                cruise_max_linear_speed_ = 0.0;
                cruise_original_linear_speed_ = 0.0;
                cruise_last_speed_time_ = rclcpp::Time(0);
                cruise_speed_timeout_enabled_ = false;
                cruise_forward_speed_reported_ = false;
                need_close_cruise = true;  // 标记需要调用handleCruiseClose()，但由调用者释放锁后调用
                return false;  // 巡航模式已关闭
            }
        return true;  // 未超时，巡航模式继续运行
    }
}

bool SceneNode::checkCruiseControlCommandTimeout(const geometry_msgs::msg::Twist& control_cmd) {
    // 快速检查：如果巡航模式未启用，直接返回（避免不必要的锁获取）
    if (!cruise_mode_enabled_.load(std::memory_order_acquire)) {
        return false;
    }
    
    rclcpp::Time current_time = this->now();
    
    // 检查最终控制命令的线速度和角速度是否均为0
    constexpr double epsilon = 1e-6;  // 浮点数比较阈值（编译时常量）
    bool is_zero_velocity = std::abs(control_cmd.linear.x) < epsilon;
    bool is_zero_angular = std::abs(control_cmd.angular.z) < epsilon;
    
    if (is_zero_velocity && is_zero_angular) {
        // 预警速度为0，重新启用8秒超时检查（如果之前已关闭）
        bool need_close = false;
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            // 双重检查：在获取锁后再次确认巡航模式状态
            if (!cruise_mode_enabled_.load(std::memory_order_relaxed)) {
                return false;
            }
            
            if (!cruise_speed_timeout_enabled_) {
                cruise_speed_timeout_enabled_ = true;
                cruise_last_speed_time_ = current_time;  // 记录预警速度为0的时间
                RCLCPP_INFO(this->get_logger(), "%s 检测到预警速度为0，重新启用8秒超时检查", LOG_PREFIX);
            }
            
            // 检查是否超过8秒预警速度一直为0
            double time_since_last_speed = (current_time - cruise_last_speed_time_).seconds();
            if (time_since_last_speed >= cruise_speed_timeout_) {
                // 超时，关闭巡航模式
                cruise_mode_enabled_.store(false, std::memory_order_release);
                decision_maker_->setCruiseModeActive(false);
                cruise_max_linear_speed_ = 0.0;
                cruise_original_linear_speed_ = 0.0;
                cruise_last_speed_time_ = rclcpp::Time(0);
                cruise_speed_timeout_enabled_ = false;
                cruise_forward_speed_reported_ = false;
                need_close = true;
            }
        }
        // 在锁外调用可能阻塞的函数，避免死锁
        if (need_close) {
            publishCruiseEndReason(2);  // 2=前向速度因障碍物后 巡航结束
            handleCruiseClose();
            RCLCPP_INFO(this->get_logger(), "%s 预警速度为0超过8秒，巡航模式已自动关闭", LOG_PREFIX);
            return false;  // 巡航模式已关闭
        }
        return true;  // 未超时，巡航模式继续运行
    } else {
        // 最终控制命令有速度，关闭8秒超时检查（如果之前已启用）
        // 优化：只在需要时才获取锁
        std::lock_guard<std::mutex> lock(mode_mutex_);
        // 双重检查：在获取锁后再次确认巡航模式状态
        if (!cruise_mode_enabled_.load(std::memory_order_relaxed)) {
            return false;
        }
        
        if (cruise_speed_timeout_enabled_) {
            cruise_speed_timeout_enabled_ = false;
            RCLCPP_INFO(this->get_logger(), "%s 检测到预警速度不为0，关闭8秒超时检查", LOG_PREFIX);
        }
        // 更新最后收到速度的时间
        cruise_last_speed_time_ = current_time;
        return true;  // 巡航模式继续运行
    }
}

// ==================== 场景模式（推行识别）====================

void SceneNode::onSceneServiceMode(const std_msgs::msg::String::SharedPtr msg) {
    if (!msg) {
        return;
    }
    const bool helpcar = (msg->data == "helpcar");
    helpcar_mode_enabled_.store(helpcar, std::memory_order_release);
    updateDecisionOutputEnabled();
}

// ==================== 车辆信息回调 ====================

void SceneNode::onCarInfoReceived(const interface::msg::CarInfo::SharedPtr msg) {
    if (!msg) {
        return;
    }
    
    const int8_t new_gear = static_cast<int8_t>(msg->gear);
    const int16_t new_mode = msg->mode_type;

    // 档位或舒适/运动模式变化时更新限速表
    if (new_gear != current_gear_ || new_mode != current_mode_type_) {
        if (new_gear != current_gear_) {
            RCLCPP_INFO(this->get_logger(), "%s 检测到档位变化: %d -> %d (mode_type=%d)",
                        LOG_PREFIX, current_gear_, new_gear, static_cast<int>(new_mode));
        }
        if (new_mode != current_mode_type_) {
            RCLCPP_INFO(this->get_logger(), "%s 检测到驾驶模式变化: %d -> %d (gear=%d)",
                        LOG_PREFIX, current_mode_type_, new_mode, static_cast<int>(new_gear));
        }

        const double new_max_speed = gearMaxLinearSpeedMs(new_gear, new_mode);

        current_gear_ = new_gear;
        current_mode_type_ = new_mode;
        current_gear_max_speed_ = new_max_speed;
        
        // 如果巡航模式开启，需要平滑降低到新档位的最高速度
        if (cruise_mode_enabled_.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            
            // 再次检查巡航模式状态（双重检查）
            if (!cruise_mode_enabled_.load(std::memory_order_relaxed)) {
                return;
            }
            
            // 档位变化时，根据当前速度与新档位限制的关系，设置平滑变速目标
            double current_speed = std::max(cruise_max_linear_speed_, cruise_original_linear_speed_);
            if (current_speed > new_max_speed) {
                // 档位降低：平滑降速到新档位最高速度
                cruise_target_speed_ = new_max_speed;
                cruise_last_update_time_ = rclcpp::Time(0);  // 重置，使下一周期使用合理 dt
                RCLCPP_INFO(this->get_logger(), 
                           "%s 巡航模式: 档位降低，开始平滑降速 %.3f -> %.3f m/s (%.1f km/h)", 
                           LOG_PREFIX, current_speed, new_max_speed, new_max_speed * 3.6);
            } else if (current_speed < new_max_speed) {
                // 档位升高：不自动加速，需用户通过摇杆获取新档位最大速度
                cruise_target_speed_ = 0.0;
                RCLCPP_INFO(this->get_logger(), 
                           "%s 巡航模式: 档位升高，当前速度 %.3f m/s，可通过摇杆加速至 %.3f m/s (%.1f km/h)", 
                           LOG_PREFIX, current_speed, new_max_speed, new_max_speed * 3.6);
            } else {
                // 当前速度已等于新档位限制，直接更新
                cruise_max_linear_speed_ = new_max_speed;
                cruise_original_linear_speed_ = new_max_speed;
                cruise_target_speed_ = 0.0;
                RCLCPP_INFO(this->get_logger(), 
                           "%s 巡航模式: 当前速度 %.3f m/s 已等于新档位限制，无需变速", 
                           LOG_PREFIX, current_speed);
            }
        }
    }

    // 当左抱闸或者右抱闸开启时，brakests_status_为true
    brakests_status_ = (msg->left_brakests == 1 || msg->right_brakests == 1) ? true : false;

    bool cruise_mode = cruise_mode_enabled_.load(std::memory_order_acquire); 

    // 如果当前为巡航模式，检测到抱闸开启时，需要退出巡航模式
    if (cruise_mode)
    {
        if(brakests_status_){
            handleCruiseClose();
        }
    }
}