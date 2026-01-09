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
    RCLCPP_INFO(this->get_logger(), "  ✓ 决策制定器 (DecisionMaker) 创建成功");
    
    // 创建车辆接口 - 负责车辆状态管理和摇杆控制接收
    vehicle_interface_ = std::make_shared<VehicleInterface>();
    if (!vehicle_interface_) {
        throw std::runtime_error("车辆接口创建失败");
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ 车辆接口 (VehicleInterface) 创建成功");
    
    // 创建控制命令发布器 - 发布经过干预的控制命令
    control_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        nav2ControlTopic, 10);
    if (!control_publisher_) {
        throw std::runtime_error("控制命令发布器创建失败");
    }
    RCLCPP_INFO(this->get_logger(), "  ✓ 控制命令发布器创建成功 (话题: %s)", nav2ControlTopic);
    
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
    RCLCPP_INFO(this->get_logger(), "%s 所有组件创建完成 (%zu个组件)", 
                LOG_PREFIX, size_t(5));
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
    
    // 4.5. 设置轮速反馈回调 (VehicleInterface → Tracking)
    RCLCPP_INFO(this->get_logger(), "  → 连接: VehicleInterface(轮速反馈) → Tracking");
    vehicle_interface_->setWheelSpeedCallback(
        [this](double left_speed, double right_speed) {
            tracking_->updateWheelSpeed(left_speed, right_speed);
        });
    
    // 5. 将跟踪器注入到决策器 (依赖注入模式)
    RCLCPP_INFO(this->get_logger(), "  → 注入: Tracking → DecisionMaker");
    decision_maker_->setTracking(tracking_);
    
    RCLCPP_INFO(this->get_logger(), "%s 组件连接建立完成 (6条数据流)", LOG_PREFIX);
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
    bool auxiliary_mode = auxiliary_mode_enabled_.load(std::memory_order_acquire);
    bool standard_mode = standard_mode_enabled_.load(std::memory_order_acquire);
    bool cruise_mode = cruise_mode_enabled_.load(std::memory_order_acquire);
    
    if (auxiliary_mode) {
        // 安全模式: 使用执行器 (通过决策器处理)
        // 不需要锁，因为只读取原子变量
        decision_maker_->updateVehicleState(vehicle_state);
        decision_maker_->processHandleControl(handle_control);
        return;
    } else if (standard_mode) {
        // 标准模式: 关闭执行器，直接透传摇杆数据
        // 不需要锁，因为只读取原子变量
        control_publisher_->publish(handle_control);
        RCLCPP_DEBUG(this->get_logger(), "%s 标准模式: 直接透传摇杆控制: 线速度=%.3f m/s, 角速度=%.3f rad/s",
                    LOG_PREFIX, handle_control.linear.x, handle_control.angular.z);
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
        if (std::abs(joystick_msg.rocker_angle) > 140.0 && joystick_msg.rocker_x < -20) {
            // 修改共享变量（当前已持有锁）
            cruise_mode_enabled_.store(false, std::memory_order_release);
            // cruise_max_linear_speed_ = 0.0;
            // cruise_original_linear_speed_ = 0.0;
            cruise_last_speed_time_ = rclcpp::Time(0);
            cruise_speed_timeout_enabled_ = false;
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
                handleCruiseClose();
                RCLCPP_INFO(this->get_logger(), "%s 8秒内未收到大于0的速度，巡航模式已自动关闭", LOG_PREFIX);
            }
            return;  // 巡航模式已关闭，直接返回
        }
        // 只记录前进速度（正数），忽略后退速度（负数）
        // 当用户通过摇杆增加速度时，更新原始线速度（不受角速度影响的目标速度）
        if (handle_control.linear.x > 0.0 && handle_control.linear.x > cruise_original_linear_speed_) {
            cruise_original_linear_speed_ = handle_control.linear.x;
            // 如果当前没有因为角速度过大而降低速度，也更新当前使用的线速度
            if (cruise_max_linear_speed_ >= cruise_original_linear_speed_ || cruise_max_linear_speed_ == 0.0) {
                cruise_max_linear_speed_ = handle_control.linear.x;
            }
            RCLCPP_INFO(this->get_logger(), "%s 巡航模式: 更新原始线速度为 %.3f m/s", 
                       LOG_PREFIX, cruise_original_linear_speed_);
        }
        
        // 获取当前角速度的绝对值
        double abs_angular_velocity = std::abs(handle_control.angular.z);
        // 计算实际使用的线速度（从当前使用的线速度开始）
        double actual_linear_speed = cruise_max_linear_speed_;
        
        // 如果原始线速度未设置，使用当前最大线速度作为原始值
        if (cruise_original_linear_speed_ <= 0.0 && cruise_max_linear_speed_ > 0.0) {
            cruise_original_linear_speed_ = cruise_max_linear_speed_;
        }
        
        // 当角速度过大时，降低线速度防止倾倒
        if (abs_angular_velocity > cruise_angular_speed_threshold_) {
            // 根据角速度大小按比例降低线速度
            // 角速度越大，线速度降低越多
            double angular_ratio = cruise_angular_speed_threshold_ / abs_angular_velocity;
            // 使用平方关系，使速度降低更明显
            angular_ratio = angular_ratio * angular_ratio;
            actual_linear_speed = cruise_original_linear_speed_ * angular_ratio;
            // 确保线速度不为负
            actual_linear_speed = std::max(0.0, actual_linear_speed);
            
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "%s 巡航模式: 角速度过大 (%.3f rad/s > %.3f rad/s)，降低线速度: %.3f -> %.3f m/s",
                LOG_PREFIX, abs_angular_velocity, cruise_angular_speed_threshold_,
                cruise_original_linear_speed_, actual_linear_speed);
        } else if (abs_angular_velocity > 0.0 && cruise_original_linear_speed_ > 0.0) {
            // 角速度在恢复过程中（小于阈值但大于0），逐步恢复线速度
            if (actual_linear_speed < cruise_original_linear_speed_) {
                actual_linear_speed += cruise_speed_recovery_rate_;
                // 确保不超过原始线速度
                actual_linear_speed = std::min(actual_linear_speed, cruise_original_linear_speed_);
                
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "%s 巡航模式: 角速度恢复中 (%.3f rad/s)，恢复线速度: %.3f -> %.3f m/s (目标: %.3f m/s)",
                    LOG_PREFIX, abs_angular_velocity, cruise_max_linear_speed_,
                    actual_linear_speed, cruise_original_linear_speed_);
            }
        } else if (abs_angular_velocity == 0.0 && cruise_original_linear_speed_ > 0.0) {
            // 角速度已恢复为零，完全恢复线速度
            if (actual_linear_speed < cruise_original_linear_speed_) {
                actual_linear_speed = cruise_original_linear_speed_;
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
        
        // 通过决策器处理 (使用执行器) - 在锁外执行，避免长时间持有锁
        decision_maker_->updateVehicleState(vehicle_state);
        decision_maker_->processHandleControl(cruise_control);
    }
}

// ==================== 服务回调函数 ====================

void SceneNode::handleAuxiliaryService(
    const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response) {
    
    std::lock_guard<std::mutex> lock(mode_mutex_);
    
    if (request->start == 1) {
        auxiliary_mode_enabled_.store(true, std::memory_order_release);
        cruise_max_linear_speed_ = 0.0;  // 重置巡航最大速度
        cruise_original_linear_speed_ = 0.0;  // 重置原始线速度
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
}

void SceneNode::handleCruiseService(
    const std::shared_ptr<interface::srv::SceneControls::Request> request,
    std::shared_ptr<interface::srv::SceneControls::Response> response) {
    
    std::lock_guard<std::mutex> lock(mode_mutex_);
    
    if (request->start == 1) {
        cruise_mode_enabled_.store(true, std::memory_order_release);
        cruise_max_linear_speed_ = 0.0;  // 重置，等待接收摇杆数据
        cruise_original_linear_speed_ = 0.0;  // 重置原始线速度
        cruise_last_speed_time_ = this->now();  // 记录开启巡航的时间
        cruise_speed_timeout_enabled_ = true;  // 启用8秒超时检查（只有刚开启巡航模式后生效）
        RCLCPP_INFO(this->get_logger(), "%s 巡航功能已开启 (启用执行器，记录最大线速度)", LOG_PREFIX);
        response->code = 0;
        response->message = "巡航功能已开启";
    } else {
        cruise_mode_enabled_.store(false, std::memory_order_release);
        cruise_max_linear_speed_ = 0.0;  // 重置巡航最大速度
        cruise_original_linear_speed_ = 0.0;  // 重置原始线速度
        cruise_last_speed_time_ = rclcpp::Time(0);  // 重置最后收到速度的时间
        cruise_speed_timeout_enabled_ = false;  // 关闭8秒超时检查
        RCLCPP_INFO(this->get_logger(), "%s 巡航功能已关闭", LOG_PREFIX);
        response->code = 0;
        response->message = "巡航功能已关闭";
    }
}

// ==================== 信号处理 ====================

void signalHandler(int signal, std::shared_ptr<SceneNode> node) {
    if (node) {
        RCLCPP_INFO(node->get_logger(), "%s 收到信号 %d，正在优雅关闭节点...", 
                    LOG_PREFIX, signal);
        rclcpp::shutdown();
    }
}

void SceneNode::handleCruiseClose() {
    auto request = std::make_shared<interface::srv::SceneControls::Request>();
    request->start = 0;
    auto response = cruise_close_client_->async_send_request(request);
    if (response.get()->code == 0) {
        RCLCPP_INFO(this->get_logger(), "%s 巡航模式已自动关闭", LOG_PREFIX);
    } else {
        RCLCPP_ERROR(this->get_logger(), "%s 巡航模式自动关闭失败", LOG_PREFIX);
    }
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
                cruise_max_linear_speed_ = 0.0;
                cruise_original_linear_speed_ = 0.0;
                cruise_last_speed_time_ = rclcpp::Time(0);
                cruise_speed_timeout_enabled_ = false;
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
                cruise_max_linear_speed_ = 0.0;
                cruise_original_linear_speed_ = 0.0;
                cruise_last_speed_time_ = rclcpp::Time(0);
                cruise_speed_timeout_enabled_ = false;
                need_close = true;
            }
        }
        // 在锁外调用可能阻塞的函数，避免死锁
        if (need_close) {
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