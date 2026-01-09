/**
 * @file main.cpp
 * @author likang (likang@freego.com)
 * @brief 场景辅助测试主程序入口
 * 
 * @version 0.1
 * @date 2025-10-16
 * @copyright Copyright (c) 2025 Freego Technology Co., Ltd.
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <signal.h>
#include "scene_node.h"

// ==================== 全局变量 ====================

/// 全局节点指针，用于信号处理
std::shared_ptr<SceneNode> g_node = nullptr;

/**
 * @brief 信号处理包装函数 - 优雅关闭节点
 * 
 * @param signal 接收到的信号类型 (SIGINT, SIGTERM等)
 */
void signalHandlerWrapper(int signal) {
    signalHandler(signal, g_node);
}

// ==================== 主函数 ====================

/**
 * @brief 程序入口点
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出码 (0=成功, 1=失败)
 * 
 * 执行流程:
 * 1. 初始化ROS2运行环境
 * 2. 创建场景辅助测试节点
 * 3. 注册信号处理器 (支持Ctrl+C优雅退出)
 * 4. 创建多线程执行器并添加所有节点组件
 * 5. 启动执行器 (阻塞直到收到关闭信号)
 * 6. 清理资源并退出
 * 
 * 设计说明:
 * - 使用MultiThreadedExecutor支持多个节点组件并发运行
 * - 各个组件在独立线程中处理回调，避免阻塞
 * - 异常处理确保程序不会异常退出
 */
int main(int argc, char** argv) {
    // ==================== 1. 初始化ROS2 ====================
    RCLCPP_INFO(rclcpp::get_logger("main"), "=================================");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  场景辅助测试系统启动");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  Scene Auxiliary Test System");
    RCLCPP_INFO(rclcpp::get_logger("main"), "=================================");
    
    rclcpp::init(argc, argv);
    
    try {
        // ==================== 2. 创建主控制节点 ====================
        RCLCPP_INFO(rclcpp::get_logger("main"), "正在创建主控制节点...");
        g_node = std::make_shared<SceneNode>();
        
        if (!g_node) {
            RCLCPP_FATAL(rclcpp::get_logger("main"), "主控制节点创建失败！");
            return 1;
        }
        
        // ==================== 3. 注册信号处理器 ====================
        RCLCPP_INFO(rclcpp::get_logger("main"), "注册信号处理器...");
        signal(SIGINT, signalHandlerWrapper);   // Ctrl+C
        signal(SIGTERM, signalHandlerWrapper);  // kill命令
        
        // ==================== 4. 创建多线程执行器 ====================
        RCLCPP_INFO(rclcpp::get_logger("main"), "创建多线程执行器...");
        rclcpp::executors::MultiThreadedExecutor executor;
        
        // 添加主节点
        executor.add_node(g_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "  ✓ 已添加主控制节点");
        
        // 添加所有子组件节点到executor (支持并发处理)
        if (auto vehicle_interface = g_node->getVehicleInterface()) {
            executor.add_node(vehicle_interface);
            RCLCPP_INFO(rclcpp::get_logger("main"), "  ✓ 已添加 VehicleInterface");
        }
        
        if (auto lidar_driver = g_node->getLidarDriver()) {
            executor.add_node(lidar_driver);
            RCLCPP_INFO(rclcpp::get_logger("main"), "  ✓ 已添加 LidarDriver");
        }
        
        if (auto decision_maker = g_node->getDecisionMaker()) {
            executor.add_node(decision_maker);
            RCLCPP_INFO(rclcpp::get_logger("main"), "  ✓ 已添加 DecisionMaker");
        }
        
        if (auto clustering = g_node->getClustering()) {
            executor.add_node(clustering);
            RCLCPP_INFO(rclcpp::get_logger("main"), "  ✓ 已添加 Clustering");
        }
        
        // ==================== 5. 启动执行器 ====================
        RCLCPP_INFO(rclcpp::get_logger("main"), "=================================");
        RCLCPP_INFO(rclcpp::get_logger("main"), "  系统启动成功！开始运行...");
        RCLCPP_INFO(rclcpp::get_logger("main"), "  按Ctrl+C可优雅退出");
        RCLCPP_INFO(rclcpp::get_logger("main"), "=================================");
        
        executor.spin();  // 阻塞，直到收到关闭信号
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "节点运行异常: %s", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("main"), "系统将退出，请检查错误信息");
        
        // 清理并返回错误码
        g_node.reset();
        rclcpp::shutdown();
        return 1;
    }
    
    // ==================== 6. 清理资源 ====================
    RCLCPP_INFO(rclcpp::get_logger("main"), "正在清理资源...");
    g_node.reset();
    rclcpp::shutdown();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "=================================");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  场景辅助测试系统已安全关闭");
    RCLCPP_INFO(rclcpp::get_logger("main"), "=================================");
    
    return 0;
}
