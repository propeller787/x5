/**
 * @file 埋点使用示例.cpp
 * @brief 埋点系统使用示例代码
 * 
 * 本文件展示了如何在实际代码中使用埋点系统
 */

#include "utils/trigger_logger.h"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <map>

// ==================== 示例1: 程序启动时初始化 ====================

void initializeTriggerLogger() {
    // 初始化埋点日志器
    utils::TriggerLogger::getInstance().initialize(
        "ws://60.205.235.193:3009",  // WebSocket服务器地址
        "VLAGO_001",                  // 车辆ID
        "1.0.0"                       // 软件版本
    );
}

// ==================== 示例2: 模式切换埋点 ====================

class SceneServices : public rclcpp::Node {
public:
    void ChangeMode(const std::string& old_mode, const std::string& new_mode) {
        // 埋点: 模式切换开始
        TRIGGER_LOG("mode_switch_start", {
            {"old_mode", old_mode},
            {"new_mode", new_mode}
        });
        
        // 业务逻辑...
        // curr_mode_ = new_mode;
        
        // 埋点: 模式切换成功
        TRIGGER_LOG("mode_switch_success", {
            {"old_mode", old_mode},
            {"new_mode", new_mode}
        });
    }
};

// ==================== 示例3: 碰撞检测埋点 ====================

class CollisionChecker {
public:
    bool checkForCollision(double linear_vel, double angular_vel) {
        // 埋点: 碰撞检测开始
        TRIGGER_LOG("collision_check_start", {
            {"linear_velocity", std::to_string(linear_vel)},
            {"angular_velocity", std::to_string(angular_vel)}
        });
        
        bool collision_detected = false;
        // 碰撞检测逻辑...
        
        // 埋点: 碰撞检测结果
        TRIGGER_LOG("collision_check_result", {
            {"collision_detected", collision_detected ? "true" : "false"},
            {"final_linear_velocity", std::to_string(linear_vel)},
            {"final_angular_velocity", std::to_string(angular_vel)}
        });
        
        return collision_detected;
    }
};

// ==================== 示例4: 服务调用埋点 ====================

class GearsControl {
public:
    void handleGearsStart(int gear, int helpcar, int sos) {
        // 埋点: 服务请求
        TRIGGER_LOG("service_gears_request", {
            {"gear", std::to_string(gear)},
            {"helpcar", std::to_string(helpcar)},
            {"sos", std::to_string(sos)}
        });
        
        // 调用服务...
        bool success = true;
        int result_code = 0;
        
        if (!success) {
            // 埋点: 服务超时
            TRIGGER_LOG("error_service_timeout", {
                {"service_name", "gears_control"},
                {"timeout_seconds", "1"}
            });
        }
        
        // 埋点: 服务响应
        TRIGGER_LOG("service_gears_response", {
            {"result_code", std::to_string(result_code)}
        });
    }
};

// ==================== 示例5: 错误处理埋点 ====================

class LxCamera {
public:
    int Check(const std::string& command, int state) {
        if (state == 0) {  // SUCCESS
            return state;
        }
        
        // 埋点: 错误发生
        TRIGGER_LOG("error_lx_camera", {
            {"command", command},
            {"error_code", std::to_string(state)},
            {"error_message", "Device error"}
        });
        
        return state;
    }
};

// ==================== 示例6: 使用节点名称 ====================

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        // 使用节点名称的埋点
        TRIGGER_LOG_WITH_NODE("node_started", {
            {"node_name", "my_node"}
        }, "my_node");
    }
    
    void processData() {
        // 使用节点名称的埋点
        TRIGGER_LOG_WITH_NODE("data_processed", {
            {"data_count", "100"}
        }, this->get_name());
    }
};

// ==================== 示例7: 主程序示例 ====================

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // 1. 初始化埋点日志器
    initializeTriggerLogger();
    
    // 2. 创建节点
    auto node = std::make_shared<MyNode>();
    
    // 3. 使用埋点
    TRIGGER_LOG("system_started", {
        {"version", "1.0.0"},
        {"node_count", "1"}
    });
    
    // 4. 业务逻辑...
    SceneServices scene;
    scene.ChangeMode("standard", "auxiliary");
    
    CollisionChecker checker;
    checker.checkForCollision(1.0, 0.5);
    
    // 5. 程序结束前关闭埋点日志器
    rclcpp::shutdown();
    utils::TriggerLogger::getInstance().shutdown();
    
    return 0;
}

