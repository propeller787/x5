#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <atomic>

namespace utils {

/**
 * @brief 埋点数据统一格式
 * 
 * 统一的埋点数据格式，用于云端上传
 */
struct TriggerData {
    // 基础字段
    std::string from;              // 数据来源: "pad" 或 "vlago"
    std::string type;              // 事件类型: "trigger" 固定值
    std::string event_name;        // 事件名称: 如 "mode_switch_start"
    int64_t timestamp;             // 时间戳(毫秒)
    std::string timestamp_str;     // 时间戳字符串: "YYYY-MM-DD HH:MM:SS"
    
    // 位置信息
    std::string node_name;         // ROS节点名称
    std::string file_path;        // 源文件路径
    int line_number;               // 源代码行号
    std::string function_name;     // 函数名称
    
    // 业务数据
    std::map<std::string, std::string> data;  // 业务数据键值对
    
    // 系统信息
    std::string vehicle_id;        // 车辆ID (可选)
    std::string version;           // 软件版本 (可选)
};

/**
 * @brief 埋点日志收集器
 * 
 * 单例模式，负责收集埋点数据并上传到云端
 */
class TriggerLogger {
public:
    /**
     * @brief 获取单例实例
     */
    static TriggerLogger& getInstance();
    
    /**
     * @brief 初始化埋点日志器
     * @param websocket_url WebSocket服务器地址
     * @param vehicle_id 车辆ID
     * @param version 软件版本
     */
    void initialize(const std::string& websocket_url = "ws://60.205.235.193:3009",
                   const std::string& vehicle_id = "",
                   const std::string& version = "");
    
    /**
     * @brief 记录埋点事件
     * @param event_name 事件名称
     * @param data 业务数据
     * @param file_path 源文件路径
     * @param line_number 源代码行号
     * @param function_name 函数名称
     * @param node_name ROS节点名称
     */
    void log(const std::string& event_name,
             const std::map<std::string, std::string>& data,
             const std::string& file_path = "",
             int line_number = 0,
             const std::string& function_name = "",
             const std::string& node_name = "");
    
    /**
     * @brief 设置车辆ID
     */
    void setVehicleId(const std::string& vehicle_id);
    
    /**
     * @brief 设置软件版本
     */
    void setVersion(const std::string& version);
    
    /**
     * @brief 停止日志器
     */
    void shutdown();
    
    /**
     * @brief 检查是否已初始化
     */
    bool isInitialized() const { return initialized_.load(); }

private:
    TriggerLogger() = default;
    ~TriggerLogger();
    TriggerLogger(const TriggerLogger&) = delete;
    TriggerLogger& operator=(const TriggerLogger&) = delete;
    
    /**
     * @brief 将TriggerData转换为JSON字符串
     */
    std::string toJson(const TriggerData& trigger_data);
    
    /**
     * @brief 上传线程函数
     */
    void uploadThread();
    
    /**
     * @brief 获取当前时间戳(毫秒)
     */
    int64_t getCurrentTimestamp();
    
    /**
     * @brief 格式化时间戳字符串
     */
    std::string formatTimestamp(int64_t timestamp_ms);
    
    // WebSocket客户端前向声明
    class WebSocketClient;
    
    std::unique_ptr<WebSocketClient> ws_client_;
    std::string websocket_url_;
    std::string vehicle_id_;
    std::string version_;
    
    std::mutex queue_mutex_;
    std::queue<TriggerData> data_queue_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> should_stop_{false};
    std::thread upload_thread_;
    
    // 队列大小限制
    static constexpr size_t MAX_QUEUE_SIZE = 10000;
};

/**
 * @brief 便捷宏定义，用于快速埋点
 * 
 * 使用示例:
 *   TRIGGER_LOG("mode_switch_start", {
 *       {"old_mode", "standard"},
 *       {"new_mode", "auxiliary"}
 *   });
 */
#define TRIGGER_LOG(event_name, data) \
    utils::TriggerLogger::getInstance().log( \
        event_name, \
        data, \
        __FILE__, \
        __LINE__, \
        __FUNCTION__, \
        "")

/**
 * @brief 带节点名称的埋点宏
 */
#define TRIGGER_LOG_WITH_NODE(event_name, data, node_name) \
    utils::TriggerLogger::getInstance().log( \
        event_name, \
        data, \
        __FILE__, \
        __LINE__, \
        __FUNCTION__, \
        node_name)

} // namespace utils

