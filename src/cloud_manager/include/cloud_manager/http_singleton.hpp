
#pragma once
#include <string>
#include <memory>
#include <mutex>
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <random>

#include "utils/topic.h"
#include "event_device_bind.pb.h"

#include "mqtt_module/signature_manager.hpp"
#include "mqtt_module/mqtt_connection.hpp"
#include "mqtt_module/mqtt_client.hpp"

using VehicleBindStatus = vehicle::device::bind::VehicleBindStatus;

namespace cloud_manager_node {
class HttpClientManager : public rclcpp::Node {
public:
    // 删除拷贝构造函数和赋值运算符
    HttpClientManager(const HttpClientManager&) = delete;
    HttpClientManager& operator=(const HttpClientManager&) = delete;
    
    // 获取单例实例
    static HttpClientManager* getInstance();
    
    // 定义请求构造函数的类型
    // 返回：tuple<完整URL, 头部, 请求体JSON>
    // template<typename... Args>
    // using HttpHandler = std::function<int(std::any)>;

    
private:
    // 私有构造函数
    HttpClientManager();
public:
    // 构造通用消息头
    cpr::Header build_headers(const mqtt_module::DeviceInfo& info, const std::string& body_json, bool use_session_key);
    // 设备绑定
    bool device_bind_msg(const std::string &url_name, const mqtt_module::DeviceInfo &info, VehicleBindStatus &pkg, std::string &err);

private:
    // 数据成员：map存储URL与处理函数的映射
    static std::mutex mutex_;
    static HttpClientManager* instance_;

    std::string base_url_ = "http://39.107.107.204:22000";
};
}