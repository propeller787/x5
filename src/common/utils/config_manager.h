/**
 * @file config_manager.h
 * @brief 统一配置管理框架
 * @author likang (likang@freego.com)
 * @date 2025-01-XX
 * 
 * 提供统一的配置管理接口，支持：
 * 1. ROS2 参数系统集成
 * 2. YAML/JSON 配置文件加载
 * 3. 配置验证和默认值
 * 4. 配置热加载（参数回调）
 * 5. 配置分层（系统级、节点级、场景级）
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <memory>
#include <functional>
#include <mutex>
#include <any>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <type_traits>

// 前向声明，避免直接依赖 yaml-cpp
namespace YAML {
    class Node;
}

namespace common::utils {

/**
 * @brief 配置管理器基类
 * 
 * 提供统一的配置管理接口，支持多种配置源
 */
class ConfigManager {
public:
    using ConfigChangeCallback = std::function<void(const std::string& key, const rclcpp::Parameter& param)>;
    
    ConfigManager(rclcpp::Node* node);
    virtual ~ConfigManager() = default;
    
    /**
     * @brief 从 ROS2 参数声明并获取参数
     * @tparam T 参数类型 (int, double, string, bool 等)
     * @param key 参数键名（支持点号分隔的层级，如 "camera.width"）
     * @param default_value 默认值
     * @param description 参数描述
     * @return 参数值
     */
    template<typename T>
    T declareAndGet(const std::string& key, const T& default_value, const std::string& description = "") {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 如果参数已声明，直接获取
        if (declared_params_.find(key) != declared_params_.end()) {
            return node_->get_parameter(key).get_value<T>();
        }
        
        // 声明参数
        rcl_interfaces::msg::ParameterDescriptor desc;
        if (!description.empty()) {
            desc.description = description;
        }
        
        if constexpr (std::is_same_v<T, int>) {
            node_->declare_parameter(key, default_value, desc);
        } else if constexpr (std::is_same_v<T, double>) {
            node_->declare_parameter(key, default_value, desc);
        } else if constexpr (std::is_same_v<T, bool>) {
            node_->declare_parameter(key, default_value, desc);
        } else if constexpr (std::is_same_v<T, std::string>) {
            node_->declare_parameter(key, default_value, desc);
        } else {
            node_->declare_parameter(key, default_value);
        }
        
        declared_params_.insert(key);
        
        // 获取参数值
        T value = node_->get_parameter(key).get_value<T>();
        config_cache_[key] = value;
        
        return value;
    }
    
    /**
     * @brief 获取已声明的参数值
     * @tparam T 参数类型
     * @param key 参数键名
     * @return 参数值
     * @throws std::runtime_error 如果参数未声明
     */
    template<typename T>
    T get(const std::string& key) const {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto it = config_cache_.find(key);
        if (it != config_cache_.end()) {
            return std::any_cast<T>(it->second);
        }
        
        if (!node_->has_parameter(key)) {
            throw std::runtime_error("Parameter not declared: " + key);
        }
        
        T value = node_->get_parameter(key).get_value<T>();
        const_cast<ConfigManager*>(this)->config_cache_[key] = value;
        
        return value;
    }
    
    /**
     * @brief 从 YAML 文件加载配置
     * @param file_path YAML 文件路径
     * @param namespace_prefix 命名空间前缀（如 "camera"）
     * @return 是否加载成功
     */
    bool loadFromYaml(const std::string& file_path, const std::string& namespace_prefix = "");
    
    /**
     * @brief 从 JSON 文件加载配置
     * @param file_path JSON 文件路径
     * @param namespace_prefix 命名空间前缀
     * @return 是否加载成功
     */
    bool loadFromJson(const std::string& file_path, const std::string& namespace_prefix = "");
    
    /**
     * @brief 注册配置变更回调
     * @param key 参数键名（空字符串表示所有参数）
     * @param callback 回调函数
     */
    void registerChangeCallback(const std::string& key, ConfigChangeCallback callback);
    
    /**
     * @brief 验证配置参数
     * @param key 参数键名
     * @param validator 验证函数
     * @return 是否验证通过
     */
    template<typename T>
    bool validate(const std::string& key, std::function<bool(const T&)> validator) {
        try {
            T value = get<T>(key);
            return validator(value);
        } catch (...) {
            return false;
        }
    }
    
    /**
     * @brief 获取所有已声明的参数
     */
    std::vector<std::string> getDeclaredParams() const;
    
    /**
     * @brief 导出配置到 YAML 文件
     */
    bool exportToYaml(const std::string& file_path) const;
    
protected:
    rclcpp::Node* node_;
    mutable std::mutex mutex_;
    std::set<std::string> declared_params_;
    std::map<std::string, std::any> config_cache_;
    std::map<std::string, std::vector<ConfigChangeCallback>> change_callbacks_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    /**
     * @brief 参数变更回调处理
     */
    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& parameters);
    
    /**
     * @brief 从 YAML 节点递归加载参数
     */
    void loadYamlNode(const YAML::Node& node, const std::string& prefix, const std::string& namespace_prefix);
    
    /**
     * @brief 触发配置变更回调
     */
    void triggerCallbacks(const std::string& key, const rclcpp::Parameter& param);
};

/**
 * @brief 配置管理器工厂
 * 
 * 提供单例模式的配置管理器实例
 */
class ConfigManagerFactory {
public:
    static std::shared_ptr<ConfigManager> getInstance(rclcpp::Node* node);
    static void reset();
    
private:
    static std::shared_ptr<ConfigManager> instance_;
    static std::mutex instance_mutex_;
};

} // namespace common::utils
