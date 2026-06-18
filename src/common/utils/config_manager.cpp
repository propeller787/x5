/**
 * @file config_manager.cpp
 * @brief 统一配置管理框架实现
 */

#include "config_manager.h"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <filesystem>
#include <sstream>

namespace common::utils {

// ==================== ConfigManager ====================

ConfigManager::ConfigManager(rclcpp::Node* node)
    : node_(node)
{
    if (!node_) {
        throw std::invalid_argument("Node pointer cannot be null");
    }
    
    // 注册参数变更回调
    param_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&ConfigManager::onSetParameters, this, std::placeholders::_1));
}

// 模板函数实现已在头文件中

bool ConfigManager::loadFromYaml(const std::string& file_path, const std::string& namespace_prefix) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!std::filesystem::exists(file_path)) {
        RCLCPP_ERROR(node_->get_logger(), "Config file not found: %s", file_path.c_str());
        return false;
    }
    
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        
        // 如果配置有 ros__parameters 节点，使用它
        if (config["ros__parameters"]) {
            loadYamlNode(config["ros__parameters"], "", namespace_prefix);
        } else {
            // 否则直接加载根节点
            loadYamlNode(config, "", namespace_prefix);
        }
        
        RCLCPP_INFO(node_->get_logger(), "Loaded config from YAML: %s", file_path.c_str());
        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse YAML config: %s, error: %s", 
                    file_path.c_str(), e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML config: %s, error: %s", 
                    file_path.c_str(), e.what());
        return false;
    }
}

void ConfigManager::loadYamlNode(const YAML::Node& node, const std::string& prefix, const std::string& namespace_prefix) {
    if (!node.IsMap()) {
        return;
    }
    
    for (const auto& item : node) {
        std::string key = item.first.as<std::string>();
        std::string full_key = prefix.empty() ? key : (prefix + "." + key);
        
        // 如果指定了命名空间前缀，添加到键名前面
        if (!namespace_prefix.empty()) {
            full_key = namespace_prefix + "." + full_key;
        }
        
        const YAML::Node& value = item.second;
        
        if (value.IsMap()) {
            // 递归处理嵌套的 map
            loadYamlNode(value, full_key, namespace_prefix);
        } else {
            // 声明并设置参数
            try {
                if (value.IsScalar()) {
                    // 尝试推断类型
                    std::string str_value = value.as<std::string>();
                    
                    // 尝试解析为数字或布尔值
                    if (str_value == "true" || str_value == "True") {
                        node_->declare_parameter(full_key, true);
                        node_->set_parameter(rclcpp::Parameter(full_key, true));
                    } else if (str_value == "false" || str_value == "False") {
                        node_->declare_parameter(full_key, false);
                        node_->set_parameter(rclcpp::Parameter(full_key, false));
                    } else {
                        // 尝试解析为数字
                        try {
                            if (str_value.find('.') != std::string::npos) {
                                double dval = value.as<double>();
                                node_->declare_parameter(full_key, dval);
                                node_->set_parameter(rclcpp::Parameter(full_key, dval));
                            } else {
                                int ival = value.as<int>();
                                node_->declare_parameter(full_key, ival);
                                node_->set_parameter(rclcpp::Parameter(full_key, ival));
                            }
                        } catch (...) {
                            // 解析为字符串
                            node_->declare_parameter(full_key, str_value);
                            node_->set_parameter(rclcpp::Parameter(full_key, str_value));
                        }
                    }
                } else if (value.IsSequence()) {
                    // 处理数组（暂时作为字符串数组处理）
                    std::vector<std::string> vec;
                    for (const auto& elem : value) {
                        vec.push_back(elem.as<std::string>());
                    }
                    node_->declare_parameter(full_key, vec);
                    node_->set_parameter(rclcpp::Parameter(full_key, vec));
                }
                
                declared_params_.insert(full_key);
            } catch (const std::exception& e) {
                RCLCPP_WARN(node_->get_logger(), "Failed to load parameter %s: %s", 
                           full_key.c_str(), e.what());
            }
        }
    }
}

bool ConfigManager::loadFromJson(const std::string& file_path, const std::string& namespace_prefix) {
    // JSON 加载可以通过转换为 YAML 或直接解析实现
    // 这里简化处理，建议统一使用 YAML
    RCLCPP_WARN(node_->get_logger(), "JSON config loading not fully implemented, use YAML instead");
    return false;
}

void ConfigManager::registerChangeCallback(const std::string& key, ConfigChangeCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    change_callbacks_[key].push_back(callback);
}

rcl_interfaces::msg::SetParametersResult ConfigManager::onSetParameters(
    const std::vector<rclcpp::Parameter>& parameters) {
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (const auto& param : parameters) {
        std::string key = param.get_name();
        
        // 更新缓存
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            config_cache_[key] = param.as_int();
        } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            config_cache_[key] = param.as_double();
        } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            config_cache_[key] = param.as_string();
        } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
            config_cache_[key] = param.as_bool();
        }
        
        // 触发回调
        triggerCallbacks(key, param);
    }
    
    return result;
}

void ConfigManager::triggerCallbacks(const std::string& key, const rclcpp::Parameter& param) {
    // 触发特定键的回调
    auto it = change_callbacks_.find(key);
    if (it != change_callbacks_.end()) {
        for (auto& callback : it->second) {
            callback(key, param);
        }
    }
    
    // 触发全局回调（空字符串键）
    auto global_it = change_callbacks_.find("");
    if (global_it != change_callbacks_.end()) {
        for (auto& callback : global_it->second) {
            callback(key, param);
        }
    }
}

std::vector<std::string> ConfigManager::getDeclaredParams() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::vector<std::string>(declared_params_.begin(), declared_params_.end());
}

bool ConfigManager::exportToYaml(const std::string& file_path) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    try {
        YAML::Node root;
        YAML::Node params = root["ros__parameters"];
        
        for (const auto& key : declared_params_) {
            if (!node_->has_parameter(key)) {
                continue;
            }
            
            auto param = node_->get_parameter(key);
            std::vector<std::string> parts;
            std::stringstream ss(key);
            std::string item;
            
            while (std::getline(ss, item, '.')) {
                parts.push_back(item);
            }
            
            YAML::Node current = params;
            for (size_t i = 0; i < parts.size() - 1; ++i) {
                if (!current[parts[i]].IsDefined()) {
                    current[parts[i]] = YAML::Node(YAML::NodeType::Map);
                }
                current = current[parts[i]];
            }
            
            std::string final_key = parts.back();
            switch (param.get_type()) {
                case rclcpp::ParameterType::PARAMETER_INTEGER:
                    current[final_key] = param.as_int();
                    break;
                case rclcpp::ParameterType::PARAMETER_DOUBLE:
                    current[final_key] = param.as_double();
                    break;
                case rclcpp::ParameterType::PARAMETER_STRING:
                    current[final_key] = param.as_string();
                    break;
                case rclcpp::ParameterType::PARAMETER_BOOL:
                    current[final_key] = param.as_bool();
                    break;
                default:
                    break;
            }
        }
        
        std::ofstream file(file_path);
        file << root;
        file.close();
        
        RCLCPP_INFO(node_->get_logger(), "Exported config to YAML: %s", file_path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to export config: %s", e.what());
        return false;
    }
}

// ==================== ConfigManagerFactory ====================

std::shared_ptr<ConfigManager> ConfigManagerFactory::instance_ = nullptr;
std::mutex ConfigManagerFactory::instance_mutex_;

std::shared_ptr<ConfigManager> ConfigManagerFactory::getInstance(rclcpp::Node* node) {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (!instance_) {
        instance_ = std::make_shared<ConfigManager>(node);
    }
    return instance_;
}

void ConfigManagerFactory::reset() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    instance_.reset();
}

} // namespace common::utils
