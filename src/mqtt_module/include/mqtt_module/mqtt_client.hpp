#pragma once
#include "mqtt_module/token_manager.hpp"
#include "mqtt_module/mqtt_connection.hpp"
#include "mqtt_module/signature_manager.hpp"

namespace mqtt_module {

struct MqttClientConfig {
    DeviceInfo device;
    MqttConnConfig mqtt;
    SignatureConfig signature;
};

class MqttClient {
public:
    explicit MqttClient(const MqttClientConfig& cfg, rclcpp::Logger logger)
        : cfg_(cfg), logger_(logger), token_manager_(std::make_shared<TokenManager>()),
          signature_mgr_(std::make_shared<SignatureManager>(cfg.signature, logger)),
          conn_(std::make_shared<MqttConnectionManager>(cfg.mqtt, logger)) {}

    bool connect_with_token_flow() {
        RCLCPP_INFO(logger_, "===== 开始MQTT v5连接流程 =====");
        RCLCPP_INFO(logger_, "设备信息: VIN=%s, 型号=%s, API地址=%s", 
                   cfg_.device.vin.c_str(), cfg_.device.model.c_str(), cfg_.device.api_base_url.c_str());
        RCLCPP_INFO(logger_, "MQTT配置: Broker=%s, ClientID=%s, 使用token认证=%s", 
                   cfg_.mqtt.broker.c_str(), cfg_.mqtt.client_id.c_str(), cfg_.mqtt.use_token_auth ? "是" : "否");
        
        // 1. 第一次登陆时 获取会话密钥对
        RCLCPP_INFO(logger_, "1. 开始获取会话密钥对...");
        auto session_key = token_manager_->get_session_key(cfg_.device);
        if (!session_key.success) {
            RCLCPP_ERROR(logger_, "❌ 获取会话密钥失败: %s", session_key.err_msg.c_str());
            RCLCPP_INFO(logger_, "===== MQTT连接流程失败 =====");
            return false;
        }
        RCLCPP_INFO(logger_, "✅ 会话密钥获取成功");
        
        // 更新设备信息与会话密钥
        cfg_.device.session_public_key = session_key.public_key;
        cfg_.device.session_private_key = session_key.private_key;
        cfg_.device.session_key_no = session_key.keyNo;
        
        // 2. 获取可信令牌（挑战应答流程）
        RCLCPP_INFO(logger_, "2. 开始获取可信令牌（挑战应答流程）...");
        auto token = token_manager_->get_trusted_token(cfg_.device);
        if (!token.success) {
            RCLCPP_ERROR(logger_, "❌ 获取令牌失败: %s", token.err_msg.c_str());
            RCLCPP_INFO(logger_, "===== MQTT连接流程失败 =====");
            return false;
        }
        RCLCPP_INFO(logger_, "✅ 可信令牌获取成功，有效期: %ld ms", token.exp);
        
        // 3. 开始MQTT连接
        RCLCPP_INFO(logger_, "3. 开始MQTT v5连接...");
        cached_token_ = token;
        bool connected = conn_->connect(token);
        
        if (connected) {
            RCLCPP_INFO(logger_, "✅ MQTT v5连接成功");
            RCLCPP_INFO(logger_, "===== MQTT v5连接流程完成 =====");
        } else {
            RCLCPP_ERROR(logger_, "❌ MQTT连接失败");
            RCLCPP_INFO(logger_, "===== MQTT连接流程失败 =====");
        }
        
        return connected;
    }

    bool connect_with_token_flow_new(const TokenResult& token){
        RCLCPP_INFO(logger_, "===== 开始MQTT v5连接流程 =====");
        RCLCPP_INFO(logger_, "设备信息: VIN=%s, 型号=%s, API地址=%s", 
                   cfg_.device.vin.c_str(), cfg_.device.model.c_str(), cfg_.device.api_base_url.c_str());
        RCLCPP_INFO(logger_, "MQTT配置: Broker=%s, ClientID=%s, 使用token认证=%s", 
                   cfg_.mqtt.broker.c_str(), cfg_.mqtt.client_id.c_str(), cfg_.mqtt.use_token_auth ? "是" : "否");
        
        if (!token.success) {
            RCLCPP_ERROR(logger_, "❌ 获取令牌失败: %s", token.err_msg.c_str());
            RCLCPP_INFO(logger_, "===== MQTT连接流程失败 =====");
            return false;
        }
        RCLCPP_INFO(logger_, "开始MQTT v5连接...");
        cached_token_ = token;
        bool connected = conn_->connect(token);
        if (connected) {
            RCLCPP_INFO(logger_, "✅ MQTT v5连接成功");
            RCLCPP_INFO(logger_, "===== MQTT v5连接流程完成 =====");
        } else {
            RCLCPP_ERROR(logger_, "❌ MQTT连接失败");
            RCLCPP_INFO(logger_, "===== MQTT连接流程失败 =====");
        }
        return connected;
    }
    bool reconnect_with_token_flow() { 
        bool connected = conn_->force_reconnect(cached_token_);
        if (connected) {
            RCLCPP_INFO(logger_, "✅ MQTT v5连接成功");
            RCLCPP_INFO(logger_, "===== MQTT v5连接流程完成 =====");
        } else {
            RCLCPP_ERROR(logger_, "❌ MQTT连接失败");
            RCLCPP_INFO(logger_, "===== MQTT连接流程失败 =====");
        }
        return connected;
    }

    std::shared_ptr<MqttConnectionManager> connection() const { return conn_; }
    std::shared_ptr<SignatureManager> signature() const { return signature_mgr_; }
    const TokenResult& token() const { return cached_token_; }

    void get_session_key(std::string& public_key, std::string& private_key, std::string& no_key){
        public_key = cfg_.device.session_public_key;
        private_key = cfg_.device.session_private_key;
        no_key = cfg_.device.session_key_no;
    }

private:
    MqttClientConfig cfg_;
    rclcpp::Logger logger_;
    std::shared_ptr<TokenManager> token_manager_;
    std::shared_ptr<SignatureManager> signature_mgr_;
    std::shared_ptr<MqttConnectionManager> conn_;
    TokenResult cached_token_;
};

} // namespace mqtt_module
