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
        auto token = token_manager_->get_trusted_token(cfg_.device);
        if (!token.success) {
            RCLCPP_ERROR(logger_, "get token failed: %s", token.err_msg.c_str());
            return false;
        }
        cached_token_ = token;
        return conn_->connect(token);
    }

    std::shared_ptr<MqttConnectionManager> connection() const { return conn_; }
    std::shared_ptr<SignatureManager> signature() const { return signature_mgr_; }
    const TokenResult& token() const { return cached_token_; }

private:
    MqttClientConfig cfg_;
    rclcpp::Logger logger_;
    std::shared_ptr<TokenManager> token_manager_;
    std::shared_ptr<SignatureManager> signature_mgr_;
    std::shared_ptr<MqttConnectionManager> conn_;
    TokenResult cached_token_;
};

} // namespace mqtt_module
