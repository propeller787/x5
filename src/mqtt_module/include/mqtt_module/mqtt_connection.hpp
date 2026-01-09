#pragma once
#include <string>
#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <mosquittopp.h>
#include <mosquitto.h>
#include "mqtt_module/token_manager.hpp"
#include <chrono>
#include <thread>
using namespace std::chrono_literals;

namespace mqtt_module {

struct MqttConnConfig {
    std::string broker = "tcp://broker.emqx.io:1883";
    std::string client_id = "ros2_vehicle_client";
    std::string username;
    std::string password;
    bool use_token_auth = true;
    std::string ca_cert_path;
    bool insecure_skip_verify = false;
    std::string tls_version = "TLSv1.2";
    std::string tls_cipher = "ECDHE-RSA-AES256-GCM-SHA384";
};

class MqttConnectionManager : public mosqpp::mosquittopp {
public:
    MqttConnectionManager(const MqttConnConfig& config, rclcpp::Logger logger)
        : mosqpp::mosquittopp(config.client_id.c_str()), config_(config), logger_(logger), connected_(false), loop_running_(false) {
        mosquitto_lib_init();
        init_client();
    }

    ~MqttConnectionManager() {
        disconnect();
        mosquitto_lib_cleanup();
    }

    bool connect(const TokenResult& token = TokenResult{});
    MqttConnectionManager* get_client() { return this; }
    bool is_connected() const { return connected_; }
    void disconnect();
    bool force_reconnect(const TokenResult& token = TokenResult{});
    void set_disconnect_callback(std::function<void(int)> cb) { disconnect_cb_ = std::move(cb); }
    int publish(const std::string& topic, const std::string& payload, int qos = 1);
    int publish_with_properties(const std::string& topic,
                                const std::string& payload,
                                int qos,
                                const std::vector<std::pair<std::string, std::string>>& user_props);
    int subscribe(const std::string& topic, int qos = 1);
    int unsubscribe(const std::string& topic);
    void set_message_callback(std::function<void(const std::string&, const std::string&)> cb) {
        message_cb_ = std::move(cb);
    }

    void on_connect(int rc) override;
    void on_message(const struct mosquitto_message *message) override;
    void on_disconnect(int rc) override;

private:
    void init_client();

    MqttConnConfig config_;
    bool connected_;
    bool loop_running_;
    rclcpp::Logger logger_;
    std::function<void(const std::string&, const std::string&)> message_cb_;
    std::function<void(int)> disconnect_cb_;
};

} // namespace mqtt_module
