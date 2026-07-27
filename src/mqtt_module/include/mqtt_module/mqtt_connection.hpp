#pragma once
#include <string>
#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
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

class MqttConnectionManager {
public:
    MqttConnectionManager(const MqttConnConfig& config, rclcpp::Logger logger)
        : config_(config), logger_(logger), connected_(false), loop_running_(false), mosq_(nullptr) {
        mosquitto_lib_init();
        init_client();
    }

    ~MqttConnectionManager() {
        disconnect();
        if (mosq_) {
            mosquitto_destroy(mosq_);
        }
        mosquitto_lib_cleanup();
    }

    bool connect(const TokenResult& token = TokenResult{});
    bool force_reconnect(const TokenResult& token = TokenResult{});
    void disconnect();
    bool is_connected() const { return connected_; }
    void set_disconnect_callback(std::function<void(int)> cb) { disconnect_cb_ = std::move(cb); }
    void set_message_callback(std::function<void(const std::string&, const std::string&, const std::vector<std::pair<std::string, std::string>>&)> cb) {
        message_cb_ = std::move(cb);
    }

    int publish(const std::string& topic, const std::string& payload, int qos = 1);
    int publish_with_properties(const std::string& topic,
                                const std::string& payload,
                                int qos,
                                const std::vector<std::pair<std::string, std::string>>& user_props);
    int subscribe(const std::string& topic, int qos = 1);
    int subscribe_v5(const std::string& topic, int qos = 1, mosquitto_property* props = nullptr);
    int unsubscribe(const std::string& topic);

private:
    void init_client();
    static void on_connect_v5_callback(struct mosquitto* mosq, void* obj, int rc, int reason_code, const mosquitto_property* props);
    static void on_message_callback(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg, const mosquitto_property* props);
    static void on_disconnect_v5_callback(struct mosquitto* mosq, void* obj, int rc, const mosquitto_property* props);

    MqttConnConfig config_;
    rclcpp::Logger logger_;
    struct mosquitto* mosq_;
    bool connected_;
    bool loop_running_;
    std::function<void(int)> disconnect_cb_;
    std::function<void(const std::string&, const std::string&, const std::vector<std::pair<std::string, std::string>>&)> message_cb_;
};

} // namespace mqtt_module
