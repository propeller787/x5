#pragma once
#include <string>
#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <mosquittopp.h>
#include <mosquitto.h>
#include "cloud_manager/token_manager.hpp"
#include <chrono>  // 新增
#include <thread>  // 新增
using namespace std::chrono_literals;  // 新增（启用"s"字面量）
namespace cloud_manager {

// MQTT配置结构体（含TLS与认证）
struct MqttConnConfig {
    // 基础配置（默认占位）
    std::string broker = "tcp://broker.emqx.io:1883";  // 示例测试服务器
    std::string client_id = "ros2_vehicle_client";     // 默认客户端ID

    // 认证
    std::string username;
    std::string password;
    bool use_token_auth = true;                        // 缺省使用 token 认证，可由用户名/密码覆盖

    // TLS 配置
    std::string ca_cert_path;                           // CA 根证书路径
    bool insecure_skip_verify = false;                  // 是否跳过服务器证书校验（仅用于测试）
    std::string tls_version = "TLSv1.2";              // 预留：可按需调整
    std::string tls_cipher = "ECDHE-RSA-AES256-GCM-SHA384"; // 可配置 cipher
    // token 认证用户名（服务端通常固定要求）
    std::string token_username = "token_user";
    // MQTT协议版本：5 或 4(=3.1.1)。默认 5，可通过参数切换。
    int mqtt_version = 5;
};

// MQTT连接管理器（使用mosquitto客户端）
class MqttConnectionManager : public mosqpp::mosquittopp {
public:
    MqttConnectionManager(const MqttConnConfig& config, rclcpp::Logger logger)
        : mosqpp::mosquittopp(config.client_id.c_str()), config_(config), logger_(logger), connected_(false), loop_running_(false) {
        // Use C++ wrapper init/cleanup to match mosquittopp lifecycle
        mosqpp::lib_init();
        init_client();
    }

    ~MqttConnectionManager() {
        disconnect();
        mosqpp::lib_cleanup();
    }

    // 核心接口：建立连接
    bool connect(const TokenResult& token = TokenResult{}) {
        RCLCPP_INFO(logger_, "🔌 开始MQTT连接：%s", config_.broker.c_str());
        
        // 设置认证
        if (!config_.username.empty() || !config_.password.empty()) {
            username_pw_set(config_.username.c_str(), config_.password.c_str());
            RCLCPP_INFO(logger_, "🔐 使用用户名/密码认证：%s/%s", config_.username.c_str(), 
                       config_.password.empty() ? "(empty)" : "***");
        } else if (config_.use_token_auth && token.success) {
            username_pw_set(config_.token_username.c_str(), token.jwt_token.c_str());
            RCLCPP_INFO(logger_, "🔐 使用token认证：%s", config_.token_username.c_str());
        }

        // 解析broker
        std::string host = config_.broker;
        int port = 1883;
        bool use_tls = false;
        if (host.rfind("mqtts://", 0) == 0) {
            host = host.substr(8);
            use_tls = true;
            port = 8883;
        } else if (host.rfind("ssl://", 0) == 0) {
            host = host.substr(6);
            use_tls = true;
            port = 8883;
        } else if (host.rfind("tcp://", 0) == 0) {
            host = host.substr(6);
        }
        size_t colon = host.find(':');
        if (colon != std::string::npos) {
            port = std::stoi(host.substr(colon + 1));
            host = host.substr(0, colon);
        }

        RCLCPP_INFO(logger_, "🌐 连接到：%s:%d (TLS: %s)", host.c_str(), port, use_tls ? "是" : "否");

        // 设置MQTT版本
        int mqtt_ver = config_.mqtt_version;
        int opts_ret = opts_set(MOSQ_OPT_PROTOCOL_VERSION, &mqtt_ver);
        if (opts_ret != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "❌ MQTT版本设置失败：%s", mosquitto_strerror(opts_ret));
            return false;
        }
        RCLCPP_INFO(logger_, "📋 MQTT版本设置为%d", mqtt_ver);

        // 设置TLS
        if (use_tls) {
            // 当跳过校验且未指定CA时，优先尝试不加载CA；如参数错误再回退系统CA
            const char* ca_path = nullptr;
            const char* ca_file = nullptr;
            bool tried_no_ca = false;
            if (!config_.ca_cert_path.empty()) {
                ca_file = config_.ca_cert_path.c_str();
                RCLCPP_INFO(logger_, "🔒 TLS启用，使用自定义CA：%s", config_.ca_cert_path.c_str());
            } else if (!config_.insecure_skip_verify) {
                ca_file = "/etc/ssl/certs/ca-certificates.crt";
                ca_path = "/etc/ssl/certs";
                RCLCPP_INFO(logger_, "🔒 TLS启用，使用系统CA");
            } else {
                tried_no_ca = true;
                RCLCPP_INFO(logger_, "🔒 TLS启用（跳过校验，先不加载CA）");
            }

            int tls_ret = tls_set(ca_file, ca_path, NULL, NULL, NULL);
            if (tls_ret == MOSQ_ERR_INVAL && tried_no_ca) {
                // mosquitto 要求提供 CA；退回系统 CA 但仍跳过校验
                ca_file = "/etc/ssl/certs/ca-certificates.crt";
                ca_path = "/etc/ssl/certs";
                RCLCPP_WARN(logger_, "⚠️ tls_set 参数不足，回退加载系统CA（仍跳过校验）");
                tls_ret = tls_set(ca_file, ca_path, NULL, NULL, NULL);
            }
            if (tls_ret != MOSQ_ERR_SUCCESS) {
                RCLCPP_ERROR(logger_, "❌ TLS设置失败：%s (code: %d)", mosquitto_strerror(tls_ret), tls_ret);
                return false;
            }

            // 按需设置校验级别与 cipher：跳过校验则 cert_reqs=0，不跳过则为1
            int cert_reqs = config_.insecure_skip_verify ? 0 : 1;
            const char* tls_ver = config_.insecure_skip_verify ? NULL : config_.tls_version.c_str();
            const std::string cipher = config_.tls_cipher.empty()
                ? std::string("ECDHE-RSA-AES256-GCM-SHA384")
                : config_.tls_cipher;
            int opts_ret = tls_opts_set(cert_reqs, tls_ver, cipher.c_str());
            if (opts_ret != MOSQ_ERR_SUCCESS) {
                RCLCPP_ERROR(logger_, "❌ TLS选项设置失败：%s", mosquitto_strerror(opts_ret));
                return false;
            }
            if (config_.insecure_skip_verify) {
                int insecure_ret = tls_insecure_set(true);
                if (insecure_ret != MOSQ_ERR_SUCCESS) {
                    RCLCPP_ERROR(logger_, "❌ TLS不安全设置失败：%s", mosquitto_strerror(insecure_ret));
                    return false;
                }
                RCLCPP_WARN(logger_, "⚠️ 已显式跳过证书校验（仅应用于测试环境），cipher=%s", cipher.c_str());
            } else {
                RCLCPP_INFO(logger_, "🔐 TLS校验证书，TLS版本=%s，cipher=%s", config_.tls_version.c_str(), cipher.c_str());
            }
        }

        // 连接（先连接，后启动loop，避免未连接状态下线程竞争导致潜在崩溃）
        int ret = mosqpp::mosquittopp::connect(host.c_str(), port, 30);
        if (ret != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "❌ MQTT连接失败：%s (code: %d)", mosquitto_strerror(ret), ret);
            if (loop_running_) {
                loop_stop();
                loop_running_ = false;
            }
            return false;
        }
        RCLCPP_INFO(logger_, "📡 connect()调用成功，开始等待连接确认...");

        // 启动循环（防止重复启动）
        if (!loop_running_) {
            int loop_rc = loop_start();
            if (loop_rc != MOSQ_ERR_SUCCESS) {
                RCLCPP_ERROR(logger_, "❌ 启动MQTT循环失败：%s", mosquitto_strerror(loop_rc));
                return false;
            }
            loop_running_ = true;
        }

        // 等待连接
        std::this_thread::sleep_for(2s);
        if (!connected_) {
            RCLCPP_ERROR(logger_, "❌ MQTT连接超时");
            return false;
        }

        RCLCPP_INFO(logger_, "✅ MQTT连接成功（服务器：%s:%d）", host.c_str(), port);
        return true;
    }

    // 获取MQTT客户端（兼容接口，返回this）
    MqttConnectionManager* get_client() { return this; }

    // 兼容接口
    bool is_connected() const { return connected_; }
    void disconnect() {
        mosqpp::mosquittopp::disconnect();
        if (loop_running_) {
            loop_stop();
            loop_running_ = false;
        }
        connected_ = false;
    }

    bool force_reconnect(const TokenResult& token = TokenResult{}) {
        disconnect();
        return connect(token);
    }

    void set_disconnect_callback(std::function<void(int)> cb) { disconnect_cb_ = std::move(cb); }

    // 发布消息（兼容Paho接口）
    int publish(const std::string& topic, const std::string& payload, int qos = 1) {
        return mosqpp::mosquittopp::publish(NULL, topic.c_str(), payload.size(), payload.c_str(), qos, false);
    }

    // 发布消息（MQTT5 User Properties），如不支持V5则回退普通发布
    int publish_with_properties(const std::string& topic,
                                const std::string& payload,
                                int qos,
                                const std::vector<std::pair<std::string, std::string>>& user_props) {
#if defined(MQTT_PROP_USER_PROPERTY)
        mosquitto_property* props = nullptr;
        for (const auto& kv : user_props) {
            mosquitto_property_add_string_pair(&props, MQTT_PROP_USER_PROPERTY, kv.first.c_str(), kv.second.c_str());
        }
        // 使用C API的 v5 发布以兼容旧版 C++ wrapper
        int rc = mosquitto_publish_v5(this->m_mosq,
                                      nullptr,
                                      topic.c_str(),
                                      payload.size(),
                                      payload.c_str(),
                                      qos,
                                      false,
                                      props);
        mosquitto_property_free_all(&props);
        return rc;
#else
        (void)user_props;
        return publish(topic, payload, qos);
#endif
    }

    // 订阅接口（供下行指令/OTA使用）
    int subscribe(const std::string& topic, int qos = 1) {
        return mosqpp::mosquittopp::subscribe(NULL, topic.c_str(), qos);
    }

    // 退订接口（OTA动态标签订阅场景）
    int unsubscribe(const std::string& topic) {
        return mosqpp::mosquittopp::unsubscribe(NULL, topic.c_str());
    }

    // 设置消息回调（topic, payload）
    // 设置消息回调（带属性）
    void set_message_callback(std::function<void(const std::string&, const std::string&)> cb) {
        message_cb_ = std::move(cb);
    }
    void set_message_callback_with_props(std::function<void(const std::string&, const std::string&, const mosquitto_property*)> cb) {
        message_prop_cb_ = std::move(cb);
    }

    void on_connect(int rc) override {
        if (rc == 0) {
            connected_ = true;
            RCLCPP_INFO(logger_, "✅ MQTT连接成功");
        } else {
            connected_ = false;
            RCLCPP_ERROR(logger_, "❌ MQTT连接失败：%s (code: %d)", mosquitto_connack_string(rc), rc);
        }
    }

    void on_message(const struct mosquitto_message *message) override {
        std::string topic = message->topic ? message->topic : "";
        std::string payload;
        if (message->payload && message->payloadlen > 0) {
            payload.assign(static_cast<const char*>(message->payload), static_cast<size_t>(message->payloadlen));
        }

        // mosquitto < 1.6 无 properties 字段，使用编译期宏安全探测
#if defined(MQTT_PROP_USER_PROPERTY) && defined(LIBMOSQUITTO_VERSION_NUMBER) && (LIBMOSQUITTO_VERSION_NUMBER >= 1006000)
        const mosquitto_property* props = message ? message->properties : nullptr;
#else
        const mosquitto_property* props = nullptr;
#endif

        if (message_prop_cb_) {
            message_prop_cb_(topic, payload, props);
        } else if (message_cb_) {
            message_cb_(topic, payload);
        }
    }

    void on_disconnect(int rc) override {
        connected_ = false;
        RCLCPP_INFO(logger_, "MQTT断开连接 (reason: %d)", rc);
        if (disconnect_cb_) {
            disconnect_cb_(rc);
        }
    }

private:
    void init_client() {
        // 设置回调（继承自mosquittopp）
    }

    MqttConnConfig config_;
    bool connected_;
    bool loop_running_;
    rclcpp::Logger logger_;
    std::function<void(const std::string&, const std::string&)> message_cb_;
    std::function<void(const std::string&, const std::string&, const mosquitto_property*)> message_prop_cb_;
    std::function<void(int)> disconnect_cb_;
};

} // namespace cloud_manager
