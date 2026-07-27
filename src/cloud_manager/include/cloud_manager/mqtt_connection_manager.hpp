#pragma once
#include <string>
#include <memory>
#include <functional>
#include <map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <mosquittopp.h>
#include <mosquitto.h>
#include <mqtt_protocol.h>
#include "cloud_manager/token_manager.hpp"
#include <chrono>  // 新增
#include <thread>  // 新增
#include <cstring> // for strlen
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
};

// MQTT连接管理器（使用mosquitto客户端）
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

    // 核心接口：建立连接
    bool connect(const TokenResult& token = TokenResult{}) {
        RCLCPP_INFO(logger_, "🔌 开始MQTT连接：%s", config_.broker.c_str());
        
        // 设置认证
        if (!config_.username.empty() || !config_.password.empty()) {
            mosquitto_username_pw_set(mosq_, config_.username.c_str(), config_.password.c_str());
            RCLCPP_INFO(logger_, "🔐 使用用户名/密码认证：%s/%s", config_.username.c_str(), 
                       config_.password.empty() ? "(empty)" : "***");
        } else if (config_.use_token_auth && token.success) {
            mosquitto_username_pw_set(mosq_, config_.client_id.c_str(), token.jwt_token.c_str());
            RCLCPP_INFO(logger_, "🔐 使用token认证：%s", config_.client_id.c_str());
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

        // 设置MQTT版本为5
        int mqtt_ver = 5;
        int opts_ret = mosquitto_opts_set(mosq_, MOSQ_OPT_PROTOCOL_VERSION, &mqtt_ver);
        if (opts_ret != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "❌ MQTT版本设置失败：%s", mosquitto_strerror(opts_ret));
            return false;
        }
        RCLCPP_INFO(logger_, "📋 MQTT版本设置为5");

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

            int tls_ret = mosquitto_tls_set(mosq_, ca_file, ca_path, NULL, NULL, NULL);
            if (tls_ret == MOSQ_ERR_INVAL && tried_no_ca) {
                // mosquitto 要求提供 CA；退回系统 CA 但仍跳过校验
                ca_file = "/etc/ssl/certs/ca-certificates.crt";
                ca_path = "/etc/ssl/certs";
                RCLCPP_WARN(logger_, "⚠️ tls_set 参数不足，回退加载系统CA（仍跳过校验）");
                tls_ret = mosquitto_tls_set(mosq_, ca_file, ca_path, NULL, NULL, NULL);
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
            int opts_ret = mosquitto_tls_opts_set(mosq_, cert_reqs, tls_ver, cipher.c_str());
            if (opts_ret != MOSQ_ERR_SUCCESS) {
                RCLCPP_ERROR(logger_, "❌ TLS选项设置失败：%s", mosquitto_strerror(opts_ret));
                return false;
            }
            if (config_.insecure_skip_verify) {
                int insecure_ret = mosquitto_tls_insecure_set(mosq_, true);
                if (insecure_ret != MOSQ_ERR_SUCCESS) {
                    RCLCPP_ERROR(logger_, "❌ TLS不安全设置失败：%s", mosquitto_strerror(insecure_ret));
                    return false;
                }
                RCLCPP_WARN(logger_, "⚠️ 已显式跳过证书校验（仅应用于测试环境），cipher=%s", cipher.c_str());
            } else {
                RCLCPP_INFO(logger_, "🔐 TLS校验证书，TLS版本=%s，cipher=%s", config_.tls_version.c_str(), cipher.c_str());
            }
        }

        // 启动循环（防止重复启动）
        if (!loop_running_) {
            int loop_rc = mosquitto_loop_start(mosq_);
            if (loop_rc != MOSQ_ERR_SUCCESS) {
                RCLCPP_ERROR(logger_, "❌ 启动MQTT循环失败：%s", mosquitto_strerror(loop_rc));
                return false;
            }
            loop_running_ = true;
        }

        // 连接
        int ret = mosquitto_connect(mosq_, host.c_str(), port, 30);
        if (ret != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "❌ MQTT连接失败：%s (code: %d)", mosquitto_strerror(ret), ret);
            if (loop_running_) {
                mosquitto_loop_stop(mosq_, false);
                loop_running_ = false;
            }
            return false;
        }
        RCLCPP_INFO(logger_, "📡 connect()调用成功，开始等待连接确认...");

        // 等待连接
        std::this_thread::sleep_for(30s);
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
        mosquitto_disconnect(mosq_);
        if (loop_running_) {
            mosquitto_loop_stop(mosq_, false);
            loop_running_ = false;
        }
        connected_ = false;
    }

    bool force_reconnect(const TokenResult& token = TokenResult{}) {
        disconnect();
        return connect(token);
    }

    void set_disconnect_callback(std::function<void(int)> cb) { disconnect_cb_ = std::move(cb); }

    // 发布消息（统一走MQTT 5.0接口；不带属性时传递空props）
    int publish(const std::string& topic, const std::string& payload, int qos = 1) {
        return mosquitto_publish_v5(
            mosq_,
            nullptr,
            topic.c_str(),
            payload.size(),
            payload.c_str(),
            qos,
            false,
            nullptr
        );
    }

    // 发布消息（MQTT5 User Properties），如不支持V5则回退普通发布
    int publish_with_properties(const std::string& topic,
                                const std::string& payload,
                                int qos,
                                const std::vector<std::pair<std::string, std::string>>& user_props) {
        mosquitto_property* props = nullptr;
        for (const auto& kv : user_props) {
            int add_rc = mosquitto_property_add_string_pair(&props, MQTT_PROP_USER_PROPERTY, kv.first.c_str(), kv.second.c_str());
            if (add_rc != MOSQ_ERR_SUCCESS) {
                RCLCPP_ERROR(logger_, "❌ 添加属性失败：%s=%s，错误：%s", kv.first.c_str(), kv.second.c_str(), mosquitto_strerror(add_rc));
            } else {
                RCLCPP_INFO(logger_, "🧩 添加属性：%s=%s", kv.first.c_str(), kv.second.c_str());
            }
        }
        RCLCPP_INFO(logger_, "📋 计划发布MQTT5属性数量：%d", (int)user_props.size());
        // 验证属性数量（确保非空且键/值未丢失）
        int prop_count = 0;
        const mosquitto_property* p = props;
        while (p) {
            if (mosquitto_property_identifier(p) == MQTT_PROP_USER_PROPERTY) {
                prop_count++;
            }
            p = mosquitto_property_next(p);
        }
        RCLCPP_INFO(logger_, "📋 已添加UserProperty数量：%d（预期%zu）", prop_count, user_props.size());
        if (prop_count == 0) {
            RCLCPP_WARN(logger_, "⚠️ UserProperty计数为0，检查键名与签名值是否为空（例如 gsign/gtime/gnonce）");
        }
        // 进一步打印将要发送的具体属性内容（从props链表读取，确保打印的是实际提交的数据）
        RCLCPP_INFO(logger_, "📦 将发送的UserProperty列表（%d）:", prop_count);
        const mosquitto_property* p2 = props;
        int idx = 0;
        while (p2) {
            if (mosquitto_property_identifier(p2) == MQTT_PROP_USER_PROPERTY) {
                char* key = nullptr;
                char* value = nullptr;
                mosquitto_property_read_string_pair(p2, MQTT_PROP_USER_PROPERTY, &key, &value, false);
                size_t klen = key ? strlen(key) : 0;
                size_t vlen = value ? strlen(value) : 0;
                RCLCPP_INFO(logger_, "  [%d] %s = %s (klen=%zu, vlen=%zu)", ++idx,
                            key ? key : "(null)", value ? value : "(null)", klen, vlen);
                if (key) free(key);
                if (value) free(value);
            }
            p2 = mosquitto_property_next(p2);
        }
        // 使用C API的 v5 发布
        int rc = mosquitto_publish_v5(mosq_,
                                      nullptr,
                                      topic.c_str(),
                                      payload.size(),
                                      payload.c_str(),
                                      qos,
                                      false,
                                      props);
        if (rc == MOSQ_ERR_SUCCESS) {
            RCLCPP_INFO(logger_, "✅ 发布成功（v5），主题=%s, QoS=%d", topic.c_str(), qos);
        } else {
            RCLCPP_ERROR(logger_, "❌ 发布失败（v5）：%s", mosquitto_strerror(rc));
        }
        mosquitto_property_free_all(&props);
        return rc;
    }

    // 订阅接口（供下行指令/OTA使用）
    int subscribe(const std::string& topic, int qos = 1) {
        return mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos);
    }

    // 订阅接口（MQTT5）
    int subscribe_v5(const std::string& topic, int qos = 1, mosquitto_property* props = nullptr) {
        return mosquitto_subscribe_v5(mosq_, nullptr, topic.c_str(), qos, 0, props);
    }

    // 退订接口（OTA动态标签订阅场景）
    int unsubscribe(const std::string& topic) {
        return mosquitto_unsubscribe(mosq_, nullptr, topic.c_str());
    }

    // 设置消息回调（topic, payload, properties）
    void set_message_callback(std::function<void(const std::string&, const std::string&, const std::map<std::string, std::string>&)> cb) {
        message_cb_ = std::move(cb);
    }

    // 设置连接成功回调
    void set_connect_callback(std::function<void()> cb) {
        connect_cb_ = std::move(cb);
    }

    void on_connect(int rc) {
        if (rc == 0) {
            connected_ = true;
            RCLCPP_INFO(logger_, "✅ MQTT连接成功");
            if (connect_cb_) {
                connect_cb_();
            }
        } else {
            connected_ = false;
            RCLCPP_ERROR(logger_, "❌ MQTT连接失败：%s (code: %d)", mosquitto_connack_string(rc), rc);
        }
    }

    void on_message(const struct mosquitto_message *message, const mosquitto_property *props) {
        // ========== 1. 打印最原始的MQTT消息信息（第一手数据） ==========
        RCLCPP_INFO(logger_, "🔍 [底层MQTT回调] 收到原始消息：");
        RCLCPP_INFO(logger_, "  主题：%s", message->topic ? message->topic : "空");
        RCLCPP_INFO(logger_, "  负载长度：%d", message->payloadlen);
        RCLCPP_INFO(logger_, "  QoS：%d", message->qos);
        RCLCPP_INFO(logger_, "  保留位：%s", message->retain ? "是" : "否");
        
        // 打印原始Properties（未做任何处理）
        RCLCPP_INFO(logger_, "  原始Properties链表：%p", props); // 打印指针，判断是否为空
        const mosquitto_property *raw_prop = props;
        int prop_count = 0;
        while (raw_prop) {
            prop_count++;
            int prop_id = mosquitto_property_identifier(raw_prop);
            
            // 只解析UserProperty类型（云端下发的gtime/gnonce/gsign都属于这个类型）
            if (prop_id == MQTT_PROP_USER_PROPERTY) {
                char *key = nullptr, *value = nullptr;
                mosquitto_property_read_string_pair(raw_prop, MQTT_PROP_USER_PROPERTY, &key, &value, false);
                RCLCPP_INFO(logger_, "  原始Property[%d]：ID=%d，Key=[%s]，Value=[%s]",
                            prop_count, prop_id, 
                            key ? key : "空", value ? value : "空");
                // 手动释放Key和Value的内存
                if (key) free(key);
                if (value) free(value);
            } else {
                RCLCPP_INFO(logger_, "  原始Property[%d]：ID=%d，非UserProperty，跳过",
                            prop_count, prop_id);
            }
            // 总是使用mosquitto_property_next前进，避免无限循环
            raw_prop = mosquitto_property_next(raw_prop);
        }
        RCLCPP_INFO(logger_, "  原始UserProperty总数：%d", prop_count);
        // ==============================================================

        if (!message_cb_) return;
        std::string topic = message->topic ? message->topic : "";
        std::string payload;
        if (message->payload && message->payloadlen > 0) {
            payload.assign(static_cast<const char*>(message->payload), static_cast<size_t>(message->payloadlen));
        }
        std::map<std::string, std::string> properties;
        // 解析MQTT 5.0 Properties
        const mosquitto_property *prop = props;
        while (prop) {
            if (mosquitto_property_identifier(prop) == MQTT_PROP_USER_PROPERTY) {
                char *key = nullptr, *value = nullptr;
                mosquitto_property_read_string_pair(prop, MQTT_PROP_USER_PROPERTY, &key, &value, false);
                RCLCPP_INFO(logger_, "  解析Property：Key=[%s]，Value=[%s]", key ? key : "null", value ? value : "null");
                if (key && value && strlen(key) > 0 && strlen(value) > 0) {
                    properties[key] = value;
                    RCLCPP_INFO(logger_, "  添加到map：%s = %s", key, value);
                } else {
                    RCLCPP_INFO(logger_, "  跳过无效Property");
                }
                // 手动释放Key和Value的内存
                if (key) free(key);
                if (value) free(value);
            } else {
                RCLCPP_INFO(logger_, "  跳过非UserProperty：ID=%d", mosquitto_property_identifier(prop));
            }
            // 总是使用mosquitto_property_next前进
            prop = mosquitto_property_next(prop);
        }
        message_cb_(topic, payload, properties);
    }

    void on_disconnect(int rc) {
        connected_ = false;
        RCLCPP_INFO(logger_, "MQTT断开连接 (reason: %d)", rc);
        if (disconnect_cb_) {
            disconnect_cb_(rc);
        }
    }
    // 静态回调包装器
    static void on_connect_wrapper(struct mosquitto *mosq, void *obj, int rc) {
        static_cast<MqttConnectionManager*>(obj)->on_connect(rc);
    }
    static void on_disconnect_wrapper(struct mosquitto *mosq, void *obj, int rc) {
        static_cast<MqttConnectionManager*>(obj)->on_disconnect(rc);
    }
    static void on_message_wrapper(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message, const mosquitto_property *props) {
        static_cast<MqttConnectionManager*>(obj)->on_message(message, props);
    }
private:
    void init_client() {
        mosq_ = mosquitto_new(config_.client_id.c_str(), true, this);
        if (!mosq_) {
            RCLCPP_ERROR(logger_, "❌ 创建mosquitto实例失败");
            return;
        }

        // ========== 强制使用MQTT 5.0协议 ==========
        int mqtt_version = MQTT_PROTOCOL_V5;
        int opts_ret = mosquitto_opts_set(mosq_, MOSQ_OPT_PROTOCOL_VERSION, &mqtt_version);
        if (opts_ret != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "❌ MQTT版本设置失败：%s", mosquitto_strerror(opts_ret));
        } else {
            RCLCPP_INFO(logger_, "📋 MQTT客户端强制设置为5.0协议");
            // 一些libmosquitto版本不提供opts_get，这里不再读取当前版本，仅记录设置成功
        }
        // ==========================================

        mosquitto_connect_callback_set(mosq_, on_connect_wrapper);
        mosquitto_disconnect_callback_set(mosq_, on_disconnect_wrapper);
        mosquitto_message_v5_callback_set(mosq_, on_message_wrapper);
    }

    struct mosquitto *mosq_;
    MqttConnConfig config_;
    bool connected_;
    bool loop_running_;
    rclcpp::Logger logger_;
    std::function<void(const std::string&, const std::string&, const std::map<std::string, std::string>&)> message_cb_;
    std::function<void()> connect_cb_;
    std::function<void(int)> disconnect_cb_;
};

} // namespace cloud_manager
