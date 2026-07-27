#include "mqtt_module/mqtt_connection.hpp"

namespace mqtt_module {

void MqttConnectionManager::init_client() {
    // 创建mosquitto客户端
    mosq_ = mosquitto_new(config_.client_id.c_str(), true, this);
    if (!mosq_) {
        RCLCPP_ERROR(logger_, "❌ 创建mosquitto客户端失败");
        return;
    }

    // 设置回调函数
    // 使用MQTT 5.0的连接回调函数，支持属性传递
    mosquitto_connect_v5_callback_set(mosq_, on_connect_v5_callback);
    // 使用MQTT 5.0的消息回调函数，支持属性传递
    mosquitto_message_v5_callback_set(mosq_, [](struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg, const mosquitto_property* props) {
        // 转换为我们的回调函数，传递属性
        MqttConnectionManager::on_message_callback(mosq, obj, msg, props);
    });
    // 使用MQTT 5.0的断开连接回调函数，支持属性传递
    mosquitto_disconnect_v5_callback_set(mosq_, on_disconnect_v5_callback);
}

void MqttConnectionManager::on_connect_v5_callback(struct mosquitto* mosq, void* obj, int rc, int reason_code, const mosquitto_property* props) {
    auto* conn = static_cast<MqttConnectionManager*>(obj);
    if (rc == MOSQ_ERR_SUCCESS) {
        conn->connected_ = true;
        RCLCPP_INFO(conn->logger_, "✅ MQTT连接成功，Reason Code: %d", reason_code);
        // 打印MQTT 5.0连接属性
        if (props) {
            RCLCPP_INFO(conn->logger_, "📋 MQTT 5.0连接属性");
            const mosquitto_property* prop = props;
            while (prop) {
                int prop_id = mosquitto_property_identifier(prop);
                RCLCPP_INFO(conn->logger_, "  属性ID: %d", prop_id);
                prop = mosquitto_property_next(prop);
            }
        }
    } else {
        conn->connected_ = false;
        RCLCPP_ERROR(conn->logger_, "❌ MQTT连接失败：%s", mosquitto_connack_string(rc));
    }
}

void MqttConnectionManager::on_disconnect_v5_callback(struct mosquitto* mosq, void* obj, int rc, const mosquitto_property* props) {
    auto* conn = static_cast<MqttConnectionManager*>(obj);
    conn->connected_ = false;
    RCLCPP_INFO(conn->logger_, "⏹️ MQTT断开连接 (rc=%d)", rc);
    // 打印MQTT 5.0断开连接属性
    if (props) {
        RCLCPP_INFO(conn->logger_, "📋 MQTT 5.0断开连接属性");
        const mosquitto_property* prop = props;
        while (prop) {
            int prop_id = mosquitto_property_identifier(prop);
            RCLCPP_INFO(conn->logger_, "  属性ID: %d", prop_id);
            prop = mosquitto_property_next(prop);
        }
    }
    if (conn->disconnect_cb_) {
        conn->disconnect_cb_(rc);
    }
}

void MqttConnectionManager::on_message_callback(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg, const mosquitto_property* props) {
    auto* conn = static_cast<MqttConnectionManager*>(obj);
    if (!conn->message_cb_) {
        return;
    }

    std::string topic = msg->topic ? msg->topic : "";
    std::string payload;
    if (msg->payload && msg->payloadlen > 0) {
        payload.assign(static_cast<const char*>(msg->payload), static_cast<size_t>(msg->payloadlen));
    }
    std::vector<std::pair<std::string, std::string>> user_props;
    if (props) {
        RCLCPP_INFO(conn->logger_, "📋 收到MQTT 5.0属性，将尝试提取User Property");
        const mosquitto_property* prop = props;
        while (prop) {
            int prop_id = mosquitto_property_identifier(prop);
            char* key = nullptr;
            char* value = nullptr;
            mosquitto_property_read_string_pair(prop, prop_id, &key, &value, false);
            // 38 是 MQTT 5.0 User Property 的标准数值常量
            if (prop_id == 38 && key && value) {
                user_props.emplace_back(key, value);
                RCLCPP_INFO(conn->logger_, "📋 收到MQTT属性: %s = %s", key, value);
            }
            prop = mosquitto_property_next(prop);
        }
        if (user_props.empty()) {
            RCLCPP_ERROR(conn->logger_, "📋 没有找到User Property");
        }
    }
    conn->message_cb_(topic, payload, user_props);
}

bool MqttConnectionManager::connect(const TokenResult& token) {
    RCLCPP_INFO(logger_, "🔌 开始MQTT连接：%s", config_.broker.c_str());
    if (!config_.username.empty() || !config_.password.empty()) {
        int ret = mosquitto_username_pw_set(mosq_, config_.username.c_str(), config_.password.c_str());
        if (ret != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "❌ 设置用户名/密码失败：%s", mosquitto_strerror(ret));
            return false;
        }
        RCLCPP_INFO(logger_, "🔐 使用用户名/密码认证");
    } else if (config_.use_token_auth && token.success) {
        int ret = mosquitto_username_pw_set(mosq_, config_.client_id.c_str(), token.jwt_token.c_str());
        if (ret != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "❌ 设置token认证失败：%s", mosquitto_strerror(ret));
            return false;
        }
        RCLCPP_INFO(logger_, "🔐 使用token认证");
    }

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

    // 设置MQTT版本为5
    int mqtt_ver = 5;
    int opts_ret = mosquitto_opts_set(mosq_, MOSQ_OPT_PROTOCOL_VERSION, &mqtt_ver);
    if (opts_ret != MOSQ_ERR_SUCCESS) {
        RCLCPP_ERROR(logger_, "❌ MQTT版本设置失败：%s", mosquitto_strerror(opts_ret));
        return false;
    }
    RCLCPP_INFO(logger_, "📋 MQTT版本设置为5");

    if (use_tls) {
        const char* ca_path = nullptr;
        const char* ca_file = nullptr;
        bool tried_no_ca = false;
        if (!config_.ca_cert_path.empty()) {
            ca_file = config_.ca_cert_path.c_str();
        } else if (!config_.insecure_skip_verify) {
            ca_file = "/etc/ssl/certs/ca-certificates.crt";
            ca_path = "/etc/ssl/certs";
        } else {
            tried_no_ca = true;
        }

        int tls_ret = mosquitto_tls_set(mosq_, ca_file, ca_path, NULL, NULL, NULL);
        if (tls_ret == MOSQ_ERR_INVAL && tried_no_ca) {
            ca_file = "/etc/ssl/certs/ca-certificates.crt";
            ca_path = "/etc/ssl/certs";
            tls_ret = mosquitto_tls_set(mosq_, ca_file, ca_path, NULL, NULL, NULL);
        }
        if (tls_ret != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "❌ TLS设置失败：%s", mosquitto_strerror(tls_ret));
            return false;
        }

        int cert_reqs = config_.insecure_skip_verify ? 0 : 1;
        const char* tls_ver = config_.insecure_skip_verify ? NULL : config_.tls_version.c_str();
        const std::string cipher = config_.tls_cipher.empty()
            ? std::string("ECDHE-RSA-AES256-GCM-SHA384")
            : config_.tls_cipher;
        int opts_rc = mosquitto_tls_opts_set(mosq_, cert_reqs, tls_ver, cipher.c_str());
        if (opts_rc != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "❌ TLS选项设置失败：%s", mosquitto_strerror(opts_rc));
            return false;
        }
        if (config_.insecure_skip_verify) {
            int insecure_ret = mosquitto_tls_insecure_set(mosq_, true);
            if (insecure_ret != MOSQ_ERR_SUCCESS) {
                RCLCPP_ERROR(logger_, "❌ TLS不安全设置失败：%s", mosquitto_strerror(insecure_ret));
                return false;
            }
        }
    }

    if (!loop_running_) {
        int loop_rc = mosquitto_loop_start(mosq_);
        if (loop_rc != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "❌ 启动MQTT循环失败：%s", mosquitto_strerror(loop_rc));
            return false;
        }
        loop_running_ = true;
    }

    int ret = mosquitto_connect(mosq_, host.c_str(), port, 30);
    if (ret != MOSQ_ERR_SUCCESS) {
        RCLCPP_ERROR(logger_, "❌ 连接MQTT服务器失败：%s", mosquitto_strerror(ret));
        if (loop_running_) {
            mosquitto_loop_stop(mosq_, true);
            loop_running_ = false;
        }
        return false;
    }

    // 等待连接成功
    int wait_count = 0;
    while (!connected_ && wait_count < 10) {
        std::this_thread::sleep_for(500ms);
        wait_count++;
    }

    return connected_;
}

bool MqttConnectionManager::force_reconnect(const TokenResult& token) {
    disconnect();
    return connect(token);
}

void MqttConnectionManager::disconnect() {
    if (loop_running_) {
        mosquitto_loop_stop(mosq_, true);
        loop_running_ = false;
    }
    connected_ = false;
    mosquitto_disconnect(mosq_);
}

int MqttConnectionManager::publish(const std::string& topic, const std::string& payload, int qos) {
    // 统一走MQTT 5.0接口；不带属性时传递空props
    return mosquitto_publish_v5(
        mosq_,
        nullptr,
        topic.c_str(),
        payload.size(),
        payload.c_str(),
        qos,
        false,
        nullptr);
}

int MqttConnectionManager::publish_with_properties(const std::string& topic,
                                const std::string& payload,
                                int qos,
                                const std::vector<std::pair<std::string, std::string>>& user_props) {
    // 打印MQTT v5属性
    RCLCPP_INFO(logger_, "📋 MQTT v5属性数量: %zu", user_props.size());
    for (const auto& prop : user_props) {
        RCLCPP_INFO(logger_, "  属性: %s = %s", prop.first.c_str(), prop.second.c_str());
    }

    // 构造MQTT 5.0属性
    mosquitto_property* props = nullptr;
    if (!user_props.empty()) {
        for (const auto& [key, value] : user_props) {
            // 添加User Property (38是MQTT 5.0 User Property的标准数值常量)
            if (mosquitto_property_add_string_pair(&props, 38, key.c_str(), value.c_str()) != MOSQ_ERR_SUCCESS) {
                RCLCPP_ERROR(logger_, "❌ 添加MQTT属性失败: %s=%s", key.c_str(), value.c_str());
            }
        }
    }

    // 使用C API的 v5 发布
    int rc = mosquitto_publish_v5(
        mosq_,
        nullptr,
        topic.c_str(),
        payload.size(),
        payload.c_str(),
        qos,
        false,
        props);

    // 释放属性资源
    if (props) {
        mosquitto_property_free_all(&props);
    }

    if (rc == MOSQ_ERR_SUCCESS) {
        RCLCPP_INFO(logger_, "✅ 发布成功（v5），主题=%s, QoS=%d", topic.c_str(), qos);
    } else {
        RCLCPP_ERROR(logger_, "❌ 发布失败（v5）：%s", mosquitto_strerror(rc));
    }

    return rc;
}

int MqttConnectionManager::subscribe(const std::string& topic, int qos) {
    // 使用v5订阅
    return subscribe_v5(topic, qos, nullptr);
}

int MqttConnectionManager::subscribe_v5(const std::string& topic, int qos, mosquitto_property* props) {
    int rc = mosquitto_subscribe_v5(mosq_, nullptr, topic.c_str(), qos, 0, props);
    if (rc == MOSQ_ERR_SUCCESS) {
        RCLCPP_INFO(logger_, "✅ 订阅成功（v5），主题=%s, QoS=%d", topic.c_str(), qos);
    } else {
        RCLCPP_ERROR(logger_, "❌ 订阅失败（v5）：%s", mosquitto_strerror(rc));
    }
    return rc;
}

int MqttConnectionManager::unsubscribe(const std::string& topic) {
    int rc = mosquitto_unsubscribe(mosq_, nullptr, topic.c_str());
    if (rc == MOSQ_ERR_SUCCESS) {
        RCLCPP_INFO(logger_, "✅ 取消订阅成功，主题=%s", topic.c_str());
    } else {
        RCLCPP_ERROR(logger_, "❌ 取消订阅失败：%s", mosquitto_strerror(rc));
    }
    return rc;
}

} // namespace mqtt_module
