#include "mqtt_module/mqtt_connection.hpp"

namespace mqtt_module {

bool MqttConnectionManager::connect(const TokenResult& token) {
    RCLCPP_INFO(logger_, "starting MQTT connect: %s", config_.broker.c_str());

    if (!config_.username.empty() || !config_.password.empty()) {
        username_pw_set(config_.username.c_str(), config_.password.c_str());
        RCLCPP_INFO(logger_, "using username/password auth");
    } else if (config_.use_token_auth && token.success) {
        username_pw_set(config_.client_id.c_str(), token.jwt_token.c_str());
        RCLCPP_INFO(logger_, "using token auth with client_id as username");
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

    int mqtt_ver = 5;
    int opts_ret = opts_set(MOSQ_OPT_PROTOCOL_VERSION, &mqtt_ver);
    if (opts_ret != MOSQ_ERR_SUCCESS) {
        RCLCPP_ERROR(logger_, "MQTT version set failed: %s", mosquitto_strerror(opts_ret));
        return false;
    }

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

        int tls_ret = tls_set(ca_file, ca_path, NULL, NULL, NULL);
        if (tls_ret == MOSQ_ERR_INVAL && tried_no_ca) {
            ca_file = "/etc/ssl/certs/ca-certificates.crt";
            ca_path = "/etc/ssl/certs";
            tls_ret = tls_set(ca_file, ca_path, NULL, NULL, NULL);
        }
        if (tls_ret != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "TLS set failed: %s", mosquitto_strerror(tls_ret));
            return false;
        }

        int cert_reqs = config_.insecure_skip_verify ? 0 : 1;
        const char* tls_ver = config_.insecure_skip_verify ? NULL : config_.tls_version.c_str();
        const std::string cipher = config_.tls_cipher.empty()
            ? std::string("ECDHE-RSA-AES256-GCM-SHA384")
            : config_.tls_cipher;
        int opts_rc = tls_opts_set(cert_reqs, tls_ver, cipher.c_str());
        if (opts_rc != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "TLS opts set failed: %s", mosquitto_strerror(opts_rc));
            return false;
        }
        if (config_.insecure_skip_verify) {
            int insecure_ret = tls_insecure_set(true);
            if (insecure_ret != MOSQ_ERR_SUCCESS) {
                RCLCPP_ERROR(logger_, "TLS insecure set failed: %s", mosquitto_strerror(insecure_ret));
                return false;
            }
        }
    }

    if (!loop_running_) {
        int loop_rc = loop_start();
        if (loop_rc != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(logger_, "loop_start failed: %s", mosquitto_strerror(loop_rc));
            return false;
        }
        loop_running_ = true;
    }

    int ret = mosqpp::mosquittopp::connect(host.c_str(), port, 30);
    if (ret != MOSQ_ERR_SUCCESS) {
        RCLCPP_ERROR(logger_, "connect failed: %s", mosquitto_strerror(ret));
        if (loop_running_) {
            loop_stop();
            loop_running_ = false;
        }
        return false;
    }

    std::this_thread::sleep_for(2s);
    if (!connected_) {
        RCLCPP_ERROR(logger_, "MQTT connect timeout");
        return false;
    }
    return true;
}

void MqttConnectionManager::disconnect() {
    mosqpp::mosquittopp::disconnect();
    if (loop_running_) {
        loop_stop();
        loop_running_ = false;
    }
    connected_ = false;
}

bool MqttConnectionManager::force_reconnect(const TokenResult& token) {
    disconnect();
    return connect(token);
}

int MqttConnectionManager::publish(const std::string& topic, const std::string& payload, int qos) {
    return mosqpp::mosquittopp::publish(NULL, topic.c_str(), payload.size(), payload.c_str(), qos, false);
}

int MqttConnectionManager::publish_with_properties(const std::string& topic,
                                const std::string& payload,
                                int qos,
                                const std::vector<std::pair<std::string, std::string>>& user_props) {
#if defined(MQTT_PROP_USER_PROPERTY)
    mosquitto_property* props = nullptr;
    for (const auto& kv : user_props) {
        mosquitto_property_add_string_pair(&props, MQTT_PROP_USER_PROPERTY, kv.first.c_str(), kv.second.c_str());
    }
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

int MqttConnectionManager::subscribe(const std::string& topic, int qos) {
    return mosqpp::mosquittopp::subscribe(NULL, topic.c_str(), qos);
}

int MqttConnectionManager::unsubscribe(const std::string& topic) {
    return mosqpp::mosquittopp::unsubscribe(NULL, topic.c_str());
}

void MqttConnectionManager::on_connect(int rc) {
    if (rc == 0) {
        connected_ = true;
        RCLCPP_INFO(logger_, "MQTT connected");
    } else {
        connected_ = false;
        RCLCPP_ERROR(logger_, "MQTT connect failed: %s", mosquitto_connack_string(rc));
    }
}

void MqttConnectionManager::on_message(const struct mosquitto_message *message) {
    if (!message_cb_) return;
    std::string topic = message->topic ? message->topic : "";
    std::string payload;
    if (message->payload && message->payloadlen > 0) {
        payload.assign(static_cast<const char*>(message->payload), static_cast<size_t>(message->payloadlen));
    }
    message_cb_(topic, payload);
}

void MqttConnectionManager::on_disconnect(int rc) {
    connected_ = false;
    RCLCPP_INFO(logger_, "MQTT disconnected (%d)", rc);
    if (disconnect_cb_) {
        disconnect_cb_(rc);
    }
}

void MqttConnectionManager::init_client() {}

} // namespace mqtt_module
