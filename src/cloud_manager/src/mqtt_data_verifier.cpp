#include <rclcpp/rclcpp.hpp>
#include <mqtt/async_client.h>
#include <mqtt/callback.h>
#include <mqtt/connect_options.h>
#include <mqtt/message.h>
#include <chrono>
#include <string>
#include <iostream>
#include "car_status.pb.h"
#include "cloud_manager/signature_manager.hpp"

using namespace std::chrono_literals;

// 兼容所有Paho MQTT版本：仅移除reason_code，保留chrono参数
class MqttDataVerifier : public rclcpp::Node, public mqtt::callback {
public:
    MqttDataVerifier() : Node("mqtt_data_verifier"), mqtt::callback() {
        // 支持通过参数注入 broker 地址 与 client_id，便于本地调试
        this->declare_parameter<std::string>("mqtt.broker", "tcp://broker.emqx.io:1883");
        this->declare_parameter<std::string>("mqtt.client_id", "");
        std::string broker = "tcp://broker.emqx.io:1883";
        std::string client_id = "data_verifier_" + std::to_string(this->now().nanoseconds());
        try { broker = this->get_parameter("mqtt.broker").as_string(); } catch(...) {}
        try { std::string cid = this->get_parameter("mqtt.client_id").as_string(); if (!cid.empty()) client_id = cid; } catch(...) {}
        // 使用 create_options 显式指定 MQTT v5（如可用），确保 User Properties 在客户端层被启用
        mqtt::create_options create_opts;
    #if defined(MQTTVERSION_5)
        try { create_opts.set_mqtt_version(MQTTVERSION_5); } catch(...) {}
    #endif
        client_ = std::make_shared<mqtt::async_client>(broker, client_id, create_opts);
        broker_ = broker;
        
        // 设置MQTT连接参数（恢复chrono类型的keep_alive_interval）
        // 并强制使用 MQTT v5，确保 User Properties 在网络层被传递
        auto builder = mqtt::connect_options_builder()
            .clean_session(true)
            .keep_alive_interval(20s);  // 恢复chrono类型，兼容库要求

    #if defined(MQTTVERSION_5)
        builder.mqtt_version(MQTTVERSION_5);
    #endif

        connOpts_ = builder.finalize();

        // 绑定回调函数
        client_->set_callback(*this);

        // 连接MQTT并订阅协议指定的两个主题（info / status）
        connect_mqtt();
        // 仅订阅协议里使用的 topic 路径，避免旧名称如 vehicle/up/realtime 引起混淆
        subscribe_topic("vehicle/up/x5/x001/1/info");
        subscribe_topic("vehicle/up/x5/x001/1/status");

        // 声明签名参数并初始化签名管理器（优先使用注入的密钥）
        this->declare_parameter<std::string>("signature.private_key", "");
        this->declare_parameter<std::string>("signature.public_key", "");
        cloud_manager::SignatureConfig sign_config;
        sign_config.sign_separator = "|";
        try {
            std::string sk = this->get_parameter("signature.private_key").as_string();
            std::string pk = this->get_parameter("signature.public_key").as_string();
            if (!sk.empty()) sign_config.private_key = sk;
            if (!pk.empty()) sign_config.public_key = pk;
        } catch (...) {
            /* ignore */
        }
        // 支持严格模式（生产）：若启用则 enforce_strict=true
        try {
            bool enforce = this->get_parameter("signature.enforce_strict").as_bool();
            sign_config.enforce_strict = enforce;
        } catch (...) {
            sign_config.enforce_strict = false;
        }

        if (sign_config.private_key.empty() && sign_config.public_key.empty()) {
            if (sign_config.enforce_strict) {
                RCLCPP_ERROR(this->get_logger(), "❌ 严格模式启用且未提供签名密钥，节点将继续运行但验签会失败");
            } else {
                sign_config.private_key = cloud_manager::generate_seed_hex();
                RCLCPP_WARN(this->get_logger(), "⚠️ 未提供签名密钥参数，已生成临时 seed（仅测试用）");
            }
        }
        signature_mgr_ = std::make_shared<cloud_manager::SignatureManager>(sign_config, this->get_logger());
        // 是否允许从 payload 中回退解析签名（兼容老协议）
        this->declare_parameter<bool>("signature.allow_payload_fallback", false);
        try { allow_payload_fallback_ = this->get_parameter("signature.allow_payload_fallback").as_bool(); } catch(...) { allow_payload_fallback_ = false; }
    }

    ~MqttDataVerifier() {
        // 断开MQTT连接
        if (client_->is_connected()) {
            try {
                client_->disconnect()->wait_for(2s);  // 用chrono类型
                RCLCPP_INFO(this->get_logger(), "🔌 MQTT验证节点已断开连接");
            } catch (const mqtt::exception& e) {
                RCLCPP_WARN(this->get_logger(), "⚠️ 断开MQTT连接时异常：%s", e.what());
            }
        }
    }

    // 连接MQTT服务器（彻底移除reason_code）
    void connect_mqtt() {
        if (!rclcpp::ok()) return;

        try {
            auto conn_token = client_->connect(connOpts_);
            conn_token->wait_for(5s);  // 等待5秒（chrono类型）
            
            // 仅通过is_complete() + is_connected()判断成功
            if (conn_token->is_complete()) {
                if (client_->is_connected()) {
                    RCLCPP_INFO(this->get_logger(), "✅ MQTT验证节点连接成功（Broker：%s）", broker_.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "❌ MQTT连接失败：令牌完成但客户端未连接");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ MQTT连接超时（5秒）");
            }
        } catch (const mqtt::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ MQTT连接异常：%s", e.what());
        }
    }

    // 订阅MQTT主题（彻底移除reason_code）
    void subscribe_topic(const std::string& topic) {
        if (!client_->is_connected()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ MQTT未连接，跳过订阅");
            return;
        }

        try {
            auto sub_token = client_->subscribe(topic, 0);  // QoS 0
            sub_token->wait_for(2s);  // 等待2秒（chrono类型）
            
            if (sub_token->is_complete()) {
                RCLCPP_INFO(this->get_logger(), "✅ 已订阅主题：%s", topic.c_str());
                subscribed_topic_ = topic;
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 订阅超时（2秒）");
            }
        } catch (const mqtt::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ 订阅异常：%s", e.what());
        }
    }

    // 消息到达回调（兼容所有版本）
    void message_arrived(mqtt::const_message_ptr msg) override {
        if (!msg || !rclcpp::ok()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 收到空消息或节点已停止");
            return;
        }

        // 读取 MQTT v5 User Properties（优先用于验签）
        std::string gtime, gnonce, gsign;
        try {
            const mqtt::properties& props = msg->get_properties();
            for (const auto& p : props) {
                if (p.type() == mqtt::property::USER_PROPERTY) {
                    auto up = mqtt::get<mqtt::string_pair>(p);
                    const std::string& key = std::get<0>(up);
                    const std::string& val = std::get<1>(up);
                    RCLCPP_DEBUG(this->get_logger(), "🔖 UserProperty: %s = %s", key.c_str(), val.c_str());
                    if (key == "gtime") gtime = val;
                    else if (key == "gnonce") gnonce = val;
                    else if (key == "gsign") gsign = val;
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "无法读取 message properties：%s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "\n==================== 接收上报数据 ====================");
        RCLCPP_INFO(this->get_logger(), "📌 主题：%s", msg->get_topic().c_str());
        RCLCPP_INFO(this->get_logger(), "📏 数据总长度：%zu 字节", msg->get_payload().size());

        // 优先使用 user-properties 中的签名信息
        std::string raw_data;
        std::string signature;
        bool has_props = !gsign.empty();
        if (has_props) {
            // 使用 gtime+gnonce+payload 作为签名原文
            raw_data = msg->get_payload_str();
            signature = gsign;
            RCLCPP_INFO(this->get_logger(), "🔍 使用 user-properties 验签 gtime=%s nonce=%s siglen=%zu", gtime.c_str(), gnonce.c_str(), signature.size());
            if (gtime.empty() || gnonce.empty()) {
                RCLCPP_ERROR(this->get_logger(), "❌ user-properties 缺少 gtime/gnonce，无法验签");
                return;
            }
        } else {
            if (!allow_payload_fallback_) {
                RCLCPP_ERROR(this->get_logger(), "❌ 缺少 MQTT v5 user-properties 签名，且未启用 payload 回退，已拒绝处理");
                return;
            }
            // 回退到原先的 payload|signature 格式解析
            std::string signed_data = msg->to_string();
            std::string separator = "|";
            size_t sep_pos = signed_data.find(separator);
            
            if (sep_pos == std::string::npos || sep_pos == 0 || sep_pos == signed_data.size()-1) {
                RCLCPP_ERROR(this->get_logger(), "❌ 数据格式错误：未找到有效分隔符");
                return;
            }

            // 提取原始数据和签名
            raw_data = signed_data.substr(0, sep_pos);
            signature = signed_data.substr(sep_pos + 1);
            RCLCPP_INFO(this->get_logger(), "🔍 使用 payload 回退验签（siglen=%zu）", signature.size());
        }

        // 验签（如果配置了 SignatureManager）
        if (signature_mgr_) {
            std::string sign_source = has_props ? (gtime + gnonce + raw_data) : raw_data;
            if (!signature_mgr_->verify_data(sign_source, signature)) {
                RCLCPP_ERROR(this->get_logger(), "❌ 数据签名验证失败，拒绝处理");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "✅ 数据签名验证通过（props=%s）", has_props ? "yes" : "no");
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 未配置 SignatureManager，跳过验签（不推荐）");
        }

        // 反序列化原始Protobuf数据为 VehicleInfo
        if (raw_data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ 原始数据为空，跳过反序列化");
            return;
        }

        car_status::VehicleInfo pb_msg;
        if (!pb_msg.ParseFromString(raw_data)) {
            RCLCPP_ERROR(this->get_logger(), "❌ 原始数据反序列化失败（格式错误，len=%zu）", raw_data.size());
            return;
        }
        if (!pb_msg.IsInitialized()) {
            RCLCPP_ERROR(this->get_logger(), "❌ 反序列化后消息未初始化完整：%s", pb_msg.InitializationErrorString().c_str());
            return;
        }

        // 打印解析后的关键数据
        RCLCPP_INFO(this->get_logger(), "✅ 数据解析成功：");
        RCLCPP_INFO(this->get_logger(), "  - 型号：%d", pb_msg.base().model());
        RCLCPP_INFO(this->get_logger(), "  - VIN：%s", pb_msg.base().vin().c_str());
        RCLCPP_INFO(this->get_logger(), "  - 时间戳(ms)：%ld", static_cast<long>(pb_msg.base().device_timestamp()));
        RCLCPP_INFO(this->get_logger(), "  - 速度：%.2f km/h", pb_msg.speed_kmh());
        RCLCPP_INFO(this->get_logger(), "  - 剩余里程：%d km", pb_msg.remaining_mileage());
        RCLCPP_INFO(this->get_logger(), "  - 电量：%d %%", pb_msg.battery());
        RCLCPP_INFO(this->get_logger(), "  - 档位：%d", pb_msg.gear());
        RCLCPP_INFO(this->get_logger(), "  - 充电状态：%d", pb_msg.charging_status());
        RCLCPP_INFO(this->get_logger(), "  - 总里程：%.2f m", pb_msg.total_mileage());
        RCLCPP_INFO(this->get_logger(), "======================================================\n");
    }

    // 连接丢失回调（兼容所有版本）
    void connection_lost(const std::string& cause) override {
        RCLCPP_WARN(this->get_logger(), "⚠️ MQTT连接丢失：%s", cause.empty() ? "未知原因" : cause.c_str());
        // 手动重连
        if (rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "🔄 尝试重新连接MQTT...");
            connect_mqtt();
            // 重连后重新订阅
            if (!subscribed_topic_.empty()) {
                subscribe_topic(subscribed_topic_);
            }
        }
    }

    // 交付完成回调（必须实现，避免编译警告）
    void delivery_complete(mqtt::delivery_token_ptr tok) override {
        (void)tok;  // 未使用参数
    }

private:
    std::shared_ptr<mqtt::async_client> client_;  // MQTT客户端
    mqtt::connect_options connOpts_;              // MQTT连接参数
    std::string broker_ = "tcp://broker.emqx.io:1883";  // MQTT服务器地址
    // std::string broker_ = "tcp://localhost:1883";  // MQTT服务器地址
    std::string subscribed_topic_;                // 已订阅的主题
    std::shared_ptr<cloud_manager::SignatureManager> signature_mgr_;  // 签名管理器
    bool allow_payload_fallback_ = false;
};

int main(int argc, char* argv[]) {
    // 初始化Protobuf和ROS2
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    rclcpp::init(argc, argv);

    // 创建并运行验证节点
    auto node = std::make_shared<MqttDataVerifier>();
    RCLCPP_INFO(node->get_logger(), "🚀 MQTT数据验证节点启动成功");
    
    rclcpp::spin(node);

    // 清理资源
    rclcpp::shutdown();
    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}