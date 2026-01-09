#include <rclcpp/rclcpp.hpp>
#include <mosquittopp.h>
#include <chrono>
#include <string>
#include <random>
#include <sstream>
#include <iomanip>

#include "vehicle_cmd_exe.pb.h"
#include "cloud_manager/signature_manager.hpp"
#include "cloud_manager/mqtt_connection_manager.hpp"

using namespace std::chrono_literals;

class MqttCmdPublisher : public rclcpp::Node {
public:
    MqttCmdPublisher() : Node("mqtt_cmd_publisher") {
        this->declare_parameter<std::string>("mqtt.broker", "tcp://localhost:1883");
        this->declare_parameter<std::string>("mqtt.client_id", "");
        this->declare_parameter<std::string>("mqtt.username", "");
        this->declare_parameter<std::string>("mqtt.password", "");
        this->declare_parameter<std::string>("mqtt.ca_cert", "");
        this->declare_parameter<bool>("mqtt.insecure_skip_verify", true);
        this->declare_parameter<std::string>("mqtt.cipher", "ECDHE-RSA-AES256-GCM-SHA384");
        this->declare_parameter<std::string>("signature.private_key", "");
        this->declare_parameter<std::string>("cmd.vin", "VIN_1234567890");
        this->declare_parameter<std::string>("cmd.cmd_id", "cmd_001");
        this->declare_parameter<std::string>("cmd.cmd", "1,param");
        this->declare_parameter<std::string>("cmd.user_id", 
            "default_user_001");

        std::string broker = this->get_parameter("mqtt.broker").as_string();
        std::string cid = this->get_parameter("mqtt.client_id").as_string();
        std::string username = this->get_parameter("mqtt.username").as_string();
        std::string password = this->get_parameter("mqtt.password").as_string();
        std::string ca_cert = this->get_parameter("mqtt.ca_cert").as_string();
        bool insecure_skip_verify = this->get_parameter("mqtt.insecure_skip_verify").as_bool();
        std::string cipher = this->get_parameter("mqtt.cipher").as_string();
        if (cid.empty()) cid = "mqtt_cmd_publisher_" + std::to_string(this->now().nanoseconds());

        // mosquitto 使用 tcp/ssl 前缀；兼容 mqtt:// mqtts://
        if (broker.rfind("mqtts://", 0) == 0) {
            broker = "ssl://" + broker.substr(8);
        } else if (broker.rfind("mqtt://", 0) == 0) {
            broker = "tcp://" + broker.substr(7);
        }

        cloud_manager::MqttConnConfig mqtt_cfg;
        mqtt_cfg.broker = broker;
        mqtt_cfg.client_id = cid;
        mqtt_cfg.username = username;
        mqtt_cfg.password = password;
        mqtt_cfg.ca_cert_path = ca_cert;
        mqtt_cfg.insecure_skip_verify = insecure_skip_verify;
        mqtt_cfg.tls_cipher = cipher;
        // 若未提供用户名密码，可回退 token（默认 use_token_auth=true）；这里带了用户名/密码则保持账号认证
        if (!mqtt_cfg.username.empty() || !mqtt_cfg.password.empty()) {
            mqtt_cfg.use_token_auth = false;
        }

        mqtt_mgr_ = std::make_shared<cloud_manager::MqttConnectionManager>(mqtt_cfg, this->get_logger());
        cloud_manager::TokenResult empty_token;
        if (!mqtt_mgr_->connect(empty_token)) {
            RCLCPP_FATAL(this->get_logger(), "❌ MQTT连接失败，发布终止");
            rclcpp::shutdown();
            return;
        }

        // signature manager
        cloud_manager::SignatureConfig sign_cfg;
        try {
            std::string sk = this->get_parameter("signature.private_key").as_string();
            if (!sk.empty()) sign_cfg.private_key = sk;
        } catch(...) {}
        if (sign_cfg.private_key.empty()) {
            sign_cfg.private_key = cloud_manager::generate_seed_hex();
            RCLCPP_WARN(this->get_logger(), "⚠️ 未提供签名私钥，已生成临时 seed（仅测试用）");
        }
        signature_mgr_ = std::make_shared<cloud_manager::SignatureManager>(sign_cfg, this->get_logger());

        // build protobuf command
        vehicle::cmd::exe::VehicleCmdExe cmd;
        cmd.set_vin(this->get_parameter("cmd.vin").as_string());
        cmd.set_cmd_id(this->get_parameter("cmd.cmd_id").as_string());
        cmd.set_cmd(this->get_parameter("cmd.cmd").as_string());
        cmd.set_user_id(this->get_parameter("cmd.user_id").as_string());

        std::string raw;
        if (!cmd.SerializeToString(&raw)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Protobuf 序列化失败");
            return;
        }

        // sign raw
        std::string signature = signature_mgr_->sign_data(raw);
        // mosquitto receiver 无 v5 properties 回调，直接用 payload 形式 raw|signature
        std::string payload = raw + "|" + signature;

        int rc = mqtt_mgr_->publish("vehicle/cmd/exe", payload, 1);
        if (rc == MOSQ_ERR_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "📤 已发布指令到 vehicle/cmd/exe（payload 含签名）");
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 发布失败：%s", mosquitto_strerror(rc));
        }

        // disconnect and exit
        try { mqtt_mgr_->disconnect(); } catch(...) {}
        rclcpp::shutdown();
    }

private:
    std::shared_ptr<cloud_manager::MqttConnectionManager> mqtt_mgr_;
    std::shared_ptr<cloud_manager::SignatureManager> signature_mgr_;
};

int main(int argc, char **argv) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MqttCmdPublisher>();
    // node runs its publish in ctor then exits
    (void)node;
    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}
