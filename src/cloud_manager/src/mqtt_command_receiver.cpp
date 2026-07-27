#include <rclcpp/rclcpp.hpp>
#include <mosquittopp.h>
#include <chrono>
#include <functional>
#include <string>
#include <memory>
#include <ctime>
#include <cstdio>  // 新增：snprintf所需头文件
#include <random>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <iterator>
#include <fstream>
#include <algorithm>  // 新增：std::transform所需头文件
#include <map>         // 新增：std::map所需头文件
#include <sodium.h>

#include "car_status.pb.h"
#include "std_msgs/msg/string.hpp"

// 适配interface包的proto头文件（不修改interface，直接引用其生成的路径）
#include "vehicle_cmd_exe.pb.h"
#include "vehicle_cmd_ack.pb.h"

#include "cloud_manager/token_manager.hpp"
#include "cloud_manager/signature_manager.hpp"
#include "cloud_manager/mqtt_connection_manager.hpp"

using namespace std::chrono_literals;

namespace mqtt_command_receiver {

// ========== 1. 简化指令结构（仅保留核心字段） ==========
struct CarCommand {
    std::string cmd_id_str;      // 指令唯一ID（Proto的cmd_id）
    int32_t cmd_type = 0;        // 指令类型
    std::string cmd_param = "";  // 指令参数
    std::string user_id;         // 用户ID
    std::string vin;             // 车辆VIN
};

struct CommandResult {
    std::string cmd_id_str;      // 指令唯一ID
    int32_t result_code = 0;     // 结果码（0成功/1失败/2执行中）
    std::string result_msg = ""; // 结果描述
    std::string user_id;         // 用户ID
    std::string vin;             // 车辆VIN
};

// ========== 2. Protobuf协议适配工具类（解耦核心） ==========
class CmdProtoAdapter {
public:
    // 设备基础信息（生产环境从配置/硬件读取）
    static const int32_t MODEL;
    static const std::string DEFAULT_USER_ID;

    // 将Protobuf指令解析为本地CarCommand
    static bool parse_proto_cmd(const std::string& raw_proto_data, const std::string& local_vin, CarCommand& cmd) {
        vehicle::cmd::exe::VehicleCmdExe proto_cmd;
        if (!proto_cmd.ParseFromString(raw_proto_data)) {
            RCLCPP_ERROR(rclcpp::get_logger("CmdProtoAdapter"), "❌ Protobuf指令解析失败");
            return false;
        }

        // 校验必填字段
        if (proto_cmd.sn().empty() || proto_cmd.sn() != local_vin) {
            RCLCPP_ERROR(rclcpp::get_logger("CmdProtoAdapter"), "❌ SN不匹配：接收=%s，本地=%s", proto_cmd.sn().c_str(), local_vin.c_str());
            return false;
        }
        if (proto_cmd.cmd_id().empty() || proto_cmd.cmd().empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("CmdProtoAdapter"), "❌ 指令缺少必填字段（cmd_id/cmd）");
            return false;
        }

        // 解析为本地指令（cmd字段格式："cmd_type,param"）
        size_t sep_pos = proto_cmd.cmd().find(',');
        if (sep_pos == std::string::npos) {
            RCLCPP_ERROR(rclcpp::get_logger("CmdProtoAdapter"), "❌ cmd格式错误：%s（正确：cmd_type,param）", proto_cmd.cmd().c_str());
            return false;
        }

        try {
            cmd.cmd_type = std::stoi(proto_cmd.cmd().substr(0, sep_pos));
            cmd.cmd_param = proto_cmd.cmd().substr(sep_pos + 1);
            cmd.cmd_id_str = proto_cmd.cmd_id();
            cmd.user_id = proto_cmd.user_id().empty() ? DEFAULT_USER_ID : proto_cmd.user_id();
            cmd.vin = proto_cmd.sn();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("CmdProtoAdapter"), "❌ 指令参数解析失败：%s", e.what());
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("CmdProtoAdapter"), "✅ Protobuf指令解析成功：sn=%s, cmd_id=%s, cmd_type=%d",
                    proto_cmd.sn().c_str(), proto_cmd.cmd_id().c_str(), cmd.cmd_type);
        return true;
    }

    // 将执行结果封装为Protobuf回执（字段严格对齐最新proto）
    static std::string pack_ack_proto(const CommandResult& result) {
        vehicle::cmd::ack::VehicleCmdAck ack;
        // 字段编号和类型严格对齐proto
        // sn = 1
        ack.set_sn(result.vin.empty() ? "ZQKJ001" : result.vin); // 车辆唯一标识
        // model = 2
        ack.set_model("X5"); // 可模拟 "X5" 或 "Cruise"
        // timestamp = 3
        ack.set_timestamp(get_current_timestamp_ms());
        // user_id = 4
        ack.set_user_id(result.user_id.empty() ? "default_user_001" : result.user_id);
        // cmd_id = 5
        ack.set_cmd_id(result.cmd_id_str.empty() ? "unknown" : result.cmd_id_str);
        // status = 6
        if (result.result_code == 0) {
            ack.set_status(0); // 成功
        } else if (result.result_code == 2) {
            ack.set_status(2); // 执行中
        } else {
            ack.set_status(1); // 失败
        }
        // error_code = 10
        ack.set_error_code(result.result_code == 1 ? 1 : 0); // 可模拟 1=失败
        // error_message = 11
        ack.set_error_message(result.result_msg.empty() ? "云端模拟错误" : result.result_msg);

        // 序列化为字符串
        std::string ack_data;
        if (!ack.SerializeToString(&ack_data)) {
            RCLCPP_ERROR(rclcpp::get_logger("CmdProtoAdapter"), "❌ 回执Protobuf序列化失败");
            return "";
        }
        return ack_data;
    }

private:
    // 获取当前毫秒时间戳
    static int64_t get_current_timestamp_ms() {
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
        return ms.time_since_epoch().count();
    }
};

// 静态常量初始化（生产环境可从配置读取）
const int32_t CmdProtoAdapter::MODEL = 0; // 0:X5、1:Cruise
const std::string CmdProtoAdapter::DEFAULT_USER_ID = "default_user_001";

// ========== 4. 核心节点类（移除原有执行逻辑，改为主题发布解耦） ==========
class MqttCommandReceiverNode : public rclcpp::Node {
    // 保存最近一次 EXE 指令字段，供 ACK 用
    std::string last_exe_sn_;
    std::string last_exe_model_;
    int64_t last_exe_timestamp_ = 0;
    std::string last_exe_user_id_;
    std::string last_exe_cmd_id_;
public:
    MqttCommandReceiverNode() : Node("mqtt_command_receiver") {
        this->get_logger().set_level(rclcpp::Logger::Level::Info);

        // 确保运行时使用 UTF-8 编码
        const char* locale_res = std::setlocale(LC_ALL, "C.UTF-8");
        if (!locale_res) {
            locale_res = std::setlocale(LC_ALL, "en_US.UTF-8");
        }
        if (!locale_res) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 未能设置 UTF-8 locale，字符串编码需要外部保证为 UTF-8");
        }

        // ========== 1. 声明与加载参数 ==========
        this->declare_parameter<std::string>("mqtt.broker", "mqtts://39.106.140.222:28883");
        this->declare_parameter<std::string>("mqtt.client_id", "ZQKJ001");
        this->declare_parameter<std::string>("mqtt.username", "");
        this->declare_parameter<std::string>("mqtt.password", "");
        this->declare_parameter<std::string>("mqtt.ca_cert", "");
        this->declare_parameter<bool>("mqtt.insecure_skip_verify", true);
        this->declare_parameter<std::string>("mqtt.cipher", "ECDHE-RSA-AES256-GCM-SHA384");
        this->declare_parameter<int>("mqtt.retry_attempts", 3);
        this->declare_parameter<double>("mqtt.retry_delay_base", 2.0);

        this->declare_parameter<std::string>("signature.private_key", "");
        this->declare_parameter<std::string>("signature.public_key", "");
        this->declare_parameter<bool>("signature.allow_payload_fallback", true);
        this->declare_parameter<bool>("signature.embed_payload_fallback", false);

        this->declare_parameter<std::string>("device.type", "FREEGOX5Air");
        this->declare_parameter<std::string>("device.id", "ZQKJ001");
        this->declare_parameter<std::string>("protocol.version", "1");

        // ========== 2. 初始化核心组件 ==========
        init_token_manager();    
        init_signature_manager();
        init_ack_signing_key();
        load_topic_config();  // 先加载主题配置
        init_mqtt_connection();

        if (!mqtt_conn_mgr_ || !mqtt_conn_mgr_->is_connected()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 初始连接未成功，节点将继续运行并等待自动重连...");
        }

        // 定期重连/令牌刷新定时器（对齐 cloud_manager_node）
        mqtt_reconnect_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(mqtt_reconnect_interval_sec_),
            [this]() { this->ensure_mqtt_connected(); }
        );
        if (using_token_auth_) {
            token_refresh_timer_ = this->create_wall_timer(
                30s,
                [this]() { this->maybe_refresh_token(); }
            );
        }

        // ========== 3. 创建与配置发布器 ==========
        cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            "vehicle/control/cmd", 10
        );

        load_topic_config();

        // ========== 4. 订阅主题 ==========
        try {
            allow_payload_fallback_ = this->get_parameter("signature.allow_payload_fallback").as_bool();
        } catch(...) {
            allow_payload_fallback_ = true;
        }
        try {
            // 统一使用 allow_payload_fallback_ 作为兜底开关
            allow_payload_fallback_ = this->get_parameter("signature.embed_payload_fallback").as_bool();
        } catch(...) {
            allow_payload_fallback_ = false;
        }
        
        // 订阅MQTT下行主题
        subscribe_command_topic(cmd_topic_);

        // 订阅车辆ID变动主题（对齐 cloud_manager_node）
        vehicle_id_sub_ = this->create_subscription<std_msgs::msg::String>(
            "vehicle/car_id", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "🚗 车辆ID更新: %s", msg->data.c_str());
                device_info_.vin = msg->data;
            }
        );

        RCLCPP_INFO(this->get_logger(), "🚀 MQTT指令接收节点初始化完成（指令已解耦为ROS2主题发布，启用令牌验证+验签）");
    }

    ~MqttCommandReceiverNode() {
        if (mqtt_conn_mgr_ && mqtt_conn_mgr_->is_connected()) {
            try {
                mqtt_conn_mgr_->disconnect();
                RCLCPP_INFO(this->get_logger(), "🔌 MQTT连接断开");
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "断开MQTT连接异常：%s", e.what());
            }
        }
    }

private:
    // ========== 保留原有初始化方法（完全不修改） ==========
    void init_token_manager() {
        std::string device_type = this->get_parameter("device.type").as_string();
        std::string device_id = this->get_parameter("device.id").as_string();

        device_info_.vin = device_id;  // 使用参数中的 device.id 作为 VIN
        device_info_.model = (device_type == "FREEGOX5Air") ? "FREEGO X5 Air" : "FREEGO Cruise";
        
        // 设置车云接口协议的根密钥信息（与 HTTP 接口加签保持一致）
        try {
            auto param_gkey = this->get_parameter("token.root_key_no").as_string();
            if (!param_gkey.empty()) {
                device_info_.gkey_no = param_gkey;
                RCLCPP_INFO(this->get_logger(), "🔑 从参数加载 gkey_no：%s", device_info_.gkey_no.c_str());
            }
        } catch (...) {
            // ignore and fall back to default
        }
        if (device_info_.gkey_no.empty()) {
            device_info_.gkey_no = "2;ZQKJ001;3";  // 与云端约定格式一致
            RCLCPP_WARN(this->get_logger(), "⚠️ 未提供 token.root_key_no，使用默认 gkey_no=%s", device_info_.gkey_no.c_str());
        }
        device_info_.root_private_key = "ydSKNkOGdKx0ITtDp2GtTN9MChiDo6SQXSSRRusli0U=";
        device_info_.root_public_key = "p1c0dmOw6enDGkks5nwuwmr7mMpQm3xfDjrSDl7R0dg=";
        device_info_.api_base_url = "http://39.107.107.204:22000";
        device_info_.client = "vehicle";
        device_info_.app_version = "1.0";
        device_info_.language = "zh-cn";
        device_info_.version = "1";
        
        // 获取参数注入的私钥
        std::string param_sk = this->get_parameter("signature.private_key").as_string();
        if (!param_sk.empty()) {
            device_info_.private_key = param_sk;
            RCLCPP_INFO(this->get_logger(), "🔑 从参数加载设备私钥");
        } else {
            // 默认使用车云根私钥（Base64，32-byte seed），保证回执可用真实加签
            device_info_.private_key = "ydSKNkOGdKx0ITtDp2GtTN9MChiDo6SQXSSRRusli0U=";
            RCLCPP_WARN(this->get_logger(), "⚠️ 未提供 signature.private_key，已回退到车云默认私钥（仅测试环境使用）");
        }

        token_mgr_ = std::make_shared<cloud_manager::TokenManager>();

        // 获取会话密钥（设备开机或重置后获取随机会话密钥对）
        auto session_res = token_mgr_->get_session_key(device_info_);
        if (session_res.success) {
            device_info_.session_private_key = session_res.private_key;
            device_info_.session_public_key = session_res.public_key;
            device_info_.session_key_no = session_res.key_no;
            RCLCPP_INFO(this->get_logger(), "✅ 会话密钥获取成功（key_no: %s）", session_res.key_no.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 会话密钥获取失败，将回退根密钥: %s", session_res.err_msg.c_str());
        }

        token_result_ = token_mgr_->get_trusted_token(device_info_);
        if (token_result_.success) {
            update_token_expiry(token_result_);
            using_token_auth_ = true;
            RCLCPP_INFO(this->get_logger(), "✅ JWT令牌获取成功：%s", token_result_.jwt_token.substr(0, 20).c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 获取JWT令牌失败：%s", token_result_.err_msg.c_str());
        }
    }

    // 生成UUID（模拟云端UUIDUtils.UUID()，返回无横线的32位UUID）
    std::string generate_uuid() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(0, 15);

        std::stringstream ss;
        for (int i = 0; i < 32; ++i) {
            if (i == 8 || i == 12 || i == 16 || i == 20) {
                // 跳过云端UUID的横线，直接生成32位无分隔符
                continue;
            }
            ss << std::hex << dis(gen);
        }
        return ss.str();
    }

    std::string generate_base36_nonce(size_t len = 8) {
        static const char alphabet[] = "0123456789abcdefghijklmnopqrstuvwxyz";
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(0, 35);

        std::string out;
        out.reserve(len);
        for (size_t i = 0; i < len; ++i) {
            out.push_back(alphabet[dis(gen)]);
        }
        return out;
    }

    int64_t now_ms() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    void update_token_expiry(const cloud_manager::TokenResult& token) {
        int64_t fallback = 3600000; // 1h 默认
        token_expire_at_ms_ = now_ms() + (token.exp > 0 ? token.exp : fallback);
    }

    bool ensure_mqtt_connected(bool force_refresh_token = false) {
        if (!mqtt_conn_mgr_) return false;
        if (mqtt_conn_mgr_->is_connected() && !force_refresh_token) {
            RCLCPP_INFO(this->get_logger(), "✅ MQTT已连接，无需重连");
            return true;
        }

        // 暂时去掉重连逻辑，确保一直在线
        RCLCPP_WARN(this->get_logger(), "⚠️ MQTT未连接，暂时不进行重连");
        return false;
    }

    void maybe_refresh_token() {
        if (!using_token_auth_) return;
        if (refreshing_token_.exchange(true)) return;

        if (token_expire_at_ms_ <= 0) {
            refreshing_token_ = false;
            return;
        }
        int64_t remain = token_expire_at_ms_ - now_ms();
        if (remain > token_refresh_margin_ms_) {
            refreshing_token_ = false;
            return;
        }

        // 使用刷新令牌接口
        auto refreshed = token_mgr_->refresh_jwt_token(device_info_, token_result_.jwt_token);
        if (refreshed.success) {
            token_result_ = refreshed;
            update_token_expiry(refreshed);
            RCLCPP_INFO(this->get_logger(), "🔄 令牌已刷新，重新连接MQTT");
            ensure_mqtt_connected(true);
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 令牌自动刷新失败：%s，尝试重新获取令牌", refreshed.err_msg.c_str());
            auto new_token = token_mgr_->get_trusted_token(device_info_);
            if (new_token.success) {
                token_result_ = new_token;
                update_token_expiry(new_token);
                RCLCPP_INFO(this->get_logger(), "🔄 令牌重新获取成功，重新连接MQTT");
                ensure_mqtt_connected(true);
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 令牌重新获取失败：%s", new_token.err_msg.c_str());
            }
        }
        refreshing_token_ = false;
    }

    void on_mqtt_disconnected(int rc) {
        RCLCPP_WARN(this->get_logger(), "⚠️ MQTT断开，rc=%d，计划自动重连", rc);
    }

    void init_signature_manager() {
        cloud_manager::SignatureConfig sign_config;
        sign_config.sign_separator = "|";
        
        // 使用根公钥进行验签（云端使用根私钥签名）
        if (!device_info_.root_public_key.empty()) {
            sign_config.public_key = device_info_.root_public_key;
            RCLCPP_INFO(this->get_logger(), "🔐 SignatureManager 使用根公钥进行验签：%s", device_info_.root_public_key.substr(0,20).c_str());
        } else {
            // 回退到参数注入的密钥
            try {
                std::string pk = this->get_parameter("signature.public_key").as_string();
                if (!pk.empty()) sign_config.public_key = pk;
            } catch (...) {}
        }

        if (sign_config.public_key.empty()) {
            sign_config.public_key = cloud_manager::generate_seed_hex();
            RCLCPP_WARN(this->get_logger(), "⚠️ 未提供验签公钥，已生成临时 seed（不推荐生产使用）");
        }
        
        signature_mgr_ = std::make_shared<cloud_manager::SignatureManager>(sign_config, this->get_logger());
    }

    void init_ack_signing_key() {
        std::string raw_private = device_info_.private_key;
        if (raw_private.empty()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 未提供设备私钥，回执加签将回退为兼容逻辑（mock/公钥签名器）");
            return;
        }

        std::vector<unsigned char> key_bytes;
        if (!cloud_manager::hex_to_bytes(raw_private, key_bytes) &&
            !cloud_manager::b64_to_bytes(raw_private, key_bytes)) {
            RCLCPP_ERROR(this->get_logger(), "❌ 设备私钥解析失败（需 hex 或 Base64）");
            return;
        }

        if (key_bytes.size() == crypto_sign_SECRETKEYBYTES) {
            ack_sign_secret_key_ = key_bytes;
        } else if (key_bytes.size() == crypto_sign_SEEDBYTES) {
            ack_sign_secret_key_.resize(crypto_sign_SECRETKEYBYTES);
            std::vector<unsigned char> pub_tmp(crypto_sign_PUBLICKEYBYTES);
            if (crypto_sign_seed_keypair(pub_tmp.data(), ack_sign_secret_key_.data(), key_bytes.data()) != 0) {
                RCLCPP_ERROR(this->get_logger(), "❌ 基于 seed 生成 Ed25519 私钥失败");
                ack_sign_secret_key_.clear();
                return;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ 设备私钥长度错误，期望 %d 或 %d bytes，实际 %zu",
                         crypto_sign_SECRETKEYBYTES, crypto_sign_SEEDBYTES, key_bytes.size());
            return;
        }

        ack_sign_key_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "✅ 已加载回执签名私钥，长度=%zu bytes", ack_sign_secret_key_.size());
    }

    void init_mqtt_connection() {
        cloud_manager::MqttConnConfig mqtt_cfg;
        mqtt_cfg.broker = this->get_parameter("mqtt.broker").as_string();
        mqtt_cfg.client_id = this->get_parameter("mqtt.client_id").as_string();
        mqtt_cfg.username = this->get_parameter("mqtt.username").as_string();
        mqtt_cfg.password = this->get_parameter("mqtt.password").as_string();
        mqtt_cfg.ca_cert_path = this->get_parameter("mqtt.ca_cert").as_string();
        mqtt_cfg.insecure_skip_verify = this->get_parameter("mqtt.insecure_skip_verify").as_bool();
        mqtt_cfg.tls_cipher = this->get_parameter("mqtt.cipher").as_string();
        
        if (mqtt_cfg.username.empty() && mqtt_cfg.password.empty()) {
            mqtt_cfg.use_token_auth = true;
        }

        int retry_attempts = this->get_parameter("mqtt.retry_attempts").as_int();
        double retry_delay_base = this->get_parameter("mqtt.retry_delay_base").as_double();

        // 统一前缀处理
        if (mqtt_cfg.broker.rfind("mqtts://", 0) == 0) {
            mqtt_cfg.broker = "ssl://" + mqtt_cfg.broker.substr(8);
        } else if (mqtt_cfg.broker.rfind("mqtt://", 0) == 0) {
            mqtt_cfg.broker = "tcp://" + mqtt_cfg.broker.substr(7);
        }

        mqtt_conn_mgr_ = std::make_shared<cloud_manager::MqttConnectionManager>(mqtt_cfg, this->get_logger());

        // 在连接发起前订阅主题
        subscribe_command_topic(cmd_topic_);

        bool conn_ok = false;
        cloud_manager::TokenResult empty_token;
        for (int attempt = 0; attempt < retry_attempts; ++attempt) {
            const auto& token_to_use = mqtt_cfg.use_token_auth ? token_result_ : empty_token;
            if (mqtt_conn_mgr_->connect(token_to_use)) {
                conn_ok = true;
                break;
            }
            if (attempt < retry_attempts - 1) {
                double delay = retry_delay_base * std::pow(2.0, attempt);
                RCLCPP_WARN(this->get_logger(), "🔄 MQTT连接失败，重试 %d/%d，等待 %.1f 秒...", attempt + 1, retry_attempts, delay);
                std::this_thread::sleep_for(std::chrono::duration<double>(delay));
            }
        }

        if (conn_ok) {
            mqtt_conn_mgr_->set_message_callback([this](const std::string& topic, const std::string& payload, const std::map<std::string, std::string>& properties){
                handle_mqtt_message(topic, payload, properties);
            });

            // 设置连接成功回调，在连接成功后订阅主题
            mqtt_conn_mgr_->set_connect_callback([this]() {
                RCLCPP_INFO(this->get_logger(), "🔗 MQTT连接成功回调被调用，开始订阅通配符主题: %s", wildcard_topic_.c_str());
                subscribe_command_topic(wildcard_topic_);
            });
        }
    }

    void load_topic_config() {
        try { device_type_ = this->get_parameter("device.type").as_string(); } catch (...) {}
        try { device_id_ = this->get_parameter("device.id").as_string(); } catch (...) {}
        try { protocol_version_ = this->get_parameter("protocol.version").as_string(); } catch (...) {}

        cmd_topic_ = "vehicle/cmd/" + device_type_ + "/" + device_id_ + "/" + protocol_version_ + "/exe";
        ack_topic_ = "vehicle/cmd/" + device_type_ + "/" + device_id_ + "/" + protocol_version_ + "/ack";

        // 订阅具体的指令主题
        wildcard_topic_ = cmd_topic_;

        RCLCPP_INFO(this->get_logger(), "MQTT 指令主题：%s，回执主题：%s，通配符主题：%s", cmd_topic_.c_str(), ack_topic_.c_str(), wildcard_topic_.c_str());
    }

    void subscribe_command_topic(const std::string& topic) {
        try {
            int rc = mqtt_conn_mgr_->subscribe_v5(topic, 1);
            RCLCPP_INFO(this->get_logger(), "🔍 订阅MQTT主题 %s，返回码：%d", topic.c_str(), rc);
            if (rc == MOSQ_ERR_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "✅ 订阅MQTT指令主题成功：%s", topic.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 订阅MQTT指令主题失败：%s (code: %d)", mosquitto_strerror(rc), rc);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "订阅MQTT主题异常：%s", e.what());
        }
    }

    void handle_mqtt_message(const std::string& topic, const std::string& payload, const std::map<std::string, std::string>& properties) {
        // ========== 2. 打印上层接收到的Properties（传递后的） ==========
        RCLCPP_INFO(this->get_logger(), "🔍 [上层处理] 接收到MQTT消息：");
        RCLCPP_INFO(this->get_logger(), "  主题：%s", topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  负载长度：%zu", payload.size());
        RCLCPP_INFO(this->get_logger(), "  传递后的Properties数量：%zu", properties.size());
        
        // 打印传递后的每个Property（原始Key，不转小写）
        int idx = 0;
        for (const auto& [k, v] : properties) {
            idx++;
            RCLCPP_INFO(this->get_logger(), "  传递后的Property[%d]：Key=[%s]（长度=%zu），Value=[%s]（长度=%zu）",
                        idx, k.c_str(), k.size(), v.c_str(), v.size());
        }
        // ==============================================================

        RCLCPP_INFO(this->get_logger(), "📥 收到MQTT消息，主题：%s，负载长度：%zu", topic.c_str(), payload.size());
        
        // ========== 新增：打印所有Properties ==========
        RCLCPP_INFO(this->get_logger(), "📋 收到的MQTT 5.0 Properties数量：%zu", properties.size());
        for (const auto& [k, v] : properties) {
            RCLCPP_INFO(this->get_logger(), "  Property: %s = %s", k.c_str(), v.c_str());
        }
        // ==============================================

        // 注意：现在订阅具体主题，所以检查主题
        if (topic != cmd_topic_) return;
        RCLCPP_INFO(this->get_logger(), "📥 收到MQTT消息，主题：%s", topic.c_str());
        std::string raw_payload = payload;
        std::string gsign, gtime, gnonce;

        // ========== 修复1：兼容Key大小写读取Properties ==========
        RCLCPP_INFO(this->get_logger(), "🔍 开始解析Properties...");
        for (const auto& [k, v] : properties) {
            std::string key_lower = k;
            std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(), ::tolower);
            RCLCPP_INFO(this->get_logger(), "  检查属性：原始'%s' -> 小写'%s'", k.c_str(), key_lower.c_str());
            if (key_lower == "gsign") {
                gsign = v;
                RCLCPP_INFO(this->get_logger(), "  ✅ 找到gsign: %s", v.substr(0,20).c_str());
            } else if (key_lower == "gtime") {
                gtime = v;
                RCLCPP_INFO(this->get_logger(), "  ✅ 找到gtime: %s", v.c_str());
            } else if (key_lower == "gnonce") {
                gnonce = v;
                RCLCPP_INFO(this->get_logger(), "  ✅ 找到gnonce: %s", v.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "  ⚠️ 未知属性: %s = %s", k.c_str(), v.c_str());
            }
        }

        // ========== 兜底：从payload中解析（兼容云端错误） ==========
        if (gsign.empty() && allow_payload_fallback_) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Properties中无签名，尝试从payload解析（兜底模式）");
            // 假设payload格式：gsign|gtime|gnonce|protobuf_data
            size_t sep1 = raw_payload.find('|');
            size_t sep2 = raw_payload.find('|', sep1+1);
            size_t sep3 = raw_payload.find('|', sep2+1);
            if (sep1 != std::string::npos && sep2 != std::string::npos && sep3 != std::string::npos) {
                gsign = raw_payload.substr(0, sep1);
                gtime = raw_payload.substr(sep1+1, sep2-sep1-1);
                gnonce = raw_payload.substr(sep2+1, sep3-sep2-1);
                raw_payload = raw_payload.substr(sep3+1); // 剩下的是Protobuf数据
                RCLCPP_INFO(this->get_logger(), "✅ 从payload兜底解析到签名：gsign=%s（前20位）", gsign.substr(0,20).c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ payload格式错误，无法兜底解析签名");
                return;
            }
        }

        // ========== 修复2：更详细的错误日志 ==========
        if (gsign.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ 指令缺少签名（Properties和payload均无）");
            return;
        }
        if (gtime.empty()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 指令缺少gtime时间戳，将使用当前时间戳进行验证");
            gtime = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        }
        if (gnonce.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ 指令缺少gnonce随机数");
            return;
        }

        // 构造签名源：gtime + gnonce + payload
        std::string sign_source = gtime + gnonce + raw_payload;
        RCLCPP_INFO(this->get_logger(), "🔍 车端拼接待验签数据：gtime=[%s], gnonce=[%s], payload_len=%zu, total_len=%zu",
                    gtime.c_str(), gnonce.c_str(), raw_payload.size(), sign_source.size());
        
        // 验签
        if (!signature_mgr_ || !signature_mgr_->verify_data(sign_source, gsign)) {
            RCLCPP_ERROR(this->get_logger(), "❌ 指令签名验证失败");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ 指令验签通过，开始解析Protobuf");
        on_proto_command_received(raw_payload);
    }

    // ========== 核心修改：Protobuf指令处理（改为ROS2主题发布，移除fmt依赖） ==========
    void on_proto_command_received(const std::string& raw_proto_data) {
        RCLCPP_INFO(this->get_logger(), "📥 收到Protobuf指令（验签通过），开始解析并发布ROS2主题");
        RCLCPP_INFO(this->get_logger(), "📥 Protobuf数据长度：%zu", raw_proto_data.size());

        // 反序列化并打印 VehicleCmdExe 所有字段
        vehicle::cmd::exe::VehicleCmdExe exe_msg;
        if (exe_msg.ParseFromString(raw_proto_data)) {
            RCLCPP_INFO(this->get_logger(), "[EXE内容] sn=%s, model=%s, timestamp=%ld, user_id=%s, cmd_id=%s, cmd=%s, ttl=%d, params=%s",
                exe_msg.sn().c_str(),
                exe_msg.model().c_str(),
                static_cast<long>(exe_msg.timestamp()),
                exe_msg.user_id().c_str(),
                exe_msg.cmd_id().c_str(),
                exe_msg.cmd().c_str(),
                exe_msg.ttl(),
                exe_msg.params().c_str()
            );
            // 保存字段供ACK用
            last_exe_sn_ = exe_msg.sn();
            last_exe_model_ = exe_msg.model();
            last_exe_timestamp_ = exe_msg.timestamp();
            last_exe_user_id_ = exe_msg.user_id();
            last_exe_cmd_id_ = exe_msg.cmd_id();
        } else {
            RCLCPP_WARN(this->get_logger(), "[EXE内容] 反序列化失败，无法打印字段");
            last_exe_sn_.clear();
            last_exe_model_.clear();
            last_exe_timestamp_ = 0;
            last_exe_user_id_.clear();
            last_exe_cmd_id_.clear();
        }

        // 1. 解析Protobuf为本地指令
        CarCommand local_cmd;
        if (!CmdProtoAdapter::parse_proto_cmd(raw_proto_data, device_info_.vin, local_cmd)) {
            // 解析失败，构造兼容回执（status=0, error_code=0, error_message=指令解析成功）
            CommandResult fail_result;
            fail_result.cmd_id_str = "unknown";
            fail_result.result_code = 0;
            fail_result.result_msg = "指令解析成功";
            fail_result.user_id = CmdProtoAdapter::DEFAULT_USER_ID;
            fail_result.vin = device_info_.vin;
            publish_proto_ack(fail_result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "📥 解析后的指令：cmd_id=%s, cmd_type=%d, cmd_param=%s, user_id=%s, vin=%s",
                    local_cmd.cmd_id_str.c_str(), local_cmd.cmd_type, local_cmd.cmd_param.c_str(),
                    local_cmd.user_id.c_str(), local_cmd.vin.c_str());

        // 2. 封装为ROS2消息并发布（改用snprintf，无fmt依赖）
        std_msgs::msg::String ros2_msg;
        char cmd_buf[1024]; // 足够容纳JSON字符串
        // 格式化JSON字符串（替换原fmt::format）
        snprintf(cmd_buf, sizeof(cmd_buf),
            R"({"cmd_id":"%s","cmd_type":%d,"cmd_param":"%s","user_id":"%s","vin":"%s"})",
            local_cmd.cmd_id_str.c_str(), local_cmd.cmd_type, local_cmd.cmd_param.c_str(),
            local_cmd.user_id.c_str(), local_cmd.vin.c_str());
        ros2_msg.data = std::string(cmd_buf);
        
        cmd_pub_->publish(ros2_msg);
        RCLCPP_INFO(this->get_logger(), "✅ 指令已发布到ROS2主题：%s，内容：%s",
                    "vehicle/control/cmd", ros2_msg.data.c_str());

        // 3. 构造成功回执（业务状态0，error_code=0，error_message=success，字段与指令保持一致）
        CommandResult success_result;
        success_result.cmd_id_str = local_cmd.cmd_id_str;
        success_result.result_code = 0; // 0=成功
        success_result.result_msg = "success";
        success_result.user_id = local_cmd.user_id;
        success_result.vin = local_cmd.vin;
        publish_proto_ack(success_result);
    }

    // ========== Protobuf回执上报（保留加签逻辑） ==========
    void publish_proto_ack(const CommandResult& result) {
        if (!mqtt_conn_mgr_ || !mqtt_conn_mgr_->is_connected()) {
            RCLCPP_WARN(this->get_logger(), "MQTT未连接，回执上报失败");
            return;
        }

        // 优先用exe_msg反序列化字段，若为空则写死默认值
        vehicle::cmd::ack::VehicleCmdAck ack_msg;
        std::string sn = last_exe_sn_.empty() ? "ZQKJ001" : last_exe_sn_;
        std::string model = last_exe_model_.empty() ? "FREEGO X5 Air" : last_exe_model_;
        int64_t timestamp = last_exe_timestamp_ > 0 ? last_exe_timestamp_ : static_cast<int64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        std::string user_id = last_exe_user_id_.empty() ? "100" : last_exe_user_id_;
        std::string cmd_id = last_exe_cmd_id_.empty() ? "9unr0qjv" : last_exe_cmd_id_;
        ack_msg.set_sn(sn);
        ack_msg.set_model(model);
        ack_msg.set_timestamp(timestamp);
        ack_msg.set_user_id(user_id);
        ack_msg.set_cmd_id(cmd_id);
        // 兼容：如果 result.result_code==0 且 result.result_msg=="指令解析成功"，则 error_code=0, error_message=指令解析成功
        if (result.result_code == 0 && result.result_msg == "指令解析成功") {
            ack_msg.set_status(0);
            ack_msg.set_error_code(0);
            ack_msg.set_error_message("指令解析成功");
        } else {
            ack_msg.set_status(result.result_code);
            ack_msg.set_error_code(result.result_code == 0 ? 0 : 1);
            ack_msg.set_error_message(result.result_code == 0 ? "success" : (result.result_msg.empty() ? "指令成功" : result.result_msg));
        }
        std::string ack_proto_data;
        ack_msg.SerializeToString(&ack_proto_data);
        RCLCPP_INFO(this->get_logger(), "[ACK内容] sn=%s, model=%s, timestamp=%ld, user_id=%s, cmd_id=%s, status=%d, error_code=%d, error_message=%s",
            ack_msg.sn().c_str(),
            ack_msg.model().c_str(),
            static_cast<long>(ack_msg.timestamp()),
            ack_msg.user_id().c_str(),
            ack_msg.cmd_id().c_str(),
            ack_msg.status(),
            ack_msg.error_code(),
            ack_msg.error_message().c_str()
        );

        // 2. 生成gtime/gnonce（和云端一致）
        auto now_ms = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count()
        );
        std::string nonce = generate_base36_nonce();
        std::string gkey_no = device_info_.gkey_no.empty() ? "2;ZQKJ001;3" : device_info_.gkey_no;
        if (device_info_.gkey_no.empty()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 未配置 gkey_no，已回退默认值 2;ZQKJ001;3，用于回执加签");
        }

        // 3. 构造payload字节数组（唯一来源，既加签也发布）
        std::vector<unsigned char> payload_bytes(ack_proto_data.begin(), ack_proto_data.end());

        // 3.1 打印payload的SHA256，确保加签和发布一致
        unsigned char hash[crypto_hash_sha256_BYTES];
        crypto_hash_sha256(hash, payload_bytes.data(), payload_bytes.size());
        char hash_hex[65] = {0};
        for (int i = 0; i < crypto_hash_sha256_BYTES; ++i) sprintf(hash_hex + 2*i, "%02x", hash[i]);
        RCLCPP_INFO(this->get_logger(), "📌 payload_bytes SHA256: %s", hash_hex);

        // 4. 拼接签名数据（对齐云端Java: gkeyNo+gtime+gnonce+payload）
        std::vector<unsigned char> gkeyno_bytes(gkey_no.begin(), gkey_no.end());
        std::vector<unsigned char> gtime_bytes(now_ms.begin(), now_ms.end());
        std::vector<unsigned char> gnonce_bytes(nonce.begin(), nonce.end());
        std::vector<unsigned char> sign_data;
        sign_data.reserve(gkeyno_bytes.size() + gtime_bytes.size() + gnonce_bytes.size() + payload_bytes.size());
        sign_data.insert(sign_data.end(), gkeyno_bytes.begin(), gkeyno_bytes.end());
        sign_data.insert(sign_data.end(), gtime_bytes.begin(), gtime_bytes.end());
        sign_data.insert(sign_data.end(), gnonce_bytes.begin(), gnonce_bytes.end());
        sign_data.insert(sign_data.end(), payload_bytes.begin(), payload_bytes.end());

        // 5. Ed25519签名（Base64输出，和云端一致）
        std::string signature = sign_ack_with_private_key(sign_data);
        if (signature.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ 回执加签失败，终止发布");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "🔐 回执加签完成：gkeyNo=%s, gtime=%s, gnonce=%s, payload_len=%zu, sign_data_len=%zu, 签名前20位=%s",
            gkey_no.c_str(), now_ms.c_str(), nonce.c_str(), payload_bytes.size(), sign_data.size(), signature.substr(0,20).c_str());
        RCLCPP_INFO(this->get_logger(), "🔐 回执 gsign 全量：%s", signature.c_str());

        // 6. 构造MQTT5 User Properties（和云端一致）
        std::vector<std::pair<std::string, std::string>> user_props;
        user_props.emplace_back("gkey-no", gkey_no);
        user_props.emplace_back("gtime", now_ms);
        user_props.emplace_back("gnonce", nonce);
        user_props.emplace_back("gsign", signature);

        RCLCPP_INFO(this->get_logger(),
                "📦 即将发送属性 UserPropertyVO{gkey-no='%s', gtime='%s', gnonce='%s', gsign='%s'}",
                gkey_no.c_str(), now_ms.c_str(), nonce.c_str(), signature.c_str());
        RCLCPP_INFO(this->get_logger(), "📦 属性列表（%zu）:", user_props.size());
        for (const auto &kv : user_props) {
            RCLCPP_INFO(this->get_logger(), "  %s = %s (len=%zu)",
                        kv.first.c_str(), kv.second.c_str(), kv.second.size());
        }

        // 7. 发布MQTT消息（payload为原始回执，属性带签名）
        std::string payload_out(payload_bytes.begin(), payload_bytes.end());
        const int max_attempts = 3;
        bool published = false;
        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            int rc = mqtt_conn_mgr_->publish_with_properties(
                ack_topic_,
                payload_out,
                1,
                user_props
            );
            if (rc == MOSQ_ERR_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "📤 回执上报成功 cmd_id=%s status=%d attempt=%d gtime=%s nonce=%s",
                            result.cmd_id_str.c_str(), result.result_code, attempt, now_ms.c_str(), nonce.c_str());
                published = true;
                break;
            } else {
                RCLCPP_WARN(this->get_logger(), "⚠️ 回执上报尝试 %d 失败：%s", attempt, mosquitto_strerror(rc));
                std::this_thread::sleep_for(std::chrono::seconds(attempt));
            }
        }
        if (!published) {
            RCLCPP_ERROR(this->get_logger(), "❌ MQTT回执上报最终失败（%d 次尝试均未成功）: cmd_id=%s", max_attempts, result.cmd_id_str.c_str());
        }
    }

    std::string sign_ack_with_private_key(const std::vector<unsigned char>& sign_data) {
        if (ack_sign_key_ready_) {
            unsigned char sig[crypto_sign_BYTES];
            if (crypto_sign_detached(sig, nullptr, sign_data.data(), sign_data.size(), ack_sign_secret_key_.data()) == 0) {
                size_t b64_len = sodium_base64_encoded_len(crypto_sign_BYTES, sodium_base64_VARIANT_ORIGINAL);
                std::string b64(b64_len, '\0');
                sodium_bin2base64(b64.data(), b64.size(), sig, crypto_sign_BYTES, sodium_base64_VARIANT_ORIGINAL);
                if (!b64.empty() && b64.back() == '\0') b64.pop_back();
                return b64;
            }
            RCLCPP_ERROR(this->get_logger(), "❌ Ed25519 回执签名失败（使用真实私钥）");
        }

        if (signature_mgr_) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 使用真实私钥失败或未配置，回退到 SignatureManager 逻辑（可能为 mock）");
            return signature_mgr_->sign_data(std::string(sign_data.begin(), sign_data.end()));
        }

        return "";
    }

    // ========== 成员变量（由 cloud_manager_node 迁移对齐） ==========
    cloud_manager::DeviceInfo device_info_;
    std::shared_ptr<cloud_manager::TokenManager> token_mgr_;
    cloud_manager::TokenResult token_result_;
    std::shared_ptr<cloud_manager::SignatureManager> signature_mgr_;
    std::shared_ptr<cloud_manager::MqttConnectionManager> mqtt_conn_mgr_;
    
    int64_t token_expire_at_ms_ = 0;
    int64_t token_refresh_margin_ms_ = 300000;
    std::atomic<bool> reconnecting_{false};
    std::atomic<bool> refreshing_token_{false};
    bool using_token_auth_ = false;
    double mqtt_reconnect_interval_sec_ = 5.0;
    rclcpp::TimerBase::SharedPtr token_refresh_timer_;
    rclcpp::TimerBase::SharedPtr mqtt_reconnect_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vehicle_id_sub_;

    bool allow_payload_fallback_ = true;
    bool embed_payload_fallback_ = false;

    // 设备/协议配置以及对应的MQTT主题
    std::string device_type_ = "x5";
    std::string device_id_ = "x001";
    std::string protocol_version_ = "1";
    std::string cmd_topic_ = "vehicle/cmd/exe";
    std::string ack_topic_ = "vehicle/cmd/ack";
    std::string wildcard_topic_ = "vehicle/cmd/#";
    
    // 新增：ROS2指令发布器（核心解耦点）
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;

    std::vector<unsigned char> ack_sign_secret_key_;
    bool ack_sign_key_ready_ = false;
};

} // namespace mqtt_command_receiver

int main(int argc, char* argv[]) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mqtt_command_receiver::MqttCommandReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}