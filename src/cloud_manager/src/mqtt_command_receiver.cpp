#include <rclcpp/rclcpp.hpp>
#include <mosquittopp.h>
#include <mosquitto.h>
#include <chrono>
#include <functional>
#include <string>
#include <memory>
#include <ctime>
#include <random>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <nlohmann/json.hpp>

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
    std::string cmd;             // 指令
    std::string params;          // 控制参数
    int32_t ttl_ms = 0;          // 有效期
    std::string user_id;         // 用户ID
    std::string sn;              // 车辆SN
    std::string model;           // 车型
    int64_t timestamp = 0;       // 毫秒时间戳
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
    static std::string VIN;
    static int32_t MODEL;
    static std::string DEFAULT_USER_ID;

    // 将Protobuf指令解析为本地CarCommand
    static bool parse_proto_cmd(const std::string& raw_proto_data, CarCommand& cmd) {
        vehicle::cmd::exe::VehicleCmdExe proto_cmd;
        if (!proto_cmd.ParseFromString(raw_proto_data)) {
            RCLCPP_ERROR(rclcpp::get_logger("CmdProtoAdapter"), "❌ Protobuf指令解析失败");
            return false;
        }

        // 校验必填字段（按实际proto字段：vin 为字符串，model 为 int32）
        if (proto_cmd.vin().empty() || proto_cmd.vin() != VIN) {
            RCLCPP_ERROR(rclcpp::get_logger("CmdProtoAdapter"), "❌ VIN不匹配：接收=%s，本地=%s", proto_cmd.vin().c_str(), VIN.c_str());
            return false;
        }
        if (proto_cmd.cmd_id().empty() || proto_cmd.cmd().empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("CmdProtoAdapter"), "❌ 指令缺少必填字段（cmd_id/cmd）");
            return false;
        }
        cmd.cmd = proto_cmd.cmd();
        cmd.params = proto_cmd.params();
        cmd.ttl_ms = proto_cmd.ttl();
        cmd.cmd_id_str = proto_cmd.cmd_id();
        cmd.user_id = proto_cmd.user_id().empty() ? DEFAULT_USER_ID : proto_cmd.user_id();
        cmd.sn = proto_cmd.vin();
        cmd.model = std::to_string(proto_cmd.model());
        cmd.timestamp = proto_cmd.timestamp();

        RCLCPP_INFO(rclcpp::get_logger("CmdProtoAdapter"), "✅ Protobuf指令解析成功：vin=%s, cmd_id=%s, cmd=%s",
                    proto_cmd.vin().c_str(), proto_cmd.cmd_id().c_str(), proto_cmd.cmd().c_str());
        return true;
    }

    // 将执行结果封装为Protobuf回执
    static std::string pack_ack_proto(const CommandResult& result) {
        vehicle::cmd::ack::VehicleCmdAck ack;
        // 填充固定字段
        ack.set_vin(result.vin);
        ack.set_model(MODEL);
        ack.set_timestamp(get_current_timestamp_ms()); // 毫秒时间戳
        ack.set_user_id(result.user_id);
        ack.set_cmd_id(result.cmd_id_str);

        // 填充业务状态
        if (result.result_code == 0) {
            ack.set_status(0); // 成功
        } else if (result.result_code == 2) {
            ack.set_status(2); // 执行中
        } else {
            ack.set_status(1); // 失败
            ack.set_error_code(result.result_code);
            ack.set_error_message(result.result_msg);
        }

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

// 静态变量初始化（可由参数覆盖）
std::string CmdProtoAdapter::VIN = "VIN_1234567890";
int32_t CmdProtoAdapter::MODEL = 0; // 0:X5、1:Cruise
std::string CmdProtoAdapter::DEFAULT_USER_ID = "default_user_001";

// ========== 4. 核心节点类（移除原有执行逻辑，改为主题发布解耦） ==========
class MqttCommandReceiverNode : public rclcpp::Node {
public:
    MqttCommandReceiverNode() : Node("mqtt_command_receiver") {
        // 声明可注入的签名参数（与数据上报保持一致）
        this->declare_parameter<std::string>("signature.private_key", "");
        this->declare_parameter<std::string>("signature.public_key", "");
        this->declare_parameter<bool>("signature.disable", false);

        // 设备与协议版本（用于组装可配置的MQTT主题），与数据上报保持一致
        this->declare_parameter<std::string>("device.model", "FREEGO X5 Air");
        this->declare_parameter<std::string>("device.id", "ZQKJ001");
        // 主题车型段按协议去除空格，默认 FREEGOX5Air，可被参数覆盖
        this->declare_parameter<std::string>("topic.model", "FREEGOX5Air");
        this->declare_parameter<std::string>("protocol.version", "1");
        // 基础车辆信息（与数据上报模块保持一致，可覆盖解析校验字段）
        this->declare_parameter<std::string>("vin", CmdProtoAdapter::VIN);
        this->declare_parameter<int>("model", CmdProtoAdapter::MODEL);
        this->declare_parameter<std::string>("user_id", CmdProtoAdapter::DEFAULT_USER_ID);
        // JSON 调试开关（指令解析回退）
        this->declare_parameter<bool>("command.allow_json", true);

        // Token获取对齐数据上报：API地址与根密钥参数
        this->declare_parameter<std::string>("api_base_url", "http://39.107.107.204:22000");
        this->declare_parameter<std::string>("token.root_key_no", "");
        this->declare_parameter<std::string>("token.root_private_key", "");
        this->declare_parameter<std::string>("token.root_public_key", "");
        this->declare_parameter<bool>("token.session_cache_enable", true);
        this->declare_parameter<std::string>("token.session_cache_path", "/tmp/cloud_manager_session_keys.json");
        // 令牌日志与直传参数（与上报对齐，默认关闭直传）
        this->declare_parameter<bool>("log.token_full", false);
        this->declare_parameter<bool>("mqtt.use_param_token", false);
        this->declare_parameter<std::string>("mqtt.jwt_token", "");
        this->declare_parameter<std::string>("mqtt.base_token", "");

        // 覆盖静态协议标识
        try { CmdProtoAdapter::VIN = this->get_parameter("vin").as_string(); } catch(...) {}
        try { CmdProtoAdapter::MODEL = this->get_parameter("model").as_int(); } catch(...) {}
        try {
            auto uid = this->get_parameter("user_id").as_string();
            if (!uid.empty()) CmdProtoAdapter::DEFAULT_USER_ID = uid;
        } catch(...) {}
        try { allow_json_cmd_fallback_ = this->get_parameter("command.allow_json").as_bool(); } catch(...) { allow_json_cmd_fallback_ = true; }
        try { signature_disabled_ = this->get_parameter("signature.disable").as_bool(); } catch(...) { signature_disabled_ = false; }

        // ========== 步骤1：保留原有初始化逻辑（令牌/签名/MQTT连接） ==========
        init_token_manager();    
        init_signature_manager();
        init_mqtt_connection();

        if (!mqtt_conn_mgr_ || !mqtt_conn_mgr_->is_connected()) {
            RCLCPP_FATAL(this->get_logger(), "❌ MQTT连接失败（令牌验证未通过），节点退出");
            rclcpp::shutdown();
            return;
        }

        // ========== 步骤2：创建ROS2主题发布器（解耦指令分发，直接转发protobuf负载） ==========
        // 通用指令发布主题（其他节点订阅该主题处理具体逻辑）
        cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            "vehicle/control/cmd", 10 // 主题名可根据项目调整
        );

        // 加载可配置的指令主题（vehicle/cmd/<type>/<id>/<ver>/exe/ack）
        load_topic_config();

        // ========== 步骤3：读取 payload 回退开关（mosquitto 无 v5 user-properties 回调） ==========
        this->declare_parameter<bool>("signature.allow_payload_fallback", true);
        try { allow_payload_fallback_ = this->get_parameter("signature.allow_payload_fallback").as_bool(); } catch(...) { allow_payload_fallback_ = true; }

        // ========== 步骤4：订阅MQTT指令主题 ==========
        subscribe_command_topic(cmd_topic_);

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
        // 构造与数据上报一致的设备信息
        cloud_manager::DeviceInfo dev_info;
        // 设备标识：优先使用参数 vin（与上报兼容，通常即SN）
        try { dev_info.vin = this->get_parameter("vin").as_string(); } catch(...) {}
        if (dev_info.vin.empty()) dev_info.vin = CmdProtoAdapter::VIN;
        // 业务型号（与上报一致）
        try { dev_info.model = this->get_parameter("device.model").as_string(); } catch(...) {}
        if (dev_info.model.empty()) dev_info.model = "x5";
        // API地址
        try { dev_info.api_base_url = this->get_parameter("api_base_url").as_string(); } catch(...) {}
        // 根密钥与 gkey-no
        try {
            const std::string key_no = this->get_parameter("token.root_key_no").as_string();
            const std::string root_sk = this->get_parameter("token.root_private_key").as_string();
            const std::string root_pk = this->get_parameter("token.root_public_key").as_string();
            if (!key_no.empty()) dev_info.key_no = key_no;
            if (!root_sk.empty()) dev_info.root_private_key = root_sk;
            if (!root_pk.empty()) dev_info.root_public_key = root_pk;
        } catch(...) {}
        // 若注入了签名私钥，则作为兜底 device_info.private_key
        try {
            const std::string sk = this->get_parameter("signature.private_key").as_string();
            if (!sk.empty()) dev_info.private_key = sk;
        } catch(...) {}

        token_mgr_ = std::make_shared<cloud_manager::TokenManager>();

        // 尝试读取/写入会话密钥缓存（与数据上报一致的参数名）
        cloud_manager::SessionKeyResult session_res;
        bool cache_enable = true; std::string cache_path = "/tmp/cloud_manager_session_keys.json";
        try { cache_enable = this->get_parameter("token.session_cache_enable").as_bool(); } catch(...) {}
        try { cache_path = this->get_parameter("token.session_cache_path").as_string(); } catch(...) {}
        auto load_session_cache = [&](cloud_manager::SessionKeyResult &out)->bool {
            try {
                if (!cache_enable) return false;
                if (!std::filesystem::exists(cache_path)) return false;
                std::ifstream in(cache_path);
                if (!in.is_open()) return false;
                nlohmann::json j; in >> j;
                if (!j.contains("private_key") || !j.contains("public_key")) return false;
                out.private_key = j.value("private_key", "");
                out.public_key  = j.value("public_key",  "");
                out.key_no      = j.value("key_no",      "");
                out.success = !(out.private_key.empty() || out.public_key.empty());
                return out.success;
            } catch(...) { return false; }
        };
        auto save_session_cache = [&](const cloud_manager::SessionKeyResult &res){
            try {
                if (!cache_enable || !res.success) return;
                std::filesystem::path p(cache_path);
                std::error_code ec; std::filesystem::create_directories(p.parent_path(), ec);
                nlohmann::json j{{"private_key", res.private_key},{"public_key",res.public_key},{"key_no",res.key_no}};
                std::ofstream out(cache_path); out << j.dump(2);
                RCLCPP_INFO(this->get_logger(), "✅ 会话密钥已缓存到: %s", cache_path.c_str());
            } catch(...) {}
        };

        if (!load_session_cache(session_res)) {
            session_res = token_mgr_->get_session_key(dev_info);
            if (session_res.success) save_session_cache(session_res);
        }
        if (session_res.success) {
            dev_info.session_private_key = session_res.private_key;
            dev_info.session_public_key  = session_res.public_key;
            if (!session_res.key_no.empty()) dev_info.key_no = session_res.key_no;
            dev_session_priv_ = session_res.private_key;
            dev_session_pub_  = session_res.public_key;
            dev_key_no_ = dev_info.key_no;
            RCLCPP_INFO(this->get_logger(), "✅ 会话密钥准备完成（公钥前20：%s）", session_res.public_key.substr(0,20).c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 会话密钥不可用，将回退根密钥流程获取令牌");
        }

        // 令牌获取策略与日志风格与上报保持一致
        const bool use_param_token = this->get_parameter("mqtt.use_param_token").as_bool();
        const std::string jwt_param = this->get_parameter("mqtt.jwt_token").as_string();
        const std::string base_token_param = this->get_parameter("mqtt.base_token").as_string();
        const bool log_token_full = this->get_parameter("log.token_full").as_bool();

        auto log_token = [&](const cloud_manager::TokenResult& token, const char* label){
            if (!token.success) return;
            if (log_token_full) {
                RCLCPP_WARN(this->get_logger(), "%s JWT（完整）：%s", label, token.jwt_token.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "%s JWT（前20）：%s", label, token.jwt_token.substr(0,20).c_str());
            }
        };

        cloud_manager::TokenResult jwt_res;
        if (use_param_token && !base_token_param.empty()) {
            jwt_res = token_mgr_->get_mqtt_token_v1(dev_info, base_token_param);
            if (jwt_res.success) log_token(jwt_res, "✅ 使用base_token获取MQTT令牌");
        } else if (use_param_token && !jwt_param.empty()) {
            jwt_res.jwt_token = jwt_param; jwt_res.exp = 3600000; jwt_res.success = true;
            log_token(jwt_res, "✅ 使用参数JWT令牌");
        }
        if (!jwt_res.success) {
            jwt_res = token_mgr_->get_trusted_token(dev_info);
            if (!jwt_res.success) {
                RCLCPP_FATAL(this->get_logger(), "❌ 获取JWT令牌失败：%s", jwt_res.err_msg.c_str());
                token_result_ = jwt_res; // 记录失败以便上层判断
                return;
            }
            log_token(jwt_res, "✅ 令牌获取成功");
        }
        token_result_ = jwt_res;
    }

    void init_signature_manager() {
        cloud_manager::SignatureConfig sign_config;
        sign_config.sign_separator = "|";
        // 默认使用会话密钥对：私钥加签、公共键验签
        if (!dev_session_priv_.empty()) sign_config.private_key = dev_session_priv_;
        if (!dev_session_pub_.empty())  sign_config.public_key  = dev_session_pub_;
        // 优先从参数读取私钥/公钥（hex 编码），可覆盖
        try {
            std::string sk = this->get_parameter("signature.private_key").as_string();
            std::string pk = this->get_parameter("signature.public_key").as_string();
            if (!sk.empty()) sign_config.private_key = sk;
            if (!pk.empty()) sign_config.public_key = pk;
        } catch (...) {
            // ignore, will fallback
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
    }

    void init_mqtt_connection() {
        // 支持通过参数注入 broker 地址与 client_id，便于本地调试/切换
        this->declare_parameter<std::string>("mqtt.broker", "mqtts://39.106.140.222:28883");
        this->declare_parameter<std::string>("mqtt.client_id", "");
        this->declare_parameter<std::string>("mqtt.username", "");
        this->declare_parameter<std::string>("mqtt.password", "");
        this->declare_parameter<std::string>("mqtt.ca_cert", "");
        this->declare_parameter<bool>("mqtt.insecure_skip_verify", true);
        this->declare_parameter<std::string>("mqtt.cipher", "ECDHE-RSA-AES256-GCM-SHA384");
        this->declare_parameter<int>("mqtt.version", 5);
        this->declare_parameter<std::string>("mqtt.token_username", "token_user");
        this->declare_parameter<int>("mqtt.retry_attempts", 3);
        this->declare_parameter<double>("mqtt.retry_delay_base", 2.0);

        cloud_manager::MqttConnConfig mqtt_cfg;
        mqtt_cfg.broker = this->get_parameter("mqtt.broker").as_string();
        mqtt_cfg.client_id = this->get_parameter("mqtt.client_id").as_string();
        if (mqtt_cfg.client_id.empty()) {
            // 默认使用设备SN作为ClientID
            try { mqtt_cfg.client_id = this->get_parameter("device.id").as_string(); } catch(...) {}
            if (mqtt_cfg.client_id.empty()) {
                mqtt_cfg.client_id = "ros2_command_client_" + std::to_string(this->now().nanoseconds());
            }
        }
        try {
            mqtt_cfg.mqtt_version = this->get_parameter("mqtt.version").as_int();
            if (mqtt_cfg.mqtt_version != 5 && mqtt_cfg.mqtt_version != 4) mqtt_cfg.mqtt_version = 5;
        } catch(...) {}
        try { mqtt_cfg.username = this->get_parameter("mqtt.username").as_string(); } catch(...) {}
        try { mqtt_cfg.password = this->get_parameter("mqtt.password").as_string(); } catch(...) {}
        try { mqtt_cfg.ca_cert_path = this->get_parameter("mqtt.ca_cert").as_string(); } catch(...) {}
        try { mqtt_cfg.insecure_skip_verify = this->get_parameter("mqtt.insecure_skip_verify").as_bool(); } catch(...) {}
        try { mqtt_cfg.tls_cipher = this->get_parameter("mqtt.cipher").as_string(); } catch(...) {}
        try { mqtt_cfg.token_username = this->get_parameter("mqtt.token_username").as_string(); } catch(...) {}
        if (mqtt_cfg.username.empty() && mqtt_cfg.password.empty()) {
            mqtt_cfg.use_token_auth = true; // 与 cloud_manager_node 一致：无用户名密码则走 token
        }
        int retry_attempts = 3;
        double retry_delay_base = 2.0;
        try { retry_attempts = this->get_parameter("mqtt.retry_attempts").as_int(); } catch(...) {}
        try { retry_delay_base = this->get_parameter("mqtt.retry_delay_base").as_double(); } catch(...) {}

        // mosquitto 支持 ssl:// 与 tcp://（cloud_manager::MqttConnectionManager 自适配 mqtts:///mqtt://）

        mqtt_conn_mgr_ = std::make_shared<cloud_manager::MqttConnectionManager>(mqtt_cfg, this->get_logger());

        // 连接重试策略（与 cloud_manager_node 保持一致）
        bool conn_ok = false;
        cloud_manager::TokenResult empty_token;
        for (int attempt = 0; attempt < retry_attempts; ++attempt) {
            // 如果指定了用户名/密码则优先使用它们，否则回退 token
            const auto& token_to_use = (!mqtt_cfg.username.empty() && !mqtt_cfg.password.empty()) ? empty_token : token_result_;
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
        if (!conn_ok) return;

        mqtt_conn_mgr_->set_message_callback_with_props([this](const std::string& topic, const std::string& payload, const mosquitto_property* props){
            handle_mqtt_message(topic, payload, props);
        });
    }

    void load_topic_config() {
        // 与数据上报保持一致：主题车型段使用 topic.model（无空格），与业务 device.model 解耦
        try { device_type_ = this->get_parameter("topic.model").as_string(); } catch (...) {}
        if (device_type_.empty()) {
            try { device_type_ = this->get_parameter("device.model").as_string(); } catch (...) {}
            device_type_.erase(std::remove(device_type_.begin(), device_type_.end(), ' '), device_type_.end());
        }
        try { device_id_ = this->get_parameter("device.id").as_string(); } catch (...) {}
        try { protocol_version_ = this->get_parameter("protocol.version").as_string(); } catch (...) {}

        cmd_topic_ = "vehicle/cmd/" + device_type_ + "/" + device_id_ + "/" + protocol_version_ + "/exe";
        ack_topic_ = "vehicle/cmd/" + device_type_ + "/" + device_id_ + "/" + protocol_version_ + "/ack";

        RCLCPP_INFO(this->get_logger(), "MQTT 指令主题：%s，回执主题：%s", cmd_topic_.c_str(), ack_topic_.c_str());
    }

    void subscribe_command_topic(const std::string& topic) {
        try {
            int rc = mqtt_conn_mgr_->subscribe(topic, 1);
            if (rc == MOSQ_ERR_SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "✅ 订阅MQTT指令主题成功：%s", topic.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 订阅MQTT指令主题失败：%s", mosquitto_strerror(rc));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "订阅MQTT主题异常：%s", e.what());
        }
    }

    void handle_mqtt_message(const std::string& topic, const std::string& payload, const mosquitto_property* props = nullptr) {
        if (topic != cmd_topic_) return;
        std::string raw_payload = payload;
        std::string gsign;
        std::string gtime;
        std::string gnonce;
        std::string gkey_no;

        // 解析 MQTT v5 user properties: gkey-no/gtime/gnonce/gsign（旧版libmosquitto可能无属性API）
#if defined(MQTT_PROP_USER_PROPERTY)
        if (props) {
            bool skip_first = false;
            while (true) {
                const char* name = nullptr;
                const char* val = nullptr;
                int rc = mosquitto_property_read_string_pair(props, MQTT_PROP_USER_PROPERTY, &name, &val, skip_first);
                if (rc != MOSQ_ERR_SUCCESS || !name || !val) break;
                std::string n(name), v(val);
                if (n == "gkey-no") gkey_no = v;
                else if (n == "gtime") gtime = v;
                else if (n == "gnonce") gnonce = v;
                else if (n == "gsign") gsign = v;
                skip_first = true;
            }
        }
#endif

        // 回退 payload raw|signature
        if (gsign.empty() && allow_payload_fallback_) {
            const std::string sep = "|";
            size_t pos = payload.find(sep);
            if (pos != std::string::npos && pos > 0 && pos < payload.size()-1) {
                raw_payload = payload.substr(0, pos);
                gsign = payload.substr(pos+1);
            }
        }

        if (!signature_disabled_) {
            if (gsign.empty()) {
                RCLCPP_ERROR(this->get_logger(), "❌ 指令缺少签名（未收到 gsign 且未命中 raw|signature 回退）");
                return;
            }
            // 按协议签名源：gkey-no + gtime + gnonce + payload
            std::string sign_source = gkey_no + gtime + gnonce + raw_payload;
            if (!signature_mgr_ || !signature_mgr_->verify_data(sign_source, gsign)) {
                RCLCPP_ERROR(this->get_logger(), "❌ 指令签名验证失败");
                return;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 已开启 signature.disable，跳过指令验签（仅调试使用）");
        }

        // 优先按 Protobuf 解析；失败时按 JSON 回退并转换为 Protobuf 再分发
        std::string proto_payload = raw_payload;
        mqtt_command_receiver::CarCommand dummy;
        if (!CmdProtoAdapter::parse_proto_cmd(raw_payload, dummy)) {
            if (!allow_json_cmd_fallback_) {
                RCLCPP_ERROR(this->get_logger(), "❌ 指令解析失败，且已关闭 JSON 回退");
                return;
            }
            if (!try_convert_json_to_proto(raw_payload, proto_payload)) {
                RCLCPP_ERROR(this->get_logger(), "❌ 指令解析失败（Protobuf/JSON 均无法解析）");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "ℹ️ 指令按 JSON 解析成功，已转换为 Protobuf 再分发");
        }

        on_proto_command_received(proto_payload);
    }

    // ========== 核心修改：Protobuf指令处理（解析校验后直接转发protobuf字节） ==========
    void on_proto_command_received(const std::string& raw_proto_data) {
        RCLCPP_INFO(this->get_logger(), "📥 收到Protobuf指令（验签通过），开始解析并发布ROS2主题");

        // 1. 解析Protobuf为本地指令
        CarCommand local_cmd;
        if (!CmdProtoAdapter::parse_proto_cmd(raw_proto_data, local_cmd)) {
            // 解析失败，构造失败回执
            CommandResult fail_result;
            fail_result.cmd_id_str = "unknown";
            fail_result.result_code = 1;
            fail_result.result_msg = "指令解析失败";
            fail_result.user_id = CmdProtoAdapter::DEFAULT_USER_ID;
            fail_result.vin = CmdProtoAdapter::VIN;
            publish_proto_ack(fail_result);
            return;
        }

        // 2. 封装为ROS2消息并发布（直接转发原始 protobuf 二进制）
        std_msgs::msg::String ros2_msg;
        ros2_msg.data = raw_proto_data; // 包含完整 VehicleCmdExe 序列化字节

        cmd_pub_->publish(ros2_msg);
        RCLCPP_INFO(this->get_logger(), "✅ 指令已发布到ROS2主题：%s，protobuf字节数=%zu",
                    "vehicle/control/cmd", ros2_msg.data.size());

        // 3. 构造成功回执（标记为"执行中"，由实际处理节点更新最终状态）
        CommandResult success_result;
        success_result.cmd_id_str = local_cmd.cmd_id_str;
        success_result.result_code = 2; // 2=执行中
        success_result.result_msg = "指令已分发到ROS2主题（protobuf），等待处理";
        success_result.user_id = local_cmd.user_id;
        success_result.vin = local_cmd.sn;
        publish_proto_ack(success_result);
    }

    // ========== Protobuf回执上报（保留加签逻辑） ==========
    void publish_proto_ack(const CommandResult& result) {
        if (!mqtt_conn_mgr_ || !mqtt_conn_mgr_->is_connected()) {
            RCLCPP_WARN(this->get_logger(), "MQTT未连接，回执上报失败");
            return;
        }

        // 1. 封装Protobuf回执
        std::string ack_proto_data = CmdProtoAdapter::pack_ack_proto(result);
        if (ack_proto_data.empty()) return;

        // 2. 生成 gtime/gnonce 并签名（签名仅对原始二进制）
        auto now_ms = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count()
        );
        // 生成简单 nonce（16 字节 hex）
        std::string nonce;
        {
            static std::random_device rd;
            static std::mt19937_64 gen(rd());
            static std::uniform_int_distribution<uint64_t> dis;
            uint64_t a = dis(gen), b = dis(gen);
            std::ostringstream ss;
            ss << std::hex << std::setw(16) << std::setfill('0') << a
               << std::hex << std::setw(16) << std::setfill('0') << b;
            nonce = ss.str();
        }
        std::string sign_source = dev_key_no_ + now_ms + nonce + ack_proto_data;
        std::string signature = signature_mgr_->sign_data(sign_source);
        RCLCPP_DEBUG(this->get_logger(), "回执签名前20：%s", signature.substr(0,20).c_str());

        const int max_attempts = 3;
        bool published = false;
        std::vector<std::pair<std::string, std::string>> user_props;
        if (!signature_disabled_) {
            if (!dev_key_no_.empty()) user_props.emplace_back("gkey-no", dev_key_no_);
            user_props.emplace_back("gtime", now_ms);
            user_props.emplace_back("gnonce", nonce);
            user_props.emplace_back("gsign", signature);
        }
        for (int attempt = 1; attempt <= max_attempts; ++attempt) {
            int rc = mqtt_conn_mgr_->publish_with_properties(
                ack_topic_,
                ack_proto_data,
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

    // ========== 成员变量（移除原有ROS客户端，新增主题发布器） ==========
    std::shared_ptr<cloud_manager::TokenManager> token_mgr_;
    cloud_manager::TokenResult token_result_;
    std::shared_ptr<cloud_manager::SignatureManager> signature_mgr_;
    std::shared_ptr<cloud_manager::MqttConnectionManager> mqtt_conn_mgr_;
    bool allow_payload_fallback_ = true;
    std::string dev_session_priv_;
    std::string dev_session_pub_;
    std::string dev_key_no_;

    // 设备/协议配置以及对应的MQTT主题
    std::string device_type_ = "x5";
    std::string device_id_ = "x001";
    std::string protocol_version_ = "1";
    std::string cmd_topic_ = "vehicle/cmd/exe";
    std::string ack_topic_ = "vehicle/cmd/ack";
    
    // 新增：ROS2指令发布器（核心解耦点）
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
        bool allow_json_cmd_fallback_ = true;
        bool signature_disabled_ = false;

        bool try_convert_json_to_proto(const std::string& json_text, std::string& out_proto) {
            try {
                auto obj = nlohmann::json::parse(json_text);
                auto vin = obj.value("vin", "");
                auto cmd_id = obj.value("cmd_id", "");
                auto cmd = obj.value("cmd", "");
                auto user_id = obj.value("user_id", CmdProtoAdapter::DEFAULT_USER_ID);
                if (vin.empty() || cmd_id.empty() || cmd.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "❌ JSON 指令缺少必填字段 vin/cmd_id/cmd");
                    return false;
                }
                if (vin != CmdProtoAdapter::VIN) {
                    RCLCPP_ERROR(this->get_logger(), "❌ JSON VIN 不匹配：接收=%s，本地=%s", vin.c_str(), CmdProtoAdapter::VIN.c_str());
                    return false;
                }
                vehicle::cmd::exe::VehicleCmdExe proto_cmd;
                proto_cmd.set_vin(vin);
                proto_cmd.set_model(CmdProtoAdapter::MODEL);
                proto_cmd.set_cmd_id(cmd_id);
                proto_cmd.set_cmd(cmd);
                proto_cmd.set_user_id(user_id);
                // 可选时间戳
                if (obj.contains("timestamp")) {
                    try { proto_cmd.set_timestamp(obj.at("timestamp").get<int64_t>()); } catch(...) {}
                }
                if (!proto_cmd.SerializeToString(&out_proto)) {
                    RCLCPP_ERROR(this->get_logger(), "❌ JSON 转 Protobuf 失败（序列化错误）");
                    return false;
                }
                return true;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "❌ JSON 指令解析异常：%s", e.what());
                return false;
            }
        }
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