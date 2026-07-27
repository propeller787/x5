#include "cloud_manager/http_singleton.hpp"

namespace cloud_manager_node
{
    // 静态成员初始化
    HttpClientManager *HttpClientManager::instance_(nullptr);
    std::mutex HttpClientManager::mutex_;
    // 获取单例实例（双检锁实现）
    HttpClientManager *HttpClientManager::getInstance()
    {
        if (instance_ == nullptr)
        {
            // 加锁
            std::lock_guard<std::mutex> lock(mutex_);
            if (instance_ == nullptr)
            {
                // 创建实例
                instance_ = new HttpClientManager();
            }
        }
        return instance_;
    }

    HttpClientManager::HttpClientManager() : rclcpp::Node("http_Service") 
    {
    }

    // 内部：构造通用头（含签名）
    cpr::Header HttpClientManager::build_headers(const mqtt_module::DeviceInfo &info, const std::string &body_json,
                                                 bool use_session_key)
    {
        cpr::Header headers;
        headers["Content-Type"] = "application/json; charset=UTF-8";
        headers["Accept-Language"] = "zh-CN/en-US"; // by ,2026.2;
        // 新协议使用小写头名称
        headers["gkey-no"] = use_session_key ? info.session_key_no : info.gkey_no;
        headers["gversion"] = info.version;
        headers["gclient"] = info.client;
        headers["gapp-version"] = info.app_version;
        auto now_ms = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count());
        headers["gtime"] = now_ms;
        // 16字节hex随机数
        std::uniform_int_distribution<uint64_t> dis;
        static thread_local std::mt19937_64 gen(std::random_device{}());
        uint64_t a = dis(gen), b = dis(gen);
        std::ostringstream ss;
        ss << std::hex << std::setw(16) << std::setfill('0') << a
           << std::hex << std::setw(16) << std::setfill('0') << b;
        std::string nonce = ss.str();
        headers["gnonce"] = nonce;
        // 选择签名私钥：业务流优先用会话密钥，获取会话密钥时用根密钥
        const std::string &signing_key = (use_session_key && !info.session_private_key.empty())
                                             ? info.session_private_key
                                             : (!info.root_private_key.empty() ? info.root_private_key
                                                                               : info.private_key);

        mqtt_module::SignatureConfig cfg;
        cfg.private_key = signing_key;
        mqtt_module::SignatureManager sm(cfg, rclcpp::get_logger("TokenManager"));

        // 根据协议要求选择不同的签名源格式
        std::string sign_source;
        // 所有接口统一使用：gkey-no+gversion+gclient+gapp-version+gtime+gnonce+body
        sign_source = headers["gkey-no"] + headers["gversion"] + headers["gclient"] +
                      headers["gapp-version"] + headers["gtime"] + headers["gnonce"] + body_json;

        headers["gsign"] = sm.sign_data(sign_source);
        return headers;
    }
 
    bool HttpClientManager::device_bind_msg(const std::string &url_name, const mqtt_module::DeviceInfo &info, VehicleBindStatus &pkg, std::string &err_msg) {
        std::string url = base_url_ + url_name;
        nlohmann::json body;
        body["sn"] = info.vin;
        body["model"] = info.model;
        std::string body_str = body.dump();

        // 打印请求参数
        RCLCPP_INFO(rclcpp::get_logger("device_bind_msg"), "📤 会话密钥请求 - sn:%s, model:%s", info.vin.c_str(), info.model.c_str());
        auto headers = build_headers(info, body_str, false);
        RCLCPP_DEBUG(rclcpp::get_logger("device_bind_msg"), "📤 会话密钥请求头 - gkey-no:%s, gversion:%s, gclient:%s, gapp-version:%s, gtime:%s, gnonce:%s, gsign:%s",
                     headers["gkey-no"].substr(0, 20).c_str(),
                     headers["gversion"].c_str(),
                     headers["gclient"].c_str(),
                     headers["gapp-version"].c_str(),
                     headers["gtime"].c_str(),
                     headers["gnonce"].substr(0, 16).c_str(),
                     headers["gsign"].substr(0, 20).c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("device_bind_msg"), "会话密钥GSign前20:%s", headers["gsign"].substr(0, 20).c_str());
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        RCLCPP_INFO(rclcpp::get_logger("device_bind_msg"), "会话密钥HTTP返回码:%ld", http_res.status_code);

        // 打印完整的响应头
        RCLCPP_INFO(rclcpp::get_logger("device_bind_msg"), "📥 会话密钥响应头完整信息:");
        for (const auto &header : http_res.header) {
            RCLCPP_INFO(rclcpp::get_logger("device_bind_msg"), "  %s: %s", header.first.c_str(), header.second.c_str());
        }

        // 打印响应头
        RCLCPP_INFO(rclcpp::get_logger("device_bind_msg"), "📥 会话密钥响应头 - gtime:%s, gnonce:%s, gsign:%s",
                    http_res.header.count("gtime") ? http_res.header["gtime"].c_str() : "",
                    http_res.header.count("gnonce") ? http_res.header["gnonce"].c_str() : "",
                    http_res.header.count("gsign") ? http_res.header["gsign"].c_str() : "");
        RCLCPP_DEBUG(rclcpp::get_logger("device_bind_msg"), "会话密钥响应:%s", http_res.text.c_str());
        if (http_res.error) {
            err_msg = http_res.error.message;
            RCLCPP_ERROR(rclcpp::get_logger("device_bind_msg"), "❌ HTTP错误 - %s", http_res.error.message.c_str());
            return false;
        }
        if (http_res.status_code != 200)
        {
            RCLCPP_ERROR(rclcpp::get_logger("device_bind_msg"), "❌ HTTP状态: %ld", http_res.status_code);
            return false;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        RCLCPP_DEBUG(rclcpp::get_logger("device_bind_msg"), "会话密钥响应:%s", http_res.text.c_str());
        if (json_resp.is_discarded())
        {
            RCLCPP_ERROR(rclcpp::get_logger("device_bind_msg"), "❌ 响应JSON解析失败");
            return false;
        }


        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        std::string sn = data.value("sn", std::string(""));
        std::string user_id = data.value("userId", std::string(""));
        int status = data.value("status", -1);
        long timestamp = data.value("timestamp", 0L);
        std::string privacy_policy = data.value("privacyPolicy", std::string(""));
        if (sn.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("device_bind_msg"), "⚠️ 事件配置响应缺少sn字段");
            return false;
        }

        pkg.set_sn(sn);
        pkg.set_timestamp(timestamp);
        pkg.set_user_id(user_id);
        pkg.set_status(status);
        pkg.set_privacy_policy(privacy_policy);
        pkg.set_model(""); // 回复缺省

        return true;
    }
}