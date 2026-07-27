// src/token_manager.cpp
#include "cloud_manager/token_manager.hpp"
#include "cloud_manager/signature_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>  // 时间戳/有效期校验
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <random>
#include <sstream>
#include <iomanip>

namespace cloud_manager {

// 对外统一接口：发送设备信息→获取可信JWT令牌
TokenResult TokenManager::get_trusted_token(const DeviceInfo& info) {
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "===== 开始获取可信JWT令牌 =====");
    
    const int max_retries = 3; // 最大重试次数
    int retry_count = 0;
    
    while (retry_count < max_retries) {
        // 步骤1：获取挑战应答（防重放攻击）
        ChallengeResult challenge_res = get_challenge(info);
        if (!challenge_res.success) {
            TokenResult res;
            res.success = false;
            res.err_msg = "获取Challenge失败：" + challenge_res.err_msg;
            RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "%s", res.err_msg.c_str());
            return res;
        }

        // 校验Challenge有效期（防重放核心）
        if (!check_challenge_exp(challenge_res)) {
            TokenResult res;
            res.success = false;
            res.err_msg = "Challenge已过期（有效期：" + std::to_string(challenge_res.exp) + "ms）";
            RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "%s", res.err_msg.c_str());
            return res;
        }

        // 步骤2：用Challenge获取JWT令牌
        TokenResult jwt_res = get_jwt_token(info, challenge_res);
        if (jwt_res.success) {
            RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "===== JWT令牌获取流程结束 =====");
            return jwt_res;
        } else if (jwt_res.err_msg == "challenge expired") {
            retry_count++;
            RCLCPP_WARN(rclcpp::get_logger("TokenManager"), "Challenge 失效，重试 %d/%d", retry_count, max_retries);
            // 继续循环，获取新 challenge
        } else {
            // 其他错误，直接返回
            RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "===== JWT令牌获取流程结束 =====");
            return jwt_res;
        }
    }
    
    // 重试次数耗尽
    TokenResult res;
    res.success = false;
    res.err_msg = "重试次数耗尽，Challenge 失效";
    RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "%s", res.err_msg.c_str());
    return res;
}

// 获取会话密钥对（设备开机/重置时调用）
SessionKeyResult TokenManager::get_session_key(const DeviceInfo& info) {
#if USE_MOCK_TOKEN
    return mock_get_session_key(info);
#else
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【真实模式】调用获取会话密钥接口（设备：%s，型号：%s）", info.vin.c_str(), info.model.c_str());
        SessionKeyResult res;
        try {
            nlohmann::json body;
            body["sn"] = info.vin;
            body["model"] = info.model;
            std::string body_str = body.dump();

            // 打印请求参数
            RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "📤 会话密钥请求 - sn:%s, model:%s", info.vin.c_str(), info.model.c_str());
            RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "📤 会话密钥请求体:%s", body_str.c_str());
            
            auto headers = build_headers(info, body_str, false, "");
            RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "📤 会话密钥请求头 - gkey-no:%s, gversion:%s, gclient:%s, gapp-version:%s, gtime:%s, gnonce:%s, gsign:%s", 
                        headers["gkey-no"].substr(0, 20).c_str(),
                        headers["gversion"].c_str(),
                        headers["gclient"].c_str(),
                        headers["gapp-version"].c_str(),
                        headers["gtime"].c_str(),
                        headers["gnonce"].substr(0, 16).c_str(),
                        headers["gsign"].substr(0, 20).c_str());
            RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "会话密钥GSign前20:%s", headers["gsign"].substr(0,20).c_str());
            std::string url = info.api_base_url + "/api/adevice/v1/auth/getSessionKey";
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "📡 会话密钥API URL: %s", url.c_str());
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "会话密钥HTTP返回码:%ld", http_res.status_code);
        
        // 打印完整的响应头
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "📥 会话密钥响应头完整信息:");
        for (const auto& header : http_res.header) {
            RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "  %s: %s", header.first.c_str(), header.second.c_str());
        }
        
        // 打印响应头
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "📥 会话密钥响应头 - gtime:%s, gnonce:%s, gsign:%s", 
                    http_res.header.count("gtime") ? http_res.header["gtime"] : "",
                    http_res.header.count("gnonce") ? http_res.header["gnonce"] : "",
                    http_res.header.count("gsign") ? http_res.header["gsign"] : "");
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "会话密钥响应:%s", http_res.text.c_str());
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP错误:" + http_res.error.message;
            return res;
        }
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP状态:" + std::to_string(http_res.status_code);
            return res;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "会话密钥响应:%s", http_res.text.c_str());
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "响应JSON解析失败";
            return res;
        }
        
        // 检查是否返回错误
        if (json_resp.contains("code") && json_resp["code"].is_number()) {
            int code = json_resp["code"];
            if (code != 0) {
                res.success = false;
                res.err_msg = json_resp.value("msg", std::string("未知错误"));
                RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "❌ 会话密钥API返回错误 - code:%d, msg:%s", code, res.err_msg.c_str());
                return res;
            }
        }
        
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        if (data.is_null()) {
            res.success = false;
            res.err_msg = "响应数据为空";
            RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "❌ 会话密钥响应数据为空");
            return res;
        }
        
        res.public_key = data.value("publicKey", std::string(""));
        res.private_key = data.value("privateKey", std::string(""));
        res.key_no = data.value("keyNo", std::string(""));
        res.success = !res.public_key.empty() && !res.private_key.empty();
        if (!res.success) {
            res.err_msg = "publicKey/privateKey 缺失";
        }
        
        // 验签：gtime + gnonce + body
        if (res.success && !info.root_public_key.empty()) {
            std::string res_gtime = http_res.header.count("gtime") ? http_res.header["gtime"] : "";
            std::string res_gnonce = http_res.header.count("gnonce") ? http_res.header["gnonce"] : "";
            std::string res_gsign = http_res.header.count("gsign") ? http_res.header["gsign"] : "";
            
            if (!res_gtime.empty() && !res_gnonce.empty() && !res_gsign.empty()) {
                std::string res_text = res_gtime + res_gnonce + http_res.text;
                SignatureConfig cfg;
                cfg.public_key = info.root_public_key;
                SignatureManager sm(cfg, rclcpp::get_logger("TokenManager"));
                bool verified = sm.verify_data_with_base64_key(res_text, res_gsign, info.root_public_key);
                if (!verified) {
                    RCLCPP_WARN(rclcpp::get_logger("TokenManager"), "⚠️ 会话密钥响应验签失败");
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "✅ 会话密钥响应验签通过");
                }
            }
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "获取会话密钥异常:" + std::string(e.what());
    }
    return res;
#endif
}

// 内部：构造通用头（含签名）
cpr::Header TokenManager::build_headers(const DeviceInfo& info, const std::string& body_json, bool use_session_key, const std::string& token) {
    cpr::Header headers;
    headers["content-type"] = "application/json; charset=UTF-8";
    // 关键：添加gkey-no头，用于身份认证
    std::string gkey_no = use_session_key && !info.session_key_no.empty() ? info.session_key_no : info.gkey_no;
    headers["gkey-no"] = gkey_no;
    headers["gversion"] = info.version;
    headers["glanguage"] = info.language;
    headers["gclient"] = info.client;
    headers["gapp-version"] = info.app_version;
    auto now_ms = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    headers["gtime"] = now_ms;

    std::uniform_int_distribution<uint64_t> dis;
    static thread_local std::mt19937_64 gen(std::random_device{}());
    uint64_t a = dis(gen), b = dis(gen);
    std::ostringstream ss;
    ss << std::hex << std::setw(16) << std::setfill('0') << a
       << std::hex << std::setw(16) << std::setfill('0') << b;
    std::string nonce = ss.str();
    headers["gnonce"] = nonce;

    // 选择签名密钥：使用会话密钥或根密钥
    const std::string& signing_key = (use_session_key && !info.session_private_key.empty())
                                         ? info.session_private_key
                                         : (!info.root_private_key.empty() ? info.root_private_key
                                                                           : info.private_key);

    SignatureConfig cfg;
    cfg.private_key = signing_key;
    SignatureManager sm(cfg, rclcpp::get_logger("mqtt_module::TokenManager"));

    // 构建签名源字符串：所有接口统一使用gkey-no+gversion+gclient+gapp-version+gtime+gnonce+body
    std::string sign_source = headers["gkey-no"] + headers["gversion"] + headers["gclient"] +
                               headers["gapp-version"] + headers["gtime"] + headers["gnonce"] + body_json;
    
    headers["gsign"] = sm.sign_data(sign_source);
    RCLCPP_DEBUG(rclcpp::get_logger("mqtt_module::TokenManager"), "签名源: %s", sign_source.c_str());
    RCLCPP_DEBUG(rclcpp::get_logger("mqtt_module::TokenManager"), "签名结果: %s", headers["gsign"].c_str());
    return headers;
}

// 步骤1：获取挑战应答（真实实现）
ChallengeResult TokenManager::get_challenge(const DeviceInfo& info) {
#if USE_MOCK_TOKEN
    return mock_get_challenge(info);
#else
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【真实模式】调用获取Challenge接口");
    ChallengeResult res;
    try {
        nlohmann::json body;
        body["sn"] = info.vin;
        body["model"] = info.model;
        std::string body_str = body.dump();

        // 打印请求参数
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "📤 Challenge请求参数 - sn:%s, model:%s", info.vin.c_str(), info.model.c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "📤 Challenge请求体:%s", body_str.c_str());

        auto headers = build_headers(info, body_str, true, "");
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "📤 Challenge请求头 - gkey-no:%s, gversion:%s, gclient:%s, gapp-version:%s, gtime:%s, gnonce:%s, gsign:%s", 
                    headers["gkey-no"].substr(0, 20).c_str(),
                    headers["gversion"].c_str(),
                    headers["gclient"].c_str(),
                    headers["gapp-version"].c_str(),
                    headers["gtime"].c_str(),
                    headers["gnonce"].substr(0, 16).c_str(),
                    headers["gsign"].substr(0, 20).c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "Challenge签名前20:%s", headers["gsign"].substr(0,20).c_str());
        std::string url = info.api_base_url + "/api/adevice/v1/auth/challenge";
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "Challenge HTTP返回码:%ld", http_res.status_code);
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "Challenge响应头 - gtime:%s, gnonce:%s, gsign:%s", 
                    http_res.header.count("gtime") ? http_res.header["gtime"] : "",
                    http_res.header.count("gnonce") ? http_res.header["gnonce"] : "",
                    http_res.header.count("gsign") ? http_res.header["gsign"].substr(0, 20) : "");
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "Challenge响应体:%s", http_res.text.c_str());
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP错误:" + http_res.error.message;
            return res;
        }
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP状态:" + std::to_string(http_res.status_code);
            return res;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "Challenge响应:%s", http_res.text.c_str());
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "响应JSON解析失败";
            return res;
        }
        
        // 检查是否返回错误
        if (json_resp.contains("code") && json_resp["code"].is_number()) {
            int code = json_resp["code"];
            if (code != 0) {
                res.success = false;
                res.err_msg = json_resp.value("msg", std::string("未知错误"));
                RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "❌ Challenge API返回错误 - code:%d, msg:%s", code, res.err_msg.c_str());
                return res;
            }
        }
        
        // 兼容两种返回：直接包含字段或 data 包裹
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        
        // 检查 data 是否为 null
        if (data.is_null()) {
            res.success = false;
            res.err_msg = "响应数据为空";
            RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "❌ Challenge响应数据为空");
            return res;
        }
        
        res.challenge = data.value("challenge", "");
        res.exp = data.value("exp", static_cast<int64_t>(0));
        res.create_time = data.value("createTime", static_cast<int64_t>(0)); // 服务端生成时间
        res.gtime = http_res.header.count("gtime") ? std::stoll(http_res.header["gtime"]) : 0; // 响应头 gtime
        res.success = !res.challenge.empty();
        if (!res.success) {
            res.err_msg = "Challenge缺失";
        }
        
        // 验签：gtime + gnonce + body
        if (res.success && !info.session_public_key.empty()) {
            std::string res_gtime = http_res.header.count("gtime") ? http_res.header["gtime"] : "";
            std::string res_gnonce = http_res.header.count("gnonce") ? http_res.header["gnonce"] : "";
            std::string res_gsign = http_res.header.count("gsign") ? http_res.header["gsign"] : "";
            
            if (!res_gtime.empty() && !res_gnonce.empty() && !res_gsign.empty()) {
                std::string res_text = res_gtime + res_gnonce + http_res.text;
                SignatureConfig cfg;
                cfg.public_key = info.session_public_key;
                SignatureManager sm(cfg, rclcpp::get_logger("TokenManager"));
                bool verified = sm.verify_data_with_base64_key(res_text, res_gsign, info.session_public_key);
                if (!verified) {
                    RCLCPP_WARN(rclcpp::get_logger("TokenManager"), "⚠️ Challenge响应验签失败");
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "✅ Challenge响应验签通过");
                }
            }
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "获取Challenge异常:" + std::string(e.what());
    }
    return res;
#endif
}

// 步骤2：用Challenge获取JWT令牌（真实实现）
TokenResult TokenManager::get_jwt_token(const DeviceInfo& info, const ChallengeResult& challenge_res) {
#if USE_MOCK_TOKEN
    return mock_get_jwt_token(info, challenge_res);
#else
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【真实模式】调用获取JWT令牌接口（携带Challenge：%s）",
                challenge_res.challenge.c_str());
    TokenResult res;
    try {
        nlohmann::json body;
        body["sn"] = info.vin;
        body["model"] = info.model;
        body["challenge"] = challenge_res.challenge;
        std::string body_str = body.dump();

        // 打印请求参数
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "📤 Token请求参数 - sn:%s, model:%s, challenge:%s", 
                    info.vin.c_str(), info.model.c_str(), challenge_res.challenge.substr(0, 20).c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "📥 Token请求体:%s", body_str.c_str());

        auto headers = build_headers(info, body_str, true, "");
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "📤 Token请求头 - gkey-no:%s, gversion:%s, gclient:%s, gapp-version:%s, gtime:%s, gnonce:%s, gsign:%s", 
                    headers["gkey-no"].substr(0, 20).c_str(),
                    headers["gversion"].c_str(),
                    headers["gclient"].c_str(),
                    headers["gapp-version"].c_str(),
                    headers["gtime"].c_str(),
                    headers["gnonce"].substr(0, 16).c_str(),
                    headers["gsign"].substr(0, 20).c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "Token签名前20:%s", headers["gsign"].substr(0,20).c_str());
        std::string url = info.api_base_url + "/api/adevice/v1/auth/getToken";
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "Token HTTP返回码:%ld", http_res.status_code);
        
        // 打印响应头
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "📥 Token响应头 - gtime:%s, gnonce:%s, gsign:%s", 
                    http_res.header.count("gtime") ? http_res.header["gtime"] : "",
                    http_res.header.count("gnonce") ? http_res.header["gnonce"] : "",
                    http_res.header.count("gsign") ? http_res.header["gsign"] : "");
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "Token响应体:%s", http_res.text.c_str());
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP错误:" + http_res.error.message;
            return res;
        }
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP状态:" + std::to_string(http_res.status_code);
            return res;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "响应JSON解析失败";
            return res;
        }
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        // 检查是否返回错误（例如 challenge 失效）
        if (data.contains("error") && data["error"].is_string()) {
            std::string error = data["error"];
            if (error == "challenge expired" || error == "challenge invalid") {
                res.success = false;
                res.err_msg = "challenge expired";
                return res;
            }
        }
        res.jwt_token = data.value("token", data.value("jwt_token", std::string("")));
        res.exp = data.value("exp", static_cast<int64_t>(0));
        res.success = !res.jwt_token.empty();
        if (!res.success) {
            res.err_msg = "token缺失";
        }
        
        // 验签：gtime + gnonce + body
        if (res.success && !info.session_public_key.empty()) {
            std::string res_gtime = http_res.header.count("gtime") ? http_res.header["gtime"] : "";
            std::string res_gnonce = http_res.header.count("gnonce") ? http_res.header["gnonce"] : "";
            std::string res_gsign = http_res.header.count("gsign") ? http_res.header["gsign"] : "";
            
            if (!res_gtime.empty() && !res_gnonce.empty() && !res_gsign.empty()) {
                std::string res_text = res_gtime + res_gnonce + http_res.text;
                SignatureConfig cfg;
                cfg.public_key = info.session_public_key;
                SignatureManager sm(cfg, rclcpp::get_logger("TokenManager"));
                bool verified = sm.verify_data_with_base64_key(res_text, res_gsign, info.session_public_key);
                if (!verified) {
                    RCLCPP_WARN(rclcpp::get_logger("TokenManager"), "⚠️ Token响应验签失败");
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "✅ Token响应验签通过");
                }
            }
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "获取JWT令牌异常:" + std::string(e.what());
    }
    return res;
#endif
}

// 刷新JWT令牌（真实实现）
TokenResult TokenManager::refresh_jwt_token(const DeviceInfo& info, const std::string& current_token) {
#if USE_MOCK_TOKEN
    TokenResult res;
    res.jwt_token = "MOCK_REFRESHED_JWT_TOKEN";
    res.exp = 3600000;
    res.success = true;
    return res;
#else
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【真实模式】调用刷新JWT令牌接口");
    TokenResult res;
    try {
        nlohmann::json body;
        body["sn"] = info.vin;
        body["token"] = current_token;
        std::string body_str = body.dump();

        auto headers = build_headers(info, body_str, true, current_token);
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "RefreshToken签名前20:%s", headers["gsign"].substr(0,20).c_str());
        std::string url = info.api_base_url + "/api/adevice/v1/auth/refreshToken";
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "RefreshToken HTTP返回码:%ld", http_res.status_code);
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP错误:" + http_res.error.message;
            return res;
        }
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP状态:" + std::to_string(http_res.status_code);
            return res;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "响应JSON解析失败";
            return res;
        }
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        res.jwt_token = data.value("token", data.value("jwt_token", std::string("")));
        res.exp = data.value("exp", static_cast<int64_t>(0));
        res.success = !res.jwt_token.empty();
        if (!res.success) {
            res.err_msg = "token缺失";
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "刷新JWT令牌异常:" + std::string(e.what());
    }
    return res;
#endif
}

// 辅助：校验Challenge有效期（防重放攻击核心）
bool TokenManager::check_challenge_exp(const ChallengeResult& challenge_res) {
    int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    
    // 使用 Challenge 的实际获取时间（从响应头 gtime 获取）
    // 如果 challenge_res.gtime 为 0，说明响应中没有 gtime 字段
    // 此时应该使用当前时间作为 Challenge 的获取时间
    int64_t challenge_create_time = (challenge_res.gtime == 0) ? now : challenge_res.gtime;
    bool is_valid = (now - challenge_create_time) < challenge_res.exp;
    
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "校验Challenge有效期：%s（当前时间%ld，生成时间%ld，有效期%ld）",
                is_valid ? "有效" : "过期", now, challenge_create_time, challenge_res.exp);
    return is_valid;
}

// 辅助：Ed25519签名（Mock/真实兼容）
std::string TokenManager::ed25519_sign(const DeviceInfo& info, const std::string& content) {
#if USE_MOCK_TOKEN
    // Mock：返回模拟签名
    std::string mock_sign = "MOCK_ED25519_SIGN_" + content.substr(0, 16);
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【Mock】Ed25519签名：%s（内容：%s）",
                mock_sign.c_str(), content.c_str());
    return mock_sign;
#else
    // 真实环境：调用Ed25519签名接口（libsodium示例）
    // unsigned char sig[64];
    // crypto_sign_ed25519_detached(sig, NULL, 
    //                              (const unsigned char*)content.c_str(), content.size(),
    //                              (const unsigned char*)info.private_key.c_str());
    // std::string real_sign(reinterpret_cast<char*>(sig), 64);
    // RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【真实】Ed25519签名完成（内容：%s）", content.c_str());
    // return real_sign;

    // 预留接口：暂返回模拟签名
    return "REAL_ED25519_SIGN_" + content.substr(0, 16);
#endif
}

// 辅助：发送HTTP请求（Mock/真实兼容）
HttpResponse TokenManager::send_http_request(const HttpRequest& req) {
#if USE_MOCK_TOKEN
    // Mock：返回成功响应
    HttpResponse res;
    res.code = 200;
    res.body = "success";
    res.success = true;
    return res;
#else
    // 真实环境：基于curl实现HTTP请求（参考此前提供的curl代码）
    HttpResponse res;
    res.code = 200;
    res.body = R"({"code":0,"data":{"challenge":"REAL_CHALLENGE_987654","exp":30000}})"; // Mock真实响应
    res.success = true;
    return res;
#endif
}

// Mock：获取Challenge（开发阶段）
ChallengeResult TokenManager::mock_get_challenge(const DeviceInfo& info) {
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【Mock】调用获取Challenge接口（设备：%s）", info.vin.c_str());
    ChallengeResult res;
    res.challenge = "MOCK_CHALLENGE_123456";
    res.exp = 30000; // 30秒有效期
    res.create_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count(); // 当前时间作为生成时间
    res.success = true;
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【Mock】获取Challenge成功：%s（有效期%ldms）",
                res.challenge.c_str(), res.exp);
    return res;
}

// Mock：获取JWT令牌（开发阶段）
TokenResult TokenManager::mock_get_jwt_token(const DeviceInfo& info, const ChallengeResult& challenge_res) {
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【Mock】调用获取JWT令牌接口（携带Challenge：%s）",
                challenge_res.challenge.c_str());
    TokenResult res;
    // 模拟 challenge expired（30%概率）
    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    if (dis(gen) < 0.3) {
        res.success = false;
        res.err_msg = "challenge expired";
        RCLCPP_WARN(rclcpp::get_logger("TokenManager"), "【Mock】模拟Challenge失效");
        return res;
    }
    res.jwt_token = "MOCK_JWT_TOKEN_eyJhbGciOiJFZDI1NTE5IiwidHlwIjoiSldUIn0.eyJ2aW4iOiJWSU5fMDAxIiwiZXhwIjoxNzY0MzE3MjY5fQ.xxx";
    res.exp = 3600000; // 1小时有效期
    res.success = true;
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【Mock】获取JWT令牌成功：%s（有效期%ldms）",
                res.jwt_token.c_str(), res.exp);
    return res;
}

// Mock：获取会话密钥（开发阶段）
SessionKeyResult TokenManager::mock_get_session_key(const DeviceInfo& info) {
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【Mock】调用获取会话密钥接口（设备：%s）", info.vin.c_str());
    SessionKeyResult res;
    res.public_key = "MOCK_SESSION_PUB_HEX";
    res.private_key = "MOCK_SESSION_PRIV_HEX";
    res.success = true;
    return res;
}

} // namespace cloud_manager