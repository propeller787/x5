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

// 内部：构造通用头（含签名，v2-小写g* + gkey-no）前置声明
static cpr::Header build_headers_v2(const DeviceInfo& info,
                                    const std::string& body_json,
                                    bool use_session_key,
                                    const std::string& token_header = "");

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

// 直接用已有 token 头获取 MQTT 连接令牌（HTTP GET）
TokenResult TokenManager::get_mqtt_token_v1(const DeviceInfo& info, const std::string& base_token) {
#if USE_MOCK_TOKEN
    TokenResult res;
    res.jwt_token = "MOCK_MQTT_TOKEN_FROM_V1";
    res.exp = 3600000;
    res.success = true;
    return res;
#else
    TokenResult res;
    try {
        cpr::Header headers;
        headers["Content-Type"] = "application/json; charset=UTF-8";
        headers["token"] = base_token;
        headers["gversion"] = info.version;
        headers["gclient"] = info.client;
        headers["gapp-version"] = info.app_version;

        std::string url = info.api_base_url + "/api/auth/v1/getMqttToken";
        auto http_res = cpr::Get(cpr::Url{url}, headers, cpr::Timeout{5000});
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "getMqttToken HTTP返回码:%ld", http_res.status_code);
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP错误:" + http_res.error.message;
            RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "getMqttToken HTTP错误详情: %s", http_res.error.message.c_str());
            return res;
        }
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP状态:" + std::to_string(http_res.status_code);
            RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "getMqttToken 错误响应正文: %s", http_res.text.c_str());
            return res;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "响应JSON解析失败";
            return res;
        }
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        res.jwt_token = data.value("token", std::string(""));
        res.exp = data.value("exp", static_cast<int64_t>(0));
        res.success = !res.jwt_token.empty();
        if (!res.success) {
            res.err_msg = "token缺失";
            } else {
                RCLCPP_INFO(rclcpp::get_logger("TokenManager"),
                            "✅ getMqttToken成功：前24=%s，exp=%ldms",
                            res.jwt_token.substr(0, 24).c_str(), res.exp);
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "获取MQTT令牌异常:" + std::string(e.what());
    }
    return res;
#endif
}

// 获取会话密钥对（设备开机/重置时调用）
SessionKeyResult TokenManager::get_session_key(const DeviceInfo& info) {
#if USE_MOCK_TOKEN
    return mock_get_session_key(info);
#else
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【真实模式】调用获取会话密钥接口");
    SessionKeyResult res;
    try {
        nlohmann::json body;
        body["sn"] = info.vin;
        body["model"] = info.model;
        std::string body_str = body.dump();

        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "会话密钥请求体:%s", body_str.c_str());
        auto headers = build_headers_v2(info, body_str, false);
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"),
                "会话密钥请求头: gkey-no=%s gtime=%s gnonce=%s",
                headers.count("gkey-no")? headers["gkey-no"].c_str():"(empty)",
                headers["gtime"].c_str(), headers["gnonce"].c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "会话密钥gsign前20:%s", headers["gsign"].substr(0,20).c_str());
        std::string url = info.api_base_url + "/api/adevice/v1/auth/getSessionKey";
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "会话密钥HTTP返回码:%ld", http_res.status_code);
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP错误:" + http_res.error.message;
            return res;
        }
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP状态:" + std::to_string(http_res.status_code);
            RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "会话密钥错误响应正文: %s", http_res.text.c_str());
            return res;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "响应JSON解析失败";
            return res;
        }
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        res.public_key = data.value("publicKey", std::string(""));
        res.private_key = data.value("privateKey", std::string(""));
        res.key_no = data.value("keyNo", std::string(""));
        res.success = !res.public_key.empty() && !res.private_key.empty();
        if (!res.success) {
            res.err_msg = "publicKey/privateKey 缺失";
        }
        if (res.success) {
            RCLCPP_INFO(rclcpp::get_logger("TokenManager"),
                        "✅ 会话密钥获取成功：keyNo=%s，pub前16=%s",
                        res.key_no.c_str(), res.public_key.substr(0, 16).c_str());
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "获取会话密钥异常:" + std::string(e.what());
    }
    return res;
#endif
}

// 内部：构造通用头（含签名，v2：小写g*，含gkey-no）
static cpr::Header build_headers_v2(const DeviceInfo& info,
                                    const std::string& body_json,
                                    bool use_session_key,
                                    const std::string& token_header) {
    cpr::Header headers;
    headers["Content-Type"] = "application/json; charset=UTF-8";
    headers["Accept-Language"] = info.language;
    headers["gversion"] = info.version;
    headers["gclient"] = info.client;
    headers["gapp-version"] = info.app_version;
    if (!token_header.empty()) {
        headers["token"] = token_header;
    }
    // 可选：语言不参与签名，若服务端无需可省略
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

    // gkey-no：服务端分配的密钥编号（仅在非空时设置）
    std::string key_no = info.key_no;
    if (!key_no.empty()) {
        headers["gkey-no"] = key_no;
    }

    // 选择签名私钥：业务流优先用会话密钥，获取会话密钥时用根密钥
    const std::string& signing_key = (use_session_key && !info.session_private_key.empty())
                                         ? info.session_private_key
                                         : (!info.root_private_key.empty() ? info.root_private_key
                                                                           : info.private_key);

    SignatureConfig cfg;
    cfg.private_key = signing_key;
    SignatureManager sm(cfg, rclcpp::get_logger("TokenManager"));

    // 按文档要求拼接签名原文：gkey-no + token(可选) + gversion + gclient + gapp-version + gtime + gnonce + body
    std::string gkey_no_val = key_no; // 若未设置header则为空字符串
    auto it_token = headers.find("token");
    const std::string token_part = (it_token != headers.end()) ? it_token->second : std::string();
    std::string sign_source = gkey_no_val + token_part + headers["gversion"] + headers["gclient"] +
                               headers["gapp-version"] + headers["gtime"] + headers["gnonce"] + body_json;
    headers["gsign"] = sm.sign_data(sign_source);
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

        auto headers = build_headers_v2(info, body_str, true);
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"),
                "Challenge请求头: gkey-no=%s gtime=%s gnonce=%s",
                headers.count("gkey-no")? headers["gkey-no"].c_str():"(empty)",
                headers["gtime"].c_str(), headers["gnonce"].c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "Challenge签名前20:%s", headers["gsign"].substr(0,20).c_str());
        std::string url = info.api_base_url + "/api/adevice/v1/auth/challenge";
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "Challenge HTTP返回码:%ld", http_res.status_code);
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP错误:" + http_res.error.message;
            return res;
        }
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP状态:" + std::to_string(http_res.status_code);
            RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "Challenge错误响应正文: %s", http_res.text.c_str());
            return res;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "响应JSON解析失败";
            return res;
        }
        // 兼容两种返回：直接包含字段或 data 包裹
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "Challenge原始响应前200: %s",
                     http_res.text.substr(0, 200).c_str());
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        res.challenge = data.value("challenge", "");
        res.exp = data.value("exp", static_cast<int64_t>(0));
        // 有些服务端不返回 createTime，这里做兜底：
        // - 优先读取 createTime/create_time/serverTime 字段
        // - 若缺失或为0，使用当前本地时间作为生成时间起点，避免被误判为过期
        {
            int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            if (data.contains("createTime")) {
                res.create_time = data.value("createTime", static_cast<int64_t>(0));
            } else if (data.contains("create_time")) {
                res.create_time = data.value("create_time", static_cast<int64_t>(0));
            } else if (data.contains("serverTime")) {
                res.create_time = data.value("serverTime", static_cast<int64_t>(0));
            } else {
                res.create_time = 0;
            }
            if (res.create_time <= 0) {
                RCLCPP_WARN(rclcpp::get_logger("TokenManager"), "⚠️ Challenge缺少生成时间，使用本地时间兜底");
                res.create_time = now;
            }
            // 兼容不同有效期字段命名
            if (res.exp <= 0) {
                if (data.contains("expire")) res.exp = data.value("expire", static_cast<int64_t>(0));
                else if (data.contains("ttl")) res.exp = data.value("ttl", static_cast<int64_t>(0));
                else if (data.contains("expireInMs")) res.exp = data.value("expireInMs", static_cast<int64_t>(0));
            }
        }
        res.success = !res.challenge.empty();
        if (!res.success) {
            res.err_msg = "Challenge缺失";
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

        auto headers = build_headers_v2(info, body_str, true);
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"),
                "Token请求头: gkey-no=%s gtime=%s gnonce=%s",
                headers.count("gkey-no")? headers["gkey-no"].c_str():"(empty)",
                headers["gtime"].c_str(), headers["gnonce"].c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "Token签名前20:%s", headers["gsign"].substr(0,20).c_str());
        std::string url = info.api_base_url + "/api/adevice/v1/auth/getToken";
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "Token HTTP返回码:%ld", http_res.status_code);
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP错误:" + http_res.error.message;
            return res;
        }
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP状态:" + std::to_string(http_res.status_code);
            RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "Token错误响应正文: %s", http_res.text.c_str());
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
        } else {
            RCLCPP_INFO(rclcpp::get_logger("TokenManager"),
                        "✅ 获取JWT成功：前24=%s，exp=%ldms",
                        res.jwt_token.substr(0, 24).c_str(), res.exp);
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "获取JWT令牌异常:" + std::string(e.what());
    }
    return res;
#endif
}

// 刷新MQTT连接令牌（真实实现）
TokenResult TokenManager::refresh_mqtt_token(const DeviceInfo& info, const std::string& current_token) {
#if USE_MOCK_TOKEN
    // Mock：直接返回新的模拟令牌
    TokenResult res;
    res.jwt_token = current_token + "_REFRESHED";
    res.exp = 3600000;
    res.success = true;
    return res;
#else
    RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "【真实模式】调用刷新JWT令牌接口");
    TokenResult res;
    try {
        nlohmann::json body;
        body["sn"] = info.vin;
        body["model"] = info.model;
        std::string body_str = body.dump();

        auto headers = build_headers_v2(info, body_str, true, current_token);
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"),
                "Refresh请求头: gkey-no=%s gtime=%s gnonce=%s",
                headers.count("gkey-no")? headers["gkey-no"].c_str():"(empty)",
                headers["gtime"].c_str(), headers["gnonce"].c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("TokenManager"), "Refresh签名前20:%s", headers["gsign"].substr(0,20).c_str());
        std::string url = info.api_base_url + "/api/adevice/v1/auth/refreshToken";
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        RCLCPP_INFO(rclcpp::get_logger("TokenManager"), "Refresh HTTP返回码:%ld", http_res.status_code);
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP错误:" + http_res.error.message;
            return res;
        }
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP状态:" + std::to_string(http_res.status_code);
            RCLCPP_ERROR(rclcpp::get_logger("TokenManager"), "Refresh错误响应正文: %s", http_res.text.c_str());
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
        } else {
            RCLCPP_INFO(rclcpp::get_logger("TokenManager"),
                        "✅ 刷新JWT成功：前24=%s，exp=%ldms",
                        res.jwt_token.substr(0, 24).c_str(), res.exp);
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
    // 使用服务端返回的真实生成时间，若缺失/为0则使用当前时间兜底
    int64_t challenge_create_time = challenge_res.create_time;
    if (challenge_create_time <= 0) {
        RCLCPP_WARN(rclcpp::get_logger("TokenManager"), "⚠️ Challenge生成时间为0，使用当前时间兜底校验");
        challenge_create_time = now;
    }
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