#include "mqtt_module/token_manager.hpp"
#include "mqtt_module/signature_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <random>
#include <sstream>
#include <iomanip>

namespace mqtt_module {

static cpr::Header build_headers(const DeviceInfo& info, const std::string& body_json, bool use_session_key);

TokenResult TokenManager::get_trusted_token(const DeviceInfo& info) {
    RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "===== 开始获取可信令牌流程 =====");
    const int max_retries = 3;
    int retry_count = 0;
    while (retry_count < max_retries) {
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "尝试次数 %d/%d", retry_count + 1, max_retries);
        
        // 步骤1: 获取会话密钥对
        // SessionKeyResult session_key_res = get_session_key(info);
        // if (!session_key_res.success) {
        //     TokenResult res;
        //     res.success = false;
        //     res.err_msg = "获取会话密钥失败:" + session_key_res.err_msg;
        //     RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "%s", res.err_msg.c_str());
        //     return res;
        // }
        // RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "✅ 会话密钥获取成功，keyNo: %s", session_key_res.keyNo.c_str());
        
        // // 更新设备信息与会话密钥
        DeviceInfo updated_info = info;
        // updated_info.session_public_key = session_key_res.public_key;
        // updated_info.session_private_key = session_key_res.private_key;
        // updated_info.session_key_no = session_key_res.keyNo;
        
        // 步骤2: 获取挑战应答
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "开始获取挑战应答...");
        ChallengeResult challenge_res = get_challenge(updated_info);
        if (!challenge_res.success) {
            TokenResult res;
            res.success = false;
            res.err_msg = "challenge failed:" + challenge_res.err_msg;
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "%s", res.err_msg.c_str());
            return res;
        }
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "✅ 挑战应答获取成功，challenge: %s, exp: %ld ms", challenge_res.challenge.c_str(), challenge_res.exp);

        if (!check_challenge_exp(challenge_res)) {
            TokenResult res;
            res.success = false;
            res.err_msg = "challenge expired";
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "%s", res.err_msg.c_str());
            return res;
        }

        // 步骤3: 获取JWT令牌
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "开始获取JWT令牌...");
        TokenResult jwt_res = get_jwt_token(updated_info, challenge_res);
        if (jwt_res.success) {
            RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "✅ JWT令牌获取成功，有效期: %ld ms", jwt_res.exp);
            RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "===== 获取可信令牌流程完成 =====");
            return jwt_res;
        } else if (jwt_res.err_msg == "challenge expired") {
            retry_count++;
            RCLCPP_WARN(rclcpp::get_logger("mqtt_module::TokenManager"), "challenge expired，重试 %d/%d", retry_count, max_retries);
        } else {
            return jwt_res;
        }
    }
    TokenResult res;
    res.success = false;
    res.err_msg = "challenge retries exhausted";
    RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "%s", res.err_msg.c_str());
    RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "===== 获取可信令牌流程失败 =====");
    return res;
}

SessionKeyResult TokenManager::get_session_key(const DeviceInfo& info) {
#if USE_MOCK_TOKEN
    return mock_get_session_key(info);
#else
    RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "===== 开始获取会话密钥 =====");
    SessionKeyResult res;
    try {
        nlohmann::json body;
        body["sn"] = info.vin;
        body["model"] = info.model;
        std::string body_str = body.dump();
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "📤 会话密钥请求 - sn:%s, model:%s", info.vin.c_str(), info.model.c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("mqtt_module::TokenManager"), "📤 会话密钥请求体:%s", body_str.c_str());

        auto headers = build_headers(info, body_str, false);
        RCLCPP_DEBUG(rclcpp::get_logger("mqtt_module::TokenManager"), "请求头: gkey-no: %s, gtime: %s, gnonce: %s", 
                     headers["gkey-no"].c_str(), headers["gtime"].c_str(), headers["gnonce"].c_str());
                      
        std::string url = info.api_base_url + "/api/adevice/v1/auth/getSessionKey";
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "调用接口: %s", url.c_str());
        
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP error:" + http_res.error.message;
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "HTTP错误: %s", res.err_msg.c_str());
            return res;
        }
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "HTTP响应状态码: %ld", http_res.status_code);
        
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP status:" + std::to_string(http_res.status_code);
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "HTTP状态码错误: %s", res.err_msg.c_str());
            return res;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("mqtt_module::TokenManager"), "响应体: %s", http_res.text.c_str());
        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "json parse failed";
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "JSON解析失败: %s", res.err_msg.c_str());
            return res;
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("mqtt_module::TokenManager"), "响应体: %s", http_res.text.c_str());
        
        // 检查是否返回错误
        if (json_resp.contains("code") && json_resp["code"].is_number()) {
            int code = json_resp["code"];
            if (code != 0) {
                res.success = false;
                res.err_msg = json_resp.value("msg", std::string("未知错误"));
                RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "❌ 会话密钥API返回错误 - code:%d, msg:%s", code, res.err_msg.c_str());
                return res;
            }
        }
        
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        if (data.is_null()) {
            res.success = false;
            res.err_msg = "响应数据为空";
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "❌ 会话密钥响应数据为空");
            return res;
        }
        
        // 安全获取字段值，避免null值异常
        res.public_key = data.contains("publicKey") && !data["publicKey"].is_null() ? data["publicKey"].get<std::string>() : std::string("");
        res.private_key = data.contains("privateKey") && !data["privateKey"].is_null() ? data["privateKey"].get<std::string>() : std::string("");
        res.keyNo = data.contains("keyNo") && !data["keyNo"].is_null() ? data["keyNo"].get<std::string>() : std::string("");
        res.success = !res.public_key.empty() && !res.private_key.empty();
        
        if (res.success) {
            RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "✅ 会话密钥获取成功，keyNo: %s", res.keyNo.c_str());
            RCLCPP_DEBUG(rclcpp::get_logger("mqtt_module::TokenManager"), "公钥: %s", res.public_key.c_str());
            RCLCPP_DEBUG(rclcpp::get_logger("mqtt_module::TokenManager"), "私钥: %s", res.private_key.c_str());
        } else {
            res.err_msg = "publicKey/privateKey missing";
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "会话密钥缺失: %s", res.err_msg.c_str());
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "session key exception:" + std::string(e.what());
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "异常: %s", res.err_msg.c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "===== 获取会话密钥完成 =====");
    return res;
#endif
}

static cpr::Header build_headers(const DeviceInfo& info, const std::string& body_json, bool use_session_key) {
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

ChallengeResult TokenManager::get_challenge(const DeviceInfo& info) {
#if USE_MOCK_TOKEN
    return mock_get_challenge(info);
#else
    RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "===== 开始获取挑战应答 =====");
    ChallengeResult res;
    try {
        nlohmann::json body;
        body["sn"] = info.vin;
        body["model"] = info.model;
        std::string body_str = body.dump();
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "📤 Challenge请求参数 - sn:%s, model:%s", info.vin.c_str(), info.model.c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("mqtt_module::TokenManager"), "📤 Challenge请求体:%s", body_str.c_str());

        auto headers = build_headers(info, body_str, true);
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "请求头: gkey-no: %s, gtime: %s, gnonce: %s", headers["gkey-no"].c_str(), headers["gtime"].c_str(), headers["gnonce"].c_str());
        std::string url = info.api_base_url + "/api/adevice/v1/auth/challenge";
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "调用接口: %s", url.c_str());
        
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP error:" + http_res.error.message;
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "HTTP错误: %s", res.err_msg.c_str());
            return res;
        }
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "HTTP响应状态码: %ld", http_res.status_code);
        RCLCPP_DEBUG(rclcpp::get_logger("mqtt_module::TokenManager"), "响应体: %s", http_res.text.c_str());
        
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP status:" + std::to_string(http_res.status_code);
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "HTTP状态码错误: %s", res.err_msg.c_str());
            return res;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "响应JSON解析失败";
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "❌ 挑战应答JSON解析失败");
            return res;
        }
        
        // 检查是否返回错误
        if (json_resp.contains("code") && json_resp["code"].is_number()) {
            int code = json_resp["code"];
            if (code != 0) {
                res.success = false;
                res.err_msg = json_resp.value("msg", std::string("未知错误"));
                RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "❌ 挑战应答API返回错误 - code:%d, msg:%s", code, res.err_msg.c_str());
                return res;
            }
        }
        
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        if (data.is_null()) {
            res.success = false;
            res.err_msg = "响应数据为空";
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "❌ 挑战应答响应数据为空");
            return res;
        }
        
        res.challenge = data.value("challenge", "");
        res.exp = data.value("exp", static_cast<int64_t>(0));
        res.create_time = data.value("createTime", static_cast<int64_t>(0));
        
        // 获取响应头中的gtime作为challenge的生成时间
        res.gtime = http_res.header.count("gtime") ? std::stoll(http_res.header["gtime"]) : 0;
        
        res.success = !res.challenge.empty();
        if (res.success) {
            RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "✅ 挑战应答获取成功，challenge: %s, exp: %ld ms, gtime: %ld", res.challenge.c_str(), res.exp, res.gtime);
        } else {
            res.err_msg = "challenge missing";
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "挑战应答缺失: %s", res.err_msg.c_str());
        }
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "===== 获取挑战应答完成 =====");
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "get challenge exception:" + std::string(e.what());
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "异常: %s", res.err_msg.c_str());
    }
    return res;
#endif
}

TokenResult TokenManager::get_jwt_token(const DeviceInfo& info, const ChallengeResult& challenge_res) {
#if USE_MOCK_TOKEN
    return mock_get_jwt_token(info, challenge_res);
#else
    RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "===== 开始获取JWT令牌 =====");
    TokenResult res;
    try {
        nlohmann::json body;
        body["sn"] = info.vin;
        body["model"] = info.model;
        body["challenge"] = challenge_res.challenge;
        std::string body_str = body.dump();
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "📤 Token请求参数 - sn:%s, model:%s, challenge:%s", info.vin.c_str(), info.model.c_str(), challenge_res.challenge.substr(0, 20).c_str());
        RCLCPP_DEBUG(rclcpp::get_logger("mqtt_module::TokenManager"), "📤 Token请求体:%s", body_str.c_str());

        auto headers = build_headers(info, body_str, true);
        
        // 打印完整请求头
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "📤 Token请求头完整信息:");
        for (const auto& header : headers) {
            RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "  %s: %s", header.first.c_str(), header.second.c_str());
        }
        
        std::string url = info.api_base_url + "/api/adevice/v1/auth/getToken";
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "📡 Token API URL: %s", url.c_str());
        
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP error:" + http_res.error.message;
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "HTTP错误: %s", res.err_msg.c_str());
            return res;
        }
        
        // 打印HTTP响应状态码
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "📥 会话密钥HTTP返回码:%ld", http_res.status_code);
        
        // 打印完整响应头
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "📥 会话密钥响应头完整信息:");
        for (const auto& header : http_res.header) {
            RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "  %s: %s", header.first.c_str(), header.second.c_str());
        }
        
        // 打印响应体
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "📥 会话密钥响应体:%s", http_res.text.c_str());
        
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP status:" + std::to_string(http_res.status_code);
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "HTTP状态码错误: %s", res.err_msg.c_str());
            return res;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "json parse failed";
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "JSON解析失败: %s", res.err_msg.c_str());
            return res;
        }
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        if (data.contains("error") && data["error"].is_string()) {
            std::string error = data["error"];
            if (error == "challenge expired" || error == "challenge invalid") {
                res.success = false;
                res.err_msg = "challenge expired";
                RCLCPP_WARN(rclcpp::get_logger("mqtt_module::TokenManager"), "challenge已过期或无效: %s", error.c_str());
                return res;
            }
        }
        res.jwt_token = data.value("token", data.value("jwt_token", std::string("")));
        res.exp = data.value("exp", static_cast<int64_t>(0));
        res.success = !res.jwt_token.empty();
        if (res.success) {
            RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "✅ JWT令牌获取成功，有效期: %ld ms", res.exp);
            RCLCPP_DEBUG(rclcpp::get_logger("mqtt_module::TokenManager"), "令牌内容: %s", res.jwt_token.c_str());
        } else {
            res.err_msg = "token missing";
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "令牌缺失: %s", res.err_msg.c_str());
        }
        RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "===== 获取JWT令牌完成 =====");
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "get jwt exception:" + std::string(e.what());
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "异常: %s", res.err_msg.c_str());
    }
    return res;
#endif
}

bool TokenManager::check_challenge_exp(const ChallengeResult& challenge_res) {
    int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    
    // 使用 Challenge 的实际获取时间（从响应头 gtime 获取）
    // 如果 challenge_res.gtime 为 0，说明响应中没有 gtime 字段
    // 此时应该使用当前时间作为 Challenge 的获取时间
    int64_t challenge_create_time = (challenge_res.gtime == 0) ? now : challenge_res.gtime;
    bool is_valid = (now - challenge_create_time) < challenge_res.exp;
    
    RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "校验Challenge有效期：%s（当前时间%ld，生成时间%ld，有效期%ld）",
                is_valid ? "有效" : "过期", now, challenge_create_time, challenge_res.exp);
    return is_valid;
}

std::string TokenManager::ed25519_sign(const DeviceInfo& info, const std::string& content) {
#if USE_MOCK_TOKEN
    std::string mock_sign = "MOCK_ED25519_SIGN_" + content.substr(0, 16);
    return mock_sign;
#else
    return "REAL_ED25519_SIGN_" + content.substr(0, 16);
#endif
}

HttpResponse TokenManager::send_http_request(const HttpRequest& req) {
#if USE_MOCK_TOKEN
    HttpResponse res;
    res.code = 200;
    res.body = "success";
    res.success = true;
    return res;
#else
    HttpResponse res;
    res.code = 200;
    res.body = R"({"code":0,"data":{"challenge":"REAL_CHALLENGE_987654","exp":30000}})";
    res.success = true;
    return res;
#endif
}

ChallengeResult TokenManager::mock_get_challenge(const DeviceInfo& info) {
    ChallengeResult res;
    res.challenge = "MOCK_CHALLENGE_123456";
    res.exp = 30000;
    res.create_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    res.success = true;
    return res;
}

TokenResult TokenManager::mock_get_jwt_token(const DeviceInfo& info, const ChallengeResult& challenge_res) {
    TokenResult res;
    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    if (dis(gen) < 0.3) {
        res.success = false;
        res.err_msg = "challenge expired";
        return res;
    }
    res.jwt_token = "MOCK_JWT_TOKEN_eyJhbGciOiJFZDI1NTE5IiwidHlwIjoiSldUIn0.eyJ2aW4iOiJWSU5fMDAxIiwiZXhwIjoxNzY0MzE3MjY5fQ.xxx";
    res.exp = 3600000;
    res.success = true;
    return res;
}

SessionKeyResult TokenManager::mock_get_session_key(const DeviceInfo& info) {
    SessionKeyResult res;
    res.public_key = "MOCK_SESSION_PUB_HEX";
    res.private_key = "MOCK_SESSION_PRIV_HEX";
    res.success = true;
    return res;
}

} // namespace mqtt_module
