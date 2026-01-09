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
    RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "start get trusted token");
    const int max_retries = 3;
    int retry_count = 0;
    while (retry_count < max_retries) {
        ChallengeResult challenge_res = get_challenge(info);
        if (!challenge_res.success) {
            TokenResult res;
            res.success = false;
            res.err_msg = "challenge failed:" + challenge_res.err_msg;
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "%s", res.err_msg.c_str());
            return res;
        }

        if (!check_challenge_exp(challenge_res)) {
            TokenResult res;
            res.success = false;
            res.err_msg = "challenge expired";
            RCLCPP_ERROR(rclcpp::get_logger("mqtt_module::TokenManager"), "%s", res.err_msg.c_str());
            return res;
        }

        TokenResult jwt_res = get_jwt_token(info, challenge_res);
        if (jwt_res.success) {
            return jwt_res;
        } else if (jwt_res.err_msg == "challenge expired") {
            retry_count++;
            RCLCPP_WARN(rclcpp::get_logger("mqtt_module::TokenManager"), "challenge expired retry %d/%d", retry_count, max_retries);
        } else {
            return jwt_res;
        }
    }
    TokenResult res;
    res.success = false;
    res.err_msg = "challenge retries exhausted";
    return res;
}

SessionKeyResult TokenManager::get_session_key(const DeviceInfo& info) {
#if USE_MOCK_TOKEN
    return mock_get_session_key(info);
#else
    RCLCPP_INFO(rclcpp::get_logger("mqtt_module::TokenManager"), "real get session key");
    SessionKeyResult res;
    try {
        nlohmann::json body;
        body["vin"] = info.vin;
        body["model"] = info.model;
        std::string body_str = body.dump();

        auto headers = build_headers(info, body_str, false);
        std::string url = info.api_base_url + "/api/auth/v1/device/getSessionKey";
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP error:" + http_res.error.message;
            return res;
        }
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP status:" + std::to_string(http_res.status_code);
            return res;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "json parse failed";
            return res;
        }
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        res.public_key = data.value("publicKey", std::string(""));
        res.private_key = data.value("privateKey", std::string(""));
        res.success = !res.public_key.empty() && !res.private_key.empty();
        if (!res.success) {
            res.err_msg = "publicKey/privateKey missing";
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "session key exception:" + std::string(e.what());
    }
    return res;
#endif
}

static cpr::Header build_headers(const DeviceInfo& info, const std::string& body_json, bool use_session_key) {
    cpr::Header headers;
    headers["Content-Type"] = "application/json; charset=UTF-8";
    headers["Gversion"] = info.version;
    headers["Glanguage"] = info.language;
    headers["GClient"] = info.client;
    headers["GApp-Version"] = info.app_version;
    auto now_ms = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    headers["Gtime"] = now_ms;

    std::uniform_int_distribution<uint64_t> dis;
    static thread_local std::mt19937_64 gen(std::random_device{}());
    uint64_t a = dis(gen), b = dis(gen);
    std::ostringstream ss;
    ss << std::hex << std::setw(16) << std::setfill('0') << a
       << std::hex << std::setw(16) << std::setfill('0') << b;
    std::string nonce = ss.str();
    headers["Gnonce"] = nonce;

    const std::string& signing_key = (use_session_key && !info.session_private_key.empty())
                                         ? info.session_private_key
                                         : (!info.root_private_key.empty() ? info.root_private_key
                                                                           : info.private_key);

    SignatureConfig cfg;
    cfg.private_key = signing_key;
    SignatureManager sm(cfg, rclcpp::get_logger("mqtt_module::TokenManager"));

    std::string sign_source = headers["Gversion"] + headers["Glanguage"] + headers["GClient"] +
                               headers["GApp-Version"] + headers["Gtime"] + headers["Gnonce"] + body_json;
    headers["GSign"] = sm.sign_data(sign_source);
    return headers;
}

ChallengeResult TokenManager::get_challenge(const DeviceInfo& info) {
#if USE_MOCK_TOKEN
    return mock_get_challenge(info);
#else
    ChallengeResult res;
    try {
        nlohmann::json body;
        body["vin"] = info.vin;
        body["model"] = info.model;
        std::string body_str = body.dump();

        auto headers = build_headers(info, body_str, true);
        std::string url = info.api_base_url + "/api/auth/v1/mqtt/device/challenge";
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP error:" + http_res.error.message;
            return res;
        }
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP status:" + std::to_string(http_res.status_code);
            return res;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "json parse failed";
            return res;
        }
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
        res.challenge = data.value("challenge", "");
        res.exp = data.value("exp", static_cast<int64_t>(0));
        res.create_time = data.value("createTime", static_cast<int64_t>(0));
        res.success = !res.challenge.empty();
        if (!res.success) {
            res.err_msg = "challenge missing";
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "get challenge exception:" + std::string(e.what());
    }
    return res;
#endif
}

TokenResult TokenManager::get_jwt_token(const DeviceInfo& info, const ChallengeResult& challenge_res) {
#if USE_MOCK_TOKEN
    return mock_get_jwt_token(info, challenge_res);
#else
    TokenResult res;
    try {
        nlohmann::json body;
        body["vin"] = info.vin;
        body["model"] = info.model;
        body["challenge"] = challenge_res.challenge;
        std::string body_str = body.dump();

        auto headers = build_headers(info, body_str, true);
        std::string url = info.api_base_url + "/api/auth/v1/mqtt/device/getToken";
        auto http_res = cpr::Post(cpr::Url{url}, headers, cpr::Body{body_str}, cpr::Timeout{5000});
        if (http_res.error) {
            res.success = false;
            res.err_msg = "HTTP error:" + http_res.error.message;
            return res;
        }
        if (http_res.status_code != 200) {
            res.success = false;
            res.err_msg = "HTTP status:" + std::to_string(http_res.status_code);
            return res;
        }

        auto json_resp = nlohmann::json::parse(http_res.text, nullptr, false);
        if (json_resp.is_discarded()) {
            res.success = false;
            res.err_msg = "json parse failed";
            return res;
        }
        nlohmann::json data = json_resp.contains("data") ? json_resp["data"] : json_resp;
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
            res.err_msg = "token missing";
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.err_msg = "get jwt exception:" + std::string(e.what());
    }
    return res;
#endif
}

bool TokenManager::check_challenge_exp(const ChallengeResult& challenge_res) {
    int64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    int64_t challenge_create_time = challenge_res.create_time;
    bool is_valid = (now - challenge_create_time) < challenge_res.exp;
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
