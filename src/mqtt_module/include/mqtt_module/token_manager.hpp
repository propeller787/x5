#pragma once
#include <string>
#include <map>
#include <cstdint>

namespace mqtt_module {

#ifndef USE_MOCK_TOKEN
#define USE_MOCK_TOKEN 0
#endif

struct DeviceInfo {
    std::string vin = "VIN_001";
    std::string model = "X5";
    std::string root_private_key = "MOCK_ED25519_PRIVATE_KEY";
    std::string root_public_key;
    std::string private_key = "MOCK_ED25519_PRIVATE_KEY";
    std::string session_private_key;
    std::string session_public_key;
    std::string api_base_url = "https://your-api-server.com";
    std::string client = "cloud_manager";
    std::string app_version = "0.0.1";
    std::string language = "zh-cn";
    std::string version = "1.0";
};

struct ChallengeResult {
    std::string challenge;
    int64_t exp;
    int64_t create_time;
    bool success;
    std::string err_msg;
};

struct TokenResult {
    std::string jwt_token;
    int64_t exp;
    bool success;
    std::string err_msg;
};

struct SessionKeyResult {
    std::string public_key;
    std::string private_key;
    bool success{false};
    std::string err_msg;
};

struct HttpRequest {
    std::string url;
    std::map<std::string, std::string> headers;
    std::map<std::string, std::string> params;
};
struct HttpResponse {
    int code;
    std::string body;
    bool success;
};

class TokenManager {
public:
    TokenResult get_trusted_token(const DeviceInfo& info);
    SessionKeyResult get_session_key(const DeviceInfo& info);

private:
    ChallengeResult get_challenge(const DeviceInfo& info);
    TokenResult get_jwt_token(const DeviceInfo& info, const ChallengeResult& challenge_res);
    std::string ed25519_sign(const DeviceInfo& info, const std::string& content);
    bool check_challenge_exp(const ChallengeResult& challenge_res);
    HttpResponse send_http_request(const HttpRequest& req);
    ChallengeResult mock_get_challenge(const DeviceInfo& info);
    TokenResult mock_get_jwt_token(const DeviceInfo& info, const ChallengeResult& challenge_res);
    SessionKeyResult mock_get_session_key(const DeviceInfo& info);
};

} // namespace mqtt_module
