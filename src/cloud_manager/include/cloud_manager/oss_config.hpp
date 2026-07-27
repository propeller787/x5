// include/cloud_manager/token_manager.hpp
#pragma once
#include <string>
#include <map>
#include <cstdint>

namespace cloud_manager {

// 编译宏：默认启用模拟模式（开发阶段）
#ifndef USE_MOCK_TOKEN
#define USE_MOCK_TOKEN 1
#endif

// 设备信息（基础）
struct DeviceInfo {
    std::string vin = "VIN_001";  // 设备唯一标识
    std::string model = "x5";     // 设备型号
    std::string private_key = "MOCK_ED25519_PRIVATE_KEY";  // 设备私钥（Mock）
};

// 挑战应答结果（防重放攻击核心）
struct ChallengeResult {
    std::string challenge;  // 服务器生成的随机数
    int64_t exp;            // 有效期（毫秒）
    bool success;           // 请求是否成功
    std::string err_msg;    // 错误信息
};

// JWT令牌结果（MQTT连接凭证）
struct TokenResult {
    std::string jwt_token;  // JWT格式连接令牌
    int64_t exp;            // 令牌有效期（毫秒）
    bool success;           // 请求是否成功
    std::string err_msg;    // 错误信息
};

// HTTP请求/响应（通用）
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

// 令牌管理器（核心：挑战应答→JWT令牌）
class TokenManager {
public:
    // 核心流程：发送设备信息→获取challenge→获取JWT令牌（对外统一接口）
    TokenResult get_trusted_token(const DeviceInfo& info);

private:
    // 步骤1：获取挑战应答（防重放攻击）
    ChallengeResult get_challenge(const DeviceInfo& info);

    // 步骤2：用challenge获取JWT令牌
    TokenResult get_jwt_token(const DeviceInfo& info, const ChallengeResult& challenge_res);

    // 辅助：Ed25519签名（Mock/真实兼容）
    std::string ed25519_sign(const DeviceInfo& info, const std::string& content);

    // 辅助：校验challenge有效期
    bool check_challenge_exp(const ChallengeResult& challenge_res);

    // 辅助：发送HTTP请求（Mock/真实兼容）
    HttpResponse send_http_request(const HttpRequest& req);

    // Mock实现：模拟挑战应答（开发阶段）
    ChallengeResult mock_get_challenge(const DeviceInfo& info);
    // Mock实现：模拟JWT令牌（开发阶段）
    TokenResult mock_get_jwt_token(const DeviceInfo& info, const ChallengeResult& challenge_res);
};

} // namespace cloud_manager