// include/cloud_manager/token_manager.hpp
#pragma once
#include <string>
#include <map>
#include <cstdint>

namespace cloud_manager {

// 编译宏：默认启用模拟模式（开发阶段）
#ifndef USE_MOCK_TOKEN
#define USE_MOCK_TOKEN 0
#endif

// 设备信息（基础）
struct DeviceInfo {
    std::string vin = "VIN_001";  // 设备唯一标识
    std::string model = "X5";     // 设备型号

    // 根密钥对（内置，用于获取会话密钥 / 设备身份认证）
    // 使用提供的Ed25519密钥（Base64或十六进制）
    std::string root_private_key = "ydSKNkOGdKx0ITtDp2GtTN9MChiDo6SQXSSRRusli0U=";
    std::string root_public_key = "p1c0dmOw6enDGkks5nwuwmr7mMpQm3xfDjrSDl7R0dg="; // 目前仅用于说明，验签在服务端

    // 兼容旧字段：默认等同于根私钥
    std::string private_key = "MOCK_ED25519_PRIVATE_KEY";

    // 会话密钥对（动态获取，用于业务数据签名/验签）
    std::string session_private_key;
    std::string session_public_key;

    std::string api_base_url = "http://39.107.107.204:22000"; // 认证服务域名
    std::string client = "vehicle";    // gclient
    std::string app_version = "1.0";   // gapp-version
    std::string language = "zh-CN";    // Accept-Language
    std::string version = "1";         // gversion
    // 当前使用的密钥编号（服务端返回，用于后续签名头 gkey-no）
    std::string key_no;
};

// 挑战应答结果（防重放攻击核心）
struct ChallengeResult {
    std::string challenge;  // 服务器生成的随机数
    int64_t exp;            // 有效期（毫秒）
    int64_t create_time;    // 服务端生成时间（毫秒时间戳）
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

// 会话密钥结果（设备启动/重置后获取一次）
struct SessionKeyResult {
    std::string public_key;   // 会话公钥，用于验签
    std::string private_key;  // 会话私钥，用于加签
    bool success{false};
    std::string err_msg;
    std::string key_no;       // 服务端分配的密钥编号
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

    // 获取MQTT连接令牌（简化版：直接用已有 token 头调用 /api/auth/v1/getMqttToken）
    TokenResult get_mqtt_token_v1(const DeviceInfo& info, const std::string& base_token);

    // 获取会话密钥对（设备开机/重置时调用）
    SessionKeyResult get_session_key(const DeviceInfo& info);

    // 刷新MQTT连接令牌（令牌即将过期时调用）
    TokenResult refresh_mqtt_token(const DeviceInfo& info, const std::string& current_token);

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

    // Mock实现：模拟会话密钥（开发阶段）
    SessionKeyResult mock_get_session_key(const DeviceInfo& info);
};

} // namespace cloud_manager