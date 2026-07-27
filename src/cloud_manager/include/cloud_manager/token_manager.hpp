// include/cloud_manager/token_manager.hpp
#pragma once
#include <string>
#include <map>
#include <cstdint>
#include <cpr/cpr.h>

namespace cloud_manager {

// 编译宏：默认启用模拟模式（开发阶段）
#ifndef USE_MOCK_TOKEN
#define USE_MOCK_TOKEN 0
#endif

// 设备信息（基础）
struct DeviceInfo {
    std::string vin = "ZQKJ001";  // 设备唯一标识
    std::string model = "x5";     // 设备型号

    // 根密钥对（内置，用于获取会话密钥 / 设备身份认证）
    std::string root_private_key = "MOCK_ED25519_PRIVATE_KEY";
    std::string root_public_key = "MOCK_ED25519_PUBLIC_KEY"; // 用于验签服务端响应

    // gkey-no: 根密钥编号（格式：2;ZQKJ001;3）
    std::string gkey_no = "2;ZQKJ001;3";
    
    // 会话密钥编号（格式：1;ZQKJ001;7）
    std::string session_key_no = "1;ZQKJ001;7";

    // 兼容旧字段：默认等同于根私钥
    std::string private_key = "MOCK_ED25519_PRIVATE_KEY";

    // 会话密钥对（动态获取，用于业务数据签名/验签）
    std::string session_private_key;
    std::string session_public_key;

    std::string api_base_url = "http://10.66.66.10:22000"; // 认证服务域名
    std::string client = "vehicle";  // gclient
    std::string app_version = "1.0";     // gapp-version
    std::string language = "zh-cn";        // glanguage
    std::string version = "1";           // gversion
};

// 挑战应答结果（防重放攻击核心）
struct ChallengeResult {
    std::string challenge;  // 服务器生成的随机数
    int64_t exp;            // 有效期（毫秒）
    int64_t create_time;    // 服务端生成时间（毫秒时间戳）
    int64_t gtime;         // 响应头 gtime（用于检查 Challenge 是否过期）
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
    std::string key_no;       // 密钥对编号
    bool success{false};
    std::string err_msg;
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

    // 获取会话密钥对（设备开机/重置时调用）
    SessionKeyResult get_session_key(const DeviceInfo& info);

    // 刷新JWT令牌（令牌即将过期时调用）
    TokenResult refresh_jwt_token(const DeviceInfo& info, const std::string& current_token);

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

    // 内部：构造通用头（含签名）
    static cpr::Header build_headers(const DeviceInfo& info, const std::string& body_json, bool use_session_key, const std::string& token);

    // Mock实现：模拟挑战应答（开发阶段）
    ChallengeResult mock_get_challenge(const DeviceInfo& info);
    // Mock实现：模拟JWT令牌（开发阶段）
    TokenResult mock_get_jwt_token(const DeviceInfo& info, const ChallengeResult& challenge_res);

    // Mock实现：模拟会话密钥（开发阶段）
    SessionKeyResult mock_get_session_key(const DeviceInfo& info);
};

} // namespace cloud_manager