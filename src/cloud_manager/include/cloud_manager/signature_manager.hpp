#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <sodium.h>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace cloud_manager {

// 签名配置（支持传入私钥/公钥的 hex 或 Base64 编码）
struct SignatureConfig {
    std::string private_key = ""; // 私钥（hex/Base64），可为空（退化为mock）
    std::string public_key = "";  // 公钥（hex/Base64），可为空（若私钥包含公钥则从私钥派生）
    std::string sign_separator = "|";                     // 数据与签名的分隔符
    bool enforce_strict = false; // 若为 true：生产模式，缺少公钥/libsodium 则验签失败
};

// 内部辅助：hex <-> bytes
static inline bool hex_to_bytes(const std::string& hex, std::vector<unsigned char>& out) {
    if (hex.size() % 2 != 0) return false;
    out.clear(); out.reserve(hex.size() / 2);
    for (size_t i = 0; i < hex.size(); i += 2) {
        unsigned int byte;
        std::istringstream iss(hex.substr(i,2));
        if (!(iss >> std::hex >> byte)) return false;
        out.push_back(static_cast<unsigned char>(byte));
    }
    return true;
}

// 内部辅助：Base64 -> bytes（返回是否成功）
static inline bool b64_to_bytes(const std::string& b64, std::vector<unsigned char>& out) {
    // 预留最大长度：Base64 放大约 4/3；这里直接分配同等长度缓冲
    out.assign(b64.size(), 0);
    size_t out_len = 0;
    if (sodium_base642bin(out.data(), out.size(),
                          b64.c_str(), b64.size(),
                          nullptr, &out_len, nullptr,
                          sodium_base64_VARIANT_ORIGINAL) != 0) {
        out.clear();
        return false;
    }
    out.resize(out_len);
    return true;
}

static inline std::string bytes_to_hex(const unsigned char* data, size_t len) {
    std::ostringstream oss;
    for (size_t i = 0; i < len; ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0') << (int)data[i];
    }
    return oss.str();
}

// 生成随机 seed（32 字节）并以 hex 返回，方便测试时在节点初始化阶段传入私钥 seed
static inline std::string generate_seed_hex() {
    std::vector<unsigned char> seed(crypto_sign_SEEDBYTES);
    randombytes_buf(seed.data(), seed.size());
    std::ostringstream oss;
    for (size_t i = 0; i < seed.size(); ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0') << (int)seed[i];
    }
    return oss.str();
}

// 签名工具类（使用 libsodium 的 Ed25519）
class SignatureManager {
public:
    SignatureManager(const SignatureConfig& config, rclcpp::Logger logger)
        : config_(config), logger_(logger), use_sodium_(false) {
        // 初始化 libsodium（可多次调用，内部保证幂等）
        if (sodium_init() < 0) {
            RCLCPP_WARN(logger_, "⚠️ libsodium 初始化失败，回退到 mock 签名");
            use_sodium_ = false;
            return;
        }

        // 解析私钥（支持 seed(32 bytes) 或 secretkey(64 bytes)，来源可为 hex 或 Base64）
        if (!config_.private_key.empty()) {
            std::vector<unsigned char> sk;
            bool parsed = false;
            if (hex_to_bytes(config_.private_key, sk)) {
                parsed = true;
            } else {
                // 尝试Base64
                if (b64_to_bytes(config_.private_key, sk)) parsed = true;
            }
            if (parsed) {
                if (sk.size() == crypto_sign_SECRETKEYBYTES) {
                    secret_key_ = sk;
                    // 若未提供公钥，则从 secret key 的后半段获取
                    if (config_.public_key.empty()) {
                        public_key_.assign(secret_key_.begin() + crypto_sign_PUBLICKEYBYTES, secret_key_.end());
                    }
                    use_sodium_ = true;
                } else if (sk.size() == crypto_sign_SEEDBYTES) {
                    // 从 seed 生成 keypair
                    secret_key_.resize(crypto_sign_SECRETKEYBYTES);
                    public_key_.resize(crypto_sign_PUBLICKEYBYTES);
                    crypto_sign_seed_keypair(public_key_.data(), secret_key_.data(), sk.data());
                    use_sodium_ = true;
                } else {
                    RCLCPP_WARN(logger_, "⚠️ 提供的私钥长度不符合 Ed25519 要求，将回退到 mock 签名");
                    use_sodium_ = false;
                }
            } else {
                RCLCPP_WARN(logger_, "⚠️ 无法解析私钥（既非hex也非Base64），将回退到 mock 签名");
            }
        } else if (!config_.public_key.empty()) {
            // 只提供了公钥（用于验签），解析公钥（hex 或 Base64）
            if (!hex_to_bytes(config_.public_key, public_key_)) {
                std::vector<unsigned char> pk_b64;
                if (b64_to_bytes(config_.public_key, pk_b64)) public_key_.swap(pk_b64);
            }
            if (!public_key_.empty()) {
                if (public_key_.size() != crypto_sign_PUBLICKEYBYTES) {
                    RCLCPP_WARN(logger_, "⚠️ 提供的公钥长度不正确，公钥验签将不可用");
                    public_key_.clear();
                } else {
                    // 可以用于验签，但不能用于加签
                    use_sodium_ = true;
                }
            }
        }

        // 如果没有传入私钥/公钥，但 libsodium 可用，则生成一对临时 keypair（便于测试/本地）
        if (!use_sodium_) {
            // 若启用了严格模式（生产），则不自动生成临时 keypair
            if (config_.enforce_strict) {
                RCLCPP_WARN(logger_, "⚠️ 严格模式启用且未提供密钥，签名功能受限（生产模式请注入密钥）");
            } else {
                // try to generate ephemeral keypair for non-strict/dev mode
                secret_key_.resize(crypto_sign_SECRETKEYBYTES);
                public_key_.resize(crypto_sign_PUBLICKEYBYTES);
                if (crypto_sign_keypair(public_key_.data(), secret_key_.data()) == 0) {
                    use_sodium_ = true;
                    RCLCPP_INFO(logger_, "✅ SignatureManager 自动生成临时 Ed25519 keypair（libsodium 已启用）");
                }
            }
        } else {
            RCLCPP_INFO(logger_, "✅ SignatureManager 已启用 libsodium Ed25519 支持");
        }
    }

    // 对数据进行 Ed25519 签名，返回 hex 编码的签名；若未启用 sodium 则返回 mock 字符串
    std::string sign_data(const std::string& raw_data) {
        if (!use_sodium_ || secret_key_.empty()) {
            // fallback mock
            std::string mock_sign = "MOCK_ED25519_" + std::to_string(raw_data.size()) + "_" +
                                   std::to_string(std::hash<std::string>{}(raw_data));
            RCLCPP_INFO(logger_, "✅ 数据加签完成（mock，签名前20位：%s）", mock_sign.substr(0,20).c_str());
            return mock_sign;
        }

        unsigned char sig[crypto_sign_BYTES];
        auto do_sign = [&](void) {
            return crypto_sign_detached(sig, nullptr,
                                        reinterpret_cast<const unsigned char*>(raw_data.data()), raw_data.size(),
                                        secret_key_.data());
        };

        // 简单重试一次，避免偶发失败；严格模式下两次失败直接报错
        int rc = do_sign();
        if (rc != 0) rc = do_sign();
        if (rc != 0) {
            RCLCPP_ERROR(logger_, "❌ libsodium 签名失败，两次重试后仍失败");
            if (config_.enforce_strict) {
                throw std::runtime_error("libsodium sign failed (strict mode)");
            }
            return std::string("MOCK_SIGN_FAIL");
        }
        // 输出 Base64（与服务端示例保持一致）
        std::vector<unsigned char> sig_vec(sig, sig + crypto_sign_BYTES);
        size_t b64_len = sodium_base64_encoded_len(sig_vec.size(), sodium_base64_VARIANT_ORIGINAL);
        std::string b64(b64_len, '\0');
        sodium_bin2base64(b64.data(), b64.size(), sig_vec.data(), sig_vec.size(), sodium_base64_VARIANT_ORIGINAL);
        // 移除可能的尾部\0
        if (!b64.empty() && b64.back() == '\0') b64.pop_back();
        RCLCPP_INFO(logger_, "✅ 数据加签完成（Base64前20位：%s）", b64.substr(0,20).c_str());
        return b64;
    }

    // 拼接原始数据+签名（服务端按分隔符拆分）
    std::string pack_signed_data(const std::string& raw_data, const std::string& signature) {
        return raw_data + config_.sign_separator + signature;
    }

    // 验签：signature 仅支持 Base64
    bool verify_data(const std::string& raw_data, const std::string& signature) {
        if (!use_sodium_ || public_key_.empty()) {
            if (config_.enforce_strict) {
                RCLCPP_ERROR(logger_, "❌ 验签不可用（缺少公钥或libsodium），严格模式下拒绝消息");
                return false;
            }
            RCLCPP_WARN(logger_, "⚠️ 验签不可用（缺少公钥或libsodium），默认通过（非严格模式）");
            return true; // 保持向后兼容，非严格模式下通过
        }

        std::vector<unsigned char> sig_bytes;
        if (!b64_to_bytes(signature, sig_bytes) || sig_bytes.size() != crypto_sign_BYTES) {
            RCLCPP_ERROR(logger_, "❌ 提供的签名不是有效的 Base64 或长度错误");
            return false;
        }

        if (crypto_sign_verify_detached(sig_bytes.data(),
                                       reinterpret_cast<const unsigned char*>(raw_data.data()), raw_data.size(),
                                       public_key_.data()) != 0) {
            RCLCPP_ERROR(logger_, "❌ Ed25519 验签失败");
            return false;
        }
        RCLCPP_INFO(logger_, "✅ 验签通过");
        return true;
    }

private:
    SignatureConfig config_;
    rclcpp::Logger logger_;
    bool use_sodium_;
    std::vector<unsigned char> secret_key_; // 64 bytes if available
    std::vector<unsigned char> public_key_; // 32 bytes
};

} // namespace cloud_manager



