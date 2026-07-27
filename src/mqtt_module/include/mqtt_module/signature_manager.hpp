#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <sodium.h>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace mqtt_module {

struct SignatureConfig {
    std::string private_key = "";
    std::string public_key = "";
    std::string sign_separator = "|";
    bool enforce_strict = false;
};

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

static inline bool b64_to_bytes(const std::string& b64, std::vector<unsigned char>& out) {
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

static inline std::string generate_seed_hex() {
    std::vector<unsigned char> seed(crypto_sign_SEEDBYTES);
    randombytes_buf(seed.data(), seed.size());
    std::ostringstream oss;
    for (size_t i = 0; i < seed.size(); ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0') << (int)seed[i];
    }
    return oss.str();
}

class SignatureManager {
public:
    SignatureManager(const SignatureConfig& config, rclcpp::Logger logger)
        : config_(config), logger_(logger), use_sodium_(false) {
        if (sodium_init() < 0) {
            RCLCPP_WARN(logger_, "libsodium init failed, using mock signature");
            use_sodium_ = false;
            return;
        }

        if (!config_.private_key.empty()) {
            std::vector<unsigned char> sk;
            bool parsed = false;
            
            // 首先尝试十六进制解析
            if (hex_to_bytes(config_.private_key, sk)) {
                parsed = true;
            } else {
                // 十六进制解析失败，尝试 Base64 解析
                if (b64_to_bytes(config_.private_key, sk)) {
                    parsed = true;
                }
            }
            
            if (parsed) {
                if (sk.size() == crypto_sign_SECRETKEYBYTES) {
                    secret_key_ = sk;
                    if (config_.public_key.empty()) {
                        public_key_.assign(secret_key_.begin() + crypto_sign_PUBLICKEYBYTES, secret_key_.end());
                    }
                    use_sodium_ = true;
                } else if (sk.size() == crypto_sign_SEEDBYTES) {
                    secret_key_.resize(crypto_sign_SECRETKEYBYTES);
                    public_key_.resize(crypto_sign_PUBLICKEYBYTES);
                    crypto_sign_seed_keypair(public_key_.data(), secret_key_.data(), sk.data());
                    use_sodium_ = true;
                } else {
                    RCLCPP_WARN(logger_, "private key length invalid, fallback to mock");
                    use_sodium_ = false;
                }
            } else {
                RCLCPP_WARN(logger_, "private key parse failed, fallback to mock");
            }
        } else if (!config_.public_key.empty()) {
            if (hex_to_bytes(config_.public_key, public_key_)) {
                if (public_key_.size() != crypto_sign_PUBLICKEYBYTES) {
                    RCLCPP_WARN(logger_, "public key length invalid, verify disabled");
                    public_key_.clear();
                } else {
                    use_sodium_ = true;
                }
            }
        }

        if (!use_sodium_) {
            if (config_.enforce_strict) {
                RCLCPP_WARN(logger_, "strict mode without key, signing limited");
            } else {
                secret_key_.resize(crypto_sign_SECRETKEYBYTES);
                public_key_.resize(crypto_sign_PUBLICKEYBYTES);
                if (crypto_sign_keypair(public_key_.data(), secret_key_.data()) == 0) {
                    use_sodium_ = true;
                    RCLCPP_INFO(logger_, "generated ephemeral Ed25519 keypair");
                }
            }
        } else {
            RCLCPP_INFO(logger_, "SignatureManager enabled with libsodium");
        }
    }

    std::string sign_data(const std::string& raw_data) {
        if (!use_sodium_ || secret_key_.empty()) {
            std::string mock_sign = "MOCK_ED25519_" + std::to_string(raw_data.size()) + "_" +
                                   std::to_string(std::hash<std::string>{}(raw_data));
            return mock_sign;
        }

        unsigned char sig[crypto_sign_BYTES];
        auto do_sign = [&]() {
            return crypto_sign_detached(sig, nullptr,
                                        reinterpret_cast<const unsigned char*>(raw_data.data()), raw_data.size(),
                                        secret_key_.data());
        };

        int rc = do_sign();
        if (rc != 0) rc = do_sign();
        if (rc != 0) {
            if (config_.enforce_strict) {
                throw std::runtime_error("libsodium sign failed (strict mode)");
            }
            return std::string("MOCK_SIGN_FAIL");
        }

        std::vector<unsigned char> sig_vec(sig, sig + crypto_sign_BYTES);
        size_t b64_len = sodium_base64_encoded_len(sig_vec.size(), sodium_base64_VARIANT_ORIGINAL);
        std::string b64(b64_len, '\0');
        sodium_bin2base64(b64.data(), b64.size(), sig_vec.data(), sig_vec.size(), sodium_base64_VARIANT_ORIGINAL);
        if (!b64.empty() && b64.back() == '\0') b64.pop_back();
        return b64;
    }

    std::string pack_signed_data(const std::string& raw_data, const std::string& signature) {
        return raw_data + config_.sign_separator + signature;
    }

    bool verify_data(const std::string& raw_data, const std::string& signature) {
        if (!use_sodium_ || public_key_.empty()) {
            if (config_.enforce_strict) {
                return false;
            }
            return true;
        }

        std::vector<unsigned char> sig_bytes;
        if (!b64_to_bytes(signature, sig_bytes) || sig_bytes.size() != crypto_sign_BYTES) {
            return false;
        }

        if (crypto_sign_verify_detached(sig_bytes.data(),
                                       reinterpret_cast<const unsigned char*>(raw_data.data()), raw_data.size(),
                                       public_key_.data()) != 0) {
            return false;
        }
        return true;
    }

private:
    SignatureConfig config_;
    rclcpp::Logger logger_;
    bool use_sodium_;
    std::vector<unsigned char> secret_key_;
    std::vector<unsigned char> public_key_;
};

} // namespace mqtt_module
