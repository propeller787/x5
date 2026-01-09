#pragma once

#include <functional>
#include <string>
#include <cstdint>

namespace ota_agent {

struct IntegrityContext {
    std::string up_id;
    std::string ecu_id;
    std::string version;
    std::string url;
    std::string staged_path;
    std::string expected_md5;
    std::string expected_sha256;
    std::string expected_version;
    std::string signature;            // base64-encoded
    std::string signature_algorithm;  // RSA | ECDSA
    std::string signature_pubkey_path;
    std::string signature_payload;    // plain text payload to verify
    std::vector<std::string> required_nodes_l3;
    std::vector<std::string> required_nodes_l4;
    uint64_t file_size = 0;
    std::function<std::string(const std::string&, std::string&)> md5_calculator;
    std::function<std::string(const std::string&, std::string&)> sha256_calculator;
};

class IntegrityValidator {
public:
    enum class Layer { L1, L2, L3, L4 };

    struct ValidationResult {
        bool ok = true;
        Layer layer = Layer::L1;
        std::string message;
    };

    using LayerCallback = std::function<ValidationResult(const IntegrityContext&)>;

    IntegrityValidator();
    void set_layer_callback(Layer layer, LayerCallback cb);
    ValidationResult run_layer(Layer layer, const IntegrityContext& ctx) const;
    bool run_all(const IntegrityContext& ctx, ValidationResult& fail) const;

private:
    LayerCallback l1_;
    LayerCallback l2_;
    LayerCallback l3_;
    LayerCallback l4_;
};

} // namespace ota_agent
