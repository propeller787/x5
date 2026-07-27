#include "ota_agent/integrity_validator.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <sstream>

namespace ota_agent {

namespace {
IntegrityValidator::ValidationResult default_l1(const IntegrityContext& ctx) {
    IntegrityValidator::ValidationResult r;
    r.layer = IntegrityValidator::Layer::L1;
    namespace fs = std::filesystem;
    if (ctx.staged_path.empty()) {
        r.ok = false;
        r.message = "staged_path_missing";
        return r;
    }
    std::error_code ec;
    if (!fs::exists(ctx.staged_path, ec)) {
        r.ok = false;
        r.message = "file_not_found";
        return r;
    }
    auto size = fs::file_size(ctx.staged_path, ec);
    if (ec) {
        r.ok = false;
        r.message = "stat_failed";
        return r;
    }
    if (size == 0) {
        r.ok = false;
        r.message = "zero_size";
        return r;
    }
    // 可选：额外 SHA256 比对
    if (!ctx.expected_sha256.empty()) {
        if (!ctx.sha256_calculator) {
            r.ok = false;
            r.message = "sha256_calculator_missing";
            return r;
        }
        std::string err;
        auto sha = ctx.sha256_calculator(ctx.staged_path, err);
        if (!err.empty()) {
            r.ok = false;
            r.message = std::string("sha256_calc_failed:") + err;
            return r;
        }
        if (sha != ctx.expected_sha256) {
            r.ok = false;
            r.message = std::string("sha256_mismatch:got=") + sha;
            return r;
        }
    }

    r.ok = true;
    return r;
}

IntegrityValidator::ValidationResult default_l2(const IntegrityContext& ctx) {
    IntegrityValidator::ValidationResult r;
    r.layer = IntegrityValidator::Layer::L2;
    if (ctx.version.empty()) {
        r.ok = false;
        r.message = "version_missing";
        return r;
    }
    if (ctx.url.empty()) {
        r.ok = false;
        r.message = "url_missing";
        return r;
    }
    auto lower_url = ctx.url;
    std::transform(lower_url.begin(), lower_url.end(), lower_url.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    if (!(lower_url.rfind("http://", 0) == 0 || lower_url.rfind("https://", 0) == 0)) {
        r.ok = false;
        r.message = "url_scheme_invalid";
        return r;
    }
    if (!ctx.expected_md5.empty()) {
        if (ctx.expected_md5.size() != 32) {
            r.ok = false;
            r.message = "md5_length_invalid";
            return r;
        }
        bool hex_only = std::all_of(ctx.expected_md5.begin(), ctx.expected_md5.end(), [](unsigned char c){
            return std::isxdigit(c) != 0;
        });
        if (!hex_only) {
            r.ok = false;
            r.message = "md5_not_hex";
            return r;
        }
    }
    r.ok = true;
    return r;
}

IntegrityValidator::ValidationResult default_l3(const IntegrityContext& ctx) {
    IntegrityValidator::ValidationResult r;
    r.layer = IntegrityValidator::Layer::L3;
    if (!ctx.expected_md5.empty()) {
        if (!ctx.md5_calculator) {
            r.ok = false;
            r.message = "md5_calculator_missing";
            return r;
        }
        std::string err;
        auto md5 = ctx.md5_calculator(ctx.staged_path, err);
        if (!err.empty()) {
            r.ok = false;
            r.message = std::string("md5_calc_failed:") + err;
            return r;
        }
        if (md5 != ctx.expected_md5) {
            r.ok = false;
            r.message = std::string("md5_mismatch:got=") + md5;
            return r;
        }
    }
    // 版本比对（若提供）
    if (!ctx.expected_version.empty()) {
        if (ctx.version != ctx.expected_version) {
            r.ok = false;
            r.message = std::string("version_mismatch:expect=") + ctx.expected_version + " got=" + ctx.version;
            return r;
        }
    }
    r.ok = true;
    return r;
}

IntegrityValidator::ValidationResult default_l4(const IntegrityContext& ctx) {
    IntegrityValidator::ValidationResult r;
    r.layer = IntegrityValidator::Layer::L4;
    if (ctx.staged_path.empty()) {
        r.ok = true;
        r.message = "skip_no_path";
        return r;
    }
    namespace fs = std::filesystem;
    std::error_code ec;
    auto space_info = fs::space(fs::path(ctx.staged_path).parent_path(), ec);
    if (ec) {
        r.ok = false;
        r.message = "space_query_failed";
        return r;
    }
    uint64_t size = ctx.file_size;
    if (size == 0) {
        size = fs::file_size(ctx.staged_path, ec);
        if (ec) {
            r.ok = false;
            r.message = "stat_failed";
            return r;
        }
    }
    // Require at least 2x image size available to avoid filling disk.
    if (space_info.available < size * 2) {
        r.ok = false;
        r.message = "insufficient_space";
        return r;
    }

    // 运行态节点检查（如有配置）
    if (!ctx.required_nodes_l4.empty()) {
        if (ctx.sha256_calculator == nullptr && ctx.md5_calculator == nullptr) {
            // 占位以保持接口对称；无须依赖
        }
        // 在外部自定义回调中使用 required_nodes_l4，默认实现仅通过
    }

    r.ok = true;
    return r;
}

} // namespace

IntegrityValidator::IntegrityValidator() {
    l1_ = default_l1;
    l2_ = default_l2;
    l3_ = default_l3;
    l4_ = default_l4;
}

void IntegrityValidator::set_layer_callback(Layer layer, LayerCallback cb) {
    switch (layer) {
        case Layer::L1: l1_ = std::move(cb); break;
        case Layer::L2: l2_ = std::move(cb); break;
        case Layer::L3: l3_ = std::move(cb); break;
        case Layer::L4: l4_ = std::move(cb); break;
    }
}

IntegrityValidator::ValidationResult IntegrityValidator::run_layer(Layer layer, const IntegrityContext& ctx) const {
    switch (layer) {
        case Layer::L1: return l1_ ? l1_(ctx) : ValidationResult{true, Layer::L1, "noop"};
        case Layer::L2: return l2_ ? l2_(ctx) : ValidationResult{true, Layer::L2, "noop"};
        case Layer::L3: return l3_ ? l3_(ctx) : ValidationResult{true, Layer::L3, "noop"};
        case Layer::L4: return l4_ ? l4_(ctx) : ValidationResult{true, Layer::L4, "noop"};
    }
    return ValidationResult{true, layer, "noop"};
}

bool IntegrityValidator::run_all(const IntegrityContext& ctx, ValidationResult& fail) const {
    // 暂移除L4检测
    // const Layer layers[] = {Layer::L1, Layer::L2, Layer::L3, Layer::L4};
    const Layer layers[] = {Layer::L1, Layer::L2, Layer::L3};
    for (auto layer : layers) {
        auto res = run_layer(layer, ctx);
        if (!res.ok) {
            fail = res;
            return false;
        }
    }
    return true;
}

} // namespace ota_agent
