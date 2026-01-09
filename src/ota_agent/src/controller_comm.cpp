#include "ota_agent/controller_comm.hpp"
#include "ota_agent/can_id_policy.hpp"

#include <fstream>
#include <array>
#include <algorithm>
#include <rcpputils/filesystem_helper.hpp>

namespace ota_agent {

CanControllerComm::CanControllerComm(std::shared_ptr<ICanTransport> can,
                                     const ControllerCommConfig& cfg,
                                     rclcpp::Logger logger)
    : can_(std::move(can)), cfg_(cfg), logger_(logger)
{
    if (can_) {
        can_->set_receive_handler([this](const CanFrame& f){ this->handle_rx(f); });
    }
}

void CanControllerComm::set_feedback_callback(FeedbackCallback cb) { cb_ = std::move(cb); }

void CanControllerComm::add_raw_listener(RawFrameCallback cb) {
    std::lock_guard<std::mutex> lk(raw_mutex_);
    raw_listeners_.push_back(std::move(cb));
}

uint32_t CanControllerComm::id_of(Endpoint ep) const {
    // 优先使用策略；否则回落到静态配置
    if (id_policy_) {
        uint32_t id = 0;
        if (id_policy_->tx_id_for(ep, id)) return id;
    }
    switch (ep) {
        case Endpoint::Domain: return cfg_.domain_can_id;
        case Endpoint::Upper:  return cfg_.upper_can_id;
        case Endpoint::Lower:  return cfg_.lower_can_id;
    }
    return cfg_.domain_can_id;
}

bool CanControllerComm::send_chunk(uint32_t can_id, const std::vector<uint8_t>& data, std::string& err) {
    if (!can_) { err = "no can transport"; return false; }
    size_t pos = 0;
    const size_t step = std::max<size_t>(1, cfg_.frame_payload_bytes);
    while (pos < data.size()) {
        const size_t n = std::min(step, data.size() - pos);
        CanFrame f; f.id = can_id; f.is_extended = (can_id > 0x7FF); f.dlc = static_cast<uint8_t>(n);
        f.data.insert(f.data.end(), data.begin() + pos, data.begin() + pos + n);
        if (!can_->send(f, err)) return false;
        pos += n;
    }
    return true;
}

bool CanControllerComm::send_tlv(uint32_t can_id, char type, const std::string& data, std::string& err) {
    // TLV: type(1) + value split into frames of 7 bytes
    const size_t chunk = (cfg_.frame_payload_bytes > 1) ? (cfg_.frame_payload_bytes - 1) : 1;
    size_t pos = 0;
    if (data.empty()) {
        std::vector<uint8_t> v; v.reserve(1);
        v.push_back(static_cast<uint8_t>(type));
        return send_chunk(can_id, v, err);
    }
    while (pos < data.size()) {
        size_t n = std::min(chunk, data.size() - pos);
        std::vector<uint8_t> v; v.reserve(n + 1);
        v.push_back(static_cast<uint8_t>(type));
        v.insert(v.end(), data.begin() + pos, data.begin() + pos + n);
        if (!send_chunk(can_id, v, err)) return false;
        pos += n;
    }
    return true;
}

bool CanControllerComm::send_pull_metadata(Endpoint ep,
                                           const std::string& version,
                                           const std::string& url,
                                           const std::string& md5,
                                           uint64_t size,
                                           std::string& err) {
    const auto id = id_of(ep);
    // Begin
    {
        std::vector<uint8_t> begin{ static_cast<uint8_t>('B') };
        if (!send_chunk(id, begin, err)) return false;
    }
    if (!send_tlv(id, 'V', version, err)) return false;
    if (!send_tlv(id, 'U', url, err)) return false;
    if (!send_tlv(id, 'M', md5, err)) return false;
    std::string size_str = size ? std::to_string(size) : std::string("");
    if (!send_tlv(id, 'S', size_str, err)) return false;
    // End
    {
        std::vector<uint8_t> end{ static_cast<uint8_t>('E') };
        if (!send_chunk(id, end, err)) return false;
    }
    return true;
}

bool CanControllerComm::send_push_file(Endpoint ep,
                                       const std::string& version,
                                       const std::string& file_path,
                                       std::string& err) {
    const auto id = id_of(ep);
    // meta: version + 0 + size
    uint64_t file_size = 0;
    try {
        file_size = rcpputils::fs::file_size(rcpputils::fs::path(file_path));
    } catch (const std::exception& e) {
        err = std::string("stat_failed:") + e.what();
        return false;
    }
    std::string size_str = std::to_string(file_size);
    std::vector<uint8_t> meta;
    meta.insert(meta.end(), version.begin(), version.end());
    meta.push_back(0);
    meta.insert(meta.end(), size_str.begin(), size_str.end());
    if (!send_chunk(id, meta, err)) return false;

    // data: chunked file content
    std::ifstream ifs(file_path, std::ios::binary);
    if (!ifs.is_open()) { err = "open_file_failed"; return false; }
    std::array<uint8_t, 4096> buf{};
    while (ifs.good()) {
        ifs.read(reinterpret_cast<char*>(buf.data()), buf.size());
        std::streamsize got = ifs.gcount();
        if (got <= 0) break;
        size_t pos = 0;
        while (pos < static_cast<size_t>(got)) {
            size_t n = std::min(cfg_.frame_payload_bytes, static_cast<size_t>(got) - pos);
            std::vector<uint8_t> slice(buf.begin() + pos, buf.begin() + pos + n);
            if (!send_chunk(id, slice, err)) return false;
            pos += n;
        }
    }
    return true;
}

void CanControllerComm::handle_rx(const CanFrame& f) {
    {
        std::lock_guard<std::mutex> lk(raw_mutex_);
        for (auto& listener : raw_listeners_) {
            if (listener) listener(f);
        }
    }

    if (!cb_) return;
    if (f.data.empty()) return;
    char t = static_cast<char>(f.data[0]);
    FeedbackEvent ev{}; ev.source_can_id = f.id;
    if (t == 'P' && f.data.size() >= 2) {
        ev.type = FeedbackEvent::Type::Progress;
        ev.progress = static_cast<int>(f.data[1]);
    } else if (t == 'S') {
        ev.type = FeedbackEvent::Type::Success;
    } else if (t == 'F') {
        ev.type = FeedbackEvent::Type::Failure;
        if (f.data.size() > 1) ev.reason.assign(reinterpret_cast<const char*>(&f.data[1]), f.data.size()-1);
    } else {
        return; // ignore unknown
    }
    cb_(ev);
}

} // namespace ota_agent
