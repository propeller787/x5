#pragma once

#include <memory>
#include <functional>
#include <string>
#include <cstdint>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "ota_agent/can_transport.hpp"

namespace ota_agent {
class ICanIdPolicy; // 前置声明

enum class Endpoint {
    Domain,   // 域控 3588
    Upper,    // 上控
    Lower     // 下控
};

struct ControllerCommConfig {
    uint32_t domain_can_id = 0x3588;
    uint32_t upper_can_id  = 0x0510;
    uint32_t lower_can_id  = 0x0520;
    size_t   frame_payload_bytes = 8; // 每帧有效载荷字节数
};

struct FeedbackEvent {
    enum class Type { Progress, Success, Failure } type;
    int progress = 0;              // 0-100 （仅 Progress）
    std::string reason;            // 失败原因（仅 Failure）
    uint32_t source_can_id = 0;    // 反馈来源
};

using FeedbackCallback = std::function<void(const FeedbackEvent&)>;
using RawFrameCallback = std::function<void(const CanFrame&)>;

class IControllerComm {
public:
    virtual ~IControllerComm() = default;
    virtual void set_feedback_callback(FeedbackCallback cb) = 0;
    virtual void add_raw_listener(RawFrameCallback cb) = 0;
    virtual bool send_pull_metadata(Endpoint ep,
                                    const std::string& version,
                                    const std::string& url,
                                    const std::string& md5,
                                    uint64_t size,
                                    std::string& err) = 0;
    virtual bool send_push_file(Endpoint ep,
                                const std::string& version,
                                const std::string& file_path,
                                std::string& err) = 0;
};

class CanControllerComm : public IControllerComm {
public:
    CanControllerComm(std::shared_ptr<ICanTransport> can,
                      const ControllerCommConfig& cfg,
                      rclcpp::Logger logger);
    ~CanControllerComm() override = default;

    void set_feedback_callback(FeedbackCallback cb) override;
    void add_raw_listener(RawFrameCallback cb) override;
    // 可选：设置CAN ID策略，实现与ID分配的完全解耦
    void set_id_policy(std::shared_ptr<const ICanIdPolicy> policy) { id_policy_ = std::move(policy); }
    bool send_pull_metadata(Endpoint ep,
                            const std::string& version,
                            const std::string& url,
                            const std::string& md5,
                            uint64_t size,
                            std::string& err) override;
    bool send_push_file(Endpoint ep,
                        const std::string& version,
                        const std::string& file_path,
                        std::string& err) override;

private:
    uint32_t id_of(Endpoint ep) const;
    bool send_chunk(uint32_t can_id, const std::vector<uint8_t>& data, std::string& err);
    bool send_tlv(uint32_t can_id, char type, const std::string& data, std::string& err);
    void handle_rx(const CanFrame& f);

    std::shared_ptr<ICanTransport> can_;
    ControllerCommConfig cfg_;
    rclcpp::Logger logger_;
    FeedbackCallback cb_;
    std::vector<RawFrameCallback> raw_listeners_;
    std::mutex raw_mutex_;
    std::shared_ptr<const ICanIdPolicy> id_policy_;
};

} // namespace ota_agent
