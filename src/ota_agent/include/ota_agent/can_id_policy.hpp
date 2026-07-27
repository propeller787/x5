#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include "ota_agent/controller_comm.hpp"

namespace ota_agent {

// 抽象：根据角色（Upper/Lower/Domain...）给出发送/接收CAN ID，完全解耦调用方
class ICanIdPolicy {
public:
    virtual ~ICanIdPolicy() = default;

    // 获取向指定端点发送报文所使用的目标 CAN ID（即对端的“接收ID”）
    virtual bool tx_id_for(Endpoint ep, uint32_t& out_can_id) const = 0;

    // 获取来自该端点回传的报文 CAN ID 列表（即对端的“发送ID们”）。用于监听或过滤，可选。
    virtual std::vector<uint32_t> peer_tx_ids_for(Endpoint ep) const = 0;

    // 广播 ID（域控用于广播）
    virtual uint32_t broadcast_id() const = 0;
};

// 按图片所示的默认策略：
// - Upper(上控)：接收ID 0x702，发送ID 0x70A
// - Lower(驱动器)：接收ID 0x701，发送ID 0x709
// - Battery(不在Endpoint中)：接收ID 0x700，发送ID 0x708（预留，未用）
// - Domain(域控)：广播 0x7DF；本端监听对端发送ID（708/709/70A）
class PictureCanIdPolicy : public ICanIdPolicy {
public:
    ~PictureCanIdPolicy() override = default;

    bool tx_id_for(Endpoint ep, uint32_t& out_can_id) const override;
    std::vector<uint32_t> peer_tx_ids_for(Endpoint ep) const override;
    uint32_t broadcast_id() const override { return 0x7DF; }
};

// 可选辅助：将“高6位节点ID + 低5位命令ID”组装成标准ID（11位）
inline uint32_t compose_can_id(uint8_t node6, uint8_t cmd5) {
    return (static_cast<uint32_t>(node6 & 0x3F) << 5) | static_cast<uint32_t>(cmd5 & 0x1F);
}

} // namespace ota_agent
