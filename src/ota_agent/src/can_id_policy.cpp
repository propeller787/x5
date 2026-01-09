#include "ota_agent/can_id_policy.hpp"

namespace ota_agent {

bool PictureCanIdPolicy::tx_id_for(Endpoint ep, uint32_t& out_can_id) const {
    switch (ep) {
        case Endpoint::Upper: // 上控接收 0x702
            out_can_id = 0x702; return true;
        case Endpoint::Lower: // 驱动器接收 0x701
            out_can_id = 0x701; return true;
        case Endpoint::Domain:
            // 域控作为主控，一般不向“域控自身”发送；这里返回广播以便上层按需使用
            out_can_id = 0x7DF; return true;
        default:
            return false;
    }
}

std::vector<uint32_t> PictureCanIdPolicy::peer_tx_ids_for(Endpoint ep) const {
    switch (ep) {
        case Endpoint::Upper:  return {0x70A}; // 上控回传
        case Endpoint::Lower:  return {0x709}; // 驱动器回传
        case Endpoint::Domain: return {0x708, 0x709, 0x70A}; // 域控关注来自所有对端
        default:               return {};
    }
}

} // namespace ota_agent
