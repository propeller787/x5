/**
 * @file driver_upgrade_debug.cpp
 * @brief 驱动器升级独立调试工具
 *
 * 功能：
 * - 通过CAN总线向驱动器（0x701）发送固件文件
 * - 监听驱动器反馈（0x709）
 * - 支持ISO-TP协议传输
 * - 不依赖ROS 2，可独立运行
 *
 * 使用方法：
 *   ./driver_upgrade_debug <固件文件路径> [CAN接口名] [CAN ID]
 *   例如：
 *   ./driver_upgrade_debug ODriveFirmware.bin can0 0x701
 */

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// SocketCAN 头文件
#include <cstring>
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

// 简单的日志宏
#define LOG_INFO(msg) std::cout << "[INFO] " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl
#define LOG_WARN(msg) std::cerr << "[WARN] " << msg << std::endl
#define LOG_DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl

// 常量定义
namespace Constants {
constexpr uint32_t DEFAULT_DRIVER_RX_ID = 0x701;
constexpr uint32_t DEFAULT_DRIVER_TX_ID = 0x709;
constexpr size_t ISO_TP_SINGLE_FRAME_MAX = 7;
constexpr size_t ISO_TP_FIRST_FRAME_PAYLOAD = 6;
constexpr size_t ISO_TP_CONTINUOUS_FRAME_PAYLOAD = 7;
constexpr size_t CAN_FRAME_MAX_DLC = 8;
constexpr uint32_t ISO_TP_MAX_LEN = 0xFFF;
constexpr int CAN_SEND_MAX_RETRIES = 5;
constexpr int CAN_SEND_RETRY_DELAY_MS = 1;
constexpr int CAN_RX_LOOP_DELAY_MS = 5;
constexpr int CAN_STABILIZE_DELAY_MS = 100;
constexpr int DEFAULT_TIMEOUT_MS = 10000;
constexpr int FLOW_CONTROL_TIMEOUT_MS = 20000;
constexpr int FLOW_CONTROL_WAIT_MS = 1000;

// UDS服务ID
constexpr uint8_t SID_REQUEST_DOWNLOAD = 0x34;
constexpr uint8_t SID_TRANSFER_DATA = 0x36;
constexpr uint8_t SID_TRANSFER_EXIT = 0x37;
constexpr uint8_t SID_ROUTINE_CONTROL = 0x31;
constexpr uint8_t SID_ERASE_MEMORY = 0x31;

// 正响应SID
constexpr uint8_t SID_POS_RESPONSE_OFFSET = 0x40;
constexpr uint8_t SID_POS_RESPONSE_DOWNLOAD = 0x74;
constexpr uint8_t SID_POS_RESPONSE_ROUTINE = 0x71;
constexpr uint8_t SID_POS_RESPONSE_EXIT = 0x77;
constexpr uint8_t SID_POS_RESPONSE_TRANSFER_DATA = 0x76;

// 流控帧
constexpr uint8_t PCI_FLOW_CONTROL = 0x30;

// HEX文件相关
constexpr uint32_t HEX_INVALID_ADDRESS = 0xFFFFFFFF;
constexpr uint8_t HEX_FILL_BYTE = 0xFF;
}

class SimpleCanTransport {
public:
    SimpleCanTransport(const std::string& ifname)
        : ifname_(ifname)
        , sock_(-1)
        , running_(false)
    {
    }

    ~SimpleCanTransport()
    {
        close();
    }

    bool open()
    {
        sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) {
            LOG_ERROR("创建SocketCAN失败");
            return false;
        }

        struct ifreq ifr {};
        std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname_.c_str());
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
            LOG_ERROR("获取CAN接口索引失败: " << ifname_);
            ::close(sock_);
            sock_ = -1;
            return false;
        }

        struct sockaddr_can addr {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            LOG_ERROR("绑定CAN接口失败: " << ifname_);
            ::close(sock_);
            sock_ = -1;
            return false;
        }

        running_ = true;
        rx_thread_ = std::thread([this]() { rx_loop(); });
        LOG_INFO("CAN接口打开成功: " << ifname_);
        return true;
    }

    void close()
    {
        running_ = false;
        if (sock_ >= 0) {
            ::shutdown(sock_, SHUT_RDWR);
            ::close(sock_);
            sock_ = -1;
        }
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
    }

    bool send(uint32_t can_id, const std::vector<uint8_t>& data)
    {
        if (sock_ < 0) {
            LOG_ERROR("Socket未就绪");
            return false;
        }

        struct can_frame cf {};
        // 检测是否为扩展帧：优先检查CAN_EFF_FLAG标志位
        bool is_extended = (can_id & CAN_EFF_FLAG) != 0;

        // 调试信息：显示传入的can_id和判断结果
        // LOG_DEBUG("send() 收到 can_id: 0x" << std::hex << can_id
        //           << ", CAN_EFF_FLAG检查: " << ((can_id & CAN_EFF_FLAG) ? "是" : "否")
        //           << ", 判断为: " << (is_extended ? "扩展帧" : "标准帧"));

        if (is_extended) {
            // 扩展帧：29位ID，提取实际ID并设置标志位
            uint32_t actual_id = can_id & CAN_EFF_MASK;
            cf.can_id = actual_id | CAN_EFF_FLAG;
        } else {
            // 标准帧：11位ID
            cf.can_id = can_id & CAN_SFF_MASK;
        }

        cf.can_dlc = std::min<uint8_t>(static_cast<uint8_t>(data.size()), Constants::CAN_FRAME_MAX_DLC);
        for (int i = 0; i < cf.can_dlc; ++i) {
            cf.data[i] = data[i];
        }

        // 重试机制
        ssize_t n = 0;
        for (int retry = 0; retry < Constants::CAN_SEND_MAX_RETRIES; ++retry) {
            n = ::write(sock_, &cf, sizeof(cf));
            if (n == sizeof(cf)) {
                break;
            }

            if (retry < Constants::CAN_SEND_MAX_RETRIES - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(Constants::CAN_SEND_RETRY_DELAY_MS));
                LOG_WARN("CAN发送失败，重试 " << (retry + 1) << "/" << Constants::CAN_SEND_MAX_RETRIES
                                              << ": " << strerror(errno));
            } else {
                if (n < 0) {
                    LOG_ERROR("CAN发送失败(已重试" << Constants::CAN_SEND_MAX_RETRIES << "次): " << strerror(errno));
                } else {
                    LOG_ERROR("CAN发送不完整(已重试" << Constants::CAN_SEND_MAX_RETRIES << "次): 期望"
                                                     << sizeof(cf) << "字节，实际" << n << "字节");
                }
                return false;
            }
        }

        return true;
    }

    void set_rx_callback(std::function<void(uint32_t can_id, const std::vector<uint8_t>& data)> cb)
    {
        rx_cb_ = std::move(cb);
    }

private:
    void rx_loop()
    {
        while (running_) {
            struct can_frame cf {};
            ssize_t n = ::read(sock_, &cf, sizeof(cf));
            if (n == sizeof(cf)) {
                // 检测是否为扩展帧
                bool is_extended = (cf.can_id & CAN_EFF_FLAG) != 0;
                uint32_t rid = is_extended ? (cf.can_id & CAN_EFF_MASK) : (cf.can_id & CAN_SFF_MASK);
                std::vector<uint8_t> data(cf.data, cf.data + cf.can_dlc);

                if (rx_cb_) {
                    rx_cb_(rid, data);
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(Constants::CAN_RX_LOOP_DELAY_MS));
            }
        }
    }

    std::string ifname_;
    int sock_;
    std::atomic<bool> running_;
    std::thread rx_thread_;
    std::function<void(uint32_t, const std::vector<uint8_t>&)> rx_cb_;
};

class DriverUpgradeDebugger {
public:
    DriverUpgradeDebugger(const std::string& firmware_path,
        const std::string& can_ifname = "can0",
        uint32_t driver_rx_id = (uint32_t)(0x701 | CAN_EFF_FLAG),
        uint32_t driver_tx_id = (uint32_t)(0x709 | CAN_EFF_FLAG))
        : firmware_path_(firmware_path)
        , can_ifname_(can_ifname)
        , driver_rx_id_(driver_rx_id)
        , driver_tx_id_(driver_tx_id)
        , can_(can_ifname)
        , upgrade_success_(false)
        , upgrade_failed_(false)
    {
    }

    bool run()
    {
        // 1. 检查固件文件
        if (!std::filesystem::exists(firmware_path_)) {
            LOG_ERROR("固件文件不存在: " << firmware_path_);
            return false;
        }

        // 2. 打开CAN接口
        if (!can_.open()) {
            LOG_ERROR("打开CAN接口失败");
            return false;
        }

        // 3. 设置接收回调
        can_.set_rx_callback([this](uint32_t can_id, const std::vector<uint8_t>& data) {
            handle_can_rx(can_id, data);
        });

        // 4. 等待CAN接口稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(Constants::CAN_STABILIZE_DELAY_MS));

        // 5. 执行完整的刷写流程（参考ota_agent_node.cpp）
        uint32_t rx_id_actual = extract_can_id(driver_rx_id_);
        bool rx_is_extended = (driver_rx_id_ & CAN_EFF_FLAG) != 0;
        LOG_INFO("===== 开始完整刷写流程 =====");
        LOG_INFO("目标CAN ID: 0x" << std::hex << rx_id_actual << std::dec
                                  << " (" << (rx_is_extended ? "扩展帧" : "标准帧") << ")");
        LOG_INFO("固件文件: " << firmware_path_);

        bool ret = flash_firmware_complete();

        if (ret) {
            LOG_INFO("✅ 驱动器刷写成功完成！");
        } else {
            LOG_ERROR("❌ 驱动器刷写失败！");
        }

        return ret;
    }

private:
    // HEX解析辅助函数
    static int hex_char_to_int(char c)
    {
        if (c >= '0' && c <= '9')
            return c - '0';
        if (c >= 'A' && c <= 'F')
            return c - 'A' + 10;
        if (c >= 'a' && c <= 'f')
            return c - 'a' + 10;
        return -1;
    }

    static int hex_str_to_byte(const std::string& hex_str, size_t pos)
    {
        if (pos + 1 >= hex_str.size())
            return -1;
        int high = hex_char_to_int(hex_str[pos]);
        int low = hex_char_to_int(hex_str[pos + 1]);
        if (high < 0 || low < 0)
            return -1;
        return (high << 4) | low;
    }

    // 提取CAN ID的实际值（去除标志位）
    uint32_t extract_can_id(uint32_t can_id_with_flags)
    {
        if (can_id_with_flags & CAN_EFF_FLAG) {
            return can_id_with_flags & CAN_EFF_MASK;
        } else {
            return can_id_with_flags & CAN_SFF_MASK;
        }
    }

    void handle_can_rx(uint32_t can_id, const std::vector<uint8_t>& data)
    {
        // 只处理来自驱动器的反馈（比较时忽略标志位）
        uint32_t expected_id = extract_can_id(driver_tx_id_);
        if (can_id != expected_id) {
            return;
        }

        if (data.empty()) {
            return;
        }

        char cmd = static_cast<char>(data[0]);
        if (cmd == 'P' && data.size() >= 2) {
            // 进度更新
            int progress = static_cast<int>(data[1]);
            LOG_INFO("📊 升级进度: " << progress << "%");
        } else if (cmd == 'S') {
            // 成功
            LOG_INFO("✅ 收到成功反馈");
            upgrade_success_.store(true);
        } else if (cmd == 'F') {
            // 失败
            std::string reason;
            if (data.size() > 1) {
                reason.assign(reinterpret_cast<const char*>(&data[1]), data.size() - 1);
            }
            LOG_ERROR("❌ 收到失败反馈: " << reason);
            upgrade_failed_.store(true);
        }
    }

    // 准备文件信息
    struct FileInfo {
        uint32_t start_address = 0;
        uint32_t data_length = 0;
        bool is_hex_file = false;
        std::vector<uint8_t> hex_binary_data;
    };

    bool prepare_file_info(FileInfo& info)
    {
        info.is_hex_file = (firmware_path_.size() >= 4 && firmware_path_.substr(firmware_path_.size() - 4) == ".hex");

        if (info.is_hex_file) {
            std::string hex_err;
            if (!parse_hex_file(firmware_path_, info.start_address, info.data_length, hex_err)) {
                LOG_ERROR("解析HEX文件失败: " << hex_err);
                return false;
            }
            LOG_INFO("HEX文件解析 | 起始地址=0x" << std::hex << info.start_address << std::dec
                                                 << " | 数据长度=" << info.data_length << " 字节");

            std::string convert_err;
            if (!hex_to_binary(firmware_path_, info.hex_binary_data, convert_err)) {
                LOG_ERROR("HEX文件转换失败: " << convert_err);
                return false;
            }
            if (info.hex_binary_data.size() != info.data_length) {
                LOG_ERROR("HEX转换后数据长度不匹配");
                return false;
            }
        } else {
            std::error_code fs_ec;
            uint64_t file_size = std::filesystem::file_size(firmware_path_, fs_ec);
            if (fs_ec) {
                LOG_ERROR("获取文件大小失败: " << fs_ec.message());
                return false;
            }
            info.start_address = 0;
            info.data_length = static_cast<uint32_t>(file_size);
        }

        LOG_INFO("文件信息 | 起始地址=0x" << std::hex << info.start_address << std::dec
                                          << " | 数据长度=" << info.data_length << " 字节");
        return true;
    }

    static std::array<uint8_t, 4> u32_to_bytes(uint32_t v)
    {
        return { { static_cast<uint8_t>((v >> 24) & 0xFF),
            static_cast<uint8_t>((v >> 16) & 0xFF),
            static_cast<uint8_t>((v >> 8) & 0xFF),
            static_cast<uint8_t>(v & 0xFF) } };
    }

    bool wait_ack(uint8_t expect_sid, const char* step)
    {
        std::vector<uint8_t> resp;
        if (!wait_for_can_response(driver_tx_id_, expect_sid, Constants::DEFAULT_TIMEOUT_MS, resp)) {
            LOG_ERROR("[" << step << "] 等待应答超时");
            return false;
        }
        if (!is_ack_success(resp, expect_sid)) {
            LOG_ERROR("[" << step << "] ACK指示失败");
            return false;
        }
        return true;
    }

    // 完整的刷写流程
    bool flash_firmware_complete()
    {
        FileInfo file_info;
        if (!prepare_file_info(file_info)) {
            return false;
        }

        size_t chunk_payload_size = Constants::ISO_TP_CONTINUOUS_FRAME_PAYLOAD;
        uint32_t crc_val = 0;

        // 步骤1：请求升级
        LOG_INFO("步骤1: 请求升级");
        {
            std::vector<uint8_t> sf = { 0x04, Constants::SID_ROUTINE_CONTROL, 0x01, 0x02, 0x03, 0x55, 0x55, 0x55 };
            if (!can_.send(driver_rx_id_, sf) || !wait_ack(Constants::SID_POS_RESPONSE_ROUTINE, "请求升级")) {
                return false;
            }
        }

        // 步骤2：擦除 flash
        LOG_INFO("步骤2: 擦除flash");
        {
            LOG_INFO("擦除字节数: " << file_info.data_length << " 字节 (起始地址: 0x"
                                    << std::hex << file_info.start_address << std::dec << ")");
            const auto addr_bytes = u32_to_bytes(file_info.start_address);
            const auto len_bytes = u32_to_bytes(file_info.data_length);
            std::vector<uint8_t> payload = { Constants::SID_ERASE_MEMORY, 0x01, 0xFF, 0x00, 0x44,
                addr_bytes[0], addr_bytes[1], addr_bytes[2], addr_bytes[3],
                len_bytes[0], len_bytes[1], len_bytes[2], len_bytes[3] };
            if (!send_iso15765_payload(driver_rx_id_, payload, Constants::ISO_TP_CONTINUOUS_FRAME_PAYLOAD) || !wait_ack(Constants::SID_POS_RESPONSE_ROUTINE, "擦除flash")) {
                return false;
            }
        }

        // 步骤3：请求下载
        LOG_INFO("步骤3: 请求下载");
        {
            const auto addr_bytes = u32_to_bytes(file_info.start_address);
            const auto len_bytes = u32_to_bytes(file_info.data_length);
            std::vector<uint8_t> payload = { Constants::SID_REQUEST_DOWNLOAD, 0x00, 0x44,
                addr_bytes[0], addr_bytes[1], addr_bytes[2], addr_bytes[3],
                len_bytes[0], len_bytes[1], len_bytes[2], len_bytes[3] };
            if (!send_iso15765_payload(driver_rx_id_, payload, Constants::ISO_TP_FIRST_FRAME_PAYLOAD)) {
                return false;
            }

            std::vector<uint8_t> resp;
            if (!wait_for_can_response(driver_tx_id_, Constants::SID_POS_RESPONSE_DOWNLOAD,
                    Constants::DEFAULT_TIMEOUT_MS, resp)
                || resp.size() < 4) {
                LOG_ERROR("未收到有效的请求下载响应");
                return false;
            }

            uint8_t max_length = resp[3];
            if (max_length == 0) {
                LOG_ERROR("请求下载响应中的传输数据长度为0");
                return false;
            }
            chunk_payload_size = max_length;
            LOG_INFO("请求下载响应: 每次传输 " << static_cast<int>(max_length) << " 字节");
        }

        // 步骤4：传输数据
        LOG_INFO("步骤4: 传输数据（每次传输长度: " << chunk_payload_size << "字节）");
        if (!send_file_iso_tp(driver_rx_id_, firmware_path_, chunk_payload_size,
                file_info.is_hex_file, file_info.hex_binary_data, crc_val)) {
            return false;
        }

        // 步骤5：传输结束
        LOG_INFO("步骤5: 传输结束");
        {
            std::vector<uint8_t> sf = { 0x01, Constants::SID_TRANSFER_EXIT, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55 };
            if (!can_.send(driver_rx_id_, sf) || !wait_ack(Constants::SID_POS_RESPONSE_EXIT, "结束传输")) {
                return false;
            }
        }

        // 步骤6：CRC32 校验
        LOG_INFO("步骤6: CRC32校验");
        {
            const auto crc_bytes = u32_to_bytes(crc_val);
            std::vector<uint8_t> payload = { Constants::SID_ROUTINE_CONTROL, 0x01, 0x02, 0x02,
                crc_bytes[0], crc_bytes[1], crc_bytes[2], crc_bytes[3] };
            if (!send_iso15765_payload(driver_rx_id_, payload, chunk_payload_size) || !wait_ack(Constants::SID_POS_RESPONSE_ROUTINE, "CRC校验")) {
                return false;
            }
        }

        LOG_INFO("✅ 完整刷写流程成功完成");
        return true;
    }

    // 发送ISO15765格式的payload（多帧传输）
    // rx_cb_to_restore: 若非空，多帧发送内部恢复回调时使用此回调而非 handle_can_rx，避免覆盖调用方的等待回调
    bool send_iso15765_payload(uint32_t can_id, const std::vector<uint8_t>& payload, size_t chunk_payload_size,
        std::function<void(uint32_t, const std::vector<uint8_t>&)> rx_cb_to_restore = nullptr)
    {
        const uint32_t max_len = Constants::ISO_TP_MAX_LEN;
        const uint32_t total_len = static_cast<uint32_t>(std::min<size_t>(payload.size(), max_len));

        chunk_payload_size = std::min<size_t>(chunk_payload_size, Constants::ISO_TP_CONTINUOUS_FRAME_PAYLOAD);
        if (chunk_payload_size == 0)
            chunk_payload_size = Constants::ISO_TP_CONTINUOUS_FRAME_PAYLOAD;

        // 单帧发送
        if (total_len <= Constants::ISO_TP_SINGLE_FRAME_MAX) {
            std::vector<uint8_t> sf;
            sf.reserve(8);
            sf.push_back(static_cast<uint8_t>(0x00 | total_len));
            sf.insert(sf.end(), payload.begin(), payload.begin() + total_len);
            while (sf.size() < 8)
                sf.push_back(0x55);
            return can_.send(can_id, sf);
        }

        // 多帧发送
        uint32_t sent = 0;
        uint8_t len_high = static_cast<uint8_t>((total_len >> 8) & 0x0F);
        uint8_t len_low = static_cast<uint8_t>(total_len & 0xFF);
        std::vector<uint8_t> ff;
        ff.reserve(8);
        ff.push_back(static_cast<uint8_t>(0x10 | len_high));
        ff.push_back(len_low);
        size_t first_payload = std::min<size_t>(Constants::ISO_TP_FIRST_FRAME_PAYLOAD, total_len);
        ff.insert(ff.end(), payload.begin(), payload.begin() + first_payload);
        while (ff.size() < 8)
            ff.push_back(0x55);

        // 在发送首帧之前就设置好回调来等待流控帧，避免竞态条件
        uint8_t fc_bs = 0;
        uint8_t fc_stmin = 0;

        // 使用shared_ptr管理状态，确保回调中的变量始终有效
        struct FlowControlState {
            std::vector<uint8_t> resp;
            std::mutex mtx;
            std::condition_variable cv;
            std::atomic<bool> received { false };
        };
        auto fc_state = std::make_shared<FlowControlState>();
        uint32_t expected_fc_id = extract_can_id(driver_tx_id_);

        // 保存原有回调：若调用方传入 rx_cb_to_restore 则使用之，否则用 handle_can_rx（避免覆盖 wait_for_can_response 的 temp_cb）
        auto original_cb = rx_cb_to_restore ? rx_cb_to_restore : std::function<void(uint32_t, const std::vector<uint8_t>&)>([this](uint32_t can_id, const std::vector<uint8_t>& data) { handle_can_rx(can_id, data); });

        // 设置临时回调：使用值捕获shared_ptr和原有回调，确保安全
        auto fc_temp_cb = [this, fc_state, expected_fc_id, original_cb](uint32_t can_id, const std::vector<uint8_t>& data) {
            // 先调用原有回调处理消息
            original_cb(can_id, data);

            // 然后检查是否是流控帧
            if (can_id == expected_fc_id && !data.empty() && data[0] == Constants::PCI_FLOW_CONTROL) {
                std::lock_guard<std::mutex> lk(fc_state->mtx);
                fc_state->resp = data;
                fc_state->received.store(true);
                fc_state->cv.notify_one();
            }
        };
        can_.set_rx_callback(fc_temp_cb);

        // 发送首帧（回调已就绪，可立即接收流控帧）
        if (!can_.send(can_id, ff)) {
            // 发送失败，恢复原有回调
            can_.set_rx_callback(original_cb);
            return false;
        }
        sent += static_cast<uint32_t>(first_payload);

        // 等待流控帧
        bool got_fc = false;
        {
            std::unique_lock<std::mutex> lk(fc_state->mtx);
            got_fc = fc_state->cv.wait_for(lk, std::chrono::milliseconds(Constants::FLOW_CONTROL_TIMEOUT_MS), [fc_state]() {
                return fc_state->received.load();
            });
        }

        // 无论是否收到流控帧，都要恢复原有回调（重要：防止回调访问已销毁的变量）
        can_.set_rx_callback(original_cb);

        // 检查是否收到流控帧
        if (!got_fc) {
            LOG_ERROR("等待流控帧超时");
            return false;
        }

        // 解析流控帧数据
        {
            std::lock_guard<std::mutex> lk(fc_state->mtx);
            if (fc_state->resp.size() >= 3 && fc_state->resp[0] == Constants::PCI_FLOW_CONTROL) {
                fc_bs = fc_state->resp[1];
                fc_stmin = fc_state->resp[2];
                LOG_INFO("收到FC: BS=" << static_cast<int>(fc_bs) << " STmin=0x" << std::hex << static_cast<int>(fc_stmin) << std::dec);
            } else {
                LOG_ERROR("收到无效的流控帧");
                return false;
            }
        }

        auto stmin_duration = [](uint8_t st) -> std::chrono::microseconds {
            if (st <= 0x7F)
                return std::chrono::milliseconds(static_cast<int>(st));
            if (st >= 0xF1 && st <= 0xF9)
                return std::chrono::microseconds((st - 0xF0) * 100);
            return std::chrono::microseconds(0);
        };
        auto st_gap = stmin_duration(fc_stmin);
        uint8_t bs_window = fc_bs;

        uint8_t seq = 1;
        while (sent < total_len) {
            if (fc_bs != 0 && bs_window == 0) {
                std::vector<uint8_t> fc_resp;
                if (!wait_for_can_response(driver_tx_id_, Constants::PCI_FLOW_CONTROL, Constants::FLOW_CONTROL_WAIT_MS, fc_resp,
                        -1, nullptr, &original_cb)) {
                    LOG_ERROR("等待下一个流控帧超时");
                    return false;
                }
                if (fc_resp.size() >= 3 && fc_resp[0] == Constants::PCI_FLOW_CONTROL) {
                    fc_bs = fc_resp[1];
                    fc_stmin = fc_resp[2];
                    bs_window = fc_bs;
                    st_gap = stmin_duration(fc_stmin);
                } else {
                    LOG_ERROR("收到无效的流控帧");
                    return false;
                }
            }

            // 计算当前连续帧的数据长度
            uint32_t remaining = total_len - sent;
            size_t n = std::min<size_t>(chunk_payload_size, remaining);

            std::vector<uint8_t> cf;
            cf.reserve(8);
            cf.push_back(static_cast<uint8_t>(0x20 | (seq & 0x0F)));
            cf.insert(cf.end(), payload.begin() + sent, payload.begin() + sent + n);
            while (cf.size() < 8)
                cf.push_back(0x55);
            if (!can_.send(can_id, cf))
                return false;
            sent += static_cast<uint32_t>(n);
            seq = static_cast<uint8_t>((seq + 1) & 0x0F);

            if (fc_bs != 0 && bs_window > 0) {
                --bs_window;
            }
            if (st_gap.count() > 0) {
                std::this_thread::sleep_for(st_gap);
            }
        }
        return true;
    }

    // 发送文件（支持HEX和二进制文件）
    // 将文件分为多次多帧发送，每次多帧发送为chunk_payload_size大小
    // 使用 send_iso15765_payload 函数进行ISO-TP多帧传输
    bool send_file_iso_tp(uint32_t can_id, const std::string& file_path, size_t chunk_payload_size,
        bool is_hex_file, const std::vector<uint8_t>& hex_binary_data, uint32_t& crc32_val)
    {
        // chunk_payload_size 是每次传输的数据块大小（例如128字节），不是每帧的数据量
        if (chunk_payload_size == 0) {
            LOG_ERROR("chunk_payload_size 不能为0");
            return false;
        }

        // 初始化CRC32值
        crc32_val = 0xFFFFFFFFU;
        uintmax_t file_size = 0;
        if (is_hex_file) {
            file_size = hex_binary_data.size();
        } else {
            try {
                file_size = std::filesystem::file_size(file_path);
            } catch (const std::exception& e) {
                LOG_ERROR("获取文件大小失败: " << e.what());
                return false;
            }
        }

        const uint32_t total_len = static_cast<uint32_t>(file_size);
        LOG_INFO("准备发送 " << total_len << " 字节，分块大小: " << chunk_payload_size << " 字节");

        std::ifstream ifs;
        if (!is_hex_file) {
            ifs.open(file_path, std::ios::binary);
            if (!ifs.is_open()) {
                LOG_ERROR("打开文件失败");
                return false;
            }
        }

        // 读取数据的辅助函数
        auto read_chunk_data = [&](uint32_t offset, uint32_t count) -> std::vector<uint8_t> {
            std::vector<uint8_t> data;
            size_t read_count = 0;

            if (is_hex_file) {
                // 检查边界
                if (offset >= hex_binary_data.size()) {
                    LOG_ERROR("读取偏移超出范围: offset=" << offset << ", hex_binary_data.size()=" << hex_binary_data.size());
                    return data; // 返回空vector
                }
                size_t available = hex_binary_data.size() - offset;
                size_t to_read = std::min<size_t>(count, available);
                if (to_read > 0) {
                    data.resize(to_read);
                    std::copy(hex_binary_data.begin() + offset,
                        hex_binary_data.begin() + offset + to_read, data.begin());
                    read_count = to_read;
                } else {
                    LOG_WARN("可读取数据为0: offset=" << offset << ", count=" << count << ", available=" << available);
                }
            } else {
                data.resize(count);
                ifs.seekg(offset, std::ios::beg);
                ifs.read(reinterpret_cast<char*>(data.data()), count);
                read_count = static_cast<size_t>(ifs.gcount());
                if (read_count < count) {
                    data.resize(read_count);
                }
            }

            return data;
        };

        // 将文件分成多个chunk，每个chunk大小为chunk_payload_size
        // 使用 send_iso15765_payload 发送每个chunk
        uint32_t total_sent = 0;
        uint32_t chunk_index = 0;

        size_t iso_tp_chunk_size = Constants::ISO_TP_CONTINUOUS_FRAME_PAYLOAD;

        while (total_sent < total_len) {
            uint32_t remaining = total_len - total_sent;
            uint32_t current_chunk_size = static_cast<uint32_t>(std::min<size_t>(chunk_payload_size, remaining));

            LOG_INFO("发送chunk #" << (chunk_index + 1) << ": " << current_chunk_size << " 字节 (总进度: "
                                   << total_sent << "/" << total_len << ")");

            // 读取当前chunk的数据
            std::vector<uint8_t> chunk_data = read_chunk_data(total_sent, current_chunk_size);
            if (chunk_data.size() != current_chunk_size) {
                LOG_ERROR("读取chunk #" << (chunk_index + 1) << " 数据失败，期望 " << current_chunk_size
                                        << " 字节，实际读取 " << chunk_data.size() << " 字节");
                return false;
            }

            // 如果是HEX文件，增量计算CRC32
            if (is_hex_file && !chunk_data.empty()) {
                crc32_val = compute_crc32(crc32_val, chunk_data.data(), chunk_data.size());
            }

            // 计算数据块序号：从1开始，到255后从0开始
            uint8_t block_number = static_cast<uint8_t>(((chunk_index % 256) + 1) % 256);

            // 在数据前面添加Transfer Data SID 和块序号
            std::vector<uint8_t> payload_with_sid;
            payload_with_sid.reserve(chunk_data.size() + 2);
            payload_with_sid.push_back(Constants::SID_TRANSFER_DATA);
            payload_with_sid.push_back(block_number);
            payload_with_sid.insert(payload_with_sid.end(), chunk_data.begin(), chunk_data.end());

            // 先设接收回调再发送，避免设备快速响应（如 06 76 29 CD F1 AB E8 55）在回调切换前到达导致超时 并打印CRC
            LOG_INFO("等待chunk #" << (chunk_index + 1) << " 的响应 (block_number=" << static_cast<int>(block_number) << ", CRC32=0x" << std::hex << crc32_val << std::dec << ")");
            std::vector<uint8_t> chunk_resp;
            if (!wait_for_can_response(driver_tx_id_, Constants::SID_POS_RESPONSE_TRANSFER_DATA,
                    Constants::DEFAULT_TIMEOUT_MS, chunk_resp, static_cast<int>(block_number),
                    [&](const std::function<void(uint32_t, const std::vector<uint8_t>&)>& restore_cb) {
                        return send_iso15765_payload(can_id, payload_with_sid, iso_tp_chunk_size, restore_cb);
                    })) {
                LOG_ERROR("发送chunk #" << (chunk_index + 1) << " 失败或等待响应超时");
                return false;
            }

            // 验证响应格式：06 76 block_number CRC32
            // 响应应该是ISO-TP格式，需要解析
            if (chunk_resp.size() < 6) {
                LOG_ERROR("chunk #" << (chunk_index + 1) << " 响应长度不足: " << chunk_resp.size() << " 字节，期望至少6字节");
                return false;
            }

            // 解析ISO-TP单帧响应
            uint8_t pci = chunk_resp[0];
            uint8_t length = pci & 0x0F;
            if (length != 6) {
                LOG_ERROR("chunk #" << (chunk_index + 1) << " 响应长度字段错误: " << static_cast<int>(length) << "，期望6");
                return false;
            }

            if (chunk_resp.size() < 7) {
                LOG_ERROR("chunk #" << (chunk_index + 1) << " 响应数据不足: " << chunk_resp.size() << " 字节，期望至少7字节");
                return false;
            }

            // 验证响应SID (0x76)
            if (chunk_resp[1] != Constants::SID_POS_RESPONSE_TRANSFER_DATA) {
                LOG_ERROR("chunk #" << (chunk_index + 1) << " 响应SID错误: 0x" << std::hex
                                    << static_cast<int>(chunk_resp[1]) << std::dec << "，期望0x76");
                return false;
            }

            // 验证block_number
            uint8_t resp_block_number = chunk_resp[2];
            if (resp_block_number != block_number) {
                LOG_ERROR("chunk #" << (chunk_index + 1) << " 响应block_number不匹配: "
                                    << static_cast<int>(resp_block_number) << "，期望" << static_cast<int>(block_number));
                return false;
            }

            // 提取CRC32（4字节，大端序）
            uint32_t resp_crc32 = (static_cast<uint32_t>(chunk_resp[3]) << 24) | (static_cast<uint32_t>(chunk_resp[4]) << 16) | (static_cast<uint32_t>(chunk_resp[5]) << 8) | static_cast<uint32_t>(chunk_resp[6]);

            LOG_INFO("chunk #" << (chunk_index + 1) << " 响应验证成功: block_number="
                               << static_cast<int>(resp_block_number) << ", CRC32=0x" << std::hex << resp_crc32 << std::dec);

            total_sent += current_chunk_size;
            chunk_index++;
        }

        // 如果是HEX文件，输出最终计算的CRC32值
        if (is_hex_file) {
            LOG_INFO("HEX文件CRC32计算完成: 0x" << std::hex << crc32_val << std::dec
                                                << " (总数据长度: " << total_sent << " 字节)");
        }

        LOG_INFO("✅ 文件发送完成: " << total_sent << " 字节，共 " << chunk_index << " 个chunk");
        return true;
    }

    // expect_byte_at_pos2: 若 >= 0，则要求 data[2] == (uint8_t)expect_byte_at_pos2，用于按 block_number 过滤 0x76 响应
    // send_after_callback_set: 若非空，在设置好接收回调之后、开始 wait 之前调用，传入当前 temp_cb 供 send_iso15765_payload 恢复
    // restore_cb_override: 若非空，wait 结束后恢复此回调；供 send_iso15765_payload 内部 wait_for_can_response 使用
    bool wait_for_can_response(uint32_t expected_id, uint8_t expect_sid,
        int timeout_ms, std::vector<uint8_t>& resp,
        int expect_byte_at_pos2 = -1,
        std::function<bool(const std::function<void(uint32_t, const std::vector<uint8_t>&)>&)> send_after_callback_set = nullptr,
        const std::function<void(uint32_t, const std::vector<uint8_t>&)>* restore_cb_override = nullptr)
    {
        // 使用shared_ptr管理状态，确保回调中的变量始终有效
        struct ResponseState {
            std::vector<uint8_t> received_data;
            std::mutex mtx;
            std::condition_variable cv;
            std::atomic<bool> received { false };
        };
        auto state = std::make_shared<ResponseState>();

        // 提取期望ID的实际值（去除标志位）
        uint32_t expected_id_actual = extract_can_id(expected_id);

        // 保存原有回调（通过值捕获，确保安全）
        auto original_cb = [this](uint32_t can_id, const std::vector<uint8_t>& data) {
            handle_can_rx(can_id, data);
        };

        // 临时设置回调来捕获响应（使用值捕获shared_ptr，确保安全）
        auto temp_cb = [this, state, expected_id_actual, expect_sid, expect_byte_at_pos2, original_cb](uint32_t can_id, const std::vector<uint8_t>& data) {
            // 先调用原有回调处理消息
            original_cb(can_id, data);

            // 然后检查是否匹配期望的响应
            if (can_id == expected_id_actual && !data.empty()) {
                bool match = false;
                if (expect_sid == Constants::PCI_FLOW_CONTROL && data[0] == Constants::PCI_FLOW_CONTROL) {
                    match = true;
                } else if (data.size() > 1 && data[1] == expect_sid) {
                    match = true;
                }
                if (match && expect_byte_at_pos2 >= 0) {
                    if (data.size() <= 2 || data[2] != static_cast<uint8_t>(expect_byte_at_pos2))
                        match = false;
                }

                if (match) {
                    std::lock_guard<std::mutex> lk(state->mtx);
                    state->received_data = data;
                    state->received.store(true);
                    state->cv.notify_one();
                }
            }
        };

        can_.set_rx_callback(temp_cb);

        // 先设回调再发送，传入 temp_cb 供 send_iso15765_payload 在内部恢复，避免覆盖导致 06 76 响应丢失
        if (send_after_callback_set && !send_after_callback_set(temp_cb)) {
            can_.set_rx_callback(restore_cb_override && *restore_cb_override ? *restore_cb_override : original_cb);
            return false;
        }

        std::unique_lock<std::mutex> lk(state->mtx);
        bool got_response = state->cv.wait_for(lk, std::chrono::milliseconds(timeout_ms), [state]() {
            return state->received.load();
        });

        if (got_response) {
            resp = state->received_data;
        }

        // 恢复原有回调（restore_cb_override 供 send_iso15765_payload 内部调用时保持调用方的 temp_cb）
        lk.unlock();
        auto cb_to_restore = (restore_cb_override && *restore_cb_override) ? *restore_cb_override : original_cb;
        can_.set_rx_callback(cb_to_restore);

        return got_response;
    }

    // 验证ACK是否成功
    bool is_ack_success(const std::vector<uint8_t>& resp, uint8_t expect_sid)
    {
        if (resp.empty() || resp.size() < 2)
            return false;

        uint8_t sid = resp[1];
        if (sid == Constants::SID_POS_RESPONSE_EXIT && expect_sid == Constants::SID_POS_RESPONSE_EXIT) {
            return true;
        }
        if (sid != Constants::SID_POS_RESPONSE_ROUTINE)
            return false;

        int length = resp[0];
        if (length > 0 && length < static_cast<int>(resp.size())) {
            uint8_t crc = resp[length];
            if (crc != 0x00) {
                LOG_WARN("CRC校验失败");
                return false;
            }
        }
        return true;
    }

    // 解析Intel HEX文件，提取起始地址和数据长度
    bool parse_hex_file(const std::string& hex_path, uint32_t& start_address, uint32_t& data_length, std::string& err)
    {
        std::ifstream ifs(hex_path, std::ios::in);
        if (!ifs.is_open()) {
            err = "无法打开HEX文件";
            return false;
        }

        start_address = Constants::HEX_INVALID_ADDRESS;
        data_length = 0;
        uint32_t upper_address = 0;
        bool first_data_record = true;

        std::string line;
        int line_num = 0;
        while (std::getline(ifs, line)) {
            line_num++;
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            if (line.empty() || line[0] != ':')
                continue;

            if (line.size() < 11)
                continue;

            int record_len = hex_str_to_byte(line, 1);
            if (record_len < 0)
                continue;

            int addr_high = hex_str_to_byte(line, 3);
            int addr_low = hex_str_to_byte(line, 5);
            if (addr_high < 0 || addr_low < 0)
                continue;
            uint16_t addr = static_cast<uint16_t>((addr_high << 8) | addr_low);

            int record_type = hex_str_to_byte(line, 7);
            if (record_type < 0)
                continue;

            size_t expected_len = 1 + 2 + 4 + 2 + record_len * 2 + 2;
            if (line.size() < expected_len)
                continue;

            switch (record_type) {
            case 0x00: { // 数据记录
                uint32_t full_address = (upper_address << 16) | addr;
                if (first_data_record) {
                    start_address = full_address;
                    first_data_record = false;
                }
                data_length += static_cast<uint32_t>(record_len);
                break;
            }
            case 0x01: // EOF
                goto parse_complete;
            case 0x04: { // 扩展线性地址
                if (record_len != 2)
                    continue;
                int upper_high = hex_str_to_byte(line, 9);
                int upper_low = hex_str_to_byte(line, 11);
                if (upper_high < 0 || upper_low < 0)
                    continue;
                upper_address = static_cast<uint32_t>((upper_high << 8) | upper_low);
                break;
            }
            default:
                break;
            }
        }

    parse_complete:
        if (start_address == Constants::HEX_INVALID_ADDRESS) {
            err = "未找到任何数据记录";
            return false;
        }
        if (data_length == 0) {
            err = "数据长度为0";
            return false;
        }

        LOG_INFO("HEX解析完成 | 起始地址=0x" << std::hex << start_address << std::dec
                                             << " | 数据长度=" << data_length << " 字节");
        return true;
    }

    // 将HEX文件转换为二进制数据
    bool hex_to_binary(const std::string& hex_path, std::vector<uint8_t>& binary_data, std::string& err)
    {
        std::ifstream ifs(hex_path, std::ios::in);
        if (!ifs.is_open()) {
            err = "无法打开HEX文件";
            return false;
        }

        binary_data.clear();
        uint32_t upper_address = 0;
        std::map<uint32_t, std::vector<uint8_t>> address_data_map;

        std::string line;
        while (std::getline(ifs, line)) {
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            if (line.empty() || line[0] != ':')
                continue;
            if (line.size() < 11)
                continue;

            int record_len = hex_str_to_byte(line, 1);
            if (record_len < 0)
                continue;

            int addr_high = hex_str_to_byte(line, 3);
            int addr_low = hex_str_to_byte(line, 5);
            if (addr_high < 0 || addr_low < 0)
                continue;
            uint16_t addr = static_cast<uint16_t>((addr_high << 8) | addr_low);

            int record_type = hex_str_to_byte(line, 7);
            if (record_type < 0)
                continue;

            size_t expected_len = 1 + 2 + 4 + 2 + record_len * 2 + 2;
            if (line.size() < expected_len)
                continue;

            switch (record_type) {
            case 0x00: { // 数据记录
                uint32_t full_address = (upper_address << 16) | addr;
                std::vector<uint8_t> data;
                for (int i = 0; i < record_len; ++i) {
                    int byte_val = hex_str_to_byte(line, 9 + i * 2);
                    if (byte_val < 0) {
                        err = "无法解析数据";
                        return false;
                    }
                    data.push_back(static_cast<uint8_t>(byte_val));
                }
                address_data_map[full_address] = std::move(data);
                break;
            }
            case 0x01:
                goto parse_complete;
            case 0x04: {
                if (record_len != 2)
                    continue;
                int upper_high = hex_str_to_byte(line, 9);
                int upper_low = hex_str_to_byte(line, 11);
                if (upper_high < 0 || upper_low < 0)
                    continue;
                upper_address = static_cast<uint32_t>((upper_high << 8) | upper_low);
                break;
            }
            default:
                break;
            }
        }

    parse_complete:
        if (address_data_map.empty()) {
            err = "未找到任何数据记录";
            return false;
        }

        uint32_t min_addr = address_data_map.begin()->first;
        uint32_t max_addr = min_addr;
        for (const auto& kv : address_data_map) {
            if (kv.first < min_addr)
                min_addr = kv.first;
            uint32_t end_addr = kv.first + kv.second.size();
            if (end_addr > max_addr)
                max_addr = end_addr;
        }

        size_t total_size = max_addr - min_addr;
        binary_data.resize(total_size, Constants::HEX_FILL_BYTE);

        for (const auto& kv : address_data_map) {
            uint32_t offset = kv.first - min_addr;
            if (offset + kv.second.size() > binary_data.size()) {
                err = "地址范围超出预期";
                return false;
            }
            std::copy(kv.second.begin(), kv.second.end(), binary_data.begin() + offset);
        }

        LOG_INFO("HEX转换完成 | 起始地址=0x" << std::hex << min_addr << std::dec
                                             << " | 数据长度=" << binary_data.size() << " 字节");
        return true;
    }

    // 通用CRC32计算函数（类似CRC32.c中的crc32函数）
    // 支持增量计算，可以分段处理数据
    // @param crc32val: 初始CRC值（通常为0xFFFFFFFF），用于增量计算
    // @param data: 数据缓冲区指针
    // @param len: 数据长度（字节数）
    // @return: 计算后的CRC32值
    uint32_t compute_crc32(uint32_t crc32val, const uint8_t* data, size_t len)
    {
        static const uint32_t crc32_tab[256] = {
            0x00000000U, 0x77073096U, 0xEE0E612CU, 0x990951BAU, 0x076DC419U, 0x706AF48FU, 0xE963A535U, 0x9E6495A3U,
            0x0EDB8832U, 0x79DCB8A4U, 0xE0D5E91EU, 0x97D2D988U, 0x09B64C2BU, 0x7EB17CBDU, 0xE7B82D07U, 0x90BF1D91U,
            0x1DB71064U, 0x6AB020F2U, 0xF3B97148U, 0x84BE41DEU, 0x1ADAD47DU, 0x6DDDE4EBU, 0xF4D4B551U, 0x83D385C7U,
            0x136C9856U, 0x646BA8C0U, 0xFD62F97AU, 0x8A65C9ECU, 0x14015C4FU, 0x63066CD9U, 0xFA0F3D63U, 0x8D080DF5U,
            0x3B6E20C8U, 0x4C69105EU, 0xD56041E4U, 0xA2677172U, 0x3C03E4D1U, 0x4B04D447U, 0xD20D85FDU, 0xA50AB56BU,
            0x35B5A8FAU, 0x42B2986CU, 0xDBBBC9D6U, 0xACBCF940U, 0x32D86CE3U, 0x45DF5C75U, 0xDCD60DCFU, 0xABD13D59U,
            0x26D930ACU, 0x51DE003AU, 0xC8D75180U, 0xBFD06116U, 0x21B4F4B5U, 0x56B3C423U, 0xCFBA9599U, 0xB8BDA50FU,
            0x2802B89EU, 0x5F058808U, 0xC60CD9B2U, 0xB10BE924U, 0x2F6F7C87U, 0x58684C11U, 0xC1611DABU, 0xB6662D3DU,
            0x76DC4190U, 0x01DB7106U, 0x98D220BCU, 0xEFD5102AU, 0x71B18589U, 0x06B6B51FU, 0x9FBFE4A5U, 0xE8B8D433U,
            0x7807C9A2U, 0x0F00F934U, 0x9609A88EU, 0xE10E9818U, 0x7F6A0DBBU, 0x086D3D2DU, 0x91646C97U, 0xE6635C01U,
            0x6B6B51F4U, 0x1C6C6162U, 0x856530D8U, 0xF262004EU, 0x6C0695EDU, 0x1B01A57BU, 0x8208F4C1U, 0xF50FC457U,
            0x65B0D9C6U, 0x12B7E950U, 0x8BBEB8EAU, 0xFCB9887CU, 0x62DD1DDFU, 0x15DA2D49U, 0x8CD37CF3U, 0xFBD44C65U,
            0x4DB26158U, 0x3AB551CEU, 0xA3BC0074U, 0xD4BB30E2U, 0x4ADFA541U, 0x3DD895D7U, 0xA4D1C46DU, 0xD3D6F4FBU,
            0x4369E96AU, 0x346ED9FCU, 0xAD678846U, 0xDA60B8D0U, 0x44042D73U, 0x33031DE5U, 0xAA0A4C5FU, 0xDD0D7CC9U,
            0x5005713CU, 0x270241AAU, 0xBE0B1010U, 0xC90C2086U, 0x5768B525U, 0x206F85B3U, 0xB966D409U, 0xCE61E49FU,
            0x5EDEF90EU, 0x29D9C998U, 0xB0D09822U, 0xC7D7A8B4U, 0x59B33D17U, 0x2EB40D81U, 0xB7BD5C3BU, 0xC0BA6CADU,
            0xEDB88320U, 0x9ABFB3B6U, 0x03B6E20CU, 0x74B1D29AU, 0xEAD54739U, 0x9DD277AFU, 0x04DB2615U, 0x73DC1683U,
            0xE3630B12U, 0x94643B84U, 0x0D6D6A3EU, 0x7A6A5AA8U, 0xE40ECF0BU, 0x9309FF9DU, 0x0A00AE27U, 0x7D079EB1U,
            0xF00F9344U, 0x8708A3D2U, 0x1E01F268U, 0x6906C2FEU, 0xF762575DU, 0x806567CBU, 0x196C3671U, 0x6E6B06E7U,
            0xFED41B76U, 0x89D32BE0U, 0x10DA7A5AU, 0x67DD4ACCU, 0xF9B9DF6FU, 0x8EBEEFF9U, 0x17B7BE43U, 0x60B08ED5U,
            0xD6D6A3E8U, 0xA1D1937EU, 0x38D8C2C4U, 0x4FDFF252U, 0xD1BB67F1U, 0xA6BC5767U, 0x3FB506DDU, 0x48B2364BU,
            0xD80D2BDAU, 0xAF0A1B4CU, 0x36034AF6U, 0x41047A60U, 0xDF60EFC3U, 0xA867DF55U, 0x316E8EEFU, 0x4669BE79U,
            0xCB61B38CU, 0xBC66831AU, 0x256FD2A0U, 0x5268E236U, 0xCC0C7795U, 0xBB0B4703U, 0x220216B9U, 0x5505262FU,
            0xC5BA3BBEU, 0xB2BD0B28U, 0x2BB45A92U, 0x5CB36A04U, 0xC2D7FFA7U, 0xB5D0CF31U, 0x2CD99E8BU, 0x5BDEAE1DU,
            0x9B64C2B0U, 0xEC63F226U, 0x756AA39CU, 0x026D930AU, 0x9C0906A9U, 0xEB0E363FU, 0x72076785U, 0x05005713U,
            0x95BF4A82U, 0xE2B87A14U, 0x7BB12BAEU, 0x0CB61B38U, 0x92D28E9BU, 0xE5D5BE0DU, 0x7CDCEFB7U, 0x0BDBDF21U,
            0x86D3D2D4U, 0xF1D4E242U, 0x68DDB3F8U, 0x1FDA836EU, 0x81BE16CDU, 0xF6B9265BU, 0x6FB077E1U, 0x18B74777U,
            0x88085AE6U, 0xFF0F6A70U, 0x66063BCAU, 0x11010B5CU, 0x8F659EFFU, 0xF862AE69U, 0x616BFFD3U, 0x166CCF45U,
            0xA00AE278U, 0xD70DD2EEU, 0x4E048354U, 0x3903B3C2U, 0xA7672661U, 0xD06016F7U, 0x4969474DU, 0x3E6E77DBU,
            0xAED16A4AU, 0xD9D65ADCU, 0x40DF0B66U, 0x37D83BF0U, 0xA9BCAE53U, 0xDEBB9EC5U, 0x47B2CF7FU, 0x30B5FFE9U,
            0xBDBDF21CU, 0xCABAC28AU, 0x53B39330U, 0x24B4A3A6U, 0xBAD03605U, 0xCDD70693U, 0x54DE5729U, 0x23D967BFU,
            0xB3667A2EU, 0xC4614AB8U, 0x5D681B02U, 0x2A6F2B94U, 0xB40BBE37U, 0xC30C8EA1U, 0x5A05DF1BU, 0x2D02EF8DU
        };

        if (data == nullptr || len == 0) {
            return crc32val;
        }

        for (size_t i = 0; i < len; i++) {
            crc32val = crc32_tab[(crc32val ^ data[i]) & 0xFFU] ^ (crc32val >> 8);
        }

        return crc32val;
    }

    // 成员变量
    std::string firmware_path_;
    std::string can_ifname_;
    uint32_t driver_rx_id_ = Constants::DEFAULT_DRIVER_RX_ID | CAN_EFF_FLAG;
    uint32_t driver_tx_id_ = Constants::DEFAULT_DRIVER_TX_ID | CAN_EFF_FLAG;
    SimpleCanTransport can_;
    std::atomic<bool> upgrade_success_;
    std::atomic<bool> upgrade_failed_;
};

void print_usage(const char* prog_name)
{
    std::cout << "驱动器升级调试工具\n"
              << "用法: " << prog_name << " <固件文件路径> [CAN接口名] [驱动器接收CAN ID] [驱动器发送CAN ID]\n"
              << "\n"
              << "参数:\n"
              << "  固件文件路径     - 要发送的固件文件路径（必需）\n"
              << "  CAN接口名        - CAN接口名称（默认: can0）\n"
              << "  驱动器接收CAN ID - 驱动器接收固件的CAN ID（默认: 0x701）\n"
              << "  驱动器发送CAN ID - 驱动器反馈的CAN ID（默认: 0x709）\n"
              << "\n"
              << "示例:\n"
              << "  " << prog_name << " ODriveFirmware.bin\n"
              << "  " << prog_name << " ODriveFirmware.bin can0 0x701 0x709\n"
              << std::endl;
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }
    std::string firmware_path = argv[1];
    std::string can_ifname = (argc > 2) ? argv[2] : "can0";
    uint32_t driver_rx_id = Constants::DEFAULT_DRIVER_RX_ID | CAN_EFF_FLAG;
    uint32_t driver_tx_id = Constants::DEFAULT_DRIVER_TX_ID | CAN_EFF_FLAG;

    if (argc > 3) {
        uint32_t rx_id_raw = std::stoul(argv[3], nullptr, 16);
        // 如果用户传入的ID没有包含CAN_EFF_FLAG，自动添加（确保0x701是扩展帧）
        if ((rx_id_raw & CAN_EFF_FLAG) == 0) {
            driver_rx_id = rx_id_raw | CAN_EFF_FLAG;
        } else {
            driver_rx_id = rx_id_raw;
        }
    }
    if (argc > 4) {
        uint32_t tx_id_raw = std::stoul(argv[4], nullptr, 16);
        // 如果用户传入的ID没有包含CAN_EFF_FLAG，自动添加
        if ((tx_id_raw & CAN_EFF_FLAG) == 0) {
            driver_tx_id = tx_id_raw | CAN_EFF_FLAG;
        } else {
            driver_tx_id = tx_id_raw;
        }
    }

    // 提取实际ID和帧类型
    bool rx_is_extended = (driver_rx_id & CAN_EFF_FLAG) != 0;
    bool tx_is_extended = (driver_tx_id & CAN_EFF_FLAG) != 0;
    uint32_t rx_id_actual = rx_is_extended ? (driver_rx_id & CAN_EFF_MASK) : (driver_rx_id & CAN_SFF_MASK);
    uint32_t tx_id_actual = tx_is_extended ? (driver_tx_id & CAN_EFF_MASK) : (driver_tx_id & CAN_SFF_MASK);

    LOG_INFO("=== 驱动器升级调试工具 ===");
    LOG_INFO("固件文件: " << firmware_path);
    LOG_INFO("CAN接口: " << can_ifname);
    LOG_INFO("驱动器接收CAN ID: 0x" << std::hex << rx_id_actual << std::dec
                                    << " (" << (rx_is_extended ? "扩展帧(EXT)" : "标准帧(STD)") << ")"
                                    << " [原始值: 0x" << std::hex << driver_rx_id << std::dec << "]");
    LOG_INFO("驱动器发送CAN ID: 0x" << std::hex << tx_id_actual << std::dec
                                    << " (" << (tx_is_extended ? "扩展帧(EXT)" : "标准帧(STD)") << ")"
                                    << " [原始值: 0x" << std::hex << driver_tx_id << std::dec << "]");
    if (rx_id_actual == 0x701 && !rx_is_extended) {
        LOG_WARN("⚠️  警告: 0x701应该是扩展帧，但当前设置为标准帧！");
    }
    LOG_INFO("");

    DriverUpgradeDebugger debugger(firmware_path, can_ifname, driver_rx_id, driver_tx_id);

    if (debugger.run()) {
        LOG_INFO("=== 升级流程完成 ===");
        return 0;
    } else {
        LOG_ERROR("=== 升级流程失败 ===");
        return 1;
    }
}
