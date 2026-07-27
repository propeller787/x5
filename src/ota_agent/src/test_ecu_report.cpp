#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <random>
#include <cstdlib> // for getpid()

// Protocol Buffers
#include "vehicle_ecu_up.pb.h"

// MQTT and Signature modules
#include "mqtt_module/signature_manager.hpp"
#include "mqtt_module/token_manager.hpp"

using namespace std::chrono_literals;

// Mock logger for testing
rclcpp::Logger get_mock_logger() {
    // Create a dummy node to get a logger
    static auto dummy_node = rclcpp::Node::make_shared("dummy_test_node");
    return dummy_node->get_logger();
}

std::string generate_base36_nonce() {
    constexpr const char* BASE36_CHARS = "0123456789abcdefghijklmnopqrstuvwxyz";
    constexpr int NONCE_LENGTH = 16;
    std::string nonce;
    nonce.reserve(NONCE_LENGTH);
    
    // 使用系统时间和随机数生成nonce
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    std::mt19937_64 rng(now_ms ^ static_cast<uint64_t>(getpid()));
    
    for (int i = 0; i < NONCE_LENGTH; ++i) {
        nonce += BASE36_CHARS[rng() % 36];
    }
    return nonce;
}

int main(int argc, char** argv) {
    std::cout << "=== ECU版本上报测试程序 ===\n";
    
    // 初始化rclcpp
    rclcpp::init(argc, argv);
    
    // 1. 初始化签名管理器
    mqtt_module::SignatureConfig sign_config;
    
    // 使用真实的根密钥作为测试
    std::string default_root_private_key = "ydSKNkOGdKx0ITtDp2GtTN9MChiDo6SQXSSRRusli0U=";
    std::string default_root_public_key = "p1c0dmOw6enDGkks5nwuwmr7mMpQm3xfDjrSDl7R0dg=";
    
    sign_config.private_key = default_root_private_key;
    sign_config.public_key = default_root_public_key;
    sign_config.sign_separator = "|";
    
    // 获取mock logger
    auto logger = get_mock_logger();
    
    auto signature_mgr = std::make_shared<mqtt_module::SignatureManager>(sign_config, logger);
    if (!signature_mgr) {
        std::cerr << "❌ 初始化签名管理器失败\n";
        rclcpp::shutdown();
        return 1;
    }
    std::cout << "✅ 签名管理器初始化成功\n";
    
    // 2. 设备信息
    mqtt_module::DeviceInfo device_info;
    device_info.vin = "ZQKJ001";
    device_info.model = "FREEGO X5 Air";
    device_info.gkey_no = "2;ZQKJ001;3";
    
    // 3. 创建ECU上报消息
    vehicle::ecu::up::VehicleEcuUp ecu_up;
    ecu_up.set_sn(device_info.vin);
    ecu_up.set_model(device_info.model);
    ecu_up.set_timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    
    // 添加模拟ECU数据
    auto* ecu1 = ecu_up.add_ecu();
    ecu1->set_ecu("PDC");
    ecu1->set_ecu_hardware_code("PDC-202401");
    ecu1->set_current_version("V1.0.0");
    
    auto* ecu2 = ecu_up.add_ecu();
    ecu2->set_ecu("MC");
    ecu2->set_ecu_hardware_code("MC-202402");
    ecu2->set_current_version("V2.1.0");
    
    auto* ecu3 = ecu_up.add_ecu();
    ecu3->set_ecu("HT");
    ecu3->set_ecu_hardware_code("HT-202403");
    ecu3->set_current_version("V1.2.3");
    
    auto* ecu4 = ecu_up.add_ecu();
    ecu4->set_ecu("3588");
    ecu4->set_ecu_hardware_code("3588-202404");
    ecu4->set_current_version("1.0.0-test");
    
    // 4. 序列化消息
    std::string payload;
    if (!ecu_up.SerializeToString(&payload)) {
        std::cerr << "❌ ECU上报消息序列化失败\n";
        return 2;
    }
    
    std::cout << "\n=== ECU上报消息内容 ===\n";
    std::cout << "SN: " << ecu_up.sn() << std::endl;
    std::cout << "Model: " << ecu_up.model() << std::endl;
    std::cout << "Timestamp: " << ecu_up.timestamp() << std::endl;
    std::cout << "ECU数量: " << ecu_up.ecu_size() << std::endl;
    for (int i = 0; i < ecu_up.ecu_size(); ++i) {
        const auto& ecu = ecu_up.ecu(i);
        std::cout << "  ECU " << i+1 << ":" << std::endl;
        std::cout << "    Name: " << ecu.ecu() << std::endl;
        std::cout << "    Hardware Code: " << ecu.ecu_hardware_code() << std::endl;
        std::cout << "    Version: " << ecu.current_version() << std::endl;
    }
    std::cout << "序列化后大小: " << payload.size() << " 字节\n";
    
    // 5. 生成gtime/gnonce
    auto now_ms = std::to_string(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count()
    );
    std::string nonce = generate_base36_nonce();
    std::string gkey_no = device_info.gkey_no;
    
    // 6. 拼接签名数据
    std::vector<unsigned char> gkeyno_bytes(gkey_no.begin(), gkey_no.end());
    std::vector<unsigned char> gtime_bytes(now_ms.begin(), now_ms.end());
    std::vector<unsigned char> gnonce_bytes(nonce.begin(), nonce.end());
    std::vector<unsigned char> payload_bytes(payload.begin(), payload.end());
    std::vector<unsigned char> sign_data;
    sign_data.reserve(gkeyno_bytes.size() + gtime_bytes.size() + gnonce_bytes.size() + payload_bytes.size());
    sign_data.insert(sign_data.end(), gkeyno_bytes.begin(), gkeyno_bytes.end());
    sign_data.insert(sign_data.end(), gtime_bytes.begin(), gtime_bytes.end());
    sign_data.insert(sign_data.end(), gnonce_bytes.begin(), gnonce_bytes.end());
    sign_data.insert(sign_data.end(), payload_bytes.begin(), payload_bytes.end());
    
    // 7. 对消息进行Ed25519签名
    std::string signature = signature_mgr->sign_data(std::string(sign_data.begin(), sign_data.end()));
    if (signature.empty()) {
        std::cerr << "❌ ECU上报消息签名失败\n";
        return 3;
    }
    
    std::cout << "\n=== 签名信息 ===\n";
    std::cout << "gkey-no: " << gkey_no << std::endl;
    std::cout << "gtime: " << now_ms << std::endl;
    std::cout << "gnonce: " << nonce << std::endl;
    std::cout << "payload长度: " << payload.size() << " 字节\n";
    std::cout << "sign_data长度: " << sign_data.size() << " 字节\n";
    std::cout << "签名前20位: " << signature.substr(0, 20) << std::endl;
    std::cout << "完整签名: " << signature << std::endl;
    
    // 8. 构造MQTT v5属性
    std::vector<std::pair<std::string, std::string>> user_props;
    user_props.emplace_back("gkey-no", gkey_no);
    user_props.emplace_back("gtime", now_ms);
    user_props.emplace_back("gnonce", nonce);
    user_props.emplace_back("gsign", signature);
    
    std::cout << "\n=== MQTT v5 属性 ===\n";
    for (const auto& prop : user_props) {
        std::cout << "  " << prop.first << " = " << prop.second << std::endl;
    }
    
    // 9. 验证签名（可选）
    std::cout << "\n=== 验证签名 ===\n";
    bool verified = signature_mgr->verify_data(std::string(sign_data.begin(), sign_data.end()), signature);
    if (verified) {
        std::cout << "✅ 签名验证成功\n";
    } else {
        std::cerr << "❌ 签名验证失败\n";
        return 4;
    }
    
    // 10. 模拟发送到的MQTT主题
    std::string model_for_topic = device_info.model;
    model_for_topic.erase(std::remove(model_for_topic.begin(), model_for_topic.end(), ' '), model_for_topic.end());
    std::string ecu_topic = "vehicle/ecu/" + model_for_topic + "/" + device_info.vin + "/1/up";
    std::cout << "\n=== 模拟MQTT发送 ===\n";
    std::cout << "📤 发送ECU版本报告到主题: " << ecu_topic << std::endl;
    std::cout << "✅ 测试完成，ECU版本上报功能正常\n";
    
    // 关闭rclcpp
    rclcpp::shutdown();
    
    return 0;
}
