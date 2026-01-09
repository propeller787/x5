#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <mutex>
#include <string>
#include <algorithm>

#include "interface/msg/hex_data.hpp"
#include "interface/msg/module_error.hpp"
#include "interface/srv/get_error_code.hpp"
#include "utils/error_event.h"
#include "utils/topic.h"

using interface::msg::HexData;
using interface::msg::ModuleError;
using interface::srv::GetErrorCode;

class ErrorCodeServiceNode : public rclcpp::Node {
public:
  ErrorCodeServiceNode() : Node("error_code_service_node") {
    sub_ = this->create_subscription<HexData>(errorEventTopic, 50, std::bind(&ErrorCodeServiceNode::on_error_event, this, std::placeholders::_1));
    srv_ = this->create_service<GetErrorCode>(
      "/cloud_manager/vlago/get_error_code",
      std::bind(&ErrorCodeServiceNode::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "ErrorCodeService ready. Topic=%s Service=%s", errorEventTopic, "/cloud_manager/vlago/get_error_code");
  }

private:
  rclcpp::Subscription<HexData>::SharedPtr sub_;
  rclcpp::Service<GetErrorCode>::SharedPtr srv_;
  std::mutex mu_;
  std::unordered_map<std::string, uint32_t> last_code_by_module_;

  static std::string normalize_module(std::string v) {
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c){ return static_cast<char>(std::toupper(c)); });
    return v;
  }



static std::string module_name_from_mm(uint8_t mm) {
  switch (mm) {
    case 0x01: return "SYS";
    case 0x02: return "CAN0";    // 红色模块：can0
    case 0x03: return "CAN1";    // 红色模块：can1
    case 0x04: return "CAN2";    // 红色模块：can2
    case 0x10: return "IMU";     // 红色模块：IMU
    case 0x11: return "GPS";     // 红色模块：GPS
    case 0x12: return "LIDAR1";  // 红色模块：单线雷达#1左
    case 0x13: return "LIDAR2";  // 红色模块：单线雷达#2右
    case 0x14: return "LIDAR3";  // 红色模块：单线雷达#3左斜
    case 0x15: return "LIDAR4";  // 红色模块：单线雷达#4右斜
    case 0x16: return "LIDAR5";  // 红色模块：单线雷达#5后
    case 0x20: return "ODRV";    // 红色模块：电机驱动器
    case 0x21: return "TOP";     // 红色模块：上控
    case 0x17: return "CAM1";
    case 0x18: return "CAM2";
    case 0x19: return "TFT";
    default: {
      char buf[16];
      snprintf(buf, sizeof(buf), "MM_%02X", mm);
      return std::string(buf);
    }
  }
}

  static uint8_t status_from_code(uint32_t code) {
    return static_cast<uint8_t>(code & 0xFF); // EE
  }

  void on_error_event(const HexData::SharedPtr msg) {
    uint32_t code = msg->hex_value;
    uint8_t mm = static_cast<uint8_t>((code >> 24) & 0xFF);
    std::string mod = module_name_from_mm(mm);
    {
      std::lock_guard<std::mutex> lk(mu_);
      last_code_by_module_[mod] = code;
    }
  }

  void handle_request(const GetErrorCode::Request::SharedPtr req, GetErrorCode::Response::SharedPtr res) {
    std::unordered_map<std::string, uint32_t> snapshot;
    {
      std::lock_guard<std::mutex> lk(mu_);
      snapshot = last_code_by_module_;
    }

    const std::string want = normalize_module(req->module);

    if (snapshot.empty()) {
      res->success = false;
      res->status_code = 1; // no records
      return;
    }

    auto append_item = [&](const std::string& mod, uint32_t code){
      ModuleError item;
      item.module = mod;
      item.code = code;
      uint8_t ee = status_from_code(code);
      // 1=发生，2=解除；其他保持原值
      if (ee == 0x01) item.status_code = 1; else if (ee == 0x02) item.status_code = 2; else item.status_code = ee;
      res->items.push_back(item);
    };

    if (want.empty()) {
      for (const auto &kv : snapshot) append_item(kv.first, kv.second);
      res->success = true;
      res->status_code = 0;
      return;
    }

    auto it = snapshot.find(want);
    if (it == snapshot.end()) {
      res->success = false;
      res->status_code = 2; // not found
      return;
    }

    append_item(it->first, it->second);
    res->success = true;
    res->status_code = 0;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ErrorCodeServiceNode>());
  rclcpp::shutdown();
  return 0;
}
