#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <fstream>
#include <string>
#include <vector>

// This demo simulates receiving an OTA bin locally, then publishes
// a DBC-backed command (audio) to the existing topic so CanAdapter
// encodes it via odrive_*_pack and sends on CAN.

class OtaDbcDemoNode : public rclcpp::Node {
public:
  OtaDbcDemoNode() : Node("ota_dbc_demo_node") {
    this->declare_parameter<std::string>("bin_path", "");
    this->declare_parameter<int>("audio_type", 1);
    this->declare_parameter<int>("publish_count", 1);
    this->declare_parameter<int>("publish_interval_ms", 200);

    audio_pub_ = this->create_publisher<std_msgs::msg::Int32>("/vlago/audio/play", 10);

    std::string bin_path = this->get_parameter("bin_path").as_string();
    int audio_type = this->get_parameter("audio_type").as_int();
    int publish_count = this->get_parameter("publish_count").as_int();
    int publish_interval_ms = this->get_parameter("publish_interval_ms").as_int();

    if (!bin_path.empty()) {
      // Simulate: read local bin file, just log size to show we loaded it.
      std::ifstream ifs(bin_path, std::ios::binary | std::ios::ate);
      if (ifs.is_open()) {
        std::streamsize size = ifs.tellg();
        RCLCPP_INFO(this->get_logger(), "Loaded bin file '%s' (%ld bytes)", bin_path.c_str(), static_cast<long>(size));
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to open bin file '%s' (simulation continues)", bin_path.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter 'bin_path' is empty; proceeding with publish only");
    }

    // Publish a few times with a timer for reliability.
    to_publish_ = publish_count;
    msg_.data = audio_type;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(std::max(1, publish_interval_ms)),
        [this]() {
          if (to_publish_ <= 0) {
            rclcpp::shutdown();
            return;
          }
          audio_pub_->publish(msg_);
          RCLCPP_INFO(this->get_logger(), "Published audio command: %d to /vlago/audio/play (remaining %d)", msg_.data, to_publish_ - 1);
          --to_publish_;
        });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr audio_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Int32 msg_;
  int to_publish_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OtaDbcDemoNode>());
  rclcpp::shutdown();
  return 0;
}
