#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include "src/ord_lidar_driver.h"

// 模拟LidarDevice的驱动
class MockOrdlidarDriver : public ordlidar::OrdlidarDriver {
public:
    MockOrdlidarDriver() : OrdlidarDriver(ORADAR_TYPE_SERIAL, ORADAR_MS200) {}
    bool Connect() override { return true; }
    void Disconnect() override {}
    bool GrabFullScanBlocking(full_scan_data_st& scan_data, int timeout) override {
        scan_data.vailtidy_point_num = 360;
        for (int i = 0; i < 360; ++i) {
            scan_data.data[i].angle = i;
            scan_data.data[i].distance = 1000; // 1 meter
            scan_data.data[i].intensity = 100;
        }
        return true;
    }
};

// 测试类
class OradarScanNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("test_node");
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
};

// 测试节点初始化
TEST_F(OradarScanNodeTest, NodeInitialization) {
    ASSERT_NE(node_, nullptr);
    ASSERT_EQ(node_->get_name(), "test_node");
}

// 测试参数设置
TEST_F(OradarScanNodeTest, ParameterDeclaration) {
    std::string port_name = "/dev/ttyUSB0";
    int baudrate = 115200;
    node_->declare_parameter("lidar1_port_name", port_name);
    node_->declare_parameter("lidar1_baudrate", baudrate);

    std::string retrieved_port_name;
    int retrieved_baudrate;
    node_->get_parameter("lidar1_port_name", retrieved_port_name);
    node_->get_parameter("lidar1_baudrate", retrieved_baudrate);

    EXPECT_EQ(retrieved_port_name, port_name);
    EXPECT_EQ(retrieved_baudrate, baudrate);
}

// 测试发布消息
TEST_F(OradarScanNodeTest, PublishLaserScan) {
    auto publisher = node_->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    auto driver = std::make_shared<MockOrdlidarDriver>();

    full_scan_data_st scan_data;
    driver->GrabFullScanBlocking(scan_data, 1000);

    rclcpp::Time start_scan_time = node_->now();
    double scan_duration = 0.1; // 假设扫描时间为0.1秒

    publish_msg(publisher, &scan_data, start_scan_time, scan_duration, "laser_frame",
                true, 0.0, 6.28, 0.1, 10.0);

    // 这里可以添加更多的断言来验证发布的消息
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}