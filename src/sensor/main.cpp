/**
 * @file main.cpp
 * @author likang (likang@zhengqiai.com)
 * @brief 
 * @version 0.1
 * @date 2025-10-15
 * 
 * @copyright Copyright (c) 2025
 */
#include "camera.hpp"
#include "chassis.hpp"
#include "global.h"
#include "gps.hpp"
#include "imu.hpp"
#include "bluetooth.hpp"
#include "bluetooth_ble.hpp"
#include "sensor_data_recorder.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>
int main(int argc, char *argv[]) {
    auto logger = rclcpp::get_logger("sensor");
    RCLCPP_INFO(logger, "sensor start!");

    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize any global resources needed by the middleware and the client library.
    // This will also parse command line arguments one day (as of Beta 1 they are not used).
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // 使用多线程 executor，让各节点的默认 mutually-exclusive callback group 可被
    // 分配到不同线程运行；这样 SensorDataRecorder 高频回调不会再阻塞 chassis 心跳定时器。
    // 线程数取节点数 + 余量；避免使用过多线程造成上下文切换。
    size_t exec_threads = std::min<size_t>(4, std::thread::hardware_concurrency());
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), exec_threads);
    rclcpp::NodeOptions options;

    sensor::Global* global = nullptr;
    std::shared_ptr<sensor::chassis::ChassisComponent> chassis = nullptr;
    std::shared_ptr<sensor::gps::GpsComponent> gps = nullptr;
    std::shared_ptr<sensor::camera::CameraComponent> camera = nullptr;
    std::shared_ptr<sensor::imu::ImuComponent> imu = nullptr;
    std::shared_ptr<sensor::recorder::SensorDataRecorder> recorder = nullptr;
    std::shared_ptr<sensor::bluetooth::BluetoothComponent> bluetooth = nullptr;
    std::shared_ptr<sensor::bluetooth::_3568::BluetoothBleComponent> bluetooth_ble = nullptr;
    try {
        global = sensor::Global::getInstance();
    } catch (...) {
        RCLCPP_ERROR(logger, "Failed to create global node");
    }
    if (global) {
        exec.add_node(global->node_);
    }

    try {
        chassis = std::make_shared<sensor::chassis::ChassisComponent>(options);
    } catch (...) {
        RCLCPP_ERROR(logger, "Failed to create chassis node");
    }
    if (chassis) {
        exec.add_node(chassis);
    }

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
