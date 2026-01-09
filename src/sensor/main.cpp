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
#include "sensor_data_recorder.hpp"
#include <rclcpp/rclcpp.hpp>
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

    // Create an executor that will be responsible for execution of callbacks for a set of nodes.
    // With this version, all callbacks will be called from within this thread (the main one).
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    auto global = sensor::Global::getInstance();
    exec.add_node(global->node_);
    // Add some nodes to the executor which provide work for the executor during its "spin" function.
    // An example of available work is executing a subscription callback, or a timer callback.

    auto chassis = std::make_shared<sensor::chassis::ChassisComponent>(options);
    exec.add_node(chassis);

    auto gps = std::make_shared<sensor::gps::GpsComponent>(options);
    exec.add_node(gps);

    auto camera = std::make_shared<sensor::camera::CameraComponent>(options);
    exec.add_node(camera);
    
    auto imu = std::make_shared<sensor::imu::ImuComponent>(options);
    exec.add_node(imu);
    // 创建数据录制组件 - 同步录制camera、imu、gps、can数据
    auto recorder = std::make_shared<sensor::recorder::SensorDataRecorder>(options);
    exec.add_node(recorder);
    
    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    exec.spin();

    rclcpp::shutdown();

    return 0;
}
