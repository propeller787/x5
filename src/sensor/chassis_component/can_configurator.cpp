#include "can_configurator.h"
#include "sensor_utils.hpp"

using namespace sensor::chassis;
using namespace sensor::utils;

CanConfigurator::CanConfigurator(std::string can)
    : can_dev_(std::move(can))
{
}

void CanConfigurator::setup()
{
    CommandExecutor exec(rclcpp::get_logger("canif"));

    // exec.run("sudo busybox devmem 0x0c303000 32 0x0000C400");
    // exec.run("sudo busybox devmem 0x0c303008 32 0x0000C458");
    // exec.run("sudo busybox devmem 0x0c303010 32 0x0000C400");
    // exec.run("sudo busybox devmem 0x0c303018 32 0x0000C458");

    // exec.run("sudo modprobe can");
    // exec.run("sudo modprobe can_raw");
    // exec.run("sudo modprobe mttcan");

    // exec.run("sudo sh -c 'echo 0x600 > /sys/devices/platform/c310000.mttcan/net/can0/tdc_offset'");
    // exec.run("sudo sh -c 'echo 0x600 > /sys/devices/platform/c320000.mttcan/net/can1/tdc_offset'");

    // exec.run(stringFormat("sudo ip link set down %s", can_dev_.c_str()));
    // exec.run(stringFormat("sudo ip link set %s type can bitrate 500000 sample-point 0.8 \
    //     dbitrate 2000000 dsample-point 0.8 berr-reporting on fd on restart-ms 100",
    //     can_dev_.c_str()));
    // exec.run(stringFormat("sudo ip link set up %s mtu 72", can_dev_.c_str()));
    // exec.run(stringFormat("sudo ifconfig %s txqueuelen 1000", can_dev_.c_str()));

    // # ip -s -d link show can0
}
