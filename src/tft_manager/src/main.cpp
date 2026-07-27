// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "service.h"
#include <rclcpp/rclcpp.hpp>
#include <csignal>

namespace {
void on_sigint(int) {
    if (rclcpp::ok()) {
        RCLCPP_INFO(rclcpp::get_logger("tft_service"), "Received Ctrl+C, shutting down node.");
        rclcpp::shutdown();
    }
}
}  // namespace

int main(int argc, char* argv[])
{
    auto logger = rclcpp::get_logger("tft_service");
    RCLCPP_INFO(logger, "tft_service start!");

    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    std::signal(SIGINT, on_sigint);

    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto serviceSptr = std::make_shared<tft_service::TftService>(options);
    exec.add_node(serviceSptr);
    exec.spin();
    rclcpp::shutdown();

    return 0;
}
