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
#include "global.h"
#include "service.h"
#include <rclcpp/rclcpp.hpp>
int main(int argc, char* argv[])
{
    auto logger = rclcpp::get_logger("sensor");
    RCLCPP_INFO(logger, "sensor start!");

    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto global = service::Global::getInstance();
    exec.add_node(global->node_);

    auto serviceSptr = std::make_shared<service::ControlService>(options);

    exec.add_node(serviceSptr);

    exec.spin();
    // rclcpp::spin(serviceSptr);
    rclcpp::shutdown();

    return 0;
}
