/**
 * @file decision_maker_params_io.h
 * @brief DecisionParams 的 ROS2 参数声明、加载与动态更新（独立于 DecisionMaker 实现文件）
 */
#pragma once

#include "decision_params.h"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace decision_maker_params_io {

/** 声明并加载参数到 params（使用 params 当前成员作为 declare 缺省） */
void declare_and_load_decision_params(rclcpp::Node& node, DecisionParams& params);

/** 校验参数约束，供启动加载与动态 set 使用 */
bool validate_decision_params(const DecisionParams& params, std::string& reason);

/**
 * 将一批 ROS 参数合并进 in_out（先拷贝再应用，校验通过后写回）。
 * 调用方应在已持有 decision_params_ 互斥锁的情况下调用。
 */
rcl_interfaces::msg::SetParametersResult apply_ros_parameter_updates(
    DecisionParams& in_out, const std::vector<rclcpp::Parameter>& parameters);

} // namespace decision_maker_params_io
