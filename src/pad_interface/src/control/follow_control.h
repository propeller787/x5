#pragma once
#include "control.h"
#include <muduo/base/Timestamp.h>
#include <muduo/net/EventLoop.h>
#include <interface/srv/scene_controls.hpp>
#include <utils/non_timer.h>
#include <std_msgs/msg/int32.hpp>

namespace service::control {

class FollowControl : public Control {

public:
    FollowControl();
    ~FollowControl();

private:
    void handleFollowMe(const std::shared_ptr<interface::srv::SceneControls::Request> request,
        std::shared_ptr<interface::srv::SceneControls::Response> response);
    void handleFollowStatus(const std::shared_ptr<interface::srv::FollowStatus::Request> request,
        std::shared_ptr<interface::srv::FollowStatus::Response> response);
    void onTrackerResult(const interface::msg::Objects msg);
    double distance(const Point& p1, const Point& p2);
    bool is_point_in_rect(const Point& p, const Rect& rect);
    std::pair<int, Point> find_closest_point_in_rect(const std::map<int, Point>& mpoint, const Rect& rect);
    void onTimer();

    void change_status(int status);

private:
    rclcpp::Service<interface::srv::SceneControls>::SharedPtr follow_me_ { nullptr };
    int index_obj_ { 0 };
    bool is_start_status = false;
    int foolow_status_ = 2; // 0 寻人中 1 跟随中 2 跟随结束
    rclcpp::Service<interface::srv::FollowStatus>::SharedPtr follow_sta_ { nullptr };
    rclcpp::Client<interface::srv::SceneControls>::SharedPtr scene_control_client_ { nullptr };
    rclcpp::Subscription<interface::msg::Objects>::SharedPtr object_sub_ { nullptr };
    rclcpp::Publisher<interface::msg::Nav2FollowMe>::SharedPtr follow_pub_ { nullptr };
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr follow_status_pub_;
    Rect rect_ { { 3.0, 3.0 }, { 0.0, -3.0 } };
    int track_id_ { 0 };
    int index_ { 0 };
    std::atomic<int> sta_index_ { 0 };
    rclcpp::Node::SharedPtr node_ptr_;

    interface::msg::Objects objs_;
    std::unique_ptr<utils::NonBlockingTimer> timer_;
};

}
