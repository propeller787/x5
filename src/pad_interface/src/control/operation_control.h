#pragma once

#include "control.h"

namespace service::control {

enum class ControlIndex {
    UP_ELEVATOR = 0, // 进电梯
    DOWN_ELEVATOR = 1, // 出电梯
    GARDEN = 2, // 去花园
    HOME = 3, // 回家
};

enum class ControlStatus {
    ON = 0, // 未触发
    UNDERWAY = 1, // 进行中
    ERROR = 2, // 不能触发
};
// class service::Global;
class OperationControl : public Control {

public:
    OperationControl();

private:
    void handleOperationScenario(const std::shared_ptr<interface::srv::OperationScenario::Request> request,
        std::shared_ptr<interface::srv::OperationScenario::Response> response);
    void handleOperationStatus(const std::shared_ptr<interface::srv::OperationStatus::Request> request,
        std::shared_ptr<interface::srv::OperationStatus::Response> response);
    void onFeatureResult(const interface::msg::Objects msg);

    void onDetectorResult(const interface::msg::Objects msg);
    double distance(const Point& p1, const Point& p2);
    Point calculatePerpendicularPoint(const Point& p1, const Point& p2, double distanceToCenter, bool symbol = true);
    Point calculatePointAtDistance(const Point& p1, const Point& p2, double distanceFromP1);
    Point calculateCPoint(const Point& A, const Point& B, double distanceBD, double distanceCD);

    void onMonnaiResult(const interface::msg::Objects msg);

private:
    rclcpp::Service<interface::srv::OperationScenario>::SharedPtr ope_Sce_ { nullptr };
    std::map<ControlIndex, ControlStatus> control_statua_;
    bool under_way_ = false;
    // 上电梯状态
    bool up_elevator = false;
    // 下电梯状态
    bool down_elevator = false;
    service::Global* global_ = service::Global::getInstance();
    // 左右开门状态 0  1 左开 2 右开
    std::atomic<int16_t> elevator_door_condition_ = 0;
    rclcpp::Service<interface::srv::OperationStatus>::SharedPtr ope_Sta_ { nullptr };
    rclcpp::Publisher<interface::msg::VlagoScene>::SharedPtr sence_pub_ { nullptr };
    // 电梯灭火器对象
    rclcpp::Subscription<interface::msg::Objects>::SharedPtr object_sub_ { nullptr };
    int mhindex_ = 0; // 发送计数器
    // 电梯门对象
    rclcpp::Subscription<interface::msg::Objects>::SharedPtr object_elevator_sub_ { nullptr };
    int ehindex_ = 0;
    int third_index_ = 0;
    // 电梯门内中心点
    rclcpp::Subscription<interface::msg::Objects>::SharedPtr object_monnai_sub_ { nullptr };

    double garden_x_ = 163.0;
    double garden_y_ = 96.5f;

    double home_x_ = 387.0;
    double home_y_ = 269.0;
};

}