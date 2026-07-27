#include "car_control.h"

using namespace service::control;
CarControl::CarControl()
    : Control("CarControl")
{
    using namespace std::placeholders;
    auto global = service::Global::getInstance();
    car_controls_ = global->node_->create_service<interface::srv::CarControls>(carControlService,
        std::bind(&CarControl::handleCarControls, this, std::placeholders::_1, std::placeholders::_2));
    control_pubs_ = global->node_->create_publisher<geometry_msgs::msg::Twist>(std::string(nav2ControlTopic), 10);
    control_sub_ = global->node_->create_subscription<interface::msg::Control>(carWheelSpeedTopic, 10,
        std::bind(&CarControl::onControlResult, this, std::placeholders::_1));
    // loop_thread_ = std::thread([this]() {
    //     loop_.loop();
    // });
}

CarControl::~CarControl()
{
    // 停止定时器
    stopTimer();
    if (loop_thread_.joinable()) {
        loop_.quit();
        loop_thread_.join();
    }
}

void CarControl::onTimer()
{
    std::cout << " onTimer : " << is_timer_running_ << std::endl;
    if (!is_timer_running_) {
        return;
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = vx_speed_;
    twist.angular.z = w_speed_;
    control_pubs_->publish(twist);
    std::cout << " vx_speed_ " << vx_speed_ << " w_speed_ " << w_speed_ << std::endl;
}

void CarControl::startTimer()
{
    std::cout << " startTimer : " << is_timer_running_ << std::endl;
    if (is_timer_running_) {
        return;
    }
    // 设置50Hz的定时器（20ms间隔）
    timer_id_ = loop_.runEvery(0.02, boost::bind(&CarControl::onTimer, this));
    is_timer_running_ = true;
}

void CarControl::stopTimer()
{
    std::cout << " stopTimer : " << is_timer_running_ << std::endl;
    if (!is_timer_running_) {
        return;
    }
    loop_.runInLoop([this]() {
        loop_.cancel(timer_id_);
        is_timer_running_ = false;
    });
}

#define SPEED_EXPONENT (1.8f) // 速度曲线指数（>1使低速更灵敏）
/* 指数缩放（使低速更灵敏） */
float exponential_scale(float value, float exponent)
{
    if (value == 0.0f)
        return 0.0f;

    if (value > 0.0f) {
        if (value > 1.0f) {
            return 1.0f;
        }
        return pow(value, exponent);
    } else {
        if (value < -1.0f) {
            return -1.0f;
        }
        return -pow(-value, exponent);
    }
}

/**
 * *@brief摇杆向量函数**
 * @param[in]val 摇杆值
 * @param[in]angle 摇杆方向
 * @param[in]gear 档位（三档）
 * @param[out]vx底盘前进速度m/s*
 * @param[out]w底盘旋转速度rad/s*
 * @return**void
 */

void CarControl::joystickDistribution(double val, double angle, unsigned int gear, float& vx, float& w)
{
    float vx_ = 0.0;
    float w_ = 0.0;
    if (angle > 360.0 || val < 0.001) {
        vx = 0.0;
        w = 0.0;
        return;
    }
    // 根据档位计算最大速度百分比
    float max_speed_percentage = 0.0;
    switch (gear) {
    case 1:
        max_speed_percentage = 0.4; // 1档最大速度40%
        break;
    case 2:
        max_speed_percentage = 0.7; // 2档最大速度70%
        break;
    case 3:
        max_speed_percentage = 1.0; // 3档最大速度100%
        break;
    default:
        max_speed_percentage = 0.4; // 默认使用1档
        break;
    }
    int multiple = static_cast<int>(angle / 90.0);
    double result = angle - multiple * 90.0;
    // std::cout << " result " << result << std::endl;
    //  将角度转换为弧度
    double angle_rad = result * M_PI / 180.0;

    // 计算线速度的正负
    float direction = 1.0;
    if (angle > 90.0 && angle < 270.0) {
        direction = -1.0;
    }

    float direction_w = 1.0;
    if (angle > 0 && angle < 180) {
        direction_w = -1.0;
    }
    // 使用正弦和余弦计算实际速度
    vx_ = direction * val * sin(angle_rad) * VX_MAX_VAL * max_speed_percentage;
    w_ = direction_w * val * cos(angle_rad) * W_MAX_VAL * max_speed_percentage; // 旋转速度乘以0.5
    w *= exponential_scale(std::fabs(w_) / 6 / 3.6, SPEED_EXPONENT - 1.0);
    // std::cout << " cos(angle_rad) " << cos(angle_rad) << " sin(angle_rad) " << sin(angle_rad) << std::endl;
    //  应用最小速度阈值
    if (std::abs(vx_) < VX_MIN_VAL) {
        vx_ = 0.0;
    }
    if (std::abs(w_) < W_MIN_VAL) {
        w_ = 0.0;
    }

    vx = vx_;
    w = w_;
    // w = 0.0;
}

void CarControl::onControlResult(const interface::msg::Control msg)
{
    car_infor_.electric = msg.electricquantity;
    car_infor_.remaining = msg.remainingmileage;
}
void CarControl::handleCarControls(const std::shared_ptr<interface::srv::CarControls::Request> request,
    std::shared_ptr<interface::srv::CarControls::Response> response)
{
    if (id_ != request->id) {
        response->set__code(-1);
        return;
    }
    joystickDistribution(request->distance, request->heading, request->position, vx_speed_, w_speed_);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = vx_speed_;
    twist.angular.z = w_speed_;
    control_pubs_->publish(twist);
    response->set__code(0);
}
