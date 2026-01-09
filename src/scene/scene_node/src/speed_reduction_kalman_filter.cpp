#include "speed_reduction_kalman_filter.h"

SpeedReductionKalmanFilter::SpeedReductionKalmanFilter()
    : SpeedReductionKalmanFilter(Config()) {}

SpeedReductionKalmanFilter::SpeedReductionKalmanFilter(const Config& cfg)
    : config_(cfg)
    , initialized_(false)
    , has_result_(false)
    , last_filtered_speed_(0.0)
{
    H_ << 1.0, 0.0;
    R_ << config_.measurement_noise;
    covariance_.setIdentity();
    state_.setZero();
}

void SpeedReductionKalmanFilter::initializeState(double initial_speed)
{
    state_ << initial_speed, 0.0;
    covariance_ = Eigen::Matrix2d::Identity() * config_.initial_variance;
    initialized_ = true;
    has_result_ = true;
    last_filtered_speed_ = initial_speed;
}

double SpeedReductionKalmanFilter::filter(double measured_speed, double dt_seconds)
{
    if (!initialized_) {
        initializeState(measured_speed);
        return measured_speed;
    }

    double dt = dt_seconds;
    if (dt <= config_.min_dt || std::isnan(dt) || std::isinf(dt)) {
        dt = config_.default_dt;
    }

    // 加速或保持速度时直接跟随，避免拖慢响应
    if (!has_result_ || measured_speed >= last_filtered_speed_) {
        initializeState(measured_speed);
        return measured_speed;
    }

    Eigen::Matrix2d F;
    F << 1.0, dt,
         0.0, 1.0;

    // 过程噪声：简化的常加速度模型
    const double q = config_.process_noise;
    Eigen::Matrix2d Q;
    Q << 0.25 * dt * dt * q, 0.5 * dt * q,
         0.5 * dt * q, q;

    // 预测
    state_ = F * state_;
    covariance_ = F * covariance_ * F.transpose() + Q;

    // 更新
    Eigen::Matrix<double, 1, 1> z;
    z << measured_speed;
    Eigen::Matrix<double, 1, 1> y = z - H_ * state_;
    Eigen::Matrix<double, 1, 1> S = H_ * covariance_ * H_.transpose() + R_;
    Eigen::Matrix<double, 2, 1> K = covariance_ * H_.transpose() * S.inverse();

    state_ = state_ + K * y;
    covariance_ = (Eigen::Matrix2d::Identity() - K * H_) * covariance_;

    last_filtered_speed_ = state_(0);
    has_result_ = true;
    return last_filtered_speed_;
}

void SpeedReductionKalmanFilter::reset(double initial_speed)
{
    initialized_ = false;
    has_result_ = false;
    last_filtered_speed_ = initial_speed;
    state_.setZero();
    covariance_.setIdentity();
    if (!std::isnan(initial_speed) && !std::isinf(initial_speed)) {
        initializeState(initial_speed);
    }
}

void SpeedReductionKalmanFilter::setConfig(const Config& cfg)
{
    config_ = cfg;
    R_ << config_.measurement_noise;
    covariance_ = Eigen::Matrix2d::Identity() * config_.initial_variance;
    initialized_ = false;
    has_result_ = false;
}

double SpeedReductionKalmanFilter::lastFilteredSpeed() const { return last_filtered_speed_; }
bool SpeedReductionKalmanFilter::hasResult() const { return has_result_; }

