/**
 * @file smooth_filter.cpp
 * @author likang (likang@freego.com)
 * @brief 平滑滤波器实现
 * @version 0.1
 * @date 2025-10-16
 * 
 * @copyright Copyright (c) 2025
 */

#include "smooth_filter.h"
#include <iostream>
#include <cmath>

SmoothFilter::SmoothFilter() 
    : filter_type_(FilterType::EXPONENTIAL)
    , window_size_(5)
    , alpha_(0.3)
    , has_last_filtered_(false)
    , last_kalman_time_(std::chrono::steady_clock::now())
    , has_last_kalman_time_(false) {
    std::cout << "初始化平滑滤波器" << std::endl;
    
    // 初始化上一状态
    last_filtered_.linear.x = 0.0;
    last_filtered_.linear.y = 0.0;
    last_filtered_.linear.z = 0.0;
    last_filtered_.angular.x = 0.0;
    last_filtered_.angular.y = 0.0;
    last_filtered_.angular.z = 0.0;
}

SmoothFilter::~SmoothFilter() {
    std::cout << "平滑滤波器析构" << std::endl;
}

void SmoothFilter::setFilterType(FilterType type) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    filter_type_ = type;
    std::cout << "设置滤波器类型: " << static_cast<int>(type) << std::endl;
}

void SmoothFilter::setWindowSize(size_t window_size) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    window_size_ = window_size;
    
    // 调整缓冲区大小
    while (linear_buffer_.size() > window_size_) {
        linear_buffer_.pop_front();
    }
    while (angular_buffer_.size() > window_size_) {
        angular_buffer_.pop_front();
    }
    
    std::cout << "设置滤波器窗口大小: " << window_size_ << std::endl;
}

void SmoothFilter::setAlpha(double alpha) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    alpha_ = std::max(0.0, std::min(1.0, alpha));
    std::cout << "设置指数平滑系数: " << alpha_ << std::endl;
}

geometry_msgs::msg::Twist SmoothFilter::filter(const geometry_msgs::msg::Twist& input) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    geometry_msgs::msg::Twist filtered_output;
    
    switch (filter_type_) {
        case FilterType::MOVING_AVERAGE:
            filtered_output = movingAverageFilter(input);
            break;
        case FilterType::EXPONENTIAL:
            filtered_output = exponentialFilter(input);
            break;
        case FilterType::MEDIAN:
            filtered_output = medianFilter(input);
            break;
        case FilterType::RATE_LIMITED:
            filtered_output = rateLimitedFilter(input);
            break;
        case FilterType::KALMAN_DECELERATION:
            filtered_output = kalmanDecelerationFilter(input);
            break;
        default:
            filtered_output = input;
            break;
    }
    
    return filtered_output;
}

geometry_msgs::msg::Twist SmoothFilter::movingAverageFilter(const geometry_msgs::msg::Twist& input) {
    // 添加新数据到缓冲区
    linear_buffer_.push_back(input);
    angular_buffer_.push_back(input);
    
    // 保持窗口大小
    if (linear_buffer_.size() > window_size_) {
        linear_buffer_.pop_front();
    }
    if (angular_buffer_.size() > window_size_) {
        angular_buffer_.pop_front();
    }
    
    geometry_msgs::msg::Twist output;
    
    if (!linear_buffer_.empty()) {
        // 计算线速度平均值
        double linear_x_sum = 0.0, linear_y_sum = 0.0, linear_z_sum = 0.0;
        for (const auto& twist : linear_buffer_) {
            linear_x_sum += twist.linear.x;
            linear_y_sum += twist.linear.y;
            linear_z_sum += twist.linear.z;
        }
        
        output.linear.x = linear_x_sum / linear_buffer_.size();
        output.linear.y = linear_y_sum / linear_buffer_.size();
        output.linear.z = linear_z_sum / linear_buffer_.size();
    } else {
        output.linear = input.linear;
    }
    
    if (!angular_buffer_.empty()) {
        // 计算角速度平均值
        double angular_x_sum = 0.0, angular_y_sum = 0.0, angular_z_sum = 0.0;
        for (const auto& twist : angular_buffer_) {
            angular_x_sum += twist.angular.x;
            angular_y_sum += twist.angular.y;
            angular_z_sum += twist.angular.z;
        }
        
        output.angular.x = angular_x_sum / angular_buffer_.size();
        output.angular.y = angular_y_sum / angular_buffer_.size();
        output.angular.z = angular_z_sum / angular_buffer_.size();
    } else {
        output.angular = input.angular;
    }
    
    return output;
}

geometry_msgs::msg::Twist SmoothFilter::exponentialFilter(const geometry_msgs::msg::Twist& input) {
    geometry_msgs::msg::Twist output;
    
    if (!has_last_filtered_) {
        // 第一次输入，直接使用输入值
        output = input;
        has_last_filtered_ = true;
    } else {
        // 指数平滑: output = alpha * input + (1 - alpha) * last_output
        output.linear.x = alpha_ * input.linear.x + (1.0 - alpha_) * last_filtered_.linear.x;
        output.linear.y = alpha_ * input.linear.y + (1.0 - alpha_) * last_filtered_.linear.y;
        output.linear.z = alpha_ * input.linear.z + (1.0 - alpha_) * last_filtered_.linear.z;
        
        output.angular.x = alpha_ * input.angular.x + (1.0 - alpha_) * last_filtered_.angular.x;
        output.angular.y = alpha_ * input.angular.y + (1.0 - alpha_) * last_filtered_.angular.y;
        output.angular.z = alpha_ * input.angular.z + (1.0 - alpha_) * last_filtered_.angular.z;
    }
    
    // 更新上一状态
    last_filtered_ = output;
    
    return output;
}

geometry_msgs::msg::Twist SmoothFilter::medianFilter(const geometry_msgs::msg::Twist& input) {
    // 添加新数据到缓冲区
    linear_buffer_.push_back(input);
    angular_buffer_.push_back(input);
    
    // 保持窗口大小
    if (linear_buffer_.size() > window_size_) {
        linear_buffer_.pop_front();
    }
    if (angular_buffer_.size() > window_size_) {
        angular_buffer_.pop_front();
    }
    
    geometry_msgs::msg::Twist output;
    
    if (!linear_buffer_.empty()) {
        // 计算线速度中值
        std::vector<double> linear_x_values, linear_y_values, linear_z_values;
        for (const auto& twist : linear_buffer_) {
            linear_x_values.push_back(twist.linear.x);
            linear_y_values.push_back(twist.linear.y);
            linear_z_values.push_back(twist.linear.z);
        }
        
        std::sort(linear_x_values.begin(), linear_x_values.end());
        std::sort(linear_y_values.begin(), linear_y_values.end());
        std::sort(linear_z_values.begin(), linear_z_values.end());
        
        size_t mid = linear_x_values.size() / 2;
        output.linear.x = linear_x_values[mid];
        output.linear.y = linear_y_values[mid];
        output.linear.z = linear_z_values[mid];
    } else {
        output.linear = input.linear;
    }
    
    if (!angular_buffer_.empty()) {
        // 计算角速度中值
        std::vector<double> angular_x_values, angular_y_values, angular_z_values;
        for (const auto& twist : angular_buffer_) {
            angular_x_values.push_back(twist.angular.x);
            angular_y_values.push_back(twist.angular.y);
            angular_z_values.push_back(twist.angular.z);
        }
        
        std::sort(angular_x_values.begin(), angular_x_values.end());
        std::sort(angular_y_values.begin(), angular_y_values.end());
        std::sort(angular_z_values.begin(), angular_z_values.end());
        
        size_t mid = angular_x_values.size() / 2;
        output.angular.x = angular_x_values[mid];
        output.angular.y = angular_y_values[mid];
        output.angular.z = angular_z_values[mid];
    } else {
        output.angular = input.angular;
    }
    
    return output;
}

void SmoothFilter::reset() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    linear_buffer_.clear();
    angular_buffer_.clear();
    has_last_filtered_ = false;
    has_last_kalman_time_ = false;
    speed_kf_.reset(0.0);
    
    // 重置上一状态
    last_filtered_.linear.x = 0.0;
    last_filtered_.linear.y = 0.0;
    last_filtered_.linear.z = 0.0;
    last_filtered_.angular.x = 0.0;
    last_filtered_.angular.y = 0.0;
    last_filtered_.angular.z = 0.0;
    
    std::cout << "平滑滤波器已重置" << std::endl;
}

bool SmoothFilter::isEmpty() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return linear_buffer_.empty() && angular_buffer_.empty();
}

size_t SmoothFilter::getCurrentSize() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return std::max(linear_buffer_.size(), angular_buffer_.size());
}

geometry_msgs::msg::Twist SmoothFilter::rateLimitedFilter(const geometry_msgs::msg::Twist& input) {
    geometry_msgs::msg::Twist output = input;
    
    // 限制前向速度（linear.x）的增量
    if (has_last_filtered_) {
        double last_linear_x = last_filtered_.linear.x;
        double current_linear_x = input.linear.x;
        double velocity_diff = current_linear_x - last_linear_x;
        
        // 如果速度增加，限制每次最多增加0.01
        if (current_linear_x > 0 && velocity_diff > 0.01) {
            output.linear.x = last_linear_x + 0.01;
        }
        // 如果速度减少，且当前或上一次速度为负（倒车），限制每次最多减少0.01
        if (last_linear_x <= 0 && current_linear_x < 0 && velocity_diff < -0.01) {
            output.linear.x = last_linear_x - 0.01;
        }
        if( std::fabs(input.angular.z) < 0.08 && std::fabs(input.linear.x) <= 0.01){
            output.angular.z = 0.0;
        }
    } else {
        // 第一次输入，直接使用输入值
        has_last_filtered_ = true;
    }
    
    // 更新上一状态
    last_filtered_ = output;
    
    return output;
}

geometry_msgs::msg::Twist SmoothFilter::kalmanDecelerationFilter(const geometry_msgs::msg::Twist& input) {
    geometry_msgs::msg::Twist output = input;

    auto now = std::chrono::steady_clock::now();
    double dt = 0.02;
    if (has_last_kalman_time_) {
        dt = std::chrono::duration<double>(now - last_kalman_time_).count();
    }
    last_kalman_time_ = now;
    has_last_kalman_time_ = true;

    // 仅对前向/后向速度进行卡尔曼减速滤波
    output.linear.x = speed_kf_.filter(input.linear.x, dt);

    // 更新上一状态，供其它模式复用
    last_filtered_ = output;
    has_last_filtered_ = true;
    return output;
}
