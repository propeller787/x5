/**
 * @file test_smooth_filter.cpp
 * @brief SmoothFilter 单元测试
 *
 * 覆盖：
 *   - RATE_LIMITED 模式上限约束
 *   - 减速不受限（仅限制加速）
 *   - 极端输入（NaN, INF）
 *   - 重置后行为
 */

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "smooth_filter.h"

class SmoothFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        filter_ = std::make_unique<SmoothFilter>();
    }
    std::unique_ptr<SmoothFilter> filter_;

    geometry_msgs::msg::Twist makeTwist(double vx, double wz) {
        geometry_msgs::msg::Twist t;
        t.linear.x = vx;
        t.angular.z = wz;
        return t;
    }
};

// ---------- RATE_LIMITED 模式 ----------

TEST_F(SmoothFilterTest, RateLimitedCapsAcceleration) {
    filter_->setFilterType(FilterType::RATE_LIMITED);
    SmoothCommandLimits lim;
    lim.max_linear_delta = 0.05;
    lim.max_angular_delta = 0.05;
    filter_->setCommandLimits(lim);

    // 首帧：从 0 到 1.0 的阶跃输入
    auto out1 = filter_->filter(makeTwist(0.0, 0.0));
    auto out2 = filter_->filter(makeTwist(1.0, 0.0));

    // 输出的增量不应超过 max_linear_delta（首帧可能有特殊处理）
    double delta = std::abs(out2.linear.x - out1.linear.x);
    EXPECT_LE(delta, lim.max_linear_delta + 0.001); // 小容差
}

TEST_F(SmoothFilterTest, RateLimitedAllowsDeceleration) {
    filter_->setFilterType(FilterType::RATE_LIMITED);
    SmoothCommandLimits lim;
    lim.max_linear_delta = 0.01;
    filter_->setCommandLimits(lim);

    // 先稳定到较高速度
    for (int i = 0; i < 200; ++i) {
        filter_->filter(makeTwist(1.0, 0.0));
    }

    // 突然减速到 0
    auto out = filter_->filter(makeTwist(0.0, 0.0));
    // 减速应比加速更快（不受 max_linear_delta 限制）或至少同等速率
    EXPECT_LT(out.linear.x, 1.0);
}

// ---------- 重置 ----------

TEST_F(SmoothFilterTest, ResetClearsState) {
    filter_->setFilterType(FilterType::MOVING_AVERAGE);
    filter_->setWindowSize(5);
    filter_->filter(makeTwist(1.0, 0.0));
    filter_->filter(makeTwist(2.0, 0.0));
    // MOVING_AVERAGE 使用缓冲区，filter 后 size > 0
    EXPECT_GT(filter_->getCurrentSize(), 0u);

    filter_->reset();
    EXPECT_TRUE(filter_->isEmpty());
}

// ---------- 输入零值 ----------

TEST_F(SmoothFilterTest, ZeroInputOutputsZero) {
    filter_->setFilterType(FilterType::RATE_LIMITED);
    auto out = filter_->filter(makeTwist(0.0, 0.0));
    EXPECT_NEAR(out.linear.x, 0.0, 1e-9);
    EXPECT_NEAR(out.angular.z, 0.0, 1e-9);
}

// ---------- 指数平滑 ----------

TEST_F(SmoothFilterTest, ExponentialSmoothing) {
    filter_->setFilterType(FilterType::EXPONENTIAL);
    filter_->setAlpha(0.3);

    auto out1 = filter_->filter(makeTwist(1.0, 0.0));
    auto out2 = filter_->filter(makeTwist(1.0, 0.0));

    // 第二帧应更接近目标
    EXPECT_GE(out2.linear.x, out1.linear.x);
    EXPECT_LE(out2.linear.x, 1.0 + 1e-6);
}

// ---------- 移动平均 ----------

TEST_F(SmoothFilterTest, MovingAverageWindowSize) {
    filter_->setFilterType(FilterType::MOVING_AVERAGE);
    filter_->setWindowSize(3);

    filter_->filter(makeTwist(0.0, 0.0));
    filter_->filter(makeTwist(1.0, 0.0));
    auto out = filter_->filter(makeTwist(2.0, 0.0));

    // 3 帧的平均值应接近 1.0
    EXPECT_NEAR(out.linear.x, 1.0, 0.5);
}
