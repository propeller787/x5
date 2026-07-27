#include "intent_avoidance/avoidance_engine.h"
#include "intent_avoidance/free_space_analyzer.h"
#include "intent_avoidance/intent_detector.h"
#include "intent_avoidance/scan_processor.h"
#include "intent_avoidance/control_generator.h"
#include "intent_avoidance/types.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace intent_avoidance;

// 测试：无可通行空间 → 强制刹车
TEST(AvoidanceEngineTest, NoFreeSpaceForceBrake) {
    AvoidanceEngine::Config cfg;
    cfg.enable_intent_avoidance = true;
    AvoidanceEngine engine(cfg);

    FreeSectorList empty;
    NarrowPassageInfo narrow;
    UserIntent intent;
    intent.has_forward_intent = true;
    SpeedInfo speed;
    speed.linear_x = 0.5;

    auto decision = engine.decide(empty, narrow, intent, speed);
    EXPECT_EQ(decision.strategy, AvoidanceDecision::Strategy::FORCE_BRAKE);
    EXPECT_TRUE(decision.braking);
    EXPECT_DOUBLE_EQ(decision.target_linear_x, 0.0);
    EXPECT_DOUBLE_EQ(decision.target_angular_z, 0.0);
}

// 测试：方向锁定初始设置
TEST(AvoidanceEngineTest, DirectionLockInitial) {
    AvoidanceEngine::Config cfg;
    cfg.enable_intent_avoidance = true;
    AvoidanceEngine engine(cfg);

    FreeSectorList sectors;
    FreeSector fs;
    fs.angle_start = -0.5;
    fs.angle_end = 0.5;
    fs.angle_center = 0.0;
    fs.width = 1.0;
    fs.min_clearance = 2.0;
    sectors.push_back(fs);

    NarrowPassageInfo narrow;
    UserIntent intent;
    intent.angle_rad = -0.8;  // 不在扇区内
    intent.has_steering_intent = true;
    intent.rocker_y = 60;
    SpeedInfo speed;
    speed.linear_x = 0.3;

    auto decision = engine.decide(sectors, narrow, intent, speed);
    // 应投影到最近的可行方向 (0.0)
    EXPECT_EQ(decision.strategy, AvoidanceDecision::Strategy::PROJECT_NEAREST);
    EXPECT_NEAR(decision.guidance_angle_rad, 0.0, 0.01);
}

// 测试：意图检测死区
TEST(IntentDetectorTest, Deadzone) {
    IntentDetector::Config cfg;
    cfg.joystick_deadzone = 15;
    IntentDetector detector(cfg);

    // 回中
    interface::msg::Joystick msg;
    msg.rocker_x = 0;
    msg.rocker_y = 0;
    auto intent = detector.detect(msg);
    EXPECT_TRUE(intent.is_neutral);
    EXPECT_FALSE(intent.has_forward_intent);
    EXPECT_FALSE(intent.has_steering_intent);

    // 前推
    msg.rocker_x = 80;
    msg.rocker_y = 0;
    intent = detector.detect(msg);
    EXPECT_TRUE(intent.has_forward_intent);
    EXPECT_FALSE(intent.has_steering_intent);

    // 右转
    msg.rocker_x = 50;
    msg.rocker_y = 60;
    intent = detector.detect(msg);
    EXPECT_TRUE(intent.has_forward_intent);
    EXPECT_TRUE(intent.has_steering_intent);
}

// 测试：光束坐标变换
TEST(ScanProcessorTest, BeamToBaseLink) {
    RadarMount mount;
    mount.x = 0.69;
    mount.y = 0.255;
    mount.yaw = 0.0;

    // 正前方射线 → base_link 下距离 = hypot(0.69+1.0, 0.255) ≈ 1.709
    auto [angle, range] = ScanProcessor::beamToBaseLink(0.0, 1.0, mount);
    EXPECT_NEAR(range, 1.709, 0.01);
    EXPECT_GT(angle, 0.0);  // 左雷达的前方在base_link中偏左（正角度）

    // 右雷达正前方
    mount.y = -0.255;
    mount.yaw = -0.03;
    auto [angle2, range2] = ScanProcessor::beamToBaseLink(0.0, 1.0, mount);
    EXPECT_NEAR(range2, 1.707, 0.01);
    EXPECT_LT(angle2, 0.0);  // 右雷达的前方偏右（负角度）
}

// 测试：控制生成器限幅
TEST(ControlGeneratorTest, AngularClamp) {
    ControlGenerator::Config cfg;
    cfg.max_angular_guidance = 0.5;
    cfg.cycle_time = 0.1;
    ControlGenerator gen(cfg);

    AvoidanceDecision dec;
    dec.target_angular_z = 1.0;  // 超过限幅
    dec.target_linear_x = 0.3;
    UserIntent intent;
    SpeedInfo speed;

    auto twist = gen.generate(dec, intent, speed);
    EXPECT_LE(twist.angular.z, 0.5);
    EXPECT_GE(twist.angular.z, -0.5);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
