/**
 * @file test_ego_motion_map.cpp
 * @brief 方案 6A 自运动补偿单测：平移搬移 / 暴露区清零 / 旋转惩罚 / 快照
 */
#include <gtest/gtest.h>

#include "safety_node/rolling_local_map.h"

#include <algorithm>

namespace
{

LocalMapConfig memoryConfig()
{
  LocalMapConfig config;
  config.enabled = true;
  config.width_m = 6.0;
  config.height_m = 6.0;
  config.resolution_m = 0.05;
  config.inflation_radius_m = 0.0;          // 只验证概率层的几何搬移
  config.lethal_threshold = 0.65;
  config.decay_halflife_s = 1000.0;         // 关闭时间衰减的干扰
  config.max_hit_age_s = 1000.0;
  config.observation_weight = 0.85;
  config.rebuild_each_frame = false;        // 跨帧记忆模式
  config.ego_motion_compensation = true;
  config.rotation_decay_threshold_rad = 0.20;
  config.rotation_decay_factor = 0.5;
  return config;
}

rclcpp::Time stampAt(double seconds)
{
  return rclcpp::Time(
    static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

}  // namespace

TEST(EgoMotionMap, ForwardMotionShiftsObstacleBackward)
{
  RollingLocalMap map(memoryConfig());
  map.integratePoints({{1.0, 0.0, 0.0}}, stampAt(1.0));
  map.finalizeFrame(stampAt(1.0));
  ASSERT_TRUE(map.lethalQuery(1.0, 0.0, 0.65));

  // 自车前进 0.5 m：世界固定的障碍在车体系应后移到 x=0.5
  map.applyEgoMotion(0.5, 0.0, 0.0);
  EXPECT_TRUE(map.lethalQuery(0.5, 0.0, 0.65));
  EXPECT_FALSE(map.lethalQuery(1.0, 0.0, 0.65));
}

TEST(EgoMotionMap, SubCellResidualAccumulates)
{
  RollingLocalMap map(memoryConfig());
  map.integratePoints({{1.0, 0.0, 0.0}}, stampAt(1.0));
  map.finalizeFrame(stampAt(1.0));

  // 每次 2 cm < 半格（2.5 cm），单次不搬移；累计 10 cm 后应整体后移 2 格
  for (int i = 0; i < 5; ++i) {
    map.applyEgoMotion(0.02, 0.0, 0.0);
  }
  EXPECT_TRUE(map.lethalQuery(0.90, 0.0, 0.65));
  EXPECT_FALSE(map.lethalQuery(1.0, 0.0, 0.65));
}

TEST(EgoMotionMap, ExposedRegionIsFree)
{
  RollingLocalMap map(memoryConfig());
  map.integratePoints({{-2.9, 0.0, 0.0}}, stampAt(1.0));
  map.finalizeFrame(stampAt(1.0));
  ASSERT_TRUE(map.lethalQuery(-2.9, 0.0, 0.65));

  // 前进 0.5 m：后缘障碍滚出地图，新暴露的前缘区域必须为 free
  map.applyEgoMotion(0.5, 0.0, 0.0);
  EXPECT_FALSE(map.lethalQuery(-2.9, 0.0, 0.65));
  EXPECT_EQ(map.queryCost(2.9, 0.0), 0.0f);
}

TEST(EgoMotionMap, RotationPenaltyDecaysMemory)
{
  RollingLocalMap map(memoryConfig());
  map.integratePoints({{1.0, 0.0, 0.0}}, stampAt(1.0));
  map.finalizeFrame(stampAt(1.0));
  ASSERT_TRUE(map.lethalQuery(1.0, 0.0, 0.65));

  // 累计角行程 0.25 rad > 阈值 0.20 → 全图概率乘 0.5：0.85→0.425 < 0.65
  map.applyEgoMotion(0.0, 0.0, 0.25);
  EXPECT_FALSE(map.lethalQuery(1.0, 0.0, 0.65));
  EXPECT_GT(map.queryCost(1.0, 0.0), 0.3f);
}

TEST(EgoMotionMap, RebuildModeIgnoresEgoMotion)
{
  LocalMapConfig config = memoryConfig();
  config.rebuild_each_frame = true;    // 单帧模式：补偿必须为空操作
  RollingLocalMap map(config);
  map.integratePoints({{1.0, 0.0, 0.0}}, stampAt(1.0));
  map.finalizeFrame(stampAt(1.0));
  map.applyEgoMotion(0.5, 0.0, 0.0);
  EXPECT_TRUE(map.lethalQuery(1.0, 0.0, 0.65));
  EXPECT_FALSE(map.lethalQuery(0.5, 0.0, 0.65));
}

TEST(EgoMotionMap, CompensationDisabledIsNoOp)
{
  LocalMapConfig config = memoryConfig();
  config.ego_motion_compensation = false;
  RollingLocalMap map(config);
  map.integratePoints({{1.0, 0.0, 0.0}}, stampAt(1.0));
  map.finalizeFrame(stampAt(1.0));
  map.applyEgoMotion(0.5, 0.0, 0.0);
  EXPECT_TRUE(map.lethalQuery(1.0, 0.0, 0.65));
}

TEST(EgoMotionMap, SnapshotPublishedOnFinalizeAndReset)
{
  RollingLocalMap map(memoryConfig());
  map.integratePoints({{1.0, 0.0, 0.0}}, stampAt(1.0));
  map.finalizeFrame(stampAt(1.0));

  const auto snapshot = map.snapshot();
  ASSERT_NE(snapshot, nullptr);
  float snapshot_max = 0.0f;
  for (const float p : snapshot->probability) {
    snapshot_max = std::max(snapshot_max, p);
  }
  EXPECT_GE(snapshot_max, 0.65f);

  // 旧快照是只读副本：reset 后旧指针内容不变，新快照已清零
  map.reset();
  const auto cleared = map.snapshot();
  ASSERT_NE(cleared, nullptr);
  float cleared_max = 0.0f;
  for (const float p : cleared->probability) {
    cleared_max = std::max(cleared_max, p);
  }
  EXPECT_EQ(cleared_max, 0.0f);
  EXPECT_GE(snapshot_max, 0.65f);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
