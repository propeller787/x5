#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "wheelchair_obstacle/obstacle_cloud_filter_core.hpp"

using wheelchair_obstacle::FilterPoint;
using wheelchair_obstacle::FilterStats;
using wheelchair_obstacle::IsolationConfig;
using wheelchair_obstacle::ObstacleCloudFilterCore;
using wheelchair_obstacle::PolygonMask;
using wheelchair_obstacle::RangeConfig;
using wheelchair_obstacle::RoiConfig;
using wheelchair_obstacle::VoxelConfig;

namespace
{
FilterPoint makePoint(float x, float y, float z, float intensity = 0.0F)
{
  return FilterPoint{x, y, z, intensity};
}

ObstacleCloudFilterCore makeDefaultCore()
{
  ObstacleCloudFilterCore core;
  core.set_self_masks(ObstacleCloudFilterCore::default_self_masks());
  return core;
}

}  // namespace

TEST(ObstacleCloudFilterCore, IsFiniteDropsNaNAndInf)
{
  auto core = makeDefaultCore();
  std::vector<FilterPoint> input{
    makePoint(1.0F, 0.0F, 0.0F),
    makePoint(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F),
    makePoint(1.0F, std::numeric_limits<float>::infinity(), 0.0F),
    makePoint(0.0F, 1.0F, 0.0F),
  };
  FilterStats stats;
  auto out = core.filter(input, stats);
  EXPECT_EQ(stats.after_finite_range, 2u);
  EXPECT_EQ(out.size(), stats.after_voxel);
}

TEST(ObstacleCloudFilterCore, RangeFilterDropsTooCloseAndTooFar)
{
  auto core = makeDefaultCore();
  core.set_self_mask_enabled(false);
  core.set_isolation_enabled(false);
  core.set_voxel_enabled(false);

  std::vector<FilterPoint> input{
    makePoint(0.02F, 0.0F, 0.0F),    // r=0.02 < range_min(0.05)
    makePoint(0.10F, 0.0F, 0.0F),    // r=0.10 ok
    makePoint(3.50F, 0.0F, 0.0F),    // r=3.50 > range_max(3.00)
  };
  FilterStats stats;
  auto out = core.filter(input, stats);
  EXPECT_EQ(stats.after_finite_range, 1u);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].x, 0.10F);
}

TEST(ObstacleCloudFilterCore, ZFilterDropsOutOfRangeHeight)
{
  auto core = makeDefaultCore();
  core.set_self_mask_enabled(false);
  core.set_isolation_enabled(false);
  core.set_voxel_enabled(false);

  std::vector<FilterPoint> input{
    makePoint(1.0F, 0.0F, -0.50F),   // z < z_min
    makePoint(1.0F, 0.0F, 0.0F),     // ok
    makePoint(1.0F, 0.0F, 1.00F),    // z > z_max
  };
  FilterStats stats;
  auto out = core.filter(input, stats);
  EXPECT_EQ(stats.after_finite_range, 1u);
  ASSERT_EQ(out.size(), 1u);
}

TEST(ObstacleCloudFilterCore, RoiDropsOutsideRectangle)
{
  auto core = makeDefaultCore();
  core.set_self_mask_enabled(false);
  core.set_isolation_enabled(false);
  core.set_voxel_enabled(false);

  std::vector<FilterPoint> input{
    makePoint(0.50F, 0.0F, 0.0F),     // inside ROI and range
    makePoint(3.0F, 0.0F, 0.0F),      // x > x_max(2.5)
    makePoint(0.0F, 2.0F, 0.0F),      // y > y_max(1.5)
    makePoint(-2.0F, 0.0F, 0.0F),     // x < x_min(-1.2)
  };
  FilterStats stats;
  auto out = core.filter(input, stats);
  EXPECT_EQ(stats.after_roi, 1u);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].x, 0.50F);
}

TEST(ObstacleCloudFilterCore, SelfMaskDropsPointsInsideChassis)
{
  auto core = makeDefaultCore();
  core.set_isolation_enabled(false);
  core.set_voxel_enabled(false);

  std::vector<FilterPoint> input{
    makePoint(0.0F, 0.0F, 0.0F),     // inside chassis -> dropped
    makePoint(0.6F, 0.0F, 0.0F),     // inside front_footrest -> dropped
    makePoint(1.2F, 0.0F, 0.0F),     // outside masks -> kept
  };
  FilterStats stats;
  auto out = core.filter(input, stats);
  EXPECT_EQ(stats.after_self_mask, 1u);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].x, 1.2F);
}

TEST(ObstacleCloudFilterCore, SelfMaskDisabledKeepsAllRoiPoints)
{
  auto core = makeDefaultCore();
  core.set_self_mask_enabled(false);
  core.set_isolation_enabled(false);
  core.set_voxel_enabled(false);

  std::vector<FilterPoint> input{
    makePoint(0.30F, 0.0F, 0.0F),
    makePoint(1.20F, 0.0F, 0.0F),
  };
  FilterStats stats;
  auto out = core.filter(input, stats);
  EXPECT_EQ(stats.after_self_mask, 2u);
  EXPECT_EQ(out.size(), 2u);
}

TEST(ObstacleCloudFilterCore, PointInPolygonBasic)
{
  std::vector<std::pair<double, double>> square{
    {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
  EXPECT_TRUE(ObstacleCloudFilterCore::point_in_polygon(1.0, 1.0, square));
  EXPECT_FALSE(ObstacleCloudFilterCore::point_in_polygon(3.0, 1.0, square));
  EXPECT_FALSE(ObstacleCloudFilterCore::point_in_polygon(-1.0, 1.0, square));
}

TEST(ObstacleCloudFilterCore, IsolationDropsLonelyPointKeepsCluster)
{
  auto core = makeDefaultCore();
  core.set_self_mask_enabled(false);
  core.set_voxel_enabled(false);
  IsolationConfig iso = core.isolation_config();
  iso.neighbor_radius = 0.08;
  iso.min_neighbors = 2;
  iso.skip_isolation_range = 0.50;
  core.set_isolation_config(iso);

  // Three close points form a cluster; one isolated point far away.
  std::vector<FilterPoint> input{
    makePoint(1.00F, 0.00F, 0.0F),
    makePoint(1.02F, 0.00F, 0.0F),
    makePoint(1.04F, 0.00F, 0.0F),
    makePoint(2.00F, 1.20F, 0.0F),  // lonely
  };
  FilterStats stats;
  auto out = core.filter(input, stats);
  EXPECT_EQ(stats.after_isolation, 3u);
  for (const auto & p : out) {
    EXPECT_NE(p.x, 2.00F);
  }
}

TEST(ObstacleCloudFilterCore, IsolationKeepsNearSinglePointForSafety)
{
  auto core = makeDefaultCore();
  core.set_self_mask_enabled(false);
  core.set_voxel_enabled(false);
  IsolationConfig iso = core.isolation_config();
  iso.min_neighbors = 5;
  iso.skip_isolation_range = 0.50;
  core.set_isolation_config(iso);

  // A single near point (r < 0.5m) must survive isolation even with no neighbors.
  std::vector<FilterPoint> input{
    makePoint(0.30F, 0.0F, 0.0F),  // r=0.30 < skip_isolation_range
  };
  FilterStats stats;
  auto out = core.filter(input, stats);
  EXPECT_EQ(stats.after_isolation, 1u);
  ASSERT_EQ(out.size(), 1u);
}

TEST(ObstacleCloudFilterCore, VoxelDownsampleKeepsNearestPerCell)
{
  auto core = makeDefaultCore();
  core.set_self_mask_enabled(false);
  core.set_isolation_enabled(false);
  VoxelConfig v = core.voxel_config();
  v.voxel_size_xy = 0.10;
  v.keep_nearest = true;
  core.set_voxel_config(v);

  // All within same 0.10m voxel; nearest to origin should win.
  std::vector<FilterPoint> input{
    makePoint(1.00F, 0.00F, 0.0F),  // r=1.0 nearest
    makePoint(1.03F, 0.02F, 0.0F),  // r~1.030
    makePoint(1.08F, 0.01F, 0.0F),  // r~1.080
  };
  FilterStats stats;
  auto out = core.filter(input, stats);
  EXPECT_EQ(stats.after_voxel, 1u);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].x, 1.00F);
}

TEST(ObstacleCloudFilterCore, VoxelDownsampleSeparatesDifferentCells)
{
  auto core = makeDefaultCore();
  core.set_self_mask_enabled(false);
  core.set_isolation_enabled(false);
  VoxelConfig v = core.voxel_config();
  v.voxel_size_xy = 0.10;
  v.keep_nearest = true;
  core.set_voxel_config(v);

  std::vector<FilterPoint> input{
    makePoint(1.00F, 0.00F, 0.0F),  // voxel (10, 0)
    makePoint(1.20F, 0.00F, 0.0F),  // voxel (12, 0)
  };
  FilterStats stats;
  auto out = core.filter(input, stats);
  EXPECT_EQ(stats.after_voxel, 2u);
}

TEST(ObstacleCloudFilterCore, EmptyInputProducesEmptyOutputWithoutCrash)
{
  auto core = makeDefaultCore();
  FilterStats stats;
  auto out = core.filter({}, stats);
  EXPECT_EQ(stats.input, 0u);
  EXPECT_TRUE(out.empty());
}

TEST(ObstacleCloudFilterCore, FullPipelinePreservesNearFieldObstacle)
{
  auto core = makeDefaultCore();

  // A vertical "leg" cluster at 1.0m ahead (outside front_footrest mask), multiple points.
  std::vector<FilterPoint> input;
  for (int i = 0; i < 5; ++i) {
    input.push_back(makePoint(1.00F, static_cast<float>(i) * 0.01F, 0.0F, 1.0F));
  }
  // Add chassis self-return noise (inside chassis mask, passes range).
  input.push_back(makePoint(0.20F, 0.0F, 0.0F));
  // Add a far isolated noise point inside ROI.
  input.push_back(makePoint(2.20F, 1.20F, 0.0F));
  // Add a NaN.
  input.push_back(makePoint(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F));

  FilterStats stats;
  auto out = core.filter(input, stats);

  EXPECT_EQ(stats.input, 8u);
  EXPECT_GT(stats.after_finite_range, 0u);
  EXPECT_GT(stats.after_roi, 0u);
  EXPECT_GT(stats.after_self_mask, 0u);
  EXPECT_GE(stats.after_isolation, 1u);
  ASSERT_GE(out.size(), 1u);
  // All surviving points should be the near-field leg cluster (x ~ 1.0).
  for (const auto & p : out) {
    EXPECT_NEAR(p.x, 1.00F, 0.05F);
    EXPECT_FLOAT_EQ(p.intensity, 1.0F);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
