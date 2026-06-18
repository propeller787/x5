#include <gtest/gtest.h>

#include "safety_node/core/obstacle_field.h"
#include "safety_node/perception_utils.h"

#include <cmath>
#include <limits>

TEST(PerceptionUtilsTest, PolarCoordinatesAreConverted)
{
  const auto [x, y] = PerceptionUtils::polarToCartesian(2.0, M_PI_2);
  EXPECT_NEAR(x, 0.0, 1e-9);
  EXPECT_NEAR(y, 2.0, 1e-9);
}

TEST(PerceptionUtilsTest, InvalidRangesAreRejected)
{
  EXPECT_FALSE(
    PerceptionUtils::validRange(
      std::numeric_limits<float>::quiet_NaN(), 0.1f, 10.0f));
  EXPECT_FALSE(
    PerceptionUtils::validRange(
      std::numeric_limits<float>::infinity(), 0.1f, 10.0f));
  EXPECT_FALSE(PerceptionUtils::validRange(0.09f, 0.1f, 10.0f));
  EXPECT_FALSE(PerceptionUtils::validRange(10.1f, 0.1f, 10.0f));
  EXPECT_TRUE(PerceptionUtils::validRange(0.1f, 0.1f, 10.0f));
  EXPECT_TRUE(PerceptionUtils::validRange(10.0f, 0.1f, 10.0f));
}

TEST(PerceptionUtilsTest, RoiBoundariesAreInclusive)
{
  EXPECT_TRUE(PerceptionUtils::inRoi(-2.5, -1.2));
  EXPECT_TRUE(PerceptionUtils::inRoi(5.0, 1.2));
  EXPECT_FALSE(PerceptionUtils::inRoi(-2.5001, 0.0));
  EXPECT_FALSE(PerceptionUtils::inRoi(5.0001, 0.0));
  EXPECT_FALSE(PerceptionUtils::inRoi(0.0, 1.2001));
}

TEST(PerceptionUtilsTest, TiltHeightBoundaryIsInclusive)
{
  EXPECT_TRUE(PerceptionUtils::validTiltZ(0.27, 0.27));
  EXPECT_TRUE(PerceptionUtils::validTiltZ(-0.27, 0.27));
  EXPECT_FALSE(PerceptionUtils::validTiltZ(0.271, 0.27));
  EXPECT_TRUE(PerceptionUtils::validTiltZ(1.50, 0.0));
  EXPECT_TRUE(PerceptionUtils::validTiltZ(-1.50, -1.0));
  EXPECT_FALSE(
    PerceptionUtils::validTiltZ(
      std::numeric_limits<double>::quiet_NaN(), 0.0));
}

TEST(PerceptionUtilsTest, SourceIdsAndColorsAreStable)
{
  const std::array<std::string, 5> names = {
    "left", "right", "back", "left_tilt", "right_tilt"};
  const std::array<std::array<uint8_t, 3>, 5> colors = {{
    {40, 240, 240},
    {80, 255, 80},
    {255, 160, 40},
    {80, 160, 255},
    {255, 80, 220},
  }};
  for (size_t i = 0; i < names.size(); ++i) {
    const uint8_t id = PerceptionUtils::sourceId(names[i]);
    EXPECT_EQ(id, i);
    EXPECT_EQ(PerceptionUtils::debugColor(id), colors[i]);
  }
}

TEST(ObstacleFieldTest, PointsInSameVoxelAreMerged)
{
  AvoidanceParams params;
  params.footprint_obstacle_radius_default = 0.01;
  const std::vector<std::tuple<double, double, double>> points = {
    {0.01, 0.01, 0.0},
    {0.02, 0.02, 0.0},
    {0.50, 0.0, 0.0},
  };
  const ObstacleField field = ObstacleFieldBuilder::buildFromPoints(
    points, rclcpp::Time(1, 0, RCL_ROS_TIME), params, 0.12);

  ASSERT_EQ(field.obstacles.size(), 2u);
  EXPECT_TRUE(field.obstacles[0].valid);
  EXPECT_NEAR(field.obstacles[0].radius, 0.01, 1e-9);
}

TEST(ObstacleFieldTest, PrimitiveLimitKeepsNearestObstacles)
{
  AvoidanceParams params;
  params.max_obstacle_primitives = 3;
  std::vector<std::tuple<double, double, double>> points;
  for (int i = 1; i <= 10; ++i) {
    points.emplace_back(static_cast<double>(i), 0.0, 0.0);
  }

  const ObstacleField field = ObstacleFieldBuilder::buildFromPoints(
    points, rclcpp::Time(1, 0, RCL_ROS_TIME), params, 0.12);

  ASSERT_EQ(field.obstacles.size(), 3u);
  for (const auto & obstacle : field.obstacles) {
    EXPECT_LE(std::hypot(obstacle.x, obstacle.y), 3.01);
  }
}

TEST(ObstacleFieldTest, NegativeAndNonFiniteCoordinatesAreHandledSafely)
{
  AvoidanceParams params;
  const std::vector<std::tuple<double, double, double>> points = {
    {-0.5, -0.25, 0.0},
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
    {0.0, std::numeric_limits<double>::infinity(), 0.0},
  };

  const ObstacleField field = ObstacleFieldBuilder::buildFromPoints(
    points, rclcpp::Time(1, 0, RCL_ROS_TIME), params, 0.12);

  ASSERT_EQ(field.obstacles.size(), 1u);
  EXPECT_NEAR(field.obstacles.front().x, -0.5, 1e-9);
  EXPECT_NEAR(field.obstacles.front().y, -0.25, 1e-9);
}
