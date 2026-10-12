#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "map_coordinate_provider/pose_math.hpp"

namespace mcp = map_coordinate_provider;

TEST(PoseMath, PositionMustBeFinite)
{
  EXPECT_TRUE(mcp::position_is_finite(1.0, -2.0, 0.5));
  EXPECT_FALSE(mcp::position_is_finite(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  EXPECT_FALSE(mcp::position_is_finite(
    0.0, std::numeric_limits<double>::infinity(), 0.0));
}

TEST(PoseMath, AbsoluteMapRangeIsBounded)
{
  EXPECT_TRUE(mcp::absolute_position_is_plausible(3.0, 4.0, 7.0));
  EXPECT_TRUE(mcp::absolute_position_is_plausible(7.0, 0.0, 7.0));
  EXPECT_FALSE(mcp::absolute_position_is_plausible(5.0, 5.0, 7.0));
  EXPECT_FALSE(mcp::absolute_position_is_plausible(153.0, -349.0, 7.0));
  EXPECT_FALSE(mcp::absolute_position_is_plausible(0.0, 0.0, 0.0));
  EXPECT_FALSE(mcp::absolute_position_is_plausible(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 7.0));
}

TEST(PoseMath, IdentityQuaternionProducesZeroYaw)
{
  mcp::NormalizedQuaternionYaw result;
  ASSERT_TRUE(mcp::normalize_quaternion_and_get_yaw(0.0, 0.0, 0.0, 1.0, result));
  EXPECT_DOUBLE_EQ(result.x, 0.0);
  EXPECT_DOUBLE_EQ(result.y, 0.0);
  EXPECT_DOUBLE_EQ(result.z, 0.0);
  EXPECT_DOUBLE_EQ(result.w, 1.0);
  EXPECT_DOUBLE_EQ(result.yaw, 0.0);
}

TEST(PoseMath, QuaternionProducesNinetyDegreeYaw)
{
  constexpr double pi = 3.14159265358979323846;
  const double half_sqrt_two = std::sqrt(0.5);
  mcp::NormalizedQuaternionYaw result;
  ASSERT_TRUE(mcp::normalize_quaternion_and_get_yaw(
    0.0, 0.0, half_sqrt_two, half_sqrt_two, result));
  EXPECT_NEAR(result.yaw, pi / 2.0, 1.0e-12);
}

TEST(PoseMath, NonUnitQuaternionIsNormalized)
{
  mcp::NormalizedQuaternionYaw result;
  ASSERT_TRUE(mcp::normalize_quaternion_and_get_yaw(0.0, 0.0, 0.0, 2.0, result));
  EXPECT_DOUBLE_EQ(result.w, 1.0);
  EXPECT_DOUBLE_EQ(result.yaw, 0.0);
}

TEST(PoseMath, InvalidQuaternionIsRejected)
{
  mcp::NormalizedQuaternionYaw result;
  EXPECT_FALSE(mcp::normalize_quaternion_and_get_yaw(0.0, 0.0, 0.0, 0.0, result));
  EXPECT_FALSE(mcp::normalize_quaternion_and_get_yaw(
    0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 1.0, result));
}

TEST(PoseMath, PlausibleMotionIsAccepted)
{
  EXPECT_TRUE(mcp::pose_motion_is_plausible(
    0.0, 0.0, 3.13, 0.03, 0.0, -3.13, 0.05,
    0.20, 0.35, 0.80, 1.50));
}

TEST(PoseMath, PositionAndYawJumpsAreRejected)
{
  EXPECT_FALSE(mcp::pose_motion_is_plausible(
    0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.10,
    0.20, 0.35, 0.80, 1.50));
  EXPECT_FALSE(mcp::pose_motion_is_plausible(
    0.0, 0.0, 0.0, 0.01, 0.0, 0.40, 0.10,
    0.20, 0.35, 0.80, 1.50));
}
