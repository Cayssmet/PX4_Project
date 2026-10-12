#include <cmath>

#include "gtest/gtest.h"

#include "waypoint_manager/competition_geometry.hpp"

TEST(CompetitionGeometry, YawZeroUsesForwardXAndLeftY)
{
  const auto target = waypoint_manager::body_relative_target(
    1.0, 2.0, 0.0, 3.0, 0.75);
  EXPECT_NEAR(target.x, 4.0, 1.0e-12);
  EXPECT_NEAR(target.y, 2.75, 1.0e-12);
  EXPECT_NEAR(target.yaw, 0.0, 1.0e-12);
}

TEST(CompetitionGeometry, PositiveLeftRotatesWithInitialYaw)
{
  constexpr double half_pi = 1.5707963267948966;
  const auto target = waypoint_manager::body_relative_target(
    1.0, 2.0, half_pi, 3.0, 0.75);
  EXPECT_NEAR(target.x, 0.25, 1.0e-12);
  EXPECT_NEAR(target.y, 5.0, 1.0e-12);
  EXPECT_NEAR(target.yaw, half_pi, 1.0e-12);
}

TEST(CompetitionGeometry, NegativeLateralOffsetMeansRight)
{
  const auto target = waypoint_manager::body_relative_target(
    0.0, 0.0, 0.0, 0.0, -0.75);
  EXPECT_NEAR(target.x, 0.0, 1.0e-12);
  EXPECT_NEAR(target.y, -0.75, 1.0e-12);
}
