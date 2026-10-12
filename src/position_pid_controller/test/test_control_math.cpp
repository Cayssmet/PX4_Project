#include <gtest/gtest.h>

#include <cmath>

#include "position_pid_controller/control_math.hpp"

namespace ppc = position_pid_controller;

TEST(ControlMath, PlanarMinimumSpeedKeepsPidDirection)
{
  const auto command = ppc::apply_planar_speed_limits(
    0.03, 0.04, 3.0, 4.0, 0.15, 0.80);
  EXPECT_NEAR(command.x, 0.09, 1.0e-12);
  EXPECT_NEAR(command.y, 0.12, 1.0e-12);
  EXPECT_NEAR(std::hypot(command.x, command.y), 0.15, 1.0e-12);
}

TEST(ControlMath, ZeroPidOutputFallsBackToPositionErrorDirection)
{
  const auto command = ppc::apply_planar_speed_limits(
    0.0, 0.0, 3.0, 4.0, 0.15, 0.80);
  EXPECT_NEAR(command.x, 0.09, 1.0e-12);
  EXPECT_NEAR(command.y, 0.12, 1.0e-12);
}

TEST(ControlMath, PlanarMaximumLimitUsesVectorMagnitude)
{
  const auto command = ppc::apply_planar_speed_limits(
    3.0, 4.0, 3.0, 4.0, 0.15, 0.80);
  EXPECT_NEAR(command.x, 0.48, 1.0e-12);
  EXPECT_NEAR(command.y, 0.64, 1.0e-12);
  EXPECT_NEAR(std::hypot(command.x, command.y), 0.80, 1.0e-12);
}

TEST(ControlMath, BodyCommandRotatesToMap)
{
  constexpr double pi = 3.14159265358979323846;

  const auto facing_map_x = ppc::body_command_to_map(0.4, -0.2, 0.0);
  EXPECT_NEAR(facing_map_x.x, 0.4, 1.0e-12);
  EXPECT_NEAR(facing_map_x.y, -0.2, 1.0e-12);

  // 机头朝 map +y 时，机体向前对应 map +y，机体向左对应 map -x。
  const auto facing_map_y = ppc::body_command_to_map(0.4, 0.2, pi / 2.0);
  EXPECT_NEAR(facing_map_y.x, -0.2, 1.0e-12);
  EXPECT_NEAR(facing_map_y.y, 0.4, 1.0e-12);
}

TEST(ControlMath, BodyCommandRejectsInvalidNumber)
{
  EXPECT_THROW(
    ppc::body_command_to_map(NAN, 0.0, 0.0),
    std::invalid_argument);
}

TEST(ControlMath, YawMinimumAndMaximumKeepDirection)
{
  EXPECT_DOUBLE_EQ(ppc::apply_yaw_rate_limits(0.02, 0.4, 0.10, 0.80), 0.10);
  EXPECT_DOUBLE_EQ(ppc::apply_yaw_rate_limits(0.0, -0.4, 0.10, 0.80), -0.10);
  // 即使很小的原始输出受微分噪声影响而反向，最小速度仍按 yaw 误差方向输出。
  EXPECT_DOUBLE_EQ(ppc::apply_yaw_rate_limits(-0.02, 0.4, 0.10, 0.80), 0.10);
  EXPECT_DOUBLE_EQ(ppc::apply_yaw_rate_limits(2.0, 0.4, 0.10, 0.80), 0.80);
}

TEST(ControlMath, YawWrapUsesShortestDirectionAcrossPi)
{
  constexpr double pi = 3.14159265358979323846;
  const double current = 179.0 * pi / 180.0;
  const double target = -179.0 * pi / 180.0;
  // 从 +179 度到 -179 度的最短方向是正向约 2 度，而不是反向 358 度。
  EXPECT_NEAR(ppc::wrap_angle(target - current), 2.0 * pi / 180.0, 1.0e-12);
}

TEST(ControlMath, HysteresisPreventsRepeatedStartStop)
{
  ppc::HysteresisHold hold(0.10, 0.15);
  EXPECT_FALSE(hold.update(0.20));
  EXPECT_TRUE(hold.update(0.10));
  EXPECT_TRUE(hold.update(0.12));
  EXPECT_TRUE(hold.update(0.15));
  EXPECT_FALSE(hold.update(0.151));
  EXPECT_FALSE(hold.update(0.12));
  EXPECT_TRUE(hold.update(0.09));
}

TEST(ControlMath, InvalidSpeedLimitsThrow)
{
  EXPECT_THROW(
    ppc::apply_planar_speed_limits(0.0, 0.0, 1.0, 0.0, 0.8, 0.1),
    std::invalid_argument);
}
