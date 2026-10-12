#include <gtest/gtest.h>

#include "position_pid_controller/pid_axis.hpp"

namespace ppc = position_pid_controller;

TEST(PidAxis, ProportionalTermAndOutputLimit)
{
  ppc::PidAxis pid(2.0, 0.0, 0.0, 10.0, 3.0, 0.8);
  EXPECT_DOUBLE_EQ(pid.update_error(1.0, 0.1), 2.0);
  EXPECT_DOUBLE_EQ(pid.update_error(2.0, 0.1), 3.0);
  EXPECT_DOUBLE_EQ(pid.update_error(-2.0, 0.1), -3.0);
}

TEST(PidAxis, IntegralIsClampedSymmetrically)
{
  ppc::PidAxis pid(0.0, 1.0, 0.0, 0.5, 10.0, 0.8);
  EXPECT_DOUBLE_EQ(pid.update_error(1.0, 1.0), 0.5);
  EXPECT_DOUBLE_EQ(pid.integral(), 0.5);
  EXPECT_DOUBLE_EQ(pid.update_error(-2.0, 1.0), -0.5);
  EXPECT_DOUBLE_EQ(pid.integral(), -0.5);
}

TEST(PidAxis, DerivativeUsesLowPassFilter)
{
  ppc::PidAxis pid(0.0, 0.0, 1.0, 10.0, 100.0, 0.8);
  // 第一次没有历史误差，微分为零。
  EXPECT_DOUBLE_EQ(pid.update_error(1.0, 0.1), 0.0);
  // 原始微分为 10，旧值权重 0.8，因此滤波后为 2。
  EXPECT_NEAR(pid.update_error(2.0, 0.1), 2.0, 1.0e-12);
  EXPECT_NEAR(pid.filtered_derivative(), 2.0, 1.0e-12);
}

TEST(PidAxis, ResetClearsAllStoredState)
{
  ppc::PidAxis pid(1.0, 1.0, 1.0, 10.0, 100.0, 0.8);
  (void)pid.update_error(1.0, 0.1);
  (void)pid.update_error(2.0, 0.1);
  pid.reset();

  EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
  EXPECT_DOUBLE_EQ(pid.filtered_derivative(), 0.0);
  EXPECT_FALSE(pid.has_previous_error());
  // 重置后再次计算仍应将微分视为零。
  EXPECT_NEAR(pid.update_error(1.0, 0.1), 1.1, 1.0e-12);
}

TEST(PidAxis, InvalidConfigurationThrows)
{
  EXPECT_THROW(ppc::PidAxis(1.0, 0.0, 0.0, -1.0, 1.0, 0.8), std::invalid_argument);
  EXPECT_THROW(ppc::PidAxis(1.0, 0.0, 0.0, 1.0, 1.0, 1.0), std::invalid_argument);
}
