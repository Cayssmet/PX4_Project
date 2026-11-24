// ENU/FLU <-> NED/FRD 坐标换算的单元测试。
//
// 这套测试的定位：**坐标系换算是本项目炸机风险第一名**，必须先在这里钉死，
// 再谈任何 offboard 控制。其中 `BodyVectorWorldConsistency` 是最关键的一条，
// 它验证"姿态换算"和"机体矢量换算"两条法则是**互相自洽**的——单看任何一条都
// 可能正确，但两条如果不同源，飞机就会往反方向飞。

#include "px4_offboard/frame_transforms.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

namespace
{

using px4_offboard::EulerAngles;
using px4_offboard::kPi;
using px4_offboard::Quaternion;
using px4_offboard::Vector3;

constexpr double kTolerance = 1e-9;

double deg2rad(const double deg)
{
  return deg * kPi / 180.0;
}

double rad2deg(const double rad)
{
  return rad * 180.0 / kPi;
}

/// 四元数比较：允许整体符号相反（q 与 -q 表示同一个旋转）。
void expect_quaternion_near(
  const Quaternion & actual, const Quaternion & expected, const double tolerance = kTolerance)
{
  const double same = std::fabs(actual.w - expected.w) + std::fabs(actual.x - expected.x) +
    std::fabs(actual.y - expected.y) + std::fabs(actual.z - expected.z);
  const double flipped = std::fabs(actual.w + expected.w) + std::fabs(actual.x + expected.x) +
    std::fabs(actual.y + expected.y) + std::fabs(actual.z + expected.z);
  EXPECT_NEAR(std::min(same, flipped), 0.0, tolerance);
}

void expect_vector_near(
  const Vector3 & actual, const double x, const double y, const double z,
  const double tolerance = kTolerance)
{
  EXPECT_NEAR(actual.x, x, tolerance);
  EXPECT_NEAR(actual.y, y, tolerance);
  EXPECT_NEAR(actual.z, z, tolerance);
}

double vector_error(const Vector3 & a, const Vector3 & b)
{
  return std::fabs(a.x - b.x) + std::fabs(a.y - b.y) + std::fabs(a.z - b.z);
}

}  // namespace

// ---------------------------------------------------------------------------
// 常量与旋转矩阵
// ---------------------------------------------------------------------------

TEST(FrameTransforms, ConstantsMatchPx4RosCom)
{
  expect_quaternion_near(
    px4_offboard::ned_enu_quaternion(),
    Quaternion{0.0, std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0, 0.0});
  expect_quaternion_near(
    px4_offboard::aircraft_baselink_quaternion(), Quaternion{0.0, 1.0, 0.0, 0.0});

  // 常量必须与"按欧拉角构造"等价，否则说明手写常量抄错了。
  expect_quaternion_near(
    px4_offboard::ned_enu_quaternion(),
    px4_offboard::quaternion_from_euler(kPi, 0.0, kPi / 2.0));
  expect_quaternion_near(
    px4_offboard::aircraft_baselink_quaternion(),
    px4_offboard::quaternion_from_euler(kPi, 0.0, 0.0));
}

TEST(FrameTransforms, EnuToNedRotationMatrixIsExpectedReflection)
{
  // NED_ENU_Q 的旋转矩阵应当是 [[0,1,0],[1,0,0],[0,0,-1]]。
  const Quaternion & q = px4_offboard::ned_enu_quaternion();

  expect_vector_near(px4_offboard::quaternion_rotate(q, Vector3{1.0, 0.0, 0.0}), 0.0, 1.0, 0.0);
  expect_vector_near(px4_offboard::quaternion_rotate(q, Vector3{0.0, 1.0, 0.0}), 1.0, 0.0, 0.0);
  expect_vector_near(px4_offboard::quaternion_rotate(q, Vector3{0.0, 0.0, 1.0}), 0.0, 0.0, -1.0);
}

TEST(FrameTransforms, AircraftToBaselinkFlipsRightAndDown)
{
  const Quaternion & q = px4_offboard::aircraft_baselink_quaternion();

  expect_vector_near(px4_offboard::quaternion_rotate(q, Vector3{1.0, 0.0, 0.0}), 1.0, 0.0, 0.0);
  expect_vector_near(px4_offboard::quaternion_rotate(q, Vector3{0.0, 1.0, 0.0}), 0.0, -1.0, 0.0);
  expect_vector_near(px4_offboard::quaternion_rotate(q, Vector3{0.0, 0.0, 1.0}), 0.0, 0.0, -1.0);
}

// ---------------------------------------------------------------------------
// 世界系位置与速度
// ---------------------------------------------------------------------------

TEST(FrameTransforms, WorldPositionSwapsXYAndFlipsZ)
{
  // 地图上"东 1 m、北 2 m、上 0.5 m"在 PX4 里应当是"北 2 m、东 1 m、下 -0.5 m"。
  expect_vector_near(
    px4_offboard::map_position_to_px4(Vector3{1.0, 2.0, 0.5}), 2.0, 1.0, -0.5);
  expect_vector_near(px4_offboard::map_velocity_to_px4(Vector3{0.3, -0.4, 0.0}), -0.4, 0.3, 0.0);
}

TEST(FrameTransforms, WorldPositionIsItsOwnInverse)
{
  const Vector3 original{1.25, -3.5, 0.75};
  const Vector3 round_tripped = px4_offboard::px4_position_to_map(
    px4_offboard::map_position_to_px4(original));

  EXPECT_LT(vector_error(original, round_tripped), kTolerance);
}

// ---------------------------------------------------------------------------
// 偏航：反射，不是偏移
// ---------------------------------------------------------------------------

TEST(FrameTransforms, YawIsReflectionNotConstantOffset)
{
  // yaw_px4 = pi/2 - yaw_map。注意这不是加一个常数：对 yaw_map = 90° 得 0°，
  // 对 yaw_map = -90° 得 180°，两者之差的符号已经反过来了。
  EXPECT_NEAR(rad2deg(px4_offboard::map_yaw_to_px4(0.0)), 90.0, 1e-9);
  EXPECT_NEAR(rad2deg(px4_offboard::map_yaw_to_px4(deg2rad(90.0))), 0.0, 1e-9);
  EXPECT_NEAR(rad2deg(px4_offboard::map_yaw_to_px4(deg2rad(-90.0))), 180.0, 1e-9);
  EXPECT_NEAR(rad2deg(px4_offboard::map_yaw_to_px4(deg2rad(180.0))), -90.0, 1e-9);

  // 自逆
  for (double yaw = -kPi; yaw <= kPi; yaw += 0.37) {
    EXPECT_NEAR(px4_offboard::px4_yaw_to_map(px4_offboard::map_yaw_to_px4(yaw)), yaw, 1e-9);
  }
}

TEST(FrameTransforms, YawRateSignFlips)
{
  EXPECT_NEAR(px4_offboard::map_yaw_rate_to_px4(1.0), -1.0, kTolerance);
  EXPECT_NEAR(px4_offboard::map_yaw_rate_to_px4(-0.5), 0.5, kTolerance);
  EXPECT_NEAR(px4_offboard::map_yaw_rate_to_px4(0.0), 0.0, kTolerance);
}

TEST(FrameTransforms, PureYawAttitudeMatchesYawFormula)
{
  for (double yaw = -kPi; yaw <= kPi; yaw += 0.29) {
    const Quaternion q_map = px4_offboard::quaternion_from_euler(0.0, 0.0, yaw);
    const Quaternion q_px4 = px4_offboard::map_attitude_to_px4(q_map);

    // 用"归一化后的角差"比较，避免 pi 与 -pi 表示同一角度时误判。
    EXPECT_NEAR(
      px4_offboard::normalize_angle(
        px4_offboard::quaternion_yaw(q_px4) - px4_offboard::map_yaw_to_px4(yaw)),
      0.0, 1e-9);
    // 纯偏航不应引入横滚/俯仰
    const EulerAngles e = px4_offboard::quaternion_to_euler(q_px4);
    EXPECT_NEAR(e.roll, 0.0, 1e-9);
    EXPECT_NEAR(e.pitch, 0.0, 1e-9);
  }
}

// ---------------------------------------------------------------------------
// 姿态的物理语义
// ---------------------------------------------------------------------------

TEST(FrameTransforms, AttitudePhysicalSemantics)
{
  // 机体水平、机头朝地图 +x（东），抬头 10°。ROS 侧: (roll 0, pitch +10°, yaw 0)
  // PX4 侧应当是: 横滚约 0、俯仰 -10°（FRD 里 z 朝下，抬头是负俯仰）、偏航 +90°（NED 里东就是 +90°）
  const Quaternion q_map = px4_offboard::quaternion_from_euler(0.0, deg2rad(10.0), 0.0);
  const EulerAngles e = px4_offboard::quaternion_to_euler(
    px4_offboard::map_attitude_to_px4(q_map));

  EXPECT_NEAR(rad2deg(e.roll), 0.0, 1e-6);
  EXPECT_NEAR(rad2deg(e.pitch), -10.0, 1e-6);
  EXPECT_NEAR(rad2deg(e.yaw), 90.0, 1e-6);
}

TEST(FrameTransforms, AttitudeRoundTrip)
{
  std::mt19937 rng(20261012);
  std::uniform_real_distribution<double> angle(-kPi, kPi);

  for (int i = 0; i < 2000; ++i) {
    const Quaternion q = px4_offboard::quaternion_from_euler(
      angle(rng), angle(rng), angle(rng));
    const Quaternion round_tripped = px4_offboard::px4_attitude_to_map(
      px4_offboard::map_attitude_to_px4(q));

    expect_quaternion_near(round_tripped, q, 1e-9);
  }
}

// ---------------------------------------------------------------------------
// 机体矢量
// ---------------------------------------------------------------------------

TEST(FrameTransforms, BodyVectorKeepsXAndFlipsYZ)
{
  expect_vector_near(px4_offboard::body_vector_flu_to_frd(Vector3{1.0, 0.0, 0.0}), 1.0, 0.0, 0.0);
  // FLU 里 +y 是"向左"，FRD 里 +y 是"向右"，同一个方向在 FRD 里是 -y。
  expect_vector_near(px4_offboard::body_vector_flu_to_frd(Vector3{0.0, 1.0, 0.0}), 0.0, -1.0, 0.0);
  expect_vector_near(px4_offboard::body_vector_flu_to_frd(Vector3{0.0, 0.0, 1.0}), 0.0, 0.0, -1.0);

  const Vector3 original{0.4, -1.2, 2.5};
  const Vector3 round_tripped = px4_offboard::frd_vector_to_flu(
    px4_offboard::body_vector_flu_to_frd(original));
  EXPECT_LT(vector_error(original, round_tripped), kTolerance);
}

// ---------------------------------------------------------------------------
// ★ 最关键的一条：两条法则必须互相自洽
// ---------------------------------------------------------------------------

TEST(FrameTransforms, BodyVectorWorldConsistency)
{
  // 不变式：把一个机体系矢量 v（FLU）先在 ROS 侧转到世界系（ENU），
  // 与"先把 v 换成 FRD、再用 PX4 姿态转到 NED"，必须得到同一个物理方向。
  //
  //   rotate(q_px4, flu_to_frd(v)) == map_velocity_to_px4(rotate(q_map, v))
  //
  // 如果姿态换算是错的、或者机体矢量法则是错的，这条会失败——而单看
  // 任何一条单独的测试都可能"通过"。
  std::mt19937 rng(20261013);
  std::uniform_real_distribution<double> angle(-kPi, kPi);
  std::uniform_real_distribution<double> component(-2.0, 2.0);

  for (int i = 0; i < 20000; ++i) {
    const Quaternion q_map = px4_offboard::quaternion_from_euler(
      angle(rng), angle(rng), angle(rng));
    const Quaternion q_px4 = px4_offboard::map_attitude_to_px4(q_map);
    const Vector3 v_flu{component(rng), component(rng), component(rng)};

    const Vector3 via_px4 = px4_offboard::quaternion_rotate(
      q_px4, px4_offboard::body_vector_flu_to_frd(v_flu));
    const Vector3 via_ros = px4_offboard::map_velocity_to_px4(
      px4_offboard::quaternion_rotate(q_map, v_flu));

    ASSERT_LT(vector_error(via_px4, via_ros), 1e-9) << "sample #" << i;
  }
}

TEST(FrameTransforms, ClosedFormCrossCheck)
{
  // 交叉校验：用一个独立推导出来的闭式再算一遍姿态。
  //   q_px4 = ((w+z), (x+y), (x-y), (w-z)) / √2
  // 注意这条闭式只用于**测试**，生产代码一律走两次显式四元数乘法。
  std::mt19937 rng(20261014);
  std::uniform_real_distribution<double> angle(-kPi, kPi);

  for (int i = 0; i < 20000; ++i) {
    const Quaternion q = px4_offboard::quaternion_from_euler(angle(rng), angle(rng), angle(rng));
    const Quaternion actual = px4_offboard::map_attitude_to_px4(q);
    const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
    const Quaternion closed_form{
      (q.w + q.z) * inv_sqrt2,
      (q.x + q.y) * inv_sqrt2,
      (q.x - q.y) * inv_sqrt2,
      (q.w - q.z) * inv_sqrt2};

    expect_quaternion_near(actual, closed_form, 1e-9);
  }
}

// ---------------------------------------------------------------------------
// 边界
// ---------------------------------------------------------------------------

TEST(FrameTransforms, AngleNormalizationRange)
{
  EXPECT_NEAR(px4_offboard::normalize_angle(3.0 * kPi), kPi, 1e-9);
  EXPECT_NEAR(px4_offboard::normalize_angle(-3.0 * kPi), kPi, 1e-9);
  EXPECT_NEAR(px4_offboard::normalize_angle(0.5), 0.5, kTolerance);

  for (double a = -10.0; a <= 10.0; a += 0.13) {
    const double wrapped = px4_offboard::normalize_angle(a);
    EXPECT_GT(wrapped, -kPi);
    EXPECT_LE(wrapped, kPi + 1e-12);
  }
}

TEST(FrameTransforms, DegenerateQuaternionIsHandled)
{
  // 全零四元数不应该产生 NaN，而是退化为单位四元数。
  const Quaternion zero{0.0, 0.0, 0.0, 0.0};
  const Quaternion normalized = px4_offboard::quaternion_normalized(zero);
  EXPECT_DOUBLE_EQ(normalized.w, 1.0);
  EXPECT_DOUBLE_EQ(normalized.x, 0.0);

  const Vector3 rotated = px4_offboard::quaternion_rotate(zero, Vector3{1.0, 2.0, 3.0});
  EXPECT_TRUE(std::isfinite(rotated.x));
  EXPECT_TRUE(std::isfinite(rotated.y));
  EXPECT_TRUE(std::isfinite(rotated.z));
}
