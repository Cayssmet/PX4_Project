#ifndef PX4_OFFBOARD__FRAME_TRANSFORMS_HPP_
#define PX4_OFFBOARD__FRAME_TRANSFORMS_HPP_

#include <cmath>

/**
 * @file frame_transforms.hpp
 * @brief ENU/FLU（ROS）与 NED/FRD（PX4）之间的坐标换算。
 *
 * 本文件**不依赖 ROS、不依赖 Eigen**，只用普通结构体和 <cmath>，
 * 因此可以脱离 ROS 环境单独编译和单元测试。
 *
 * ## 四套坐标系
 *
 * | 用途 | ROS 侧 | PX4 侧 | 轴向 |
 * | --- | --- | --- | --- |
 * | 世界（地图） | ENU / `odom` / `map` | NED | ROS: x=东 y=北 z=上；PX4: x=北 y=东 z=下 |
 * | 机体 | FLU / `base_link` | FRD | ROS: x=前 y=左 z=上；PX4: x=前 y=右 z=下 |
 *
 * ## 三条换算法则（**互相之间不一样，混淆就会前后左右全反**）
 *
 * 1. **世界系位置与速度**：`(x, y, z)_ENU -> (y, x, -z)_NED`
 *    —— 这个映射是**对合**（自己就是自己的逆），做两次回到原值。
 * 2. **机体矢量**：`(x, y, z)_FLU -> (x, -y, -z)_FRD`
 *    —— **不换位**，只把 y、z 取反。
 * 3. **姿态**：两次显式 Hamilton 四元数乘法
 *    `q_px4 = NED_ENU_Q ⊗ q_enu ⊗ AIRCRAFT_BASELINK_Q`
 *
 * ## 为什么姿态绝不用"闭式"
 *
 * 该换算可以化简成闭式，但历史教训是：手工推导闭式极易出错（本项目验证过程中
 * 连续推错三次、得到三个互相矛盾的表达式）。因此这里**一律用两次显式四元数
 * 乘法**，与 PX4 官方 `px4_ros_com` 的 `frame_transforms.cpp` 逐字一致。
 * 具体闭式只在单元测试里作为"交叉校验"出现。
 *
 * ## 偏航是反射，不是常数偏移
 *
 * ENU 的 z 朝上、NED 的 z 朝下，两边绕 z 的正方向相反，因此
 * `yaw_px4 = pi/2 - yaw_map`。**不存在**能对所有航向都成立的常数 offset。
 * 同理偏航角速度变号：`yaw_rate_px4 = -yaw_rate_map`。
 */
namespace px4_offboard
{

/// 圆周率常量。自己定义，避免依赖 `M_PI` 这类平台相关宏。
constexpr double kPi = 3.14159265358979323846;

/// 三维矢量。世界系与机体系共用，语义由调用方决定。
struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// 四元数，Hamilton 约定，分量顺序 (w, x, y, z)，表示 body -> world 的被动旋转。
struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// 欧拉角，弧度。按 ZYX 内旋顺序（先 yaw、再 pitch、最后 roll）。
struct EulerAngles
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

// ---------------------------------------------------------------------------
// 基础四元数运算
// ---------------------------------------------------------------------------

/// 把角度归一化到 (-pi, pi]。
double normalize_angle(double angle);

/// 由 ZYX 欧拉角构造四元数（与 px4_ros_com 的 quaternion_from_euler 一致）。
Quaternion quaternion_from_euler(double roll, double pitch, double yaw);

/// 四元数归一化。输入接近零四元数时返回单位四元数。
Quaternion quaternion_normalized(const Quaternion & q);

/// Hamilton 乘积 a ⊗ b，等价于"先施加 b 再施加 a"。
Quaternion quaternion_multiply(const Quaternion & a, const Quaternion & b);

/// 共轭（单位四元数下即逆）。
Quaternion quaternion_conjugate(const Quaternion & q);

/// 用四元数旋转矢量：v' = q ⊗ v ⊗ q*。
Vector3 quaternion_rotate(const Quaternion & q, const Vector3 & v);

/// 四元数转 ZYX 欧拉角。
EulerAngles quaternion_to_euler(const Quaternion & q);

/// 只取偏航角（等价于 quaternion_to_euler(q).yaw，但更省）。
double quaternion_yaw(const Quaternion & q);

// ---------------------------------------------------------------------------
// 固定常量
// ---------------------------------------------------------------------------

/// ENU -> NED 的姿态部分，等价于 `quaternion_from_euler(pi, 0, pi/2)`，即 (0, √2/2, √2/2, 0)。
const Quaternion & ned_enu_quaternion();

/// FLU -> FRD 的姿态部分，等价于 `quaternion_from_euler(pi, 0, 0)`，即 (0, 1, 0, 0)。
const Quaternion & aircraft_baselink_quaternion();

// ---------------------------------------------------------------------------
// ROS(ENU/FLU) -> PX4(NED/FRD)
// ---------------------------------------------------------------------------

/// 世界系位置：`(x, y, z)_ENU -> (y, x, -z)_NED`。自逆。
Vector3 map_position_to_px4(const Vector3 & p_enu);

/// 世界系速度：同 `map_position_to_px4`（位置与速度共用同一法则）。
Vector3 map_velocity_to_px4(const Vector3 & v_enu);

/// 姿态：`q_px4 = NED_ENU_Q ⊗ q_enu ⊗ AIRCRAFT_BASELINK_Q`。
Quaternion map_attitude_to_px4(const Quaternion & q_enu_flu);

/// 机体矢量：`(x, y, z)_FLU -> (x, -y, -z)_FRD`。自逆。
Vector3 body_vector_flu_to_frd(const Vector3 & v_flu);

/// 偏航角：`yaw_px4 = pi/2 - yaw_map`（结果归一化）。自逆。
double map_yaw_to_px4(double yaw_map);

/// 偏航角速度：纯变号（ENU 逆时针为正，NED 顺时针为正）。自逆。
double map_yaw_rate_to_px4(double yaw_rate_map);

// ---------------------------------------------------------------------------
// PX4(NED/FRD) -> ROS(ENU/FLU)  —— 回读飞控状态时用
// ---------------------------------------------------------------------------

/// 世界系位置回读。因为该映射是对合，实现与 `map_position_to_px4` 完全相同。
Vector3 px4_position_to_map(const Vector3 & p_ned);

/// 世界系速度回读。同上。
Vector3 px4_velocity_to_map(const Vector3 & v_ned);

/// 姿态回读：`q_map = NED_ENU_Q⁻¹ ⊗ q_px4 ⊗ AIRCRAFT_BASELINK_Q⁻¹`。
Quaternion px4_attitude_to_map(const Quaternion & q_ned_frd);

/// 机体矢量回读。自逆。
Vector3 frd_vector_to_flu(const Vector3 & v_frd);

/// 偏航角回读：`yaw_map = pi/2 - yaw_px4`（结果归一化）。自逆。
double px4_yaw_to_map(double yaw_px4);

/// 偏航角速度回读。自逆。
double px4_yaw_rate_to_map(double yaw_rate_px4);

}  // namespace px4_offboard

#endif  // PX4_OFFBOARD__FRAME_TRANSFORMS_HPP_
