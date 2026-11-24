# -*- coding: utf-8 -*-
"""frames.py —— ROS 2（ENU/FLU）与 PX4（NED/FRD）之间的坐标与姿态换算。

纯标准库，不依赖 numpy / ROS，可以脱离一切环境单独测试。

四条法则（背下来，别现推）
--------------------------
1. **世界系位置/速度**：`(x, y, z)_ENU -> (y, x, -z)_NED`  ← 交换 x/y，z 取反
2. **世界系姿态**：`q_ned_frd = NED_ENU_Q ⊗ q_enu_flu ⊗ AIRCRAFT_BASELINK_Q`
   其中 `NED_ENU_Q = (0, √2/2, √2/2, 0)`、`AIRCRAFT_BASELINK_Q = (0, 1, 0, 0)`
3. **机体系矢量**：`(x, y, z)_FLU -> (x, -y, -z)_FRD`  ← **不换位**，只 y/z 取反
4. **偏航**：`yaw_ned = π/2 - yaw_enu`，`yaw_rate_ned = -yaw_rate_enu`

⚠ 第 1 条和第 3 条**法则不一样**。把世界系的「换 x/y」套到机体速度上，
前后左右会整体转 90°，这是这类项目最常见的一次性炸机原因。

⚠ 第 4 条是**反射**不是常量偏移：ENU 的 z 朝上、NED 的 z 朝下，两边正方向相反，
所以不存在能对全部航向都成立的常数 offset。

为什么代码里用两次四元数乘法而不是闭式
--------------------------------------
闭式 `((w+z),(x+y),(x-y),(w-z))/√2` 是对的，但本项目验证过程中手工推闭式
**连续推错三次、得到三个互相矛盾的结果**。两次显式 Hamilton 积与
`px4_ros_com` 的 `frame_transforms` 逐字一致，且已用 20 万组随机位姿
数值证明等价 —— 所以代码里一律走乘法，闭式只在测试里用作交叉校验。
"""

from __future__ import annotations

import math

SQRT_HALF = math.sqrt(0.5)
PI = math.pi


# ---------------------------------------------------------------------------
# 角度
# ---------------------------------------------------------------------------


def normalize_angle(a: float) -> float:
    """归一到 (-π, π]。"""
    return math.remainder(a, 2.0 * PI)


# ---------------------------------------------------------------------------
# 四元数（Hamilton，(w, x, y, z)，body->world）
# ---------------------------------------------------------------------------


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """ZYX 内旋欧拉角 -> 四元数。与 px4_ros_com 的四式逐字一致。"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def quaternion_normalized(q):
    """归一化。范数非正或非有限时退化为单位四元数，**绝不产生 NaN**。

    这一点很重要：NaN 四元数一旦喂给飞控，飞控会直接拒绝整条视觉位姿。
    """
    n = math.sqrt(sum(c * c for c in q))
    if not math.isfinite(n) or n <= 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(c / n for c in q)


def quaternion_multiply(a, b):
    """Hamilton 积 a⊗b（先施加 b 的旋转，再施加 a）。"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quaternion_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quaternion_rotate(q, v):
    """用四元数旋转矢量：v' = q ⊗ v ⊗ q*。"""
    w, x, y, z = q
    vx, vy, vz = v
    # u = q 的矢量部分
    uv = (y * vz - z * vy, z * vx - x * vz, x * vy - y * vx)      # u × v
    uuv = (y * uv[2] - z * uv[1], z * uv[0] - x * uv[2], x * uv[1] - y * uv[0])
    return (
        vx + 2.0 * (w * uv[0] + uuv[0]),
        vy + 2.0 * (w * uv[1] + uuv[1]),
        vz + 2.0 * (w * uv[2] + uuv[2]),
    )


def quaternion_to_euler(q):
    """四元数 -> ZYX 内旋欧拉角 (roll, pitch, yaw)。"""
    w, x, y, z = quaternion_normalized(q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def quaternion_yaw(q) -> float:
    return quaternion_to_euler(q)[2]


def rotation_matrix(q):
    """四元数 -> 3x3 旋转矩阵（嵌套 tuple，行优先）。测试与调试用。"""
    w, x, y, z = quaternion_normalized(q)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


# ---------------------------------------------------------------------------
# 两个固定旋转
# ---------------------------------------------------------------------------


def ned_enu_quaternion():
    """把 ENU 矢量转到 NED 的固定旋转，等价于旋转矩阵 [[0,1,0],[1,0,0],[0,0,-1]]。"""
    return (0.0, SQRT_HALF, SQRT_HALF, 0.0)


def aircraft_baselink_quaternion():
    """FLU -> FRD 的固定旋转：绕 x 轴转 180°。"""
    return (0.0, 1.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# 正向：ROS 2 -> PX4
# ---------------------------------------------------------------------------


def map_position_to_px4(p):
    """ENU 位置 -> NED 位置：(x, y, z) -> (y, x, -z)。"""
    return (p[1], p[0], -p[2])


def map_velocity_to_px4(v):
    """ENU 世界系速度 -> NED 世界系速度：同位置法则。"""
    return (v[1], v[0], -v[2])


def map_attitude_to_px4(q_flu):
    """ENU/FLU 姿态 -> NED/FRD 姿态。两次显式 Hamilton 积，刻意不用闭式。"""
    return quaternion_normalized(
        quaternion_multiply(
            quaternion_multiply(ned_enu_quaternion(), quaternion_normalized(q_flu)),
            aircraft_baselink_quaternion(),
        )
    )


def body_vector_flu_to_frd(v):
    """机体系矢量 FLU -> FRD：(x, y, z) -> (x, -y, -z)。**不换位**。"""
    return (v[0], -v[1], -v[2])


def map_yaw_to_px4(yaw_enu: float) -> float:
    """偏航：yaw_ned = π/2 - yaw_enu（反射）。"""
    return normalize_angle(PI * 0.5 - yaw_enu)


def map_yaw_rate_to_px4(yaw_rate_enu: float) -> float:
    """偏航角速度取反。"""
    return -yaw_rate_enu


# ---------------------------------------------------------------------------
# 反向：PX4 -> ROS 2
# ---------------------------------------------------------------------------


def px4_position_to_map(p):
    """NED -> ENU：(x, y, z) -> (y, x, -z)（此变换自逆）。"""
    return (p[1], p[0], -p[2])


def px4_velocity_to_map(v):
    return (v[1], v[0], -v[2])


def px4_attitude_to_map(q_frd):
    """NED/FRD 姿态 -> ENU/FLU 姿态。两个固定旋转的逆各自取共轭。"""
    return quaternion_normalized(
        quaternion_multiply(
            quaternion_multiply(quaternion_conjugate(ned_enu_quaternion()),
                                quaternion_normalized(q_frd)),
            quaternion_conjugate(aircraft_baselink_quaternion()),
        )
    )


def frd_vector_to_flu(v):
    return (v[0], -v[1], -v[2])


def px4_yaw_to_map(yaw_ned: float) -> float:
    return normalize_angle(PI * 0.5 - yaw_ned)


def px4_yaw_rate_to_map(yaw_rate_ned: float) -> float:
    return -yaw_rate_ned
