# -*- coding: utf-8 -*-
"""坐标与姿态换算自检（ENU/FLU ↔ NED/FRD）。

这里的断言不是「跑通就行」，而是**物理语义**：
* 地图 (1, 2, 0.5)（东 1、北 2、上 0.5）→ PX4 必须是 (2, 1, -0.5)
* 机体抬头 10°（FLU pitch=+10°）→ PX4 FRD 必须是 pitch=-10°、yaw=+90°
* 机体系矢量与姿态共用同一套法则（`BodyVectorWorldConsistency` 这条不变式）

单独运行：
    python3 test/test_frames.py
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from px4_offboard import frames as fr  # noqa: E402

TOL = 1e-9


def close(a, b, tol=TOL):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def angle_close(a, b, tol=1e-9):
    return abs(fr.normalize_angle(a - b)) <= tol


def test_constants():
    assert close(fr.ned_enu_quaternion(), fr.quaternion_from_euler(math.pi, 0.0, math.pi / 2))
    assert close(fr.aircraft_baselink_quaternion(), fr.quaternion_from_euler(math.pi, 0.0, 0.0))
    assert close(fr.ned_enu_quaternion(), (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0))
    assert close(fr.aircraft_baselink_quaternion(), (0.0, 1.0, 0.0, 0.0))


def test_ned_enu_matrix_is_expected_reflection():
    R = fr.rotation_matrix(fr.ned_enu_quaternion())
    expect = ((0, 1, 0), (1, 0, 0), (0, 0, -1))
    for i in range(3):
        assert close(R[i], expect[i]), f"row {i}: {R[i]} != {expect[i]}"


def test_aircraft_to_baselink_flips_right_and_down():
    R = fr.rotation_matrix(fr.aircraft_baselink_quaternion())
    # 绕 x 转 180°：x 不变，y/z 取反
    assert close(R[0], (1, 0, 0))
    assert close(R[1], (0, -1, 0))
    assert close(R[2], (0, 0, -1))


def test_world_position_swaps_xy_and_flips_z():
    assert close(fr.map_position_to_px4((1.0, 2.0, 0.5)), (2.0, 1.0, -0.5))
    assert close(fr.map_position_to_px4((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
    assert close(fr.map_position_to_px4((-3.0, 7.0, -1.0)), (7.0, -3.0, 1.0))


def test_world_position_is_its_own_inverse():
    for p in [(1.0, 2.0, 3.0), (-4.5, 0.25, -9.0)]:
        assert close(fr.px4_position_to_map(fr.map_position_to_px4(p)), p)


def test_world_velocity_same_rule_as_position():
    assert close(fr.map_velocity_to_px4((0.5, -0.2, 0.1)), (-0.2, 0.5, -0.1))


def test_yaw_is_reflection_not_constant_offset():
    """ENU z 朝上、NED z 朝下 ⇒ 偏航是反射，不存在能对全部航向成立的常数偏移。"""
    assert angle_close(fr.map_yaw_to_px4(0.0), math.pi / 2)
    assert angle_close(fr.map_yaw_to_px4(math.pi / 2), 0.0)
    assert angle_close(fr.map_yaw_to_px4(-math.pi / 2), math.pi)
    assert angle_close(fr.map_yaw_to_px4(math.pi), -math.pi / 2)
    # 自逆
    for y in [0.0, 0.3, -1.2, 2.9]:
        assert angle_close(fr.px4_yaw_to_map(fr.map_yaw_to_px4(y)), y)


def test_yaw_rate_sign_flips():
    assert fr.map_yaw_rate_to_px4(1.0) == -1.0
    assert fr.px4_yaw_rate_to_map(-2.5) == 2.5


def test_pure_yaw_attitude_matches_yaw_formula():
    for yaw in [0.0, 0.5, -1.0, 2.5, 3.0]:
        q = fr.quaternion_from_euler(0.0, 0.0, yaw)
        qn = fr.map_attitude_to_px4(q)
        roll, pitch, got_yaw = fr.quaternion_to_euler(qn)
        assert abs(roll) < 1e-9 and abs(pitch) < 1e-9, f"纯偏航不该出横滚俯仰: {roll},{pitch}"
        assert angle_close(got_yaw, fr.map_yaw_to_px4(yaw)), f"yaw={yaw}"


def test_attitude_physical_semantics():
    """机体抬头 10°（FLU pitch=+10°）→ FRD 里 pitch=-10°、yaw=+90°。

    FRD 的 z 朝下，所以「抬头」是负俯仰；地图 +X 是东，NED 里东就是 +90°。
    """
    q = fr.quaternion_from_euler(0.0, math.radians(10.0), 0.0)
    roll, pitch, yaw = fr.quaternion_to_euler(fr.map_attitude_to_px4(q))
    assert abs(math.degrees(roll) - 0.0) < 1e-6
    assert abs(math.degrees(pitch) + 10.0) < 1e-6, f"pitch={math.degrees(pitch)}"
    assert abs(math.degrees(yaw) - 90.0) < 1e-6, f"yaw={math.degrees(yaw)}"


def test_attitude_round_trip():
    rng = random.Random(20261124)
    for _ in range(2000):
        q = fr.quaternion_normalized(
            fr.quaternion_from_euler(*[rng.uniform(-math.pi, math.pi) for _ in range(3)]))
        back = fr.px4_attitude_to_map(fr.map_attitude_to_px4(q))
        # q 与 -q 表示同一姿态
        same = close(back, q, 1e-9)
        neg = close(back, tuple(-c for c in q), 1e-9)
        assert same or neg, f"{q} -> {back}"


def test_body_vector_keeps_x_and_flips_yz():
    assert close(fr.body_vector_flu_to_frd((1.0, 2.0, 3.0)), (1.0, -2.0, -3.0))
    assert close(fr.frd_vector_to_flu((1.0, -2.0, -3.0)), (1.0, 2.0, 3.0))


def test_body_vector_world_consistency():
    """★ 最重要的一条不变式。

    把机体系速度先转 FRD 再按姿态转到世界，必须等于
    「先按地图姿态转到世界再按世界法则转换」。两条法则若不是同源，这条必挂。
    """
    rng = random.Random(424242)
    worst = 0.0
    for _ in range(20000):
        q_flu = fr.quaternion_normalized(
            fr.quaternion_from_euler(*[rng.uniform(-math.pi, math.pi) for _ in range(3)]))
        v_flu = tuple(rng.uniform(-2, 2) for _ in range(3))

        q_frd = fr.map_attitude_to_px4(q_flu)
        v_world_from_frd = fr.quaternion_rotate(q_frd, fr.body_vector_flu_to_frd(v_flu))

        v_world_flu = fr.quaternion_rotate(q_flu, v_flu)
        v_world_expected = fr.map_velocity_to_px4(v_world_flu)

        d = max(abs(a - b) for a, b in zip(v_world_from_frd, v_world_expected))
        worst = max(worst, d)
    assert worst < 1e-9, f"最大偏差 {worst}"


def test_closed_form_cross_check():
    """闭式 ((w+z),(x+y),(x-y),(w-z))/√2 与两次显式乘法一致（允许整体符号相反）。

    闭式**只在这里出现**，生产代码一律用乘法。
    """
    rng = random.Random(999)
    for _ in range(20000):
        q = fr.quaternion_normalized(
            fr.quaternion_from_euler(*[rng.uniform(-math.pi, math.pi) for _ in range(3)]))
        ref = fr.map_attitude_to_px4(q)
        w, x, y, z = q
        got = tuple(c * math.sqrt(0.5) for c in
                    ((w + z), (x + y), (x - y), (w - z)))
        assert close(got, ref, 1e-9) or close(got, tuple(-c for c in ref), 1e-9)


def test_angle_normalization_range():
    for a in [0.0, 3.141592653589793, -3.141592653589793, 7.0, -7.0, 100.0]:
        n = fr.normalize_angle(a)
        assert -math.pi - 1e-12 < n <= math.pi + 1e-12, f"{a} -> {n}"


def test_degenerate_quaternion_is_handled():
    """NaN / 零四元数绝不能传出去 —— 飞控会拒绝整条视觉位姿。"""
    for bad in [(0.0, 0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0, 0.0),
                (float("inf"), 0.0, 0.0, 0.0)]:
        q = fr.quaternion_normalized(bad)
        assert q == (1.0, 0.0, 0.0, 0.0), q
        out = fr.map_attitude_to_px4(bad)
        assert all(math.isfinite(c) for c in out), out


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    fns = [(k, v) for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    fails = 0
    for name, fn in fns:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as exc:  # noqa: BLE001
            print(f"  FAIL  {name}: {exc}")
            fails += 1
    print(f"\n{len(fns) - fails}/{len(fns)} 通过")
    sys.exit(1 if fails else 0)
