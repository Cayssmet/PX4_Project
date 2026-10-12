"""数值验证：ROS/ENU/FLU  <->  PX4/NED-FRD 的坐标与姿态换算。

目的：
1. 确认 px4_ros_com 的 q_px4 = NED_ENU_Q * q_enu * AIRCRAFT_BASELINK_Q 与
   "旋转矩阵 C 作用在 ENU 矢量上 + 机体系 FLU->FRD 取负 y,z" 完全等价。
2. 确认位置换算 (x,y,z)_ENU -> (y,x,-z)。
3. 尝试验证闭式 ((w+z),(x+y),(x-y),(w-z))/sqrt2 与 ((w+z),(x+y),(z-w),(y-x))/sqrt2。
"""
import math
import numpy as np

SQ = math.sqrt(0.5)


# ---------- 四元数工具（Hamilton, (w,x,y,z)，body->world 被动旋转）----------
def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quat_rot(q, v):
    """用四元数旋转矢量：v' = q v q*"""
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    return R @ np.asarray(v, dtype=float)


def quat_to_euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


QC = euler_to_quat(math.pi, 0.0, math.pi / 2)   # NED_ENU_Q
QD = euler_to_quat(math.pi, 0.0, 0.0)           # AIRCRAFT_BASELINK_Q

print("QC =", tuple(round(v, 12) for v in QC), " 期望 (0, 0.7071, 0.7071, 0)")
print("QD =", tuple(round(v, 12) for v in QD), " 期望 (0, 1, 0, 0)")
print()

# C = NED_ENU_Q 的旋转矩阵（作用在 ENU 矢量上得到 NED 矢量）
C = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
C_from_q = np.array([
    [1 - 2 * (QC[2] ** 2 + QC[3] ** 2), 2 * (QC[1] * QC[2] - QC[0] * QC[3]), 2 * (QC[1] * QC[3] + QC[0] * QC[2])],
    [2 * (QC[1] * QC[2] + QC[0] * QC[3]), 1 - 2 * (QC[1] ** 2 + QC[3] ** 2), 2 * (QC[2] * QC[3] - QC[0] * QC[1])],
    [2 * (QC[1] * QC[3] - QC[0] * QC[2]), 2 * (QC[2] * QC[3] + QC[0] * QC[1]), 1 - 2 * (QC[1] ** 2 + QC[2] ** 2)],
])
print("QC 旋转矩阵 =")
print(np.round(C_from_q, 12))
print("与期望 C 一致:", np.allclose(C_from_q, C))
print()

fails = []
rng = np.random.default_rng(20260805)

for trial in range(200000):
    # 随机的 SLAM 位姿：ENU 位置 + FLU 姿态
    p_enu = rng.uniform(-5, 5, 3)
    q_enu = euler_to_quat(*rng.uniform(-math.pi, math.pi, 3))
    # 归一化
    n = math.sqrt(sum(c * c for c in q_enu))
    q_enu = tuple(c / n for c in q_enu)
    if q_enu[0] < 0:                      # 规范到 w>=0 半空间
        q_enu = tuple(-c for c in q_enu)

    # --- 路径 1：px4_ros_com 的做法 ---
    q_px4 = quat_mul(quat_mul(QC, q_enu), QD)
    p_px4 = np.array([p_enu[1], p_enu[0], -p_enu[2]])

    # --- 路径 2：直接从物理定义构造 R_NED_FRD = C @ R_enu_flu @ D ---
    R_enu_flu = np.array([
        [1 - 2 * (q_enu[2] ** 2 + q_enu[3] ** 2), 2 * (q_enu[1] * q_enu[2] - q_enu[0] * q_enu[3]), 2 * (q_enu[1] * q_enu[3] + q_enu[0] * q_enu[2])],
        [2 * (q_enu[1] * q_enu[2] + q_enu[0] * q_enu[3]), 1 - 2 * (q_enu[1] ** 2 + q_enu[3] ** 2), 2 * (q_enu[2] * q_enu[3] - q_enu[0] * q_enu[1])],
        [2 * (q_enu[1] * q_enu[3] - q_enu[0] * q_enu[2]), 2 * (q_enu[2] * q_enu[3] + q_enu[0] * q_enu[1]), 1 - 2 * (q_enu[1] ** 2 + q_enu[2] ** 2)],
    ])
    D = np.diag([1.0, -1.0, -1.0])
    R_ref = C @ R_enu_flu @ D

    R_px4 = np.array([
        [1 - 2 * (q_px4[2] ** 2 + q_px4[3] ** 2), 2 * (q_px4[1] * q_px4[2] - q_px4[0] * q_px4[3]), 2 * (q_px4[1] * q_px4[3] + q_px4[0] * q_px4[2])],
        [2 * (q_px4[1] * q_px4[2] + q_px4[0] * q_px4[3]), 1 - 2 * (q_px4[1] ** 2 + q_px4[3] ** 2), 2 * (q_px4[2] * q_px4[3] - q_px4[0] * q_px4[1])],
        [2 * (q_px4[1] * q_px4[3] - q_px4[0] * q_px4[2]), 2 * (q_px4[2] * q_px4[3] + q_px4[0] * q_px4[1]), 1 - 2 * (q_px4[1] ** 2 + q_px4[2] ** 2)],
    ])
    if not np.allclose(R_px4, R_ref, atol=1e-9):
        fails.append(("R", trial))
        break

    # --- 位置一致性：C @ p_enu ---
    if not np.allclose(C @ p_enu, p_px4, atol=1e-12):
        fails.append(("p", trial))
        break

print("姿态+位置一致性（路径1 vs 物理定义，20 万随机位姿）:", "全部通过" if not fails else f"失败 {fails[0]}")
print()

# ---------- 闭式候选验证 ----------
print("闭式候选验证（取随机位姿比较）:")
cands = {
    "((w+z),(x+y),(x-y),(w-z))": lambda w, x, y, z: ((w + z), (x + y), (x - y), (w - z)),
    "((w+z),(x+y),(z-w),(y-x))": lambda w, x, y, z: ((w + z), (x + y), (z - w), (y - x)),
    "(-(x+y),(x+y),(x-y),(w-z))": lambda w, x, y, z: (-(x + y), (x + y), (x - y), (w - z)),
}
for name, f in cands.items():
    ok = True
    for _ in range(20000):
        q = euler_to_quat(*rng.uniform(-math.pi, math.pi, 3))
        n = math.sqrt(sum(c * c for c in q))
        q = tuple(c / n for c in q)
        ref = quat_mul(quat_mul(QC, q), QD)
        got = tuple(v * SQ for v in f(*q))
        # 允许整体符号相反
        if not (np.allclose(got, ref, atol=1e-9) or np.allclose(got, [-v for v in ref], atol=1e-9)):
            ok = False
            break
    print(f"  {name:32s} -> {'正确' if ok else '错误'}")
print()

# ---------- 用"物理语义"做端到端校验（这才是真正要保的） ----------
print("物理语义校验（无人机朝地图 +X 平飞，机头抬 10 度）:")
p_enu = np.array([1.0, 2.0, 0.5])           # ENU: 东1 北2 上0.5
q_flu = euler_to_quat(0.0, math.radians(10), 0.0)   # FLU: 抬头10度
q_px4 = quat_mul(quat_mul(QC, q_flu), QD)
r, p, y = quat_to_euler(q_px4)
print(f"  ENU 位置 {p_enu} -> PX4 位置 {(p_enu[1], p_enu[0], -p_enu[2])}")
print(f"  FLU 姿态 (roll=0, pitch=+10deg, yaw=0) -> PX4 FRD 姿态 "
      f"(roll={math.degrees(r):+.2f}, pitch={math.degrees(p):+.2f}, yaw={math.degrees(y):+.2f})")
print("  期望：roll 0, pitch +10（FRD 下抬头仍是 +pitch）, yaw 0")
print()

print("机体系矢量换算校验：FLU '向右'=(0,+1,0) 在 FRD 中应为 (0,-1,0) 即右为 +y:")
print("  D @ (0,1,0) =", np.round(D @ np.array([0.0, 1.0, 0.0]), 12))
