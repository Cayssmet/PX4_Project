"""用 MAVLink 直连飞控，检查「光流 + 测距」传感器是否真的被飞控收到。

判定逻辑：
  * SYS_STATUS 的 onboard_control_sensors_present/enabled 里有没有
    OPTICAL_FLOW(bit24) / LASER_POSITION(bit20)  —— 这是飞控"认到传感器"的直接证据
  * 有没有在输出 DISTANCE_SENSOR / OPTICAL_FLOW_RAD / OPTICAL_FLOW —— 这是"数据在流"的证据

注意：PX4 对 INT32 参数会以 float 的位模式发出来（denormal），这里会自动还原成整数。

用法（必须先让桥 / QGC 释放 COM9）：
    python mavlink_probe.py [COM9] [115200]
"""

import re
import struct
import sys
import time

from pymavlink import mavutil

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM9"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200


def decode_value(v):
    """PX4 会把 INT32 参数按 float 位模式发出；denormal 还原成整数。"""
    if v != v:
        return "NaN"
    if v != 0.0 and abs(v) < 1e-30:
        return struct.unpack("<i", struct.pack("<f", v))[0]
    if abs(v - round(v)) < 1e-6:
        return int(round(v))
    return round(v, 6)


SENSOR_BITS = [
    (1 << 0, "GYRO"), (1 << 1, "ACCEL"), (1 << 2, "MAG"), (1 << 3, "ABSOLUTE_PRESSURE"),
    (1 << 4, "DIFFERENTIAL_PRESSURE"), (1 << 5, "GPS"), (1 << 6, "OPTICAL_FLOW_LEGACY"),
    (1 << 7, "VISION_POSITION"), (1 << 8, "LASER_POSITION_LEGACY"), (1 << 9, "EXTERNAL_GROUND_TRUTH"),
    (1 << 10, "ANGULAR_RATE_CONTROL"), (1 << 11, "ATTITUDE_STABILIZATION"), (1 << 12, "YAW_POSITION"),
    (1 << 13, "Z_ALTITUDE_CONTROL"), (1 << 14, "XY_POSITION_CONTROL"), (1 << 15, "MOTOR_OUTPUTS"),
    (1 << 16, "RC_RECEIVER"), (1 << 17, "GYRO2"), (1 << 18, "ACCEL2"), (1 << 19, "MAG2"),
    (1 << 20, "LASER_POSITION"), (1 << 21, "GYRO3"), (1 << 22, "ACCEL3"),
    (1 << 24, "OPTICAL_FLOW"), (1 << 25, "VISION_POSITION2"), (1 << 26, "PROXIMITY"),
    (1 << 27, "SATCOM"), (1 << 28, "BATTERY2"),
]


def bits(mask):
    return [n for b, n in SENSOR_BITS if mask & b] or ["(none)"]


print(f"[probe] 打开 {PORT} @ {BAUD} …", flush=True)
master = mavutil.mavlink_connection(PORT, baud=BAUD, autoreconnect=False)

print("[probe] 等心跳 …", flush=True)
hb = master.wait_heartbeat(timeout=15)
if hb is None:
    print("[probe] 没收到心跳", flush=True)
    sys.exit(1)
print(f"[probe] 心跳 OK system={master.target_system} type={hb.type} autopilot={hb.autopilot}", flush=True)

# ---------- SYS_STATUS：飞控认到哪些传感器 ----------
print("\n[probe] 读 SYS_STATUS …", flush=True)
sys_status = None
t_end = time.time() + 6
while time.time() < t_end:
    m = master.recv_match(type="SYS_STATUS", blocking=True, timeout=2)
    if m is not None:
        sys_status = m
        break
if sys_status is not None:
    print(f"    present : {', '.join(bits(sys_status.onboard_control_sensors_present))}", flush=True)
    print(f"    enabled : {', '.join(bits(sys_status.onboard_control_sensors_enabled))}", flush=True)
    print(f"    healthy : {', '.join(bits(sys_status.onboard_control_sensors_health))}", flush=True)
    for label, bitname, mask in (
        ("光流 OPTICAL_FLOW", "OPTICAL_FLOW", 1 << 24),
        ("测距 LASER_POSITION", "LASER_POSITION", 1 << 20),
        ("光流(旧位) OPTICAL_FLOW_LEGACY", "OPTICAL_FLOW_LEGACY", 1 << 6),
        ("测距(旧位) LASER_POSITION_LEGACY", "LASER_POSITION_LEGACY", 1 << 8),
    ):
        p = bool(sys_status.onboard_control_sensors_present & mask)
        e = bool(sys_status.onboard_control_sensors_enabled & mask)
        h = bool(sys_status.onboard_control_sensors_health & mask)
        print(f"    {label:<34} present={p} enabled={e} healthy={h}", flush=True)
else:
    print("    没收到 SYS_STATUS", flush=True)

# ---------- 参数 ----------
print("\n[probe] 请求全部参数 …", flush=True)
master.param_fetch_all()
params = {}
expect = None
deadline = time.time() + 90
while time.time() < deadline:
    msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
    if msg is None:
        continue
    params[msg.param_id.strip("\x00")] = msg.param_value
    if expect is None:
        expect = msg.param_count
        print(f"[probe] 飞控报告参数总数 {expect}", flush=True)
    if expect is not None and len(params) >= expect:
        break
print(f"[probe] 收到 {len(params)} 个参数", flush=True)

def dump(title, regex):
    rx = re.compile(regex)
    names = sorted(n for n in params if rx.search(n))
    print(f"\n[probe] {title}（{len(names)} 个）", flush=True)
    for n in names:
        print(f"    {n:<28} = {decode_value(params[n])}", flush=True)

dump("所有 SER_* 串口参数", r"^SER_")
dump("所有 MAV_*_CONFIG / FLOW_CTRL", r"^MAV_")
dump("UXRCE / 光流 / 测距 / 高度源相关", r"^(UXRCE_|EKF2_(OF|RNG|HGT_REF|EV_CTRL)|SENS_EN|SENS_TFLOW|MPC_ALT_MODE|MPC_POS_MODE|SYS_HITL|CBRK_)")

# ---------- 消息流 ----------
WANT = {
    mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR: "DISTANCE_SENSOR",
    mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: "OPTICAL_FLOW_RAD",
    mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW: "OPTICAL_FLOW",
}
print("\n[probe] 请求 10 Hz 推送 DISTANCE_SENSOR / OPTICAL_FLOW_RAD …", flush=True)
for msgid in WANT:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msgid, int(1e6 / 10), 0, 0, 0, 0, 0,
    )
    time.sleep(0.15)
# 同时打开全部数据流（有些消息只受 DATA_STREAM 控制）
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1,
)

window = 10.0
print(f"[probe] 监听 {window:.0f} 秒 …", flush=True)
counts = {v: 0 for v in WANT.values()}
last = {}
t_end = time.time() + window
while time.time() < t_end:
    msg = master.recv_match(type=list(WANT.values()), blocking=True, timeout=1)
    if msg is None:
        continue
    k = msg.get_type()
    counts[k] = counts.get(k, 0) + 1
    last[k] = msg

print("\n[probe] ===== 消息流结果 =====", flush=True)
for k, c in counts.items():
    print(f"  {k:<20} {c:>4} 条 ({c/window:.1f} Hz)", flush=True)
if "DISTANCE_SENSOR" in last:
    m = last["DISTANCE_SENSOR"]
    print(f"  -> 距离 {m.current_distance/100:.3f} m  type={m.type} id={m.id} orient={m.orientation}", flush=True)
if "OPTICAL_FLOW_RAD" in last:
    m = last["OPTICAL_FLOW_RAD"]
    print(f"  -> 光流 quality={m.quality} flow_x={m.integrated_x:.5f} flow_y={m.integrated_y:.5f} rad  dist={m.distance:.3f} m", flush=True)
if "OPTICAL_FLOW" in last:
    m = last["OPTICAL_FLOW"]
    print(f"  -> 光流 quality={m.quality} flow_x={m.flow_x} flow_y={m.flow_y} dist={m.distance:.3f} m", flush=True)

print("\n[probe] ===== 判定 =====", flush=True)
flow_present = bool(sys_status and (sys_status.onboard_control_sensors_present & ((1 << 24) | (1 << 6))))
rng_present = bool(sys_status and (sys_status.onboard_control_sensors_present & ((1 << 20) | (1 << 8))))
has_flow = counts.get("OPTICAL_FLOW_RAD", 0) + counts.get("OPTICAL_FLOW", 0) > 0
has_rng = counts.get("DISTANCE_SENSOR", 0) > 0
print(f"  飞控认到光流={flow_present}  认到测距={rng_present}  光流有数据={has_flow}  测距有数据={has_rng}", flush=True)
master.close()
