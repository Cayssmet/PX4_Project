# -*- coding: utf-8 -*-
"""把 MAV_PROTO_VER 写回 2（先试普通浮点数，再试 denormal 位模式），成功后重启飞控。

!!! 实测无效，保留仅为留证 !!!
  2026-11-08 实测：普通浮点 2.0 与 denormal 位模式 (struct.unpack('<f', struct.pack('<i', 2))
  = 2.802596928649634e-45) 各发 5 拍 PARAM_SET，读回始终是 raw 1.4013e-45 (= 1)，
  随后 mavutil.mavserial 报 "Device COM9 is dead"。
  => PX4 1.17.0 不接受从 MAVLink 侧写入这个参数。
  正确做法：在 QGC 参数页里改，然后 Tools -> Reboot Vehicle。
  详见 docs/QGC中文使用手册.md 第 9 节。
"""
import sys, time, struct
from pymavlink import mavutil

PORT = "COM9"; BAUD = 115200
NAME = "MAV_PROTO_VER"
WANT = 2

def pname(raw):
    if isinstance(raw, bytes):
        return raw.split(b'\x00', 1)[0].decode('ascii', 'replace')
    return str(raw).split('\x00', 1)[0]

def denorm(v):
    if v != v or v == 0:
        return v
    if abs(v) < 1e-30:
        return struct.unpack('<i', struct.pack('<f', v))[0]
    return int(round(v))

def read_param(m, hb):
    m.mav.param_request_read_send(1, 1, NAME.encode(), -1)
    t_end = time.time() + 5
    while time.time() < t_end:
        hb()
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg and pname(msg.param_id) == NAME:
            return msg.param_value
    return None

def main():
    m = mavutil.mavserial(PORT, baud=BAUD, autoreconnect=False)
    m.wait_heartbeat(timeout=15)
    m.target_system = 1
    m.target_component = 1

    def hb():
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    raw = read_param(m, hb)
    print(f"起始 {NAME} = raw {raw} -> {denorm(raw) if raw is not None else '?'}")

    attempts = [
        ("普通浮点 2.0", 2.0),
        ("denormal 位模式(2)", struct.unpack('<f', struct.pack('<i', WANT))[0]),
    ]
    ok = False
    for label, payload in attempts:
        print(f"--- 尝试写入：{label}  payload={payload!r}")
        for _ in range(5):
            hb()
            m.mav.param_set_send(1, 1, NAME.encode(), float(payload),
                                 mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.2)
        time.sleep(0.6)
        raw2 = read_param(m, hb)
        got = denorm(raw2) if raw2 is not None else None
        print(f"    读回 raw {raw2} -> {got}")
        if got == WANT:
            ok = True
            break

    if not ok:
        print("❌ 两种编码都没写进去，放弃重启（避免白重启）")
        m.close()
        return

    print("✅ 写入成功，发送重启命令")
    for _ in range(5):
        hb()
        m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(0.2)
    try:
        m.close()
    except Exception:
        pass

if __name__ == '__main__':
    main()
