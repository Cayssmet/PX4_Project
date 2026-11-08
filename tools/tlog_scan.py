#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QGC 遥测日志（.tlog）离线扫描器 —— 用来在不占用串口的前提下查飞控状态

为什么需要它
------------
QGC 一启动就会独占 COM9，这时任何外部探针都打不开端口（Access denied）。
但 QGC 会把收到的全部 MAVLink 消息原样写进 .tlog。
所以扫一遍 tlog，就能在不打扰 QGC 的情况下知道：
  * 飞控到底有没有在发 DISTANCE_SENSOR / OPTICAL_FLOW_RAD
  * 飞控自己的 SYS_STATUS 里，光流(bit24) / 激光测距(bit20) 的 present / enabled / health 位
  * 光流与测距相关参数的当前值

用法
----
    set PYTHONPATH=D:\\09_PX4\\PX4_Project\\tools\\.pylibs
    python -u tlog_scan.py                      # 自动找最新的 tlog
    python -u tlog_scan.py <路径.tlog>

注意
----
* PX4 把 INT32 参数按 float 位模式发送（denormal），脚本会还原。
* tlog 里的时间戳是写入时刻，不代表消息的原始时间。
"""

import glob
import os
import struct
import sys
from collections import Counter

from pymavlink import mavutil

TELEMETRY_DIR = os.path.join(os.path.expanduser("~"), "Documents",
                             "QGroundControl", "Telemetry")

WATCH_PARAMS = [
    "MAV_PROTO_VER", "MAV_2_CONFIG", "MAV_2_MODE", "MAV_2_RATE",
    "SER_TEL4_BAUD", "SER_TEL1_BAUD",
    "EKF2_OF_CTRL", "EKF2_RNG_CTRL", "EKF2_HGT_REF",
    "SENS_FLOW_ROT", "SENS_TFLOW_CFG",
    "EKF2_OF_DELAY", "EKF2_OF_QMIN", "EKF2_RNG_CTRL", "EKF2_RNG_A_VMAX",
    "EKF2_RNG_A_HMAX", "EKF2_RNG_QUAL",
]

BIT_LASER = 1 << 20      # LASER_POSITION
BIT_OPTFLOW = 1 << 24    # OPTICAL_FLOW


def decode_param_value(enc_type, raw):
    """PX4 把 INT32 参数按 float 位模式发送（denormal），必须还原。"""
    try:
        if enc_type in (mavutil.mavlink.MAV_PARAM_TYPE_INT32,
                        mavutil.mavlink.MAV_PARAM_TYPE_UINT32):
            if raw != 0 and abs(raw) < 1e-30:
                return struct.unpack("<i", struct.pack("<f", raw))[0]
    except Exception:
        pass
    return raw


def newest_tlog():
    if not os.path.isdir(TELEMETRY_DIR):
        return None
    files = glob.glob(os.path.join(TELEMETRY_DIR, "*.tlog"))
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else newest_tlog()
    if not path or not os.path.isfile(path):
        print("[!!] 找不到 tlog")
        return 2

    size = os.path.getsize(path)
    print("=" * 72)
    print("扫描：%s" % path)
    print("大小：%d 字节 (%.1f MB)" % (size, size / 1048576.0))
    print("=" * 72)

    conn = mavutil.mavlink_connection(path)
    hist = Counter()
    laser_samples = []
    flow_samples = []
    last_sys = None
    params = {}

    n = 0
    while True:
        msg = conn.recv_match(blocking=False)
        if msg is None:
            break
        n += 1
        t = msg.get_type()
        hist[t] += 1

        if t == "BAD_DATA":
            continue
        if t == "DISTANCE_SENSOR":
            if len(laser_samples) < 3:
                laser_samples.append((msg.time_boot_ms, msg.min_distance,
                                      msg.max_distance, msg.current_distance,
                                      msg.orientation))
        elif t == "OPTICAL_FLOW_RAD":
            if len(flow_samples) < 3:
                flow_samples.append((msg.integration_time_us, msg.integrated_x,
                                     msg.integrated_y, msg.quality,
                                     msg.distance, msg.temperature))
        elif t == "SYS_STATUS":
            last_sys = msg
        elif t == "PARAM_VALUE":
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("utf-8", "ignore")
            params[pid.rstrip(chr(0))] = decode_param_value(
                msg.param_type, msg.param_value)

    print()
    print("总消息数：%d，消息种类：%d" % (n, len(hist)))
    print()
    print("--- 关注的消息条数 ---")
    for name in ("DISTANCE_SENSOR", "OPTICAL_FLOW_RAD", "OPTICAL_FLOW",
                 "HEARTBEAT", "SYS_STATUS", "PARAM_VALUE"):
        print("  %-22s %d" % (name, hist.get(name, 0)))

    print()
    print("--- 消息种类全表（按条数降序，前 30）---")
    for name, cnt in hist.most_common(30):
        print("  %-30s %d" % (name, cnt))

    print()
    print("--- SYS_STATUS 传感器位（飞控自己的判断）---")
    if last_sys is None:
        print("  没收到 SYS_STATUS")
    else:
        for label, bit in (("LASER_POSITION (测距, bit20)", BIT_LASER),
                           ("OPTICAL_FLOW   (光流, bit24)", BIT_OPTFLOW)):
            print("  %s: present=%s enabled=%s health=%s" % (
                label,
                bool(last_sys.onboard_control_sensors_present & bit),
                bool(last_sys.onboard_control_sensors_enabled & bit),
                bool(last_sys.onboard_control_sensors_health & bit)))

    print()
    print("--- 光流/测距相关参数（tlog 里见到的）---")
    for name in WATCH_PARAMS:
        if name in params:
            print("  %-18s = %s" % (name, params[name]))
        else:
            print("  %-18s = (tlog 里没有)" % name)
    print("  （tlog 里共见到 %d 个参数）" % len(params))

    if laser_samples:
        print()
        print("--- DISTANCE_SENSOR 样本 ---")
        for s in laser_samples:
            print("  t=%s min=%s max=%s cur=%s orient=%s" % s)
    if flow_samples:
        print()
        print("--- OPTICAL_FLOW_RAD 样本 ---")
        for s in flow_samples:
            print("  int_us=%s ix=%.4f iy=%.4f q=%s dist=%.3f temp=%.1f" % (
                s[0], s[1], s[2], s[3], s[4], s[5]))

    print()
    if hist.get("DISTANCE_SENSOR", 0) == 0 and hist.get("OPTICAL_FLOW_RAD", 0) == 0:
        print("[结论] 这份 tlog 里一条 DISTANCE_SENSOR / OPTICAL_FLOW_RAD 都没有")
        print("       => 飞控在整段会话里从没收到过 MTF-01 的数据。")
    else:
        print("[结论] 飞控确实在往外发光流/测距数据，链路是通的。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
