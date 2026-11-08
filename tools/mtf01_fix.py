#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MTF-01 光流测距传感器 —— 飞控侧连通性修复脚本

背景
----
MicoAir 官方文档《MTF-01光流测距一体传感器用户手册》在 PX4 1.17.x 一节的步骤是：
    MAV_1_CONFIG = TELEM n  ->  重启飞控
    MAV_1_MODE   = Normal
    SER_TELn_BAUD = 115200 8N1
    EKF2_OF_CTRL  = Enabled
    EKF2_RNG_CTRL = Enabled
    EKF2_HGT_REF  = Range sensor  ->  重启飞控
    SENS_FLOW_ROT = No rotation
    ★ 1.17.x 起唯一新增的一条：MAV_PROTO_VER = 1，然后重启飞控

本项目实测：TELEM4 已开 MAVLink(MAV_2_CONFIG=104)、SER_TEL4_BAUD=115200、
EKF2_OF_CTRL=1、EKF2_RNG_CTRL=1 都对了，但 MAV_PROTO_VER 仍是 2，
且 MAVLink Inspector / 探针都收不到 DISTANCE_SENSOR 与 OPTICAL_FLOW_RAD。
本脚本就是把 MAV_PROTO_VER 改成 1 并重启飞控。

用法
----
    set PYTHONPATH=D:\\09_PX4\\PX4_Project\\tools\\.pylibs
    python -u mtf01_fix.py COM9 115200            # 只读当前参数，不做任何改动
    python -u mtf01_fix.py COM9 115200 --apply    # 改 MAV_PROTO_VER=1 并重启飞控

注意
----
* 改完必须重启飞控才生效，重启期间 COM9 会消失十几秒。
* MAV_PROTO_VER 是**全局**参数：改成 1 后飞控所有 MAVLink 链路都用 MAVLink v1 说话。
  QGC 与 pymavlink 都能正确处理 v1，但这是全局影响，要知道自己在做什么。
"""

import sys
import time
import struct

from pymavlink import mavutil

TARGET = (1, 1)  # sysid 1 / compid 1（AUTOPILOT）

# 需要核对的光流/测距相关参数
WATCH = [
    # 协议版本与 MAVLink 实例（光流走 MAV_1 = TELEM2 = UART4 = /dev/ttyS3）
    "MAV_PROTO_VER",
    "MAV_1_CONFIG", "MAV_1_MODE", "MAV_1_RATE", "MAV_1_FORWARD", "MAV_1_FLOW_CTRL",
    "MAV_2_CONFIG", "MAV_2_MODE", "MAV_2_RATE",
    # 串口波特率
    "SER_TEL1_BAUD", "SER_TEL2_BAUD", "SER_TEL3_BAUD", "SER_TEL4_BAUD", "SER_URT6_BAUD",
    # EKF2 光流/测距/高度源
    "EKF2_OF_CTRL", "EKF2_RNG_CTRL", "EKF2_HGT_REF",
    "EKF2_OF_DELAY", "EKF2_OF_QMIN", "EKF2_RNG_A_VMAX", "EKF2_RNG_A_HMAX",
    # 驱动层流/测距使能（PX4 内置驱动，MTF-01 是外部 MAVLink 外设，这些都应为 0）
    "SENS_FLOW_ROT", "SENS_TFLOW_CFG",
]

# 传感器健康位（MAV_SYS_STATUS_SENSOR 位掩码）
BIT_LASER = 1 << 20
BIT_OPTFLOW = 1 << 24


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


def connect(port, baud, timeout=25.0, attempts=6):
    """打开串口并等心跳。

    Windows 上串口是"谁先拿到谁独占"，而且上一个进程刚 close 时句柄未必立刻释放，
    所以这里做指数式重试而不是一次失败就退出。
    """
    print("[..] 连接 %s @ %d ..." % (port, baud))
    # 不能用 mavlink_connection()：它会按设备名猜类型，Windows 的 COM9 会被
    # 当成日志文件去 open()，直接抛 PermissionError。显式用 mavserial。
    last_err = None
    master = None
    for i in range(1, attempts + 1):
        try:
            master = mavutil.mavserial(port, baud=baud, autoreconnect=False)
            break
        except Exception as exc:
            last_err = exc
            print("      第 %d/%d 次打开失败：%s" % (i, attempts, exc))
            time.sleep(1.5 * i)
    if master is None:
        print("[!!] 串口打不开：%s" % last_err)
        print("     多半是 QGC 或桥还占着这个口。先停掉它们再试。")
        return None
    hb = master.wait_heartbeat(timeout=timeout)
    if hb is None:
        print("[!!] 等不到心跳")
        return None
    print("[ok] 心跳：sysid=%d compid=%d autopilot=%d type=%d"
          % (master.target_system, master.target_component,
             hb.autopilot, hb.type))
    return master


def fetch_params(master):
    """主动拉全量参数（PX4 会在连上后自动发，但显式请求更保险）。"""
    print("[..] 拉取参数表 ...")
    master.mav.param_request_list_send(*TARGET)
    got = {}
    deadline = time.time() + 12.0
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        try:
            name = msg.param_id
            if isinstance(name, bytes):
                name = name.decode("utf-8", "ignore")
            name = name.rstrip("\x00")
            got[name] = (decode_param_value(msg.param_type, msg.param_value), msg.param_type)
            if len(got) % 200 == 0:
                print("      已收 %d 个参数 ..." % len(got))
        except Exception:
            continue
    print("[ok] 收到 %d 个参数" % len(got))
    return got


def read_sys_status(master, seconds=3.0):
    """读传感器健康位。"""
    master.mav.request_data_stream_send(*TARGET,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    deadline = time.time() + seconds
    last = None
    while time.time() < deadline:
        msg = master.recv_match(type="SYS_STATUS", blocking=True, timeout=0.5)
        if msg is not None:
            last = msg
    if last is None:
        print("[!!] 没收到 SYS_STATUS")
        return
    print("[..] SYS_STATUS 传感器位：")
    print("      测距 LASER_POSITION(bit20): present=%s enabled=%s healthy=%s"
          % (bool(last.onboard_control_sensors_present & BIT_LASER),
             bool(last.onboard_control_sensors_enabled & BIT_LASER),
             bool(last.onboard_control_sensors_health & BIT_LASER)))
    print("      光流 OPTICAL_FLOW   (bit24): present=%s enabled=%s healthy=%s"
          % (bool(last.onboard_control_sensors_present & BIT_OPTFLOW),
             bool(last.onboard_control_sensors_enabled & BIT_OPTFLOW),
             bool(last.onboard_control_sensors_health & BIT_OPTFLOW)))


def count_flow_messages(master, seconds=8.0):
    """主动要 10 Hz 推送，数 DISTANCE_SENSOR / OPTICAL_FLOW_RAD 到达条数。"""
    for mid, name in ((mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, "DISTANCE_SENSOR"),
                      (mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, "OPTICAL_FLOW_RAD")):
        master.mav.command_long_send(*TARGET,
                                     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                     mid, 100000, 0, 0, 0, 0, 0)
    print("[..] 监听 %.0f 秒，数光流/测距消息 ..." % seconds)
    counts = {"DISTANCE_SENSOR": 0, "OPTICAL_FLOW_RAD": 0}
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(type=["DISTANCE_SENSOR", "OPTICAL_FLOW_RAD"],
                                blocking=True, timeout=0.5)
        if msg is not None:
            counts[msg.get_type()] += 1
    for k, v in counts.items():
        print("      %-20s %d 条" % (k, v))
    return counts


def set_param(master, name, value_int):
    print("[..] 设置 %s = %d ..." % (name, value_int))
    master.mav.param_set_send(*TARGET, name.encode(), float(value_int),
                              mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    deadline = time.time() + 8.0
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        pid = msg.param_id
        if isinstance(pid, bytes):
            pid = pid.decode("utf-8", "ignore")
        if pid.rstrip("\x00") == name:
            got = decode_param_value(msg.param_type, msg.param_value)
            print("[ok] 飞控回报 %s = %s" % (name, got))
            return got == value_int
    print("[!!] 没等到 %s 的 PARAM_VALUE 回报" % name)
    return False


def reboot(master):
    print("[..] 请求飞控重启 ...")
    master.mav.command_long_send(*TARGET,
                                 mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                 0, 1, 0, 0, 0, 0, 0, 0)
    try:
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5.0)
        if ack is not None:
            print("[ok] COMMAND_ACK result=%d" % ack.result)
    except Exception:
        pass


def main():
    args = [a for a in sys.argv[1:]]
    apply_fix = "--apply" in args
    args = [a for a in args if not a.startswith("--")]
    port = args[0] if len(args) > 0 else "COM9"
    baud = int(args[1]) if len(args) > 1 else 115200

    master = connect(port, baud)
    if master is None:
        return 2

    params = fetch_params(master)
    print()
    print("=" * 68)
    print("光流/测距相关参数当前值")
    print("=" * 68)
    for name in WATCH:
        if name in params:
            val, _ = params[name]
            print("  %-18s = %s" % (name, val))
        else:
            print("  %-18s = (飞控没有这个参数)" % name)
    print()

    read_sys_status(master)
    print()
    counts = count_flow_messages(master)

    ok = counts["DISTANCE_SENSOR"] > 0 or counts["OPTICAL_FLOW_RAD"] > 0
    print()
    if ok:
        print("[结论] 已经能看到光流/测距消息，飞控侧配置通了。")
    else:
        print("[结论] 仍然收不到 DISTANCE_SENSOR / OPTICAL_FLOW_RAD。")

    proto = params.get("MAV_PROTO_VER", (None, None))[0]
    print("[诊断] MAV_PROTO_VER = %s（官方要求 1.17.x 配 MTF-01 时设为 1）" % proto)

    if apply_fix and proto != 1 and not ok:
        print()
        print("=" * 68)
        print("开始修复：MAV_PROTO_VER -> 1，然后重启飞控")
        print("=" * 68)
        if not set_param(master, "MAV_PROTO_VER", 1):
            print("[!!] 设置失败，放弃")
            return 3
        time.sleep(1.0)
        reboot(master)
        print("[ok] 已请求重启。等 25 秒后重新跑本脚本（不加 --apply）复查。")
    else:
        if proto != 1:
            print("[提示] 加 --apply 可以自动把 MAV_PROTO_VER 改成 1 并重启飞控。")

    try:
        master.close()
    except Exception:
        pass
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
