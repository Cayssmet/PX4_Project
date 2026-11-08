#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过 UDP 桥改飞控参数（不需要独占 COM 口）。

为什么需要这个：
    飞控的 USB 串口 COM9 由 mavlink_usb_udp_bridge.py 独占（计划任务 PX4_QGC_Bridge
    会把它拉起来），任何别的程序再去开 COM9 都会 PermissionError。但桥在
    UDP 14551 上开着"PC -> 飞控"的入口：往那里发的字节会被原样写进串口。
    所以调参数不必抢串口，直接投一个 PARAM_SET / COMMAND_LONG 进去就行。

局限：
    看不到飞控的回复（回复被桥送到 14550，也就是 QGC）。所以设置结果要靠
    事后验证 —— 用 tools/tlog_scan.py 扫 QGC 的 tlog（PARAM_VALUE 会被记录），
    或者趁桥重连的空档再独占串口复查。

用法：
    python param_set_via_bridge.py MAV_1_MODE=0 SER_TEL2_BAUD=115200
    python param_set_via_bridge.py --reboot                # 只重启飞控
    python param_set_via_bridge.py --bridge-port 14551 ...

参数名区分大小写；值按 int 解析，带小数点则按 float 发。
"""

import argparse
import socket
import sys
import time

from pymavlink.dialects.v20 import common as mavlink2

TARGET_SYSTEM = 1
TARGET_COMPONENT = 1
GCS_SYSTEM = 255
GCS_COMPONENT = 190          # MAV_COMP_ID_MISSIONPLANNER

MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246


class _NullFile:
    """pymavlink 的 MAVLink() 需要一个 file 对象；这里只用来 pack()，不写盘。"""

    def write(self, data):
        return len(data)

    def flush(self):
        pass


def build_encoder():
    return mavlink2.MAVLink(_NullFile(), srcSystem=GCS_SYSTEM, srcComponent=GCS_COMPONENT)


def param_type_for(value):
    if isinstance(value, float):
        return mavlink2.MAV_PARAM_TYPE_REAL32
    return mavlink2.MAV_PARAM_TYPE_INT32


def parse_assignment(text):
    if "=" not in text:
        raise SystemExit("[!!] 参数格式应为 NAME=VALUE，收到：%r" % text)
    name, raw = text.split("=", 1)
    name = name.strip()
    raw = raw.strip()
    try:
        value = int(raw)
    except ValueError:
        try:
            value = float(raw)
        except ValueError:
            raise SystemExit("[!!] 无法解析数值：%r" % raw)
    return name, value


def encode_param_set(mav, name, value):
    """pymavlink 2.4.50 的 MAVLink_param_set_message 内部会对 param_id 做
    `param_id.split(b"\\x00")[0].decode(...)`，也就是说它期望 **bytes**；
    传 str 会抛 `TypeError: must be str or None, not bytes`。这里两种都试一遍，
    免得以后 pymavlink 改了签名又要改代码。
    """
    ptype = param_type_for(value)
    try:
        return mav.param_set_encode(
            TARGET_SYSTEM, TARGET_COMPONENT, name.encode("utf-8"), float(value), ptype)
    except TypeError:
        return mav.param_set_encode(
            TARGET_SYSTEM, TARGET_COMPONENT, name, float(value), ptype)


def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("assignments", nargs="*", help="NAME=VALUE ...")
    ap.add_argument("--bridge-port", type=int, default=14551,
                    help="桥的 PC->飞控 UDP 入口，默认 14551")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--reboot", action="store_true", help="发完参数后重启飞控")
    ap.add_argument("--rounds", type=int, default=4, help="每个参数重复发送轮数，默认 4")
    args = ap.parse_args()

    if not args.assignments and not args.reboot:
        raise SystemExit("[!!] 没有要做的事：既没给参数也没 --reboot")

    mav = build_encoder()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    dest = (args.host, args.bridge_port)

    print("[..] 目标桥入口 %s:%d（桥会把收到的字节写进飞控串口）" % dest)

    # 先发几拍地面站心跳，让飞控知道有个新的 GCS 端点
    for _ in range(3):
        sock.sendto(mav.heartbeat_encode(
            mavlink2.MAV_TYPE_GCS, mavlink2.MAV_AUTOPILOT_INVALID, 0, 0, 0).pack(mav), dest)
        time.sleep(0.1)

    items = [parse_assignment(a) for a in args.assignments]
    for name, value in items:
        print("[..] %s = %s (%s)" % (name, value, "float" if isinstance(value, float) else "int"))

    for r in range(1, args.rounds + 1):
        for name, value in items:
            sock.sendto(encode_param_set(mav, name, value).pack(mav), dest)
            time.sleep(0.15)
        print("[..] 第 %d/%d 轮已发" % (r, args.rounds))
        time.sleep(0.4)

    if args.reboot:
        print("[..] 发送重启命令（PREFLIGHT_REBOOT_SHUTDOWN）")
        for _ in range(5):
            cmd = mav.command_long_encode(
                TARGET_SYSTEM, TARGET_COMPONENT,
                MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                1, 0, 0, 0, 0, 0, 0)
            sock.sendto(cmd.pack(mav), dest)
            time.sleep(0.3)

    sock.close()
    print("[ok] 发送完毕。注意：这条通路看不到飞控回复，设置结果需要事后验证。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
