#!/usr/bin/env python3
"""
px4_nsh.py -- 通过 MAVLink 在 PX4 上执行 nsh（NuttX shell）命令，不依赖 QGC。

存在理由
--------
QGC 的 MAVLink Console 本质上就是 MAVLink SERIAL_CONTROL(126) 报文。
这台机器上 QGC 时好时坏，所以自己实现一份，方便脚本化诊断。

用法
----
    set PYTHONPATH=D:\\09_PX4\\PX4_Project\\tools\\.pylibs
    python px4_nsh.py COM9 "uxrce_dds_client status"
    python px4_nsh.py COM9 "uorb top -1" --seconds 3
    python px4_nsh.py COM9 "listener distance_sensor -n 1"

参数
----
    --seconds  收集输出的秒数（默认 5）
    --baud     串口波特率（默认 115200）

★ 踩坑记录
  - SERIAL_CONTROL 的 device 字段要填 SERIAL_CONTROL_DEV_SHELL(10)。
  - flags 必须带 RESPOND(2)|EXCLUSIVE(4)，否则不返回输出。
  - 命令末尾要带 '\\n'。
  - 输出是按 70 字节一片回来的，要按到达顺序拼接，不要试图排序。
  - 本脚本独占串口，运行前必须关掉 QGC 和任何桥接进程。
"""

import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("缺少 pymavlink。先运行：\n"
             "  python -m pip install --target D:\\09_PX4\\PX4_Project\\tools\\.pylibs pymavlink\n"
             "并设置 PYTHONPATH=D:\\09_PX4\\PX4_Project\\tools\\.pylibs")

SERIAL_CONTROL_DEV_SHELL = 10
FLAG_REPLY = 1
FLAG_RESPOND = 2
FLAG_EXCLUSIVE = 4


def main():
    ap = argparse.ArgumentParser(description="通过 MAVLink 在 PX4 上执行 nsh 命令")
    ap.add_argument("port", help="串口，例如 COM9")
    ap.add_argument("command", help="要执行的 nsh 命令，例如 \"uxrce_dds_client status\"")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=5.0, help="收集输出秒数")
    args = ap.parse_args()

    m = mavutil.mavserial(args.port, baud=args.baud, autoreconnect=False)
    m.port.flushInput()
    print(f"[连接] {args.port} @ {args.baud} ...", flush=True)
    hb = m.wait_heartbeat(timeout=20)
    if hb is None:
        sys.exit("[失败] 没收到心跳。检查飞控是否上电、QGC 是否占着串口。")
    print(f"[连接] 成功 system={m.target_system}", flush=True)

    # GCS 心跳，防止链路被判为丢失
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    payload = (args.command.rstrip('\n') + '\n').encode('utf-8')
    print(f"[命令] {args.command}", flush=True)
    print("-" * 60, flush=True)

    flags = FLAG_RESPOND | FLAG_EXCLUSIVE
    deadline = time.time() + args.seconds
    sent = False
    chunks = []
    while time.time() < deadline:
        if not sent:
            m.mav.serial_control_send(
                SERIAL_CONTROL_DEV_SHELL, flags, 0, 0, len(payload),
                payload.ljust(70, b'\x00'))
            sent = True
        msg = m.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=0.3)
        if msg is None:
            # 每 1.5 秒重发一次请求（很多实现只在收到请求时才回吐缓冲）
            if time.time() + 1.5 > deadline:
                break
            m.mav.serial_control_send(
                SERIAL_CONTROL_DEV_SHELL, flags, 0, 0, 0,
                b'\x00' * 70)
            continue
        if msg.count and msg.data:
            chunks.append(bytes(msg.data[:msg.count]))

    out = b''.join(chunks)
    text = out.decode('utf-8', 'replace').replace('\r\n', '\n')
    print(text, flush=True)
    print("-" * 60, flush=True)
    print(f"[完成] 收到 {len(out)} 字节", flush=True)
    m.close()


if __name__ == "__main__":
    main()
