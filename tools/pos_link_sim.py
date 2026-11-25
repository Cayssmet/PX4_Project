#!/usr/bin/env python3
"""假 ESP32：按 1 Hz 往树莓派推坐标帧，用来在没有板子的情况下验链路。

    # 在树莓派上（和 bridge 同机）
    python3 tools/pos_link_sim.py

    # 在笔记本上（指向树莓派，走网线 192.168.100.1）
    python3 tools/pos_link_sim.py --host 192.168.100.1

    # 发 10 帧就退出（自动化测试用）
    python3 tools/pos_link_sim.py --count 10

故意不 import pos_link.frame：这个脚本要在没 colcon build、没 source 工作区的
情况下也能跑，所以帧的拼装在这里独立实现一份。两份实现的一致性由
src/comm/pos_link/test/test_frame.py 里的参考帧卡住。
"""

import argparse
import socket
import struct
import sys
import time

HEAD = b"\xAA\x55"
TAIL = b"\x0D\x0A"
TYPE_POS = 0x01


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE：poly 0x1021、init 0xFFFF、不反射、无终异或。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(x: int, y: int, seq: int) -> bytes:
    body = HEAD + struct.pack("<BBhh", TYPE_POS, seq & 0xFF, x, y)
    return body + struct.pack("<H", crc16_ccitt(body)) + TAIL


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="假 ESP32 坐标发包器")
    ap.add_argument("--host", default="127.0.0.1", help="树莓派 IP（默认本机）")
    ap.add_argument("--port", type=int, default=5000, help="TCP 端口（默认 5000）")
    ap.add_argument("--period", type=float, default=1.0, help="发包周期，秒（默认 1.0）")
    ap.add_argument("--count", type=int, default=0, help="发多少帧后退出，0 = 一直发")
    ap.add_argument("--x0", type=int, default=100, help="x 起点")
    ap.add_argument("--y0", type=int, default=200, help="y 起点")
    ap.add_argument("--dx", type=int, default=7, help="每帧 x 增量")
    ap.add_argument("--dy", type=int, default=3, help="每帧 y 增量")
    args = ap.parse_args(argv)

    print(f"连接 {args.host}:{args.port} ...", flush=True)
    sock = socket.create_connection((args.host, args.port), timeout=5.0)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("已连接，开始发包（Ctrl-C 结束）", flush=True)

    seq = 0
    sent = 0
    try:
        while args.count == 0 or sent < args.count:
            # 坐标保持在 0–999：超出三位数就回绕，模拟赛题场地里的循环走动
            x = (args.x0 + args.dx * sent) % 1000
            y = (args.y0 + args.dy * sent) % 1000
            frame = build_frame(x, y, seq)
            sock.sendall(frame)
            print(f"seq={seq:3d} x={x:3d} y={y:3d}  {frame.hex(' ').upper()}", flush=True)
            seq = (seq + 1) & 0xFF
            sent += 1
            if args.count == 0 or sent < args.count:
                time.sleep(args.period)
    except KeyboardInterrupt:
        print("\n用户中断", flush=True)
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
