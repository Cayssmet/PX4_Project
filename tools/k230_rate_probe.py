#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""k230_rate_probe.py —— 实测 K230 视觉上报的真实帧率与抖动。

用途：
    协议设计里 VISION_STATE 的目标是 30 Hz，但 K230 上跑检测能不能稳住 30 Hz
    从来没有实测过。接上串口后先跑这个脚本，用数字说话。

用法：
    python3 tools/k230_rate_probe.py --dev /dev/k230 --seconds 10
    python3 tools/k230_rate_probe.py --dev /dev/ttyAMA0        # GPIO 接法
    python3 tools/k230_rate_probe.py --dev /dev/flight_controller   # 用来查是否插反

判据（协议文档 §7 Q3）：
    平均帧率 >= 20 Hz  且  相邻帧间隔 p95 < 80 ms
    => 可以做连续视觉伺服；否则改用「到位 -> 停 -> 再测」的间歇策略。

★ 顺带的自检：如果读到的是 MAVLink 帧（0xFE / 0xFD 开头），会提示「这个口上是飞控，
  不是 K230」——用来判断两条 CH340 线有没有插反。

依赖：pyserial（树莓派上已装 3.5）
"""

import argparse
import os
import statistics
import sys
import time

# 复用 proto_selftest.py 里的唯一参考实现，避免两份 CRC 各写一遍
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from proto_selftest import StreamParser, decode_vision_state  # noqa: E402


def percentile(values, p):
    """线性插值的分位数（不依赖 numpy）。"""
    if not values:
        return 0.0
    s = sorted(values)
    if len(s) == 1:
        return float(s[0])
    k = (len(s) - 1) * (p / 100.0)
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    frac = k - lo
    return s[lo] * (1.0 - frac) + s[hi] * frac


def main():
    ap = argparse.ArgumentParser(description="实测 K230 VISION_STATE 帧率与抖动")
    ap.add_argument("--dev", default="/dev/k230", help="串口设备（默认 /dev/k230）")
    ap.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200）")
    ap.add_argument("--seconds", type=float, default=10.0, help="采样时长（默认 10 秒）")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        print("缺少 pyserial：sudo apt install python3-serial", file=sys.stderr)
        return 2

    try:
        port = serial.Serial(args.dev, args.baud, timeout=0.2)
    except Exception as exc:  # noqa: BLE001
        print(f"打不开 {args.dev}：{exc}", file=sys.stderr)
        print("提示：确认设备名（ls -l /dev/k230）、权限（dialout 组）、以及没有被别的进程占用。",
              file=sys.stderr)
        return 2

    parser = StreamParser()
    stamps = []          # 每帧 VISION_STATE 的到达时刻（monotonic）
    payloads = []        # 最近若干帧，用来看 flags/高度
    type_counts = {}
    mavlink_seen = 0     # 疑似 MAVLink 帧头计数
    total_bytes = 0

    print(f"监听 {args.dev} @ {args.baud} 共 {args.seconds:.0f} 秒 ……")
    t_end = time.monotonic() + args.seconds

    try:
        while time.monotonic() < t_end:
            chunk = port.read(4096)
            if not chunk:
                continue
            total_bytes += len(chunk)
            # 粗判 MAVLink：MAVLink v1 以 0xFE 开头、v2 以 0xFD 开头，
            # 而我们的自定义帧固定以 0xAA 0x55 开头，两者不会混淆。
            mavlink_seen += sum(1 for b in chunk if b in (0xFE, 0xFD))
            for msg_type, payload in parser.feed(chunk):
                type_counts[msg_type] = type_counts.get(msg_type, 0) + 1
                if msg_type == 0x81:  # VISION_STATE
                    stamps.append(time.monotonic())
                    if len(payloads) < 8:
                        payloads.append(payload)
    except KeyboardInterrupt:
        print("\n（被 Ctrl-C 打断，用已采到的数据统计）")
    finally:
        port.close()

    print()
    print("=" * 56)
    print(f"收到字节 {total_bytes} B，耗时 {args.seconds:.1f} s "
          f"=> {total_bytes / max(args.seconds, 1e-3):.0f} B/s")
    if type_counts:
        names = {0x81: "VISION_STATE", 0x82: "EVENT", 0x83: "STATUS"}
        for t, n in sorted(type_counts.items()):
            print(f"  TYPE 0x{t:02X} {names.get(t, '(未知)'):<14} {n:>6} 帧")

    if mavlink_seen and not type_counts:
        print()
        print("★ 这个口上收到的像是 MAVLink 帧（0xFE/0xFD 开头），不是 K230 的 AA 55 帧。")
        print("  极可能是两条 CH340 线插反了 —— 检查 /dev/flight_controller 与 /dev/k230。")
        return 1

    if len(stamps) < 5:
        print()
        print("帧数太少，无法统计。排查顺序：K230 有没有在跑 -> 波特率 -> TX/RX 是否交叉 -> 共地。")
        return 1

    # 只统计中间段，避免第一帧和最后一帧把间隔拉长
    gaps_ms = [(b - a) * 1000.0 for a, b in zip(stamps, stamps[1:])]
    span = stamps[-1] - stamps[0]
    rate = (len(stamps) - 1) / span if span > 0 else 0.0

    print()
    print(f"帧数 {len(stamps)}，跨度 {span:.2f} s")
    print(f"平均帧率        {rate:8.2f} Hz")
    print(f"帧间隔 中位数   {statistics.median(gaps_ms):8.1f} ms")
    print(f"帧间隔 p95      {percentile(gaps_ms, 95):8.1f} ms")
    print(f"帧间隔 最大     {max(gaps_ms):8.1f} ms")

    if payloads:
        print()
        print("前几帧解码：")
        for p in payloads[:4]:
            try:
                d = decode_vision_state(p)
            except Exception as exc:  # noqa: BLE001
                print(f"  解码失败：{exc}")
                continue
            print(f"  found={d['found']} mode={d['mode']} off=({d['off_x_cm']},{d['off_y_cm']})cm "
                  f"h={d['height_cm']}cm flags=0x{d['flags']:04X} q={d['quality']}")
            flags = d["flags"]
            if flags & (1 << 5):
                print("    ⚠ bit5 HEIGHT_FIXED：K230 在用固定高度，cm 偏移的缩放可能不准")
            if flags & (1 << 6):
                print("    ⚠ bit6 OVER_RATE：K230 自己认为降频了")

    print()
    print("=" * 56)
    ok_rate = rate >= 20.0
    ok_jitter = percentile(gaps_ms, 95) < 80.0
    print(f"判据：平均 >= 20 Hz {'✅' if ok_rate else '❌'}   "
          f"p95 间隔 < 80 ms {'✅' if ok_jitter else '❌'}")
    if ok_rate and ok_jitter:
        print("结论：可以做连续视觉伺服。")
        return 0
    print("结论：帧率/抖动不够，视觉伺服请改用「到位 -> 停 -> 再测」的间歇策略。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
