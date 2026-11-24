#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car_link_sim.py —— 假消防车（TCP Client），用来在没有真车的情况下联调树莓派。

它做两件事：
  1. 连上树莓派的 TCP Server，把收到的 DRONE_POSE / FIRE_REPORT / MILEAGE /
     MISSION_STATE 全部解码打印出来；
  2. **自己按收到的位置点独立累加航程**，和树莓派发来的 MILEAGE 帧对比 ——
     这样能验证「树莓派算的航程」和「车侧算的航程」是不是同一个数（赛题要求车上显示累计航程）。

用法
----
    # 只收（默认）：看树莓派在发什么
    python3 tools/car_link_sim.py --host 192.168.4.1

    # 收发都要：模拟车上线，回心跳与状态
    python3 tools/car_link_sim.py --host 192.168.4.1 --reply

    # 连本机（树莓派上自测）
    python3 tools/car_link_sim.py --host 127.0.0.1 --seconds 20

注意
----
* 树莓派是 **Server**，本脚本是 **Client**。所以要先起树莓派上的 `car_link`，再跑这个。
* 走 TCP 时靠 TCP 的可靠性，不需要重传；帧封装仍带 CRC，是为了和串口链路统一。
* 只依赖标准库。
"""

import argparse
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from proto_selftest import (  # noqa: E402
    StreamParser,
    B_HEARTBEAT, B_DRONE_POSE, B_FIRE_REPORT, B_MILEAGE, B_MISSION_STATE,
    B_CAR_STATUS, B_FIRE_EXTINGUISHED, B_ACK,
    build_frame, decode_drone_pose, decode_fire_report,
    decode_mileage, decode_mission_state, encode_car_status,
)

CAR_STATE_IDLE, CAR_STATE_MOVING, CAR_STATE_EXTINGUISHING, CAR_STATE_DONE = 0, 1, 2, 3


def main():
    ap = argparse.ArgumentParser(description="假消防车 TCP Client（联调用）")
    ap.add_argument("--host", default="192.168.4.1", help="树莓派在热点网段里的 IP")
    ap.add_argument("--port", type=int, default=9000, help="树莓派监听端口（默认 9000）")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="运行时长，0 表示一直跑到 Ctrl-C")
    ap.add_argument("--reply", action="store_true",
                    help="回心跳与 CAR_STATUS（模拟真车上线）")
    ap.add_argument("--verbose", action="store_true", help="打印每一帧")
    args = ap.parse_args()

    print(f"连接 {args.host}:{args.port} ……")
    try:
        sock = socket.create_connection((args.host, args.port), timeout=5)
    except Exception as exc:  # noqa: BLE001
        print(f"连不上：{exc}", file=sys.stderr)
        print("排查：树莓派上 car_link 起来了吗？防火墙？热点网段 IP 对吗？"
              "（在 Pi 上 `nmcli -f IP4.ADDRESS device show wlan0` 查）", file=sys.stderr)
        return 2

    sock.setblocking(False)
    parser = StreamParser()

    # 车侧自己算的航程
    last_xy = None
    my_mileage_cm = 0.0
    pi_mileage_cm = 0
    pose_count = 0
    seq_last = None
    seq_gaps = 0
    fire_reports = []
    last_state = None
    last_pose_print = 0.0
    t_start = time.monotonic()
    t_end = (t_start + args.seconds) if args.seconds > 0 else None
    next_heartbeat = t_start
    next_status = t_start

    print("已连接。Ctrl-C 退出。\n")

    try:
        while True:
            now = time.monotonic()
            if t_end is not None and now >= t_end:
                break

            # ---- 收 ----
            try:
                chunk = sock.recv(4096)
                if chunk == b"":
                    print("对端关闭了连接。")
                    break
                for msg_type, payload in parser.feed(chunk):
                    if msg_type == B_DRONE_POSE:
                        d = decode_drone_pose(payload)
                        pose_count += 1
                        if seq_last is not None and d["seq"] != (seq_last + 1) & 0xFFFF:
                            seq_gaps += 1
                        seq_last = d["seq"]
                        if d["pos_valid"]:
                            xy = (d["x_cm"], d["y_cm"])
                            if last_xy is not None:
                                my_mileage_cm += math.hypot(xy[0] - last_xy[0],
                                                            xy[1] - last_xy[1])
                            last_xy = xy
                        if args.verbose or (now - last_pose_print) > 1.0:
                            last_pose_print = now
                            print(f"[POSE] ({d['x_cm']:>4},{d['y_cm']:>4},{d['z_cm']:>4})cm "
                                  f"hdg={d['heading_d10'] / 10.0:6.1f}° seq={d['seq']:<5} "
                                  f"pos={'Y' if d['pos_valid'] else 'N'} "
                                  f"hgt={'Y' if d['hgt_valid'] else 'N'} "
                                  f"fly={'Y' if d['flying'] else 'N'} "
                                  f"mission={'Y' if d['mission_active'] else 'N'}")
                    elif msg_type == B_FIRE_REPORT:
                        d = decode_fire_report(payload)
                        fire_reports.append(d)
                        print(f"[FIRE] #{d['fire_id']} at ({d['x_cm']},{d['y_cm']})cm "
                              f"ts={d['ts_ms']}")
                    elif msg_type == B_MILEAGE:
                        d = decode_mileage(payload)
                        pi_mileage_cm = d["total_cm"]
                        if args.verbose:
                            print(f"[MILE] 树莓派 total={d['total_cm']}cm delta={d['delta_cm']}cm")
                    elif msg_type == B_MISSION_STATE:
                        d = decode_mission_state(payload)
                        if d["state"] != last_state:
                            last_state = d["state"]
                            print(f"[STATE] -> {d['state_name']}  "
                                  f"（已发现火情 {d['fire_count']} 处）")
                    elif msg_type == B_HEARTBEAT:
                        if args.verbose:
                            print("[HB] 树莓派心跳")
                    elif msg_type == B_CAR_STATUS:
                        pass  # 不该收到（那是本脚本发的）
                    else:
                        print(f"[?] 未知帧类型 0x{msg_type:02X}，{len(payload)} 字节")
            except BlockingIOError:
                pass

            # ---- 发（可选）----
            if args.reply:
                if now >= next_heartbeat:
                    sock.sendall(build_frame(B_HEARTBEAT))
                    next_heartbeat = now + 1.0
                if now >= next_status:
                    # 车侧自己估的坐标：这里只是占位，真车要填里程计/定位结果
                    cx = int(last_xy[0]) if last_xy else 0
                    cy = int(last_xy[1]) if last_xy else 0
                    sock.sendall(encode_car_status(cx, cy, 0, CAR_STATE_IDLE, 100))
                    next_status = now + 1.0

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n（Ctrl-C）")
    finally:
        sock.close()

    # ---- 汇总 ----
    elapsed = time.monotonic() - t_start
    print()
    print("=" * 60)
    print(f"运行 {elapsed:.1f} s")
    print(f"DRONE_POSE {pose_count} 帧 => {pose_count / max(elapsed, 1e-3):.2f} Hz"
          f"（协议要求 2 Hz）")
    if seq_gaps:
        print(f"⚠ 序号不连续 {seq_gaps} 次（丢帧或对端跳号）")
    print(f"火情报告 {len(fire_reports)} 条")
    for d in fire_reports:
        in_range = 0 <= d["x_cm"] <= 480 and 0 <= d["y_cm"] <= 400
        print(f"  #{d['fire_id']} ({d['x_cm']},{d['y_cm']})cm "
              f"{'✅ 在场地内' if in_range else '❌ 超出 480×400 场地范围'}")
    print()
    print(f"车侧自算航程 {my_mileage_cm / 100.0:8.3f} m")
    print(f"树莓派报告   {pi_mileage_cm / 100.0:8.3f} m")
    diff = abs(my_mileage_cm - pi_mileage_cm) / 100.0
    if pi_mileage_cm:
        print(f"差值         {diff:8.3f} m"
              f"（{'一致 ✅' if diff < 0.5 else '差异偏大 ⚠ 检查两边的算法是否相同'}）")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
