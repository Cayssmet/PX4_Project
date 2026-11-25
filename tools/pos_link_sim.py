#!/usr/bin/env python3
"""假 ESP32 —— 连上树莓派的 pos_link 端口，收帧、校验、打印。

用途：手边没有真板子时，验证「服务端在发、帧收全了、序号连续」。

    python3 tools/pos_link_sim.py                     # 默认连 192.168.50.1:5000
    python3 tools/pos_link_sim.py --host 127.0.0.1    # 在本机连本机
    python3 tools/pos_link_sim.py --seconds 20 --quiet

真 ESP32 走的是同一套流程（TCP 客户端 + 逐字节找 `AA 55`），
所以这个脚本收得对，板子基本也就收得对。

故意**不 import** `pos_link.frame`：没 `colcon build` 也能直接跑，
顺便当第二份独立实现，和包里的实现互相印证。

帧格式见 docs/protocols/D_UAV_ESP32_POS_LINK_v1.0.md。
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from typing import List, Optional, Tuple

HEAD = b"\xAA\x55"
TAIL = b"\x0D\x0A"
FRAME_LEN = 12
TYPE_POS = 0x01

Frame = Tuple[int, int, int]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def parse_frame(frame: bytes) -> Optional[Frame]:
    if len(frame) != FRAME_LEN or frame[:2] != HEAD or frame[10:] != TAIL:
        return None
    if frame[2] != TYPE_POS:
        return None
    if crc16_ccitt(frame[:8]) != struct.unpack_from("<H", frame, 8)[0]:
        return None
    _, seq, x, y = struct.unpack("<BBhh", frame[2:8])
    return seq, x, y


class Reader:
    """把 TCP 字节流切成帧，并统计好帧/坏帧/序号跳变。"""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.good = 0
        self.bad = 0
        self.gaps = 0
        self._last_seq: Optional[int] = None

    def feed(self, data: bytes) -> List[Frame]:
        out: List[Frame] = []
        self._buf.extend(data)
        while True:
            start = self._buf.find(HEAD)
            if start < 0:
                if len(self._buf) > 1:
                    del self._buf[:-1]
                return out
            if start:
                del self._buf[:start]
            if len(self._buf) < FRAME_LEN:
                return out

            frame = bytes(self._buf[:FRAME_LEN])
            del self._buf[:FRAME_LEN]
            parsed = parse_frame(frame)
            if parsed is None:
                self.bad += 1
                continue

            seq = parsed[0]
            if self._last_seq is not None:
                expected = (self._last_seq + 1) & 0xFF
                if seq != expected:
                    self.gaps += 1
            self._last_seq = seq
            self.good += 1
            out.append(parsed)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="假 ESP32：收树莓派下发的坐标帧")
    ap.add_argument("--host", default="192.168.50.1", help="树莓派地址（默认 192.168.50.1）")
    ap.add_argument("--port", type=int, default=5000, help="桥接端口（默认 5000）")
    ap.add_argument("--seconds", type=float, default=0.0, help="跑多久后退出，0 = 一直跑")
    ap.add_argument("--quiet", action="store_true", help="不逐帧打印，只打统计")
    return ap.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    reader = Reader()
    deadline = None if args.seconds <= 0 else time.monotonic() + args.seconds

    def time_left() -> bool:
        return deadline is None or time.monotonic() < deadline

    sock: Optional[socket.socket] = None
    try:
        while time_left():
            if sock is None:
                try:
                    sock = socket.create_connection((args.host, args.port), timeout=5.0)
                except OSError as exc:
                    print(f"连接 {args.host}:{args.port} 失败：{exc}（1 秒后重试）", file=sys.stderr)
                    time.sleep(1.0)
                    continue
                print(f"已连接 {args.host}:{args.port}，等坐标帧……（Ctrl-C 退出）")
                sock.settimeout(3.0)

            try:
                data = sock.recv(256)
            except socket.timeout:
                print("3 秒没收到数据：树莓派那边还没有坐标源？", file=sys.stderr)
                continue
            except OSError as exc:
                print(f"接收出错：{exc}", file=sys.stderr)
                sock.close()
                sock = None
                continue

            if not data:
                print("对端关闭了连接（桥接节点重启了？），重连中……", file=sys.stderr)
                sock.close()
                sock = None
                continue

            for seq, x, y in reader.feed(data):
                if not args.quiet:
                    print(f"帧 seq={seq:3d} x={x:6d} y={y:6d}")
    except KeyboardInterrupt:
        print()
    finally:
        if sock is not None:
            sock.close()

    print(
        f"共收到 {reader.good} 个好帧，{reader.bad} 个坏帧，"
        f"{reader.gaps} 次序号跳变"
    )
    return 0 if reader.good else 1


if __name__ == "__main__":
    raise SystemExit(main())
