"""pos_link 帧编解码 —— 12 字节定长坐标帧。

本模块只用标准库，不依赖 ROS，方便在树莓派上单独跑测试。

帧格式（12 字节，小端）：

    [0]  0xAA          帧头
    [1]  0x55          帧头
    [2]  0x01          类型：坐标帧
    [3]  seq           序号 0~255 循环
    [4]  x & 0xFF      X 低字节
    [5]  x >> 8        X 高字节   (int16)
    [6]  y & 0xFF      Y 低字节
    [7]  y >> 8        Y 高字节   (int16)
    [8]  crc & 0xFF    CRC-16/CCITT-FALSE 低字节，覆盖 [0]~[7]
    [9]  crc >> 8      CRC-16/CCITT-FALSE 高字节
    [10] 0x0D          帧尾
    [11] 0x0A          帧尾

方向：**树莓派 → ESP32**。树莓派算好坐标、打包、推给 ESP32；
ESP32 收到后直接拿去驱动执行机构。见 docs/protocols/D_UAV_ESP32_POS_LINK_v1.0.md。
"""

from __future__ import annotations

import struct
from typing import List, Optional, Tuple

HEAD = b"\xAA\x55"
TAIL = b"\x0D\x0A"
FRAME_LEN = 12
TYPE_POS = 0x01

INT16_MIN = -32768
INT16_MAX = 32767

#: parse_frame() 的返回类型：(seq, x, y)
PosFrame = Tuple[int, int, int]


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE：poly 0x1021，init 0xFFFF，不反射，输出不异或。"""
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
    """把 (x, y) 打包成 12 字节帧。坐标超出 int16 直接抛 ValueError。

    >>> build_frame(100, 200, 0).hex(" ").upper()
    'AA 55 01 00 64 00 C8 00 E5 FC 0D 0A'
    """
    for name, value in (("x", x), ("y", y)):
        if not INT16_MIN <= value <= INT16_MAX:
            raise ValueError(f"{name}={value} 超出 int16 范围 [{INT16_MIN}, {INT16_MAX}]")

    body = HEAD + struct.pack("<BBhh", TYPE_POS, seq & 0xFF, x, y)  # 8 字节
    crc = crc16_ccitt(body)
    return body + struct.pack("<H", crc) + TAIL


def parse_frame(frame: bytes) -> Optional[PosFrame]:
    """解析一帧，返回 (seq, x, y)；帧头/帧尾/类型/CRC 任何一项不对都返回 None。"""
    if len(frame) != FRAME_LEN:
        return None
    if frame[:2] != HEAD or frame[10:] != TAIL:
        return None
    if frame[2] != TYPE_POS:
        return None
    if crc16_ccitt(frame[:8]) != struct.unpack_from("<H", frame, 8)[0]:
        return None

    _, seq, x, y = struct.unpack("<BBhh", frame[2:8])
    return seq, x, y


class FrameReader:
    """增量帧解析器：喂任意长度的字节流，吐出其中的完整好帧。

    为什么要有它：TCP 是字节流，一次 recv 可能只拿到半帧、也可能一次拿到三帧
    外加半截垃圾。固件 `main.c` 里是同一套状态机（逐字节找 `AA 55`），
    这个类让那套逻辑在 Python 侧也能被测到——`tools/pos_link_sim.py` 就是拿它
    冒充 ESP32 收帧的。
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes) -> List[PosFrame]:
        """喂一段字节，返回这次凑齐的好帧（可能不止一个）。"""
        frames: List[PosFrame] = []
        self._buf.extend(data)

        while True:
            start = self._buf.find(HEAD)
            if start < 0:
                # 没有帧头：只留最后一个字节，防止 0xAA 被切在两次 feed 之间
                if len(self._buf) > 1:
                    del self._buf[:-1]
                break
            if start:
                del self._buf[:start]
            if len(self._buf) < FRAME_LEN:
                break

            frame = bytes(self._buf[:FRAME_LEN])
            del self._buf[:FRAME_LEN]
            parsed = parse_frame(frame)
            if parsed is not None:
                frames.append(parsed)

        return frames
