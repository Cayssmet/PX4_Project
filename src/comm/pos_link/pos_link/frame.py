"""12 字节定长帧的编码、解码与 CRC 校验。

帧结构（小端）：

======  ======  ===========================================
偏移    长度    字段
======  ======  ===========================================
0       2       帧头 ``0xAA 0x55``
2       1       类型，``0x01`` = 坐标帧
3       1       序号 seq，0–255 循环
4       2       x，int16
6       2       y，int16
8       2       CRC16-CCITT-FALSE，覆盖偏移 0–7
10      2       帧尾 ``0x0D 0x0A``
======  ======  ===========================================

这个定义和 ESP32 侧的 ``firmware/esp32_pos_tx/main/main.c`` 里的 ``build_frame()``
是同一份东西，**改一边必须改另一边**，否则表现为「能连上但一帧也解不出来」。

为什么是定长而不是变长：本链路 1 Hz、每帧 12 字节，带宽从来不是瓶颈，
定长帧让接收端只需要一个 12 字节窗口 + 两个常数比较就能切帧，
不用维护长度字段和状态机，出错面最小。
"""

import struct

HEAD = b"\xAA\x55"
TAIL = b"\x0D\x0A"
FRAME_LEN = 12
TYPE_POS = 0x01

#: CRC 参数：poly 0x1021、init 0xFFFF、不反射、无终异或（即 CRC-16/CCITT-FALSE）
CRC_POLY = 0x1021
CRC_INIT = 0xFFFF


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE。

    自测向量：``crc16_ccitt(b"123456789") == 0x29B1``。
    """
    crc = CRC_INIT
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(x: int, y: int, seq: int) -> bytes:
    """按帧格式拼出 12 字节，主要给测试和假 ESP32 用。"""
    x = int(x)
    y = int(y)
    if not (-32768 <= x <= 32767 and -32768 <= y <= 32767):
        raise ValueError(f"坐标超出 int16 范围：x={x} y={y}")
    body = HEAD + struct.pack("<BBhh", TYPE_POS, int(seq) & 0xFF, x, y)
    return body + struct.pack("<H", crc16_ccitt(body)) + TAIL


def parse_frame(frame: bytes):
    """解一帧。成功返回 ``(seq, x, y)``，任何一项不合法都返回 ``None``。

    这个函数不负责找帧头 —— 那是接收缓冲区的事（见 ``bridge_node._drain``）。
    它只回答一个问题：这 12 个字节是不是一帧好的坐标帧。
    """
    if len(frame) != FRAME_LEN:
        return None
    if frame[0:2] != HEAD or frame[10:12] != TAIL:
        return None
    if frame[2] != TYPE_POS:
        return None
    crc_rx = struct.unpack_from("<H", frame, 8)[0]
    if crc16_ccitt(frame[0:8]) != crc_rx:
        return None
    x, y = struct.unpack_from("<hh", frame, 4)
    return frame[3], x, y
