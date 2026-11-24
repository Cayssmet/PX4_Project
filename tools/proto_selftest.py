#!/usr/bin/env python3
"""三条自定义链路的帧编解码 + CRC 自检。

协议见 docs/通信协议设计_v1.md。这个脚本是那条协议的**唯一参考实现**：
先把它跑通（`python3 tools/proto_selftest.py`），再拿它去接 K230 和小车。

设计要点
--------
* 帧封装三条链路共用：SOF(2) VER(1) TYPE(1) LEN(2) PAYLOAD(n) CRC16(2)
* CRC-16/CCITT-FALSE：poly 0x1021, init 0xFFFF, 不反转, 不异或输出，覆盖 VER..PAYLOAD
* 全部小端
* 解析器按字节流工作，丢字节后能从下一个 0xAA 0x55 重新同步

只依赖标准库，树莓派上没有任何 pip 包也能跑。
"""

from __future__ import annotations

import struct
import sys

SOF = b"\xAA\x55"
VER = 1
MAX_PAYLOAD = 64

# --------------------------------------------------------------------------
# CRC-16/CCITT-FALSE
# --------------------------------------------------------------------------


def crc16_ccitt_false(data: bytes) -> int:
    """poly=0x1021 init=0xFFFF refin=false refout=false xorout=0x0000"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# --------------------------------------------------------------------------
# 组帧 / 解析
# --------------------------------------------------------------------------


def build_frame(msg_type: int, payload: bytes = b"", ver: int = VER) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD}")
    body = struct.pack("<BBH", ver, msg_type & 0xFF, len(payload)) + payload
    crc = crc16_ccitt_false(body)
    return SOF + body + struct.pack("<H", crc)


class StreamParser:
    """喂字节流，吐出 (type, payload)。自带重新同步。"""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.dropped_bytes = 0

    def feed(self, data: bytes):
        self._buf.extend(data)
        out = []
        while True:
            # 1) 找到帧头
            idx = self._buf.find(SOF)
            if idx < 0:
                # 没有帧头：只保留最后 1 字节（可能是半个 SOF）
                if len(self._buf) > 1:
                    self.dropped_bytes += len(self._buf) - 1
                    del self._buf[:-1]
                break
            if idx > 0:
                self.dropped_bytes += idx
                del self._buf[:idx]

            # 2) 头部是否到齐
            if len(self._buf) < 6:
                break
            ver, msg_type, length = struct.unpack_from("<BBH", self._buf, 2)

            if ver != VER or length > MAX_PAYLOAD:
                # 头部不可信：跳过这个 SOF 继续找
                self.dropped_bytes += 1
                del self._buf[:1]
                continue

            total = 6 + length + 2
            if len(self._buf) < total:
                break  # 等更多字节

            payload = bytes(self._buf[6:6 + length])
            got_crc = struct.unpack_from("<H", self._buf, 6 + length)[0]
            want_crc = crc16_ccitt_false(bytes(self._buf[2:6 + length]))
            if got_crc != want_crc:
                # 校验失败：从下一个字节重新搜帧头
                self.dropped_bytes += 1
                del self._buf[:1]
                continue

            out.append((msg_type, payload))
            del self._buf[:total]
        return out


# --------------------------------------------------------------------------
# 链路 A：树莓派 <-> K230
# --------------------------------------------------------------------------

A_HEARTBEAT = 0x00
A_CMD_MODE = 0x01
A_CMD_SET_ROI = 0x02
A_CMD_SET_HEIGHT = 0x03

A_VISION_STATE = 0x81
A_EVENT = 0x82
A_STATUS = 0x83

MODE_STANDBY, MODE_SEARCH, MODE_ALIGN, MODE_STOP = 0, 1, 2, 3

FLAG_ALIGNED = 1 << 0
FLAG_LOST = 1 << 1
FLAG_READY_DROP = 1 << 2
FLAG_LOW_LIGHT = 1 << 3
FLAG_SATURATED = 1 << 4
FLAG_HEIGHT_FIXED = 1 << 5   # K230 在用参数里的固定高度，cm 缩放可能不准
FLAG_OVER_RATE = 1 << 6      # 实际帧率低于目标 30 Hz

# CMD_SET_HEIGHT 的 valid 位
HEIGHT_VALID = 1 << 0        # 高度可信
HEIGHT_FROM_RANGE = 1 << 1   # 用的是测距（而不是气压/推算）

_VISION_STATE_FMT = "<BBHHhhhHHI"  # 20 字节
_CMD_SET_HEIGHT_FMT = "<hBB"       # 4 字节


def encode_vision_state(found, mode, px, py, off_x_cm, off_y_cm,
                        height_cm, flags, quality, ts_ms) -> bytes:
    return build_frame(
        A_VISION_STATE,
        struct.pack(_VISION_STATE_FMT, found, mode, px, py,
                    off_x_cm, off_y_cm, height_cm, flags, quality, ts_ms),
    )


def decode_vision_state(payload: bytes) -> dict:
    (found, mode, px, py, off_x_cm, off_y_cm,
     height_cm, flags, quality, ts_ms) = struct.unpack(_VISION_STATE_FMT, payload)
    return {
        "found": bool(found), "mode": mode, "px": px, "py": py,
        "off_x_cm": off_x_cm, "off_y_cm": off_y_cm, "height_cm": height_cm,
        "flags": flags,
        "aligned": bool(flags & FLAG_ALIGNED), "lost": bool(flags & FLAG_LOST),
        "ready_drop": bool(flags & FLAG_READY_DROP),
        "low_light": bool(flags & FLAG_LOW_LIGHT),
        "saturated": bool(flags & FLAG_SATURATED),
        "height_fixed": bool(flags & FLAG_HEIGHT_FIXED),
        "over_rate": bool(flags & FLAG_OVER_RATE),
        "quality": quality, "ts_ms": ts_ms,
    }


def encode_cmd_mode(mode: int) -> bytes:
    return build_frame(A_CMD_MODE, struct.pack("<B", mode))


def decode_cmd_mode(payload: bytes) -> dict:
    (mode,) = struct.unpack("<B", payload)
    return {"mode": mode}


def encode_cmd_set_height(height_cm: int, valid: int = HEIGHT_VALID) -> bytes:
    return build_frame(A_CMD_SET_HEIGHT,
                       struct.pack(_CMD_SET_HEIGHT_FMT, height_cm, valid, 0))


def decode_cmd_set_height(payload: bytes) -> dict:
    height_cm, valid, _res = struct.unpack(_CMD_SET_HEIGHT_FMT, payload)
    return {"height_cm": height_cm, "valid": valid,
            "height_valid": bool(valid & HEIGHT_VALID),
            "from_range": bool(valid & HEIGHT_FROM_RANGE)}



# --------------------------------------------------------------------------
# 链路 B：树莓派 <-> 消防车
# --------------------------------------------------------------------------

B_HEARTBEAT = 0x00
B_DRONE_POSE = 0x01
B_FIRE_REPORT = 0x02
B_MILEAGE = 0x03
B_MISSION_STATE = 0x04

B_CAR_STATUS = 0x81
B_FIRE_EXTINGUISHED = 0x82
B_ACK = 0x83

_DL = 1 << 0
_DH = 1 << 1
_DF = 1 << 2
_DM = 1 << 3


def encode_drone_pose(x_cm, y_cm, z_cm, heading_d10, seq, valid) -> bytes:
    return build_frame(
        B_DRONE_POSE,
        struct.pack("<hhhhHBB", x_cm, y_cm, z_cm, heading_d10, seq, valid, 0),
    )


def decode_drone_pose(payload: bytes) -> dict:
    x, y, z, h, seq, valid, _res = struct.unpack("<hhhhHBB", payload)
    return {"x_cm": x, "y_cm": y, "z_cm": z, "heading_d10": h, "seq": seq,
            "pos_valid": bool(valid & _DL), "hgt_valid": bool(valid & _DH),
            "flying": bool(valid & _DF), "mission_active": bool(valid & _DM)}


def encode_fire_report(fire_id, x_cm, y_cm, ts_ms) -> bytes:
    return build_frame(B_FIRE_REPORT, struct.pack("<BBhhI", fire_id, 0, x_cm, y_cm, ts_ms))


def decode_fire_report(payload: bytes) -> dict:
    fire_id, _res, x, y, ts = struct.unpack("<BBhhI", payload)
    return {"fire_id": fire_id, "x_cm": x, "y_cm": y, "ts_ms": ts}


# 累计航程：单位 cm（int32 够装 21 km，场地只有 480cm 量级）
def encode_mileage(total_cm, delta_cm, ts_ms) -> bytes:
    return build_frame(B_MILEAGE, struct.pack("<iiI", total_cm, delta_cm, ts_ms))


def decode_mileage(payload: bytes) -> dict:
    total, delta, ts = struct.unpack("<iiI", payload)
    return {"total_cm": total, "delta_cm": delta, "ts_ms": ts}


# 任务状态
MS_IDLE, MS_TAKEOFF, MS_PATROL, MS_APPROACH = 0, 1, 2, 3
MS_ALIGN, MS_DROP, MS_RETURN, MS_LAND, MS_DONE, MS_ABORT = 4, 5, 6, 7, 8, 9

MISSION_STATE_NAMES = {
    MS_IDLE: "IDLE", MS_TAKEOFF: "TAKEOFF", MS_PATROL: "PATROL",
    MS_APPROACH: "APPROACH", MS_ALIGN: "ALIGN", MS_DROP: "DROP",
    MS_RETURN: "RETURN", MS_LAND: "LAND", MS_DONE: "DONE", MS_ABORT: "ABORT",
}


def encode_mission_state(state, fire_count, ts_ms) -> bytes:
    return build_frame(B_MISSION_STATE, struct.pack("<BBI", state, fire_count, ts_ms))


def decode_mission_state(payload: bytes) -> dict:
    state, fire_count, ts = struct.unpack("<BBI", payload)
    return {"state": state,
            "state_name": MISSION_STATE_NAMES.get(state, f"?{state}"),
            "fire_count": fire_count, "ts_ms": ts}



def encode_car_status(x_cm, y_cm, heading_d10, state, battery_pct) -> bytes:
    return build_frame(B_CAR_STATUS, struct.pack("<hhhBB", x_cm, y_cm, heading_d10,
                                                 state, battery_pct))


def decode_car_status(payload: bytes) -> dict:
    x, y, h, state, bat = struct.unpack("<hhhBB", payload)
    return {"x_cm": x, "y_cm": y, "heading_d10": h, "state": state, "battery_pct": bat}


# --------------------------------------------------------------------------
# 自检
# --------------------------------------------------------------------------

_fail = 0


def check(name: str, cond: bool, extra: str = "") -> None:
    global _fail
    if cond:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}  {extra}")
        _fail += 1


def main() -> int:
    print("=" * 70)
    print("CRC 参考向量（协议文档 §1）")
    print("=" * 70)
    body = struct.pack("<BBH", VER, A_VISION_STATE, 12) + b"\x00" * 12
    vec = crc16_ccitt_false(body)
    print(f"  CRC-16/CCITT-FALSE over VER..PAYLOAD = 0x{vec:04X}")
    print(f"  完整帧 hex = {(SOF + body + struct.pack('<H', vec)).hex(' ')}")

    print()
    print("=" * 70)
    print("编码 -> 解析 往返")
    print("=" * 70)

    # 链路 A
    frame = encode_vision_state(1, MODE_ALIGN, 640, 360, -37, 12, 180,
                                FLAG_ALIGNED | FLAG_READY_DROP, 870, 123456)
    p = StreamParser()
    msgs = p.feed(frame)
    check("A 单帧解析出 1 条", len(msgs) == 1, f"got {len(msgs)}")
    if msgs:
        check("A 帧类型正确", msgs[0][0] == A_VISION_STATE)
        d = decode_vision_state(msgs[0][1])
        check("A found", d["found"] is True)
        check("A off_x_cm 负数正确", d["off_x_cm"] == -37, str(d["off_x_cm"]))
        check("A aligned", d["aligned"] is True)
        check("A ready_drop", d["ready_drop"] is True)
        check("A quality", d["quality"] == 870)
        check("A ts_ms", d["ts_ms"] == 123456)

    # 链路 B
    frame = encode_drone_pose(123, 456, 180, 900, 42, _DL | _DF | _DM)
    p2 = StreamParser()
    msgs = p2.feed(frame)
    check("B 单帧解析出 1 条", len(msgs) == 1)
    if msgs:
        d = decode_drone_pose(msgs[0][1])
        check("B x/y/z", (d["x_cm"], d["y_cm"], d["z_cm"]) == (123, 456, 180))
        check("B heading_d10", d["heading_d10"] == 900)
        check("B seq", d["seq"] == 42)
        check("B valid 位", d["pos_valid"] and d["flying"] and d["mission_active"]
              and not d["hgt_valid"])

    # 粘包
    blob = encode_cmd_mode(MODE_SEARCH) + encode_vision_state(
        0, MODE_SEARCH, 0, 0, 0, 0, 0, 0, 0, 1) + encode_cmd_mode(MODE_STOP)
    p3 = StreamParser()
    msgs = p3.feed(blob)
    check("粘包解析出 3 条", len(msgs) == 3, f"got {len(msgs)}")
    check("粘包顺序", [m[0] for m in msgs] == [A_CMD_MODE, A_VISION_STATE, A_CMD_MODE])

    # 分片
    frame = encode_fire_report(2, 240, 160, 999)
    p4 = StreamParser()
    got = []
    for i in range(0, len(frame), 3):
        got.extend(p4.feed(frame[i:i + 3]))
    check("分成 3 字节片仍能解析", len(got) == 1 and got[0][0] == B_FIRE_REPORT)
    if got:
        d = decode_fire_report(got[0][1])
        check("分片后内容正确", (d["fire_id"], d["x_cm"], d["y_cm"], d["ts_ms"])
              == (2, 240, 160, 999))

    # 坏 CRC
    frame = bytearray(encode_cmd_mode(MODE_ALIGN))
    frame[-1] ^= 0xFF
    p5 = StreamParser()
    msgs = p5.feed(bytes(frame) + encode_cmd_mode(MODE_STOP))
    check("坏 CRC 被丢弃、后续帧仍能解析",
          len(msgs) == 1 and msgs[0][1] == struct.pack("<B", MODE_STOP),
          f"got {len(msgs)}")
    check("坏 CRC 计入丢弃字节", p5.dropped_bytes > 0)

    # 垃圾前缀
    p6 = StreamParser()
    msgs = p6.feed(b"\x11\x22\x33" + encode_cmd_mode(MODE_SEARCH))
    check("垃圾前缀后能重新同步", len(msgs) == 1)

    print()
    print("=" * 70)
    print("v1.0 之后新增的消息")
    print("=" * 70)

    # CMD_SET_HEIGHT（链路 A）
    frame = encode_cmd_set_height(180, HEIGHT_VALID | HEIGHT_FROM_RANGE)
    p7 = StreamParser()
    msgs = p7.feed(frame)
    check("SET_HEIGHT 解析出 1 条", len(msgs) == 1 and msgs[0][0] == A_CMD_SET_HEIGHT)
    if msgs:
        d = decode_cmd_set_height(msgs[0][1])
        check("SET_HEIGHT 高度", d["height_cm"] == 180, str(d["height_cm"]))
        check("SET_HEIGHT 有效位", d["height_valid"] is True)
        check("SET_HEIGHT 来自测距", d["from_range"] is True)

    # 高度失效时只发 valid=0
    d = decode_cmd_set_height(encode_cmd_set_height(0, 0)[6:-2])
    check("SET_HEIGHT valid=0 时 height_valid 为假", d["height_valid"] is False)

    # VISION_STATE 的两个降级位
    d = decode_vision_state(
        encode_vision_state(1, MODE_ALIGN, 10, 20, 5, -5, 180,
                            FLAG_ALIGNED | FLAG_HEIGHT_FIXED | FLAG_OVER_RATE,
                            500, 7)[6:-2])
    check("flags bit5 HEIGHT_FIXED", d["height_fixed"] is True)
    check("flags bit6 OVER_RATE", d["over_rate"] is True)
    check("降级位不影响 ALIGNED", d["aligned"] is True)

    # MILEAGE（链路 B）
    msgs = StreamParser().feed(encode_mileage(2720, -3, 424242))
    check("MILEAGE 解析出 1 条", len(msgs) == 1 and msgs[0][0] == B_MILEAGE)
    if msgs:
        d = decode_mileage(msgs[0][1])
        check("MILEAGE 总值/增量", (d["total_cm"], d["delta_cm"]) == (2720, -3),
              f"got {d}")
        check("MILEAGE 时间戳", d["ts_ms"] == 424242)

    # MISSION_STATE（链路 B）
    msgs = StreamParser().feed(encode_mission_state(MS_DROP, 2, 11))
    check("MISSION_STATE 解析出 1 条", len(msgs) == 1 and msgs[0][0] == B_MISSION_STATE)
    if msgs:
        d = decode_mission_state(msgs[0][1])
        check("MISSION_STATE 名称", d["state_name"] == "DROP", d["state_name"])
        check("MISSION_STATE 火情计数", d["fire_count"] == 2)

    # 越界值
    try:
        build_frame(0x01, b"\x00" * (MAX_PAYLOAD + 1))
        check("超长载荷被拒绝", False, "没有抛异常")
    except ValueError:
        check("超长载荷被拒绝", True)

    print()
    if _fail == 0:
        print(f"全部通过 ✅  （CRC 向量 0x{vec:04X}）")
        return 0
    print(f"有 {_fail} 项失败 ❌")
    return 1


if __name__ == "__main__":
    sys.exit(main())
