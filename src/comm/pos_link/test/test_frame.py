"""frame.py 的单元测试。

跑法（不需要 ROS，纯标准库 + pytest）::

    cd src/comm/pos_link && python3 -m pytest test -q

参考帧的字节来自协议文档 docs/protocols/D_UAV_ESP32_POS_LINK_v1.0.md，
和固件 main.c 里 build_frame() 的输出必须逐字节一致。
"""

import struct

import pytest

from pos_link.frame import (
    FRAME_LEN,
    HEAD,
    TAIL,
    build_frame,
    crc16_ccitt,
    parse_frame,
)


def test_crc_vector():
    """CRC-16/CCITT-FALSE 的标准自测向量。"""
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_build_frame_known_bytes():
    assert build_frame(100, 200, 0) == bytes.fromhex("AA5501006400C800E5FC0D0A")


def test_build_frame_known_bytes_seq1():
    assert build_frame(7, 3, 1) == bytes.fromhex("AA55010107000300145E0D0A")


def test_frame_layout():
    f = build_frame(1, 2, 3)
    assert len(f) == FRAME_LEN
    assert f[0:2] == HEAD
    assert f[2] == 0x01
    assert f[3] == 3
    assert struct.unpack_from("<hh", f, 4) == (1, 2)
    assert f[10:12] == TAIL


def test_parse_roundtrip():
    for seq, x, y in [(0, 0, 0), (1, 100, 200), (255, 999, 999), (42, -5, -6)]:
        assert parse_frame(build_frame(x, y, seq)) == (seq, x, y)


def test_worst_case_magnitude():
    """三位数坐标的上限，距离 int16 还很远 —— 这里只是把上限钉住。"""
    assert parse_frame(build_frame(999, 999, 0)) == (0, 999, 999)


def test_parse_rejects_bad_crc():
    f = bytearray(build_frame(100, 200, 0))
    f[8] ^= 0xFF
    assert parse_frame(bytes(f)) is None


def test_parse_rejects_bad_tail():
    f = bytearray(build_frame(100, 200, 0))
    f[11] = 0x00
    assert parse_frame(bytes(f)) is None


def test_parse_rejects_bad_head():
    f = bytearray(build_frame(100, 200, 0))
    f[0] = 0x00
    assert parse_frame(bytes(f)) is None


def test_parse_rejects_unknown_type():
    f = bytearray(build_frame(100, 200, 0))
    f[2] = 0x02
    # 改类型会让 CRC 一起失效；为了单独测类型检查，重算 CRC。
    f[8:10] = struct.pack("<H", crc16_ccitt(bytes(f[0:8])))
    assert parse_frame(bytes(f)) is None


def test_parse_rejects_wrong_length():
    f = build_frame(100, 200, 0)
    assert parse_frame(f[:-1]) is None
    assert parse_frame(f + b"\x00") is None


def test_negative_coordinates_roundtrip():
    """固件现在只发 0–999，但 int16 支持负数，先钉住行为。"""
    assert parse_frame(build_frame(-1, -32768, 9)) == (9, -1, -32768)


def test_out_of_range_raises():
    with pytest.raises(ValueError):
        build_frame(40000, 0, 0)
    with pytest.raises(ValueError):
        build_frame(0, -40000, 0)


def test_seq_wraps_to_byte():
    assert build_frame(0, 0, 256)[3] == 0
    assert build_frame(0, 0, -1)[3] == 255
