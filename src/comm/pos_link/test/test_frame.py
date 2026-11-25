"""pos_link 帧编解码的离线测试。

不需要 ROS、不需要硬件：`cd src/comm/pos_link && python3 -m pytest test -q`。
没装 pytest 也能跑：`python3 test/test_frame.py`。

代码里写进两个「参考帧」，和协议文档 D_UAV_ESP32_POS_LINK_v1.0.md、
固件 main.c 里的 build_frame 是同一份期望值——三处不一致就说明改坏了。
"""

from __future__ import annotations

import pytest

from pos_link.frame import (
    FRAME_LEN,
    FrameReader,
    build_frame,
    crc16_ccitt,
    parse_frame,
)

# 参考帧：改协议必须同时改这里、协议文档、固件
REF_A = bytes.fromhex("AA 55 01 00 64 00 C8 00 E5 FC 0D 0A")  # x=100 y=200 seq=0
REF_B = bytes.fromhex("AA 55 01 01 07 00 03 00 14 5E 0D 0A")  # x=7 y=3 seq=1


def test_crc_standard_vector():
    """CRC-16/CCITT-FALSE 的标准校验向量。"""
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_reference_frames():
    assert build_frame(100, 200, 0) == REF_A
    assert build_frame(7, 3, 1) == REF_B


@pytest.mark.parametrize(
    "x,y,seq",
    [
        (0, 0, 0),
        (-1, -1, 255),
        (100, 200, 3),
        (32767, -32768, 127),
        (498, 372, 200),
    ],
)
def test_roundtrip(x, y, seq):
    assert parse_frame(build_frame(x, y, seq)) == (seq, x, y)


def test_seq_wraps_at_256():
    assert build_frame(1, 1, 256)[3] == 0
    assert build_frame(1, 1, 257)[3] == 1


def test_out_of_range_rejected():
    with pytest.raises(ValueError):
        build_frame(40000, 0, 0)
    with pytest.raises(ValueError):
        build_frame(0, -40000, 0)


@pytest.mark.parametrize(
    "mutate",
    [
        lambda f: b"\x00" + f[1:],  # 坏帧头
        lambda f: f[:10] + b"\x00\x00",  # 坏帧尾
        lambda f: f[:2] + b"\x02" + f[3:],  # 未知类型
        lambda f: f[:5] + bytes([f[5] ^ 0xFF]) + f[6:],  # 坐标被改，CRC 不匹配
        lambda f: f[:-1],  # 长度不对
    ],
)
def test_bad_frames_rejected(mutate):
    assert parse_frame(mutate(REF_A)) is None


def test_reader_handles_split_frames():
    """一帧被拆成两段喂进来。"""
    reader = FrameReader()
    assert reader.feed(REF_A[:5]) == []
    assert reader.feed(REF_A[5:]) == [(0, 100, 200)]


def test_reader_handles_byte_by_byte():
    reader = FrameReader()
    out = []
    for byte in REF_A + REF_B:
        out += reader.feed(bytes([byte]))
    assert out == [(0, 100, 200), (1, 7, 3)]


def test_reader_back_to_back():
    reader = FrameReader()
    assert reader.feed(REF_A + REF_B) == [(0, 100, 200), (1, 7, 3)]


def test_reader_skips_garbage_before_head():
    reader = FrameReader()
    assert reader.feed(b"\x11\x22\x33" + REF_A) == [(0, 100, 200)]


def test_reader_drops_bad_crc_and_recovers():
    reader = FrameReader()
    bad = REF_A[:9] + b"\x00\x00" + REF_A[11:]
    assert reader.feed(bad + REF_A) == [(0, 100, 200)]


def test_reader_keeps_trailing_head_byte():
    """0xAA 被切在两次 feed 之间，不能丢。"""
    reader = FrameReader()
    assert reader.feed(b"\x00\xAA") == []
    assert reader.feed(b"\x55" + REF_A[2:]) == [(0, 100, 200)]


def test_frame_len_is_12():
    assert FRAME_LEN == 12
    assert len(REF_A) == FRAME_LEN


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
