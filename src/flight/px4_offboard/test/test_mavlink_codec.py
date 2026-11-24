# -*- coding: utf-8 -*-
"""MAVLink 编解码自检。

两个「金标准」帧是在真机上用 pymavlink 生成、并已成功被 PX4 接受的字节，
直接用它们比对，比凭结构体规则推断可靠得多。

单独运行（不需要 ROS、不需要 pytest）：
    python3 test/test_mavlink_codec.py
"""

import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from px4_offboard import mavlink_codec as mc  # noqa: E402

# pymavlink 生成的参考帧（sysid=255 compid=190）
GOLDEN_HEARTBEAT = bytes.fromhex("fe0900ffbe000000000006080004034921")
GOLDEN_CMD_SET_INTERVAL = bytes.fromhex(
    "fe2101ffbe4c00000042005043470000000000000000000000000000000000000000"
    "f400010100fb82")
GOLDEN_PARAM_READ = bytes.fromhex(
    "fe1402ffbe14ffff010155585243455f4444535f434647000000"
    "f318")


def test_golden_heartbeat():
    enc = mc.MavlinkEncoder(sysid=255, compid=190)
    assert enc.heartbeat() == GOLDEN_HEARTBEAT


def test_golden_command_long():
    enc = mc.MavlinkEncoder(sysid=255, compid=190)
    enc.heartbeat()  # 消耗 seq=0，让下一条的 seq=1
    frame = enc.command_long(mc.MAV_CMD_SET_MESSAGE_INTERVAL, 32.0, 50000.0)
    assert frame == GOLDEN_CMD_SET_INTERVAL


def test_golden_param_request_read():
    enc = mc.MavlinkEncoder(sysid=255, compid=190)
    enc.heartbeat()
    enc.heartbeat()
    frame = enc.param_request_read("UXRCE_DDS_CFG")
    assert frame == GOLDEN_PARAM_READ


def test_crc_extra_complete():
    """要用到的消息必须都在 CRC_EXTRA 表里，否则发出去飞控静默丢弃。"""
    need = [mc.MSG_HEARTBEAT, mc.MSG_SET_MODE, mc.MSG_PARAM_REQUEST_READ,
            mc.MSG_PARAM_SET, mc.MSG_ATTITUDE, mc.MSG_LOCAL_POSITION_NED,
            mc.MSG_COMMAND_LONG, mc.MSG_COMMAND_ACK,
            mc.MSG_SET_POSITION_TARGET_LOCAL_NED,
            mc.MSG_VISION_POSITION_ESTIMATE, mc.MSG_ESTIMATOR_STATUS,
            mc.MSG_EXTENDED_SYS_STATE, mc.MSG_STATUSTEXT]
    missing = [m for m in need if m not in mc.CRC_EXTRA]
    assert not missing, f"缺 CRC_EXTRA: {missing}"


def test_parser_roundtrip_v1():
    enc = mc.MavlinkEncoder()
    p = mc.MavlinkParser()
    frame = enc.command_long(mc.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
    out = p.feed(frame)
    assert len(out) == 1
    msgid, payload, crc_ok = out[0]
    assert msgid == mc.MSG_COMMAND_LONG and crc_ok is True
    assert len(payload) == 33


def test_parser_detects_bad_crc():
    enc = mc.MavlinkEncoder()
    p = mc.MavlinkParser()
    frame = bytearray(enc.heartbeat())
    frame[-1] ^= 0xFF
    out = p.feed(bytes(frame) + enc.heartbeat())
    assert p.bad_crc >= 1
    # 坏帧照样会被解析出结构（crc_ok=False），好帧必须 crc_ok=True
    assert any(ok is True for _, _, ok in out)


def test_parser_v2_header_is_10_bytes():
    """★ 反复踩过的坑：v2 帧头 10 字节，写成 12 会让载荷整体错位 2 字节。"""
    payload = struct.pack("<Iffffff", 1, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    msgid = mc.MSG_LOCAL_POSITION_NED
    extra = mc.CRC_EXTRA[msgid]
    hdr = struct.pack("<BBBBBBBBBB", 0xFD, len(payload), 0, 0, 1, 1, 1,
                      msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF)
    crc = mc.crc_calc(hdr[1:] + payload, extra)
    frame = hdr + payload + struct.pack("<H", crc)

    p = mc.MavlinkParser()
    out = p.feed(frame)
    assert len(out) == 1
    mid, pl, ok = out[0]
    assert mid == msgid and ok is True
    d = mc.decode_local_position_ned(pl)
    assert (d["x"], d["y"], d["z"]) == (2.0, 3.0, 4.0)
    assert d["height"] == -4.0        # NED z 向下为正，高度取负


def test_parser_handles_split_stream():
    enc = mc.MavlinkEncoder()
    p = mc.MavlinkParser()
    frames = [enc.heartbeat(), enc.request_message(32), enc.heartbeat()]
    blob = b"".join(frames)
    got = []
    for i in range(0, len(blob), 3):
        got.extend(p.feed(blob[i:i + 3]))
    assert len(got) == 3


def test_parser_resync_after_garbage():
    enc = mc.MavlinkEncoder()
    p = mc.MavlinkParser()
    out = p.feed(b"\x11\x22\x33\x44" + enc.heartbeat())
    assert len(out) == 1 and out[0][2] is True


def test_decode_heartbeat_modes():
    # custom_mode = 4<<16 | 3<<24 = AUTO_LOITER
    cm = (3 << 24) | (4 << 16)
    payload = struct.pack("<IBBBBB", cm, 2, 12, 0x80 | 0x01, 4, 3)
    d = mc.decode_heartbeat(payload)
    assert d["main_mode"] == 4
    assert d["mode_name"] == "AUTO_LOITER"
    assert d["armed"] is True

    cm_offboard = (0 << 24) | (6 << 16)
    d = mc.decode_heartbeat(struct.pack("<IBBBBB", cm_offboard, 2, 12, 0, 4, 3))
    assert d["mode_name"] == "OFFBOARD"
    assert d["armed"] is False


def test_decode_param_value_int32():
    """★ PX4 把整数参数的位模式当 float 传，必须按 param_type 还原。"""
    raw = struct.unpack("<f", struct.pack("<i", 4001))[0]
    payload = struct.pack("<fHH16sB", raw, 997, 12,
                          b"SYS_AUTOSTART".ljust(16, b"\x00"), 6)
    d = mc.decode_param_value(payload)
    assert d["name"] == "SYS_AUTOSTART"
    assert d["value"] == 4001, d
    assert d["param_type_name"] == "i32"


def test_decode_param_value_float_passthrough():
    payload = struct.pack("<fHH16sB", 0.58, 997, 3,
                          b"MPC_THR_HOVER".ljust(16, b"\x00"), 9)
    d = mc.decode_param_value(payload)
    assert abs(d["value"] - 0.58) < 1e-6


def test_decode_extended_sys_state():
    d = mc.decode_extended_sys_state(bytes([0, 1]))
    assert d["landed"] is True and d["airborne"] is False
    d = mc.decode_extended_sys_state(bytes([0, 2]))
    assert d["landed"] is False and d["airborne"] is True


def test_decode_command_ack():
    payload = struct.pack("<HB", 400, 0) + bytes([0xFF]) + struct.pack("<i", 0)
    d = mc.decode_command_ack(payload)
    assert d["command"] == 400 and d["result_name"] == "ACCEPTED"


def test_vision_position_estimate_layout():
    enc = mc.MavlinkEncoder()
    frame = enc.vision_position_estimate(123456789, 1.0, 2.0, -3.0, 0.0, 0.0, 1.57)
    p = mc.MavlinkParser()
    mid, payload, ok = p.feed(frame)[0]
    assert mid == mc.MSG_VISION_POSITION_ESTIMATE and ok is True
    assert len(payload) == 32          # v1 不含协方差
    usec = struct.unpack("<Q", payload[:8])[0]
    assert usec == 123456789


def test_set_position_target_layout():
    enc = mc.MavlinkEncoder()
    frame = enc.set_position_target_local_ned(
        1000, vx=0.1, vy=-0.2, vz=-0.5, yaw_rate=0.3,
        type_mask=mc.POS_IGNORE | mc.ACC_IGNORE | mc.FORCE_IGNORE)
    p = mc.MavlinkParser()
    mid, payload, ok = p.feed(frame)[0]
    assert mid == mc.MSG_SET_POSITION_TARGET_LOCAL_NED and ok is True
    assert len(payload) == 53
    d = struct.unpack("<I11fHBBB", payload)
    assert d[0] == 1000
    assert abs(d[4] - 0.1) < 1e-6 and abs(d[5] + 0.2) < 1e-6 and abs(d[6] + 0.5) < 1e-6
    assert d[12] == (mc.POS_IGNORE | mc.ACC_IGNORE | mc.FORCE_IGNORE)
    assert d[15] == mc.MAV_FRAME_LOCAL_NED


def test_request_message_is_a_command():
    """512 不是独立消息，而是 COMMAND_LONG 里的命令号。"""
    enc = mc.MavlinkEncoder()
    p = mc.MavlinkParser()
    mid, payload, ok = p.feed(enc.request_message(32))[0]
    assert mid == mc.MSG_COMMAND_LONG
    command = struct.unpack("<H", payload[28:30])[0]
    assert command == mc.MAV_CMD_REQUEST_MESSAGE
    assert struct.unpack("<f", payload[0:4])[0] == 32.0


def test_unknown_msgid_raises():
    enc = mc.MavlinkEncoder()
    try:
        enc.pack(9999, b"\x00")
    except KeyError:
        return
    raise AssertionError("未知 msgid 应该抛 KeyError，而不是发一个 CRC 错的帧出去")


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    fns = [(k, v) for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    fails = 0
    for name, fn in fns:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as exc:  # noqa: BLE001
            print(f"  FAIL  {name}: {exc}")
            fails += 1
    print(f"\n{len(fns) - fails}/{len(fns)} 通过")
    sys.exit(1 if fails else 0)
