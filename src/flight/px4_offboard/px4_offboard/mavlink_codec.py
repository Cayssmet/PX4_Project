# -*- coding: utf-8 -*-
"""mavlink_codec.py —— 零依赖的 MAVLink v1 编解码（只实现本项目用到的消息）。

为什么不用 pymavlink
--------------------
树莓派上 `pip3` 都没有，PyPI 也不一定能连。而这里只需要十几条消息，
手工组帧的字节已经用 pymavlink 生成的参考帧**逐字节核对过**，
所以干脆不依赖任何第三方包。

已验证的依据（v3.0.1 之前用 `tools/` 下的探针在真机上跑通）：
* `HEARTBEAT` 与 `COMMAND_LONG` 组帧与 pymavlink 输出**完全一致**
* `PARAM_REQUEST_READ` 的字段顺序是 `<hBB16s`（按类型大小降序重排）
* 收到过 `COMMAND_ACK`、`PARAM_VALUE`、`LOCAL_POSITION_NED`、`DISTANCE_SENSOR`

两个反复踩过的坑（写在这里就不用再踩）
--------------------------------------
1. **v2 帧头是 10 字节**（不是 12）。带签名再 +13。写错会让载荷整体错位。
2. **PX4 把整数参数的位模式当 float 传**，`PARAM_VALUE.value` 要按
   `param_type` 还原，否则 `SYS_AUTOSTART=4001` 读出来是 `0.0000`。

字段顺序规则
------------
MAVLink 1.0 按**字段类型大小降序**排列，同尺寸保持 XML 里的声明顺序。
例如 `PARAM_REQUEST_READ` 的 XML 顺序是
`param_index, target_system, target_component, param_id`，
但 `param_id[16]` 最大所以排最前 —— **这是错的**，实测应为 `<hBB16s`，
即 i16, u8, u8, char[16]：先最小后最大的**升序**。

> 以实测为准：`<hBB16s`。新加消息时，**必须**用 `pymavlink` 生成参考帧比对一次，
> 不要凭规则推断。
"""

from __future__ import annotations

import struct

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

MAVLINK_STX_V1 = 0xFE
MAVLINK_STX_V2 = 0xFD

# 我们扮演地面站（GCS）
DEFAULT_SYSID = 255
DEFAULT_COMPID = 190          # MAV_COMP_ID_MISSIONPLANNER

# MAVLink 消息 ID
MSG_HEARTBEAT = 0
MSG_SET_MODE = 11
MSG_PARAM_REQUEST_READ = 20
MSG_PARAM_VALUE = 22
MSG_PARAM_SET = 23
MSG_ATTITUDE = 30
MSG_LOCAL_POSITION_NED = 32
MSG_COMMAND_LONG = 76
MSG_COMMAND_ACK = 77
MSG_SET_POSITION_TARGET_LOCAL_NED = 84
MSG_VISION_POSITION_ESTIMATE = 102
MSG_ESTIMATOR_STATUS = 230
MSG_EXTENDED_SYS_STATE = 245
MSG_STATUSTEXT = 253

# CRC_EXTRA：每条消息一个常数，参与校验。抄错一位飞控就会静默丢弃帧。
# 这里的值都是已经在真机上验证过能正常收发的。
CRC_EXTRA = {
    0: 50, 1: 124, 11: 89, 20: 214, 21: 159, 22: 220, 23: 168, 24: 24,
    30: 39, 31: 246, 32: 185, 33: 104, 36: 222, 62: 183, 65: 118,
    74: 20, 76: 152, 77: 143, 82: 49, 84: 143, 85: 140,
    102: 158, 105: 93, 106: 138, 111: 34, 132: 85, 141: 47, 147: 154,
    230: 163, 245: 130, 253: 83, 331: 236,
}

MSG_NAME = {
    0: "HEARTBEAT", 11: "SET_MODE", 20: "PARAM_REQUEST_READ", 22: "PARAM_VALUE",
    23: "PARAM_SET", 30: "ATTITUDE", 32: "LOCAL_POSITION_NED", 76: "COMMAND_LONG",
    77: "COMMAND_ACK", 84: "SET_POSITION_TARGET_LOCAL_NED",
    102: "VISION_POSITION_ESTIMATE", 230: "ESTIMATOR_STATUS",
    245: "EXTENDED_SYS_STATE", 253: "STATUSTEXT",
}


def msg_name(msgid: int) -> str:
    return MSG_NAME.get(msgid, f"MSG_{msgid}")


# MAV_MODE_FLAG
MAV_MODE_FLAG_SAFETY_ARMED = 0x80
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 0x01

# MAV_RESULT
MAV_RESULT = {
    0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
    3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED",
}

# 常用命令号
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_REQUEST_MESSAGE = 512
MAV_CMD_SET_MESSAGE_INTERVAL = 244
MAV_CMD_RUN_PREARM_CHECKS = 401

# PX4 custom_mode 拆解：bit16-23 主模式，bit24-31 子模式
PX4_MAIN_MODE = {
    1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
    5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE",
}
PX4_AUTO_SUB_MODE = {
    1: "AUTO_READY", 2: "AUTO_TAKEOFF", 3: "AUTO_LOITER", 4: "AUTO_MISSION",
    5: "AUTO_RTL", 6: "AUTO_LAND", 7: "AUTO_RTGS", 8: "AUTO_FOLLOW_TARGET",
    9: "AUTO_PRECLAND", 10: "AUTO_VTOL_TAKEOFF",
}

# MAV_STATE
MAV_STATE = {
    0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY",
    4: "ACTIVE", 5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF",
    8: "FLIGHT_TERMINATION",
}

# MAV_LANDED_STATE
LANDED_STATE = {
    0: "UNDEFINED", 1: "ON_GROUND", 2: "IN_AIR", 3: "TAKEOFF",
    4: "LANDING",
}

# MAV_FRAME
MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_LOCAL_OFFSET_NED = 7

# POSITION_TARGET_TYPEMASK
POS_IGNORE = 1 << 0
VEL_IGNORE = 1 << 1
ACC_IGNORE = 1 << 2
FORCE_IGNORE = 1 << 3
YAW_IGNORE = 1 << 4
YAW_RATE_IGNORE = 1 << 5

# MAV_PARAM_TYPE
MAV_PARAM_TYPE = {
    1: "u8", 2: "i8", 3: "u16", 4: "i16", 5: "u32", 6: "i32",
    7: "u64", 8: "i64", 9: "f32", 10: "f64",
}
# 用于把「按 float 位模式传输」的整数还原
_INT_FMT = {1: "<B", 2: "<b", 3: "<H", 4: "<h", 5: "<I", 6: "<i", 7: "<Q", 8: "<q"}


# ---------------------------------------------------------------------------
# CRC-16/MCRF4XX（MAVLink 用的那个，与 CCITT-FALSE 不同）
# ---------------------------------------------------------------------------


def _crc_acc(byte: int, crc: int) -> int:
    tmp = (byte ^ (crc & 0xFF)) & 0xFF
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def crc_calc(data: bytes, extra: int) -> int:
    crc = 0xFFFF
    for b in data:
        crc = _crc_acc(b, crc)
    return _crc_acc(extra & 0xFF, crc)


# ---------------------------------------------------------------------------
# 编码
# ---------------------------------------------------------------------------


class MavlinkEncoder:
    """组 MAVLink v1 帧。seq 自增（飞控不严格要求连续，但便于抓包对帧）。"""

    def __init__(self, sysid: int = DEFAULT_SYSID, compid: int = DEFAULT_COMPID):
        self.sysid = sysid
        self.compid = compid
        self.seq = 0

    def pack(self, msgid: int, payload: bytes) -> bytes:
        extra = CRC_EXTRA.get(msgid)
        if extra is None:
            raise KeyError(f"消息 {msgid} 没有 CRC_EXTRA，先补表再用")
        hdr = bytes([
            len(payload) & 0xFF,
            self.seq & 0xFF,
            self.sysid & 0xFF,
            self.compid & 0xFF,
            msgid & 0xFF,
        ])
        self.seq = (self.seq + 1) & 0xFF
        crc = crc_calc(hdr + payload, extra)
        return bytes([MAVLINK_STX_V1]) + hdr + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    # ---- 具体消息 ----

    def heartbeat(self, mav_type: int = 6, autopilot: int = 8) -> bytes:
        """type=6 是 MAV_TYPE_GCS，autopilot=8 是 MAV_AUTOPILOT_INVALID。

        飞控要靠这个心跳认识我们（有些版本不收到心跳就不往下发遥测）。
        """
        return self.pack(MSG_HEARTBEAT, struct.pack("<IBBBBB", 0, mav_type, autopilot, 0, 4, 3))

    def command_long(self, command: int, p1: float = 0.0, p2: float = 0.0,
                     p3: float = 0.0, p4: float = 0.0, target_system: int = 1,
                     target_component: int = 1) -> bytes:
        payload = struct.pack("<7fHBBB", p1, p2, p3, p4, 0.0, 0.0, 0.0,
                              command, target_system, target_component, 0)
        return self.pack(MSG_COMMAND_LONG, payload)

    def request_message(self, msgid: int) -> bytes:
        """`MAV_CMD_REQUEST_MESSAGE`：点名要一条消息。

        ★ 这是本项目在 MAVLink 上的**核心机制**：`SET_MESSAGE_INTERVAL(244)`
        在 PX4 1.17 上回 UNSUPPORTED，而 512 回 ACCEPTED 且每次都能拉出一条，
        所以我们用「定时点名」代替「设置流频率」，频率由我们自己控。
        """
        return self.command_long(MAV_CMD_REQUEST_MESSAGE, float(msgid))

    def param_request_read(self, name: str) -> bytes:
        pid = name.encode("ascii")[:16].ljust(16, b"\x00")
        return self.pack(MSG_PARAM_REQUEST_READ, struct.pack("<hBB16s", -1, 1, 1, pid))

    def param_set(self, name: str, value: float, param_type: int = 9,
                  target_system: int = 1, target_component: int = 1) -> bytes:
        pid = name.encode("ascii")[:16].ljust(16, b"\x00")
        # 字段（升序）：param_value f32, target_system u8, target_component u8,
        #              param_id char16, param_type u8
        payload = struct.pack("<fBB16sB", float(value), target_system,
                              target_component, pid, param_type)
        return self.pack(MSG_PARAM_SET, payload)

    def set_mode(self, custom_mode: int, custom_sub_mode: int = 0,
                 target_system: int = 1) -> bytes:
        """`SET_MODE`：custom_mode = 主模式<<16 | 子模式<<24。"""
        cm = ((custom_sub_mode & 0xFF) << 24) | ((custom_mode & 0xFF) << 16)
        return self.pack(MSG_SET_MODE, struct.pack("<IBB", cm, target_system,
                                                   MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))

    def vision_position_estimate(self, usec: int, x: float, y: float, z: float,
                                 roll: float, pitch: float, yaw: float) -> bytes:
        """`VISION_POSITION_ESTIMATE`(102)：PX4 会把它当外部视觉位姿（FRD 系）。

        v1 只有 32 字节（不含协方差）。PX4 的 `EKF2_EV_CTRL=1` 只用水平位置。
        """
        return self.pack(MSG_VISION_POSITION_ESTIMATE,
                         struct.pack("<Q6f", int(usec), x, y, z, roll, pitch, yaw))

    def set_position_target_local_ned(self, time_boot_ms: int,
                                      x: float = 0.0, y: float = 0.0, z: float = 0.0,
                                      vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
                                      afx: float = 0.0, afy: float = 0.0, afz: float = 0.0,
                                      yaw: float = 0.0, yaw_rate: float = 0.0,
                                      type_mask: int = 0,
                                      coordinate_frame: int = MAV_FRAME_LOCAL_NED,
                                      target_system: int = 1,
                                      target_component: int = 1) -> bytes:
        """`SET_POSITION_TARGET_LOCAL_NED`(84)：offboard 设定点。

        ★ **在 MAVLink 路线上，这条流本身就是 offboard 心跳**：以 ≥2 Hz 持续发
        PX4 才认为外部控制在线；停发（默认 1 s，`COM_OF_LOSS_T`）就退出 offboard。
        不需要像 DDS 那样另发 `OffboardControlMode`。
        """
        payload = struct.pack("<I11fHBBB", int(time_boot_ms),
                              x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate,
                              type_mask & 0xFFFF, target_system, target_component,
                              coordinate_frame)
        return self.pack(MSG_SET_POSITION_TARGET_LOCAL_NED, payload)


# ---------------------------------------------------------------------------
# 解析
# ---------------------------------------------------------------------------


class MavlinkParser:
    """增量解析器。喂字节，吐 `(msgid, payload, crc_ok)`。"""

    def __init__(self):
        self._buf = bytearray()
        self.dropped_bytes = 0
        self.bad_crc = 0

    def feed(self, data: bytes):
        self._buf.extend(data)
        out = []
        buf = self._buf
        i = 0
        n = len(buf)
        while i < n:
            magic = buf[i]
            if magic == MAVLINK_STX_V1:
                if i + 8 > n:
                    break
                length = buf[i + 1]
                total = 8 + length
                if i + total > n:
                    break
                msgid = buf[i + 5]
                payload = bytes(buf[i + 6:i + 6 + length])
                got_crc = buf[i + 6 + length] | (buf[i + 7 + length] << 8)
                out.append((msgid, payload, self._check_crc(buf, i, length, got_crc)))
                i += total
            elif magic == MAVLINK_STX_V2:
                if i + 10 > n:
                    break
                length = buf[i + 1]
                incompat = buf[i + 2]
                # ★ v2 头固定 10 字节；有签名再 +13
                hdr_len = 10 + (13 if (incompat & 0x01) else 0)
                total = hdr_len + length + 2
                if i + total > n:
                    break
                msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
                payload = bytes(buf[i + hdr_len:i + hdr_len + length])
                got_crc = buf[i + hdr_len + length] | (buf[i + hdr_len + length + 1] << 8)
                out.append((msgid, payload, self._check_crc_v2(buf, i, hdr_len, length, got_crc)))
                i += total
            else:
                i += 1
                self.dropped_bytes += 1
        if i > 0:
            del buf[:i]
        return out

    def _check_crc(self, buf, i, length, got_crc):
        extra = CRC_EXTRA.get(buf[i + 5])
        if extra is None:
            return None            # 表里没有，无法校验
        want = crc_calc(bytes(buf[i + 1:i + 6 + length]), extra)
        if want == got_crc:
            return True
        self.bad_crc += 1
        return False

    def _check_crc_v2(self, buf, i, hdr_len, length, got_crc):
        extra = CRC_EXTRA.get(buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16))
        if extra is None:
            return None
        # v2 校验不含 SOF，也不含签名
        want = crc_calc(bytes(buf[i + 1:i + hdr_len + length]), extra)
        if want == got_crc:
            return True
        self.bad_crc += 1
        return False


# ---------------------------------------------------------------------------
# 解码
# ---------------------------------------------------------------------------


def decode_heartbeat(payload: bytes) -> dict:
    cm, mav_type, autopilot, base_mode, system_status, mavlink_version = \
        struct.unpack("<IBBBBB", payload[:9])
    main = (cm >> 16) & 0xFF
    sub = (cm >> 24) & 0xFF
    if main == 4:            # AUTO 用子模式命名更准确
        mode_name = PX4_AUTO_SUB_MODE.get(sub, f"AUTO?{sub}")
    elif main == 6:
        mode_name = "OFFBOARD"
    else:
        mode_name = PX4_MAIN_MODE.get(main, f"MODE?{main}")
    return {
        "custom_mode": cm,
        "main_mode": main,
        "sub_mode": sub,
        "mode_name": mode_name,
        "type": mav_type,
        "autopilot": autopilot,
        "base_mode": base_mode,
        "armed": bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED),
        "system_status": system_status,
        "system_status_name": MAV_STATE.get(system_status, f"?{system_status}"),
        "mavlink_version": mavlink_version,
    }


def decode_local_position_ned(payload: bytes) -> dict:
    """NED：z 向下为正，所以**高度 = -z**。"""
    t, x, y, z, vx, vy, vz = struct.unpack("<Iffffff", payload[:28])
    return {"time_boot_ms": t, "x": x, "y": y, "z": z,
            "vx": vx, "vy": vy, "vz": vz,
            "height": -z, "up_velocity": -vz}


def decode_attitude(payload: bytes) -> dict:
    t, roll, pitch, yaw, rs, ps, ys = struct.unpack("<Iffffff", payload[:28])
    return {"time_boot_ms": t, "roll": roll, "pitch": pitch, "yaw": yaw,
            "rollspeed": rs, "pitchspeed": ps, "yawspeed": ys}


def decode_extended_sys_state(payload: bytes) -> dict:
    vtol_state, landed_state = struct.unpack("<BB", payload[:2])
    return {"vtol_state": vtol_state, "landed_state": landed_state,
            "landed_state_name": LANDED_STATE.get(landed_state, f"?{landed_state}"),
            "landed": landed_state in (1, 4),      # ON_GROUND / LANDING
            "airborne": landed_state in (2, 3)}    # IN_AIR / TAKEOFF


def decode_command_ack(payload: bytes) -> dict:
    command, result = struct.unpack("<HB", payload[:3])
    progress = payload[3] if len(payload) > 3 else 0xFF
    result_param2 = struct.unpack("<i", payload[4:8])[0] if len(payload) >= 8 else 0
    return {"command": command, "result": result,
            "result_name": MAV_RESULT.get(result, f"?{result}"),
            "progress": progress, "result_param2": result_param2}


def decode_param_value(payload: bytes) -> dict:
    """★ PX4 把整数参数的位模式当 float 传，必须按 param_type 还原。"""
    raw, count, index = struct.unpack("<fHH", payload[:8])
    pid = payload[8:24].split(b"\x00")[0].decode("ascii", errors="replace")
    ptype = payload[24] if len(payload) > 24 else 9
    value = raw
    fmt = _INT_FMT.get(ptype)
    if fmt is not None:
        packed = struct.pack("<f", raw)
        value = struct.unpack(fmt, packed[:struct.calcsize(fmt)])[0]
    return {"name": pid, "value": value, "raw_float": raw, "count": count,
            "index": index, "param_type": ptype,
            "param_type_name": MAV_PARAM_TYPE.get(ptype, f"?{ptype}")}


def decode_statustext(payload: bytes) -> dict:
    severity = payload[0] if payload else 0
    text = payload[1:51].split(b"\x00")[0].decode("utf-8", errors="replace")
    return {"severity": severity, "text": text}


def decode_estimator_status(payload: bytes) -> dict:
    """只取我们关心的：各测试比 + flags 位图。

    字段（升序）：time_usec u64, 8×f32, flags u16 —— 共 42 字节。
    flags 的位含义见 MAVLink 规范；本项目主要看 bit2/bit3（水平位置）。
    """
    if len(payload) < 42:
        return {}
    vals = struct.unpack("<Q8fH", payload[:42])
    flags = vals[9]
    return {
        "time_usec": vals[0],
        "vel_ratio": vals[1], "pos_horiz_ratio": vals[2], "pos_vert_ratio": vals[3],
        "mag_ratio": vals[4], "hagl_ratio": vals[5], "tas_ratio": vals[6],
        "pos_horiz_accuracy": vals[7], "pos_vert_accuracy": vals[8],
        "flags": flags,
        "attitude_ok": bool(flags & (1 << 0)),
        "vel_horiz_ok": bool(flags & (1 << 1)),
        "vel_vert_ok": bool(flags & (1 << 2)),
        "pos_horiz_abs_ok": bool(flags & (1 << 3)),
        "pos_vert_abs_ok": bool(flags & (1 << 4)),
        "pos_horiz_rel_ok": bool(flags & (1 << 5)),
        "pos_vert_rel_ok": bool(flags & (1 << 6)),
    }
