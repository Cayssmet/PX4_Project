#!/usr/bin/env python3
"""D-UAV UART V2.0伪飞控，只用于无桨协议联调。"""

import argparse
import os
import pty
import select
import signal
import sys
import time


RUNNING = True


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF
                if crc & 0x8000
                else (crc << 1) & 0xFFFF
            )
    return crc


def read_i16(data):
    return int.from_bytes(data, "big", signed=True)


def extract_frame(buffer):
    while len(buffer) >= 2:
        start = buffer.find(b"\xA5\x5A")
        if start < 0:
            return None, buffer[-1:]
        buffer = buffer[start:]
        if len(buffer) < 30:
            return None, buffer
        frame = buffer[:30]
        if (
            frame[2] != 2
            or frame[3] != 1
            or frame[7] != 0
            or frame[6] & 0xFE
            or frame[28:30] != b"\x0D\x0A"
            or crc16_ccitt_false(frame[2:26]) != int.from_bytes(frame[26:28], "big")
        ):
            buffer = buffer[1:]
            continue
        return frame, buffer[30:]
    return None, buffer


def sequence_relation(last_sequence, new_sequence):
    if last_sequence is None:
        return "new"
    delta = (new_sequence - last_sequence) & 0xFF
    if delta == 0:
        return "duplicate"
    return "new" if delta <= 127 else "old"


def make_status(state, sequence):
    flags = 0
    flags |= 0x01 if state["height_valid"] else 0
    flags |= 0x02 if state["height_reached"] else 0
    flags |= 0x04 if state["armed"] else 0
    flags |= 0x08 if state["airborne"] else 0
    flags |= 0x10 if state["landed"] else 0
    flags |= 0x20 if state["action_result"] in (1, 2, 4) else 0
    flags |= 0x40 if state["action_result"] == 2 else 0
    flags |= 0x80 if state["failsafe"] else 0

    payload = bytearray(24)
    payload[0] = 2
    payload[1] = 2
    payload[2] = sequence
    payload[3] = state["mode"]
    payload[4] = flags
    payload[5] = state["error_code"]
    payload[6:10] = state["session_id"].to_bytes(4, "big")
    payload[10:14] = state["action_id"].to_bytes(4, "big")
    payload[14:16] = state["height_mm"].to_bytes(2, "big")
    payload[16:18] = state["vertical_velocity"].to_bytes(2, "big", signed=True)
    payload[18:20] = (10 if state["height_valid"] else 0xFFFF).to_bytes(2, "big")
    payload[20:22] = (0).to_bytes(2, "big")
    payload[22] = state["action_result"]
    payload[23] = 0
    crc = crc16_ccitt_false(payload)
    return bytes([0x5A, 0xA5]) + bytes(payload) + crc.to_bytes(2, "big") + b"\x0D\x0A"


def initial_state(force_failsafe=False):
    return {
        "mode": 0,
        "session_id": 0,
        "action_id": 0,
        "last_action_id": 0,
        "action_key": None,
        "action_result": 0,
        "action_started": None,
        "height_valid": True,
        "height_reached": False,
        "height_mm": 0,
        "vertical_velocity": 0,
        "armed": False,
        "airborne": False,
        "landed": True,
        "failsafe": force_failsafe,
        "error_code": 0,
    }


def update_timed_action(state, now, height_delay, land_delay):
    if state["action_result"] != 1 or state["action_started"] is None:
        return
    elapsed = now - state["action_started"]
    if state["action_key"][0] == 1 and elapsed >= height_delay:
        state["height_mm"] = state["action_key"][1]
        state["height_reached"] = True
        state["armed"] = True
        state["airborne"] = True
        state["landed"] = False
        state["mode"] = 2
        state["action_result"] = 2
    elif state["action_key"][0] == 4 and elapsed >= land_delay:
        state["height_mm"] = 0
        state["height_reached"] = False
        state["armed"] = False
        state["airborne"] = False
        state["landed"] = True
        state["action_result"] = 2


def process_command(state, frame, now):
    mode = frame[5]
    control_valid = bool(frame[6] & 1)
    session_id = int.from_bytes(frame[8:12], "big")
    action_id = int.from_bytes(frame[12:16], "big")
    target_height = int.from_bytes(frame[24:26], "big")
    key = (mode, target_height)
    state["error_code"] = 0

    if not control_valid:
        state["vertical_velocity"] = 0
        return
    if mode > 5:
        state["error_code"] = 0x08
        return
    if session_id == 0:
        state["error_code"] = 0x09
        return
    if state["session_id"] == 0 and mode == 0 and action_id == 0:
        state["session_id"] = session_id
        state["mode"] = 0
        return
    if session_id != state["session_id"]:
        state["error_code"] = 0x09
        return

    if mode in (0, 2, 3):
        if action_id != 0:
            state["error_code"] = 0x0B
            return
        state["mode"] = mode
        if mode == 2:
            state["height_mm"] = target_height
            state["height_reached"] = True
            state["vertical_velocity"] = 0
        elif mode == 3:
            state["vertical_velocity"] = read_i16(frame[20:22])
            state["height_mm"] = max(target_height, state["height_mm"] + state["vertical_velocity"] // 50)
        else:
            state["vertical_velocity"] = 0
        return

    if action_id == 0:
        state["error_code"] = 0x0B
        return
    if action_id < state["last_action_id"]:
        state["error_code"] = 0x11
        return
    if action_id == state["last_action_id"]:
        if key != state["action_key"]:
            state["error_code"] = 0x0A
        return

    state["last_action_id"] = action_id
    state["action_id"] = action_id
    state["action_key"] = key
    state["action_started"] = now
    if mode == 1:
        state["mode"] = 1
        state["action_result"] = 1
        state["armed"] = True
        state["landed"] = False
    elif mode == 4:
        state["mode"] = 4
        state["action_result"] = 1
    elif mode == 5:
        if not state["landed"]:
            state["action_result"] = 3
            state["error_code"] = 0x0E
        else:
            state["mode"] = 0
            state["action_result"] = 2
            state["action_started"] = None


def stop(_signum, _frame):
    global RUNNING
    RUNNING = False


def main():
    parser = argparse.ArgumentParser(description="D-UAV UART V2.0伪飞控")
    parser.add_argument("--takeoff-delay", type=float, default=0.8)
    parser.add_argument("--land-delay", type=float, default=1.5)
    parser.add_argument("--failsafe", action="store_true")
    args = parser.parse_args()

    master_fd, slave_fd = pty.openpty()
    os.set_blocking(master_fd, False)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    slave_path = os.ttyname(slave_fd)
    print("V2伪飞控已启动（仅限无桨测试）")
    print(f"虚拟串口：{slave_path}")
    print(
        "ros2 run flight_controller_bridge flight_controller_bridge_node "
        f"--ros-args -p protocol_version:=2 -p serial_device:={slave_path}"
    )
    print("随后向 /flight/directive 发布STANDBY、TAKEOFF等指令。", flush=True)

    buffer = b""
    last_sequence = None
    state = initial_state(args.failsafe)
    last_print = 0.0
    try:
        while RUNNING:
            readable, _, _ = select.select([master_fd], [], [], 0.05)
            if not readable:
                continue
            try:
                buffer += os.read(master_fd, 4096)
            except BlockingIOError:
                continue
            while True:
                frame, buffer = extract_frame(buffer)
                if frame is None:
                    break
                sequence = frame[4]
                if sequence_relation(last_sequence, sequence) != "new":
                    continue
                last_sequence = sequence
                now = time.monotonic()
                update_timed_action(state, now, args.takeoff_delay, args.land_delay)
                process_command(state, frame, now)
                update_timed_action(state, now, args.takeoff_delay, args.land_delay)
                os.write(master_fd, make_status(state, sequence))
                if now - last_print >= 0.5:
                    print(
                        f"seq={sequence:3d} mode={frame[5]} valid={frame[6] & 1} "
                        f"session={int.from_bytes(frame[8:12], 'big')} "
                        f"action={int.from_bytes(frame[12:16], 'big')} -> "
                        f"mode={state['mode']} result={state['action_result']} "
                        f"height={state['height_mm']}mm error={state['error_code']}",
                        flush=True,
                    )
                    last_print = now
    finally:
        os.close(master_fd)
        os.close(slave_fd)
        print("V2伪飞控已退出。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
