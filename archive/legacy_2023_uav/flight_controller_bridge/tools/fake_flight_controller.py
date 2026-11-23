#!/usr/bin/env python3
"""伪飞控：用 Linux 伪终端接收命令帧并返回状态帧，严禁把它用于真实飞行。"""

import argparse
import math
import os
import pty
import select
import signal
import sys
import time


RUNNING = True


def crc16_ccitt_false(data):
    """与树莓派桥和飞控端完全相同的 CRC16-CCITT-FALSE。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def read_i16_be(data):
    raw = (data[0] << 8) | data[1]
    return raw - 0x10000 if raw & 0x8000 else raw


def extract_frame(buffer):
    """寻找一帧合法的 16 字节树莓派命令；坏帧只丢一个字节后重同步。"""
    while len(buffer) >= 2:
        start = buffer.find(b"\xAA\x55")
        if start < 0:
            return None, buffer[-1:]
        buffer = buffer[start:]
        if len(buffer) < 16:
            return None, buffer
        frame = buffer[:16]
        if frame[14:16] != b"\x0D\x0A":
            buffer = buffer[1:]
            continue
        expected = crc16_ccitt_false(frame[2:12])
        received = (frame[12] << 8) | frame[13]
        if expected != received:
            buffer = buffer[1:]
            continue
        return frame, buffer[16:]
    return None, buffer


def make_status(sequence, height_reached, land_accepted, land_completed, failsafe):
    flags = 0
    flags |= 0x01 if height_reached else 0
    flags |= 0x02 if land_accepted else 0
    flags |= 0x04 if land_completed else 0
    flags |= 0x08 if failsafe else 0
    payload = bytes([sequence, flags])
    crc = crc16_ccitt_false(payload)
    return bytes([0x55, 0xAA, sequence, flags, crc >> 8, crc & 0xFF, 0x0D, 0x0A])


def stop(_signum, _frame):
    global RUNNING
    RUNNING = False


def main():
    parser = argparse.ArgumentParser(description="flight_controller_bridge 串口伪飞控")
    parser.add_argument(
        "--height-delay", type=float, default=0.5,
        help="普通有效命令持续多久后报告高度到达，默认 0.5 秒",
    )
    parser.add_argument(
        "--land-delay", type=float, default=2.0,
        help="接受降落后多久报告降落完成，默认 2 秒",
    )
    parser.add_argument(
        "--failsafe", action="store_true",
        help="在状态帧中持续置 failsafe，验证树莓派安全门",
    )
    args = parser.parse_args()

    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    os.set_blocking(master_fd, False)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    print("=" * 68)
    print("伪飞控已启动（只能用于无桨软件测试）")
    print(f"虚拟串口从端：{slave_path}")
    print("另开终端运行：")
    print(
        "ros2 run flight_controller_bridge flight_controller_bridge_node "
        f"--ros-args -p serial_device:={slave_path} -p print_status:=true"
    )
    print("=" * 68, flush=True)

    receive_buffer = b""
    normal_started = None
    land_started = None
    last_print = 0.0

    try:
        while RUNNING:
            readable, _, _ = select.select([master_fd], [], [], 0.1)
            if not readable:
                continue
            try:
                receive_buffer += os.read(master_fd, 4096)
            except BlockingIOError:
                continue

            while True:
                frame, receive_buffer = extract_frame(receive_buffer)
                if frame is None:
                    break

                now = time.monotonic()
                sequence = frame[2]
                control_valid = bool(frame[3] & 0x01)
                land_request = bool(frame[3] & 0x02)
                vx = read_i16_be(frame[4:6]) / 1000.0
                vy = read_i16_be(frame[6:8]) / 1000.0
                yaw_rate = read_i16_be(frame[8:10]) / 1000.0
                target_height = ((frame[10] << 8) | frame[11]) / 1000.0

                if control_valid and not land_request:
                    normal_started = normal_started or now
                else:
                    normal_started = None
                if control_valid and land_request:
                    land_started = land_started or now

                height_reached = (
                    normal_started is not None
                    and now - normal_started >= max(0.0, args.height_delay)
                )
                land_accepted = land_started is not None
                land_completed = (
                    land_started is not None
                    and now - land_started >= max(0.0, args.land_delay)
                )

                os.write(
                    master_fd,
                    make_status(
                        sequence,
                        height_reached,
                        land_accepted,
                        land_completed,
                        args.failsafe,
                    ),
                )

                if now - last_print >= 0.5:
                    print(
                        f"seq={sequence:3d} valid={control_valid} land={land_request} | "
                        f"vx={vx:+.3f} vy={vy:+.3f} m/s "
                        f"yaw={yaw_rate:+.3f} rad/s height={target_height:.3f} m | "
                        f"height_ok={height_reached} land_done={land_completed}",
                        flush=True,
                    )
                    last_print = now
    finally:
        os.close(master_fd)
        os.close(slave_fd)
        print("伪飞控已退出。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
