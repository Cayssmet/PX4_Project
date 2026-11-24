# -*- coding: utf-8 -*-
"""px4_node.py —— 树莓派 ↔ PX4 的 MAVLink 桥（第一阶段：只读）。

这一版**只做下行**：定时点名拉几条约定的消息，把它们整理成
`/flight/status`（`uav_interfaces/msg/FlightStatus`）发布出去。
**不发任何控制量**，所以可以放心在飞机通电、装桨的状态下跑。

为什么用 MAVLink 而不是 uXRCE-DDS
--------------------------------
实测（v3.0.0 那套 DDS 方案）飞控固定向外灌 27 条话题、约 35 KB/s，
而 CH340 这条线在 115200 只有 11.5 KB/s、460800 直接 0 数据 —— 线路被撑爆。
MAVLink 只发几条基础消息，其余按需点名，实测 **6.5 KB/s 就拿到 25 Hz 位置**。

★ MAVLink 没有 `OffboardControlMode`
------------------------------------
DDS 路线上要另发 `OffboardControlMode` 当心跳。MAVLink 路线上**设定点流本身就是心跳**：
`SET_POSITION_TARGET_LOCAL_NED` 以 ≥2 Hz 持续发，PX4 才认为外部控制在线；
停发超过 `COM_OF_LOSS_T`（默认 1 s）就退出 offboard。第二阶段写上行时记住这点。

线程模型
--------
rclpy 的默认定时器不适合「阻塞读串口」，所以串口读写放在一个独立线程里，
共享状态用锁保护；ROS 侧只用定时器发 `/flight/status`。
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from uav_interfaces.msg import FlightStatus

from . import mavlink_codec as mc


# 需要定时点名的消息：msgid -> 参数名后缀
POLL_TARGETS = {
    mc.MSG_LOCAL_POSITION_NED: "local_position_ned_hz",
    mc.MSG_ATTITUDE: "attitude_hz",
    mc.MSG_EXTENDED_SYS_STATE: "extended_sys_state_hz",
    mc.MSG_ESTIMATOR_STATUS: "estimator_status_hz",
}


class Px4Node(Node):

    def __init__(self):
        super().__init__("px4_node")

        # ---------------- 参数 ----------------
        self.declare_parameter("serial_device", "/dev/flight_controller")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("heartbeat_hz", 2.0)
        self.declare_parameter("local_position_ned_hz", 25.0)
        self.declare_parameter("attitude_hz", 10.0)
        self.declare_parameter("extended_sys_state_hz", 2.0)
        self.declare_parameter("estimator_status_hz", 2.0)
        self.declare_parameter("status_publish_hz", 10.0)
        self.declare_parameter("stale_timeout_ms", 500.0)
        self.declare_parameter("reconnect_interval_ms", 2000)
        self.declare_parameter("print_status", True)

        p = self.get_parameter
        self._dev = p("serial_device").value
        self._baud = int(p("baud").value)
        self._status_hz = float(p("status_publish_hz").value)
        self._stale_s = float(p("stale_timeout_ms").value) / 1000.0
        self._reconnect_s = float(p("reconnect_interval_ms").value) / 1000.0
        self._verbose = bool(p("print_status").value)

        self._poll_interval = {}
        for msgid, name in POLL_TARGETS.items():
            hz = float(p(name).value)
            if hz > 0.0:
                self._poll_interval[msgid] = 1.0 / hz
        self._heartbeat_interval = (1.0 / float(p("heartbeat_hz").value)
                                    if float(p("heartbeat_hz").value) > 0 else 0.0)

        # ---------------- 状态 ----------------
        self._lock = threading.Lock()
        self._hb = None
        self._hb_t = 0.0
        self._lpn = None
        self._lpn_t = 0.0
        self._att = None
        self._att_t = 0.0
        self._ext = None
        self._ext_t = 0.0
        self._est = None
        self._est_t = 0.0
        self._serial_ok = False
        self._bytes_in = 0
        self._bytes_out = 0
        self._acks = 0
        self._texts = []

        # ---------------- ROS 接口 ----------------
        self._status_pub = self.create_publisher(FlightStatus, "/flight/status", 10)

        # 阶段二会用到的订阅（现在先建好但不发上行，便于观察数据是否到位）
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        qos.history = HistoryPolicy.KEEP_LAST

        self._thread = threading.Thread(target=self._io_loop, daemon=True)
        self._thread.start()

        self.create_timer(1.0 / max(self._status_hz, 1.0), self._publish_status)
        self.get_logger().info(
            f"px4_node 启动（只读模式）：{self._dev} @ {self._baud}，"
            f"点名 {sorted(self._poll_interval)}，状态 {self._status_hz:.0f} Hz")

    # ------------------------------------------------------------------
    # 串口线程
    # ------------------------------------------------------------------

    def _io_loop(self):
        try:
            import serial  # pyserial
        except ImportError:
            self.get_logger().error("缺少 pyserial：sudo apt install python3-serial")
            return

        enc = mc.MavlinkEncoder()
        parser = mc.MavlinkParser()
        next_due = {mid: 0.0 for mid in self._poll_interval}
        next_hb = 0.0
        ser = None
        warned = False

        while rclpy.ok():
            # ---- 连接（掉线自动重连）----
            if ser is None:
                try:
                    ser = serial.Serial(self._dev, self._baud, timeout=0.01,
                                        write_timeout=0.2)
                    time.sleep(0.05)
                    with self._lock:
                        self._serial_ok = True
                    self.get_logger().info(f"串口已打开：{self._dev} @ {self._baud}")
                    warned = False
                except Exception as exc:  # noqa: BLE001
                    if not warned:
                        self.get_logger().warn(
                            f"打不开 {self._dev}（{exc}），{self._reconnect_s:.0f} 秒后重试。"
                            f" 常见原因：micro-xrce-agent 还占着它 / 设备名不对 / 不在 dialout 组")
                        warned = True
                    time.sleep(self._reconnect_s)
                    continue

            now = time.monotonic()

            # ---- 发送：心跳 + 定时点名 ----
            try:
                if self._heartbeat_interval > 0.0 and now >= next_hb:
                    self._write(ser, enc.heartbeat())
                    next_hb = now + self._heartbeat_interval
                for msgid, interval in self._poll_interval.items():
                    if now >= next_due[msgid]:
                        self._write(ser, enc.request_message(msgid))
                        # 防止一次循环里补发一堆：若落后超过 3 个周期就重新对齐
                        next_due[msgid] = now + interval if next_due[msgid] + 3 * interval < now \
                            else next_due[msgid] + interval

                # ---- 接收 ----
                chunk = ser.read(4096)
                if chunk:
                    with self._lock:
                        self._bytes_in += len(chunk)
                    for msgid, payload, crc_ok in parser.feed(chunk):
                        if crc_ok is False:
                            continue
                        self._dispatch(msgid, payload)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"串口异常，重连：{exc}")
                try:
                    ser.close()
                except Exception:  # noqa: BLE001
                    pass
                ser = None
                with self._lock:
                    self._serial_ok = False

        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

    def _write(self, ser, data: bytes):
        ser.write(data)
        with self._lock:
            self._bytes_out += len(data)

    def _dispatch(self, msgid: int, payload: bytes):
        now = time.monotonic()
        with self._lock:
            if msgid == mc.MSG_HEARTBEAT:
                self._hb = mc.decode_heartbeat(payload)
                self._hb_t = now
            elif msgid == mc.MSG_LOCAL_POSITION_NED:
                self._lpn = mc.decode_local_position_ned(payload)
                self._lpn_t = now
            elif msgid == mc.MSG_ATTITUDE:
                self._att = mc.decode_attitude(payload)
                self._att_t = now
            elif msgid == mc.MSG_EXTENDED_SYS_STATE:
                self._ext = mc.decode_extended_sys_state(payload)
                self._ext_t = now
            elif msgid == mc.MSG_ESTIMATOR_STATUS:
                self._est = mc.decode_estimator_status(payload)
                self._est_t = now
            elif msgid == mc.MSG_COMMAND_ACK:
                ack = mc.decode_command_ack(payload)
                self._acks += 1
                if self._verbose:
                    self.get_logger().info(
                        f"ACK cmd={ack['command']} {ack['result_name']}")
            elif msgid == mc.MSG_STATUSTEXT:
                txt = mc.decode_statustext(payload)
                if txt["text"]:
                    self._texts.append(txt["text"])
                    del self._texts[:-5]
                    self.get_logger().warn(f"[飞控] {txt['text']}")

    # ------------------------------------------------------------------
    # 发布
    # ------------------------------------------------------------------

    def _publish_status(self):
        now = time.monotonic()
        with self._lock:
            hb, hb_age = self._hb, now - self._hb_t if self._hb_t else 1e9
            lpn, lpn_age = self._lpn, now - self._lpn_t if self._lpn_t else 1e9
            ext = self._ext
            serial_ok = self._serial_ok

        msg = FlightStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "px4"

        hb_fresh = hb_age < self._stale_s
        lpn_fresh = lpn_age < self._stale_s
        msg.communication_ok = bool(serial_ok and hb_fresh and lpn_fresh)

        if hb is not None:
            msg.armed = hb["armed"]
            msg.current_mode = hb["main_mode"]
            msg.failsafe = hb["system_status"] in (5, 6, 8)   # CRITICAL/EMERGENCY/TERMINATION
        else:
            msg.armed = False
            msg.current_mode = 0
            msg.failsafe = True

        if ext is not None:
            msg.landed = ext["landed"]
            msg.airborne = ext["airborne"]
        else:
            msg.landed = False
            msg.airborne = False

        if lpn is not None:
            msg.current_height = float(lpn["height"])
            msg.vertical_velocity = float(lpn["up_velocity"])
            msg.height_valid = lpn_fresh
            msg.height_age_ms = int(min(lpn_age, 60.0) * 1000.0)
        else:
            msg.height_valid = False
            msg.height_age_ms = 60000

        # 本项目还没有「飞控回执」这一套，先按最保守的值填
        msg.protocol_version = FlightStatus.PROTOCOL_V2
        msg.action_result = FlightStatus.ACTION_NONE
        msg.action_accepted = False
        msg.action_completed = False
        msg.land_accepted = False
        msg.land_completed = False
        msg.height_reached = False
        msg.error_code = 0
        msg.active_session_id = 0
        msg.active_action_id = 0
        msg.command_age_ms = 0

        self._status_pub.publish(msg)

    # ------------------------------------------------------------------

    def snapshot(self) -> dict:
        """给调试/日志用的一份快照（不含锁外引用）。"""
        now = time.monotonic()
        with self._lock:
            return {
                "serial_ok": self._serial_ok,
                "bytes_in": self._bytes_in,
                "bytes_out": self._bytes_out,
                "acks": self._acks,
                "heartbeat_age_s": (now - self._hb_t) if self._hb_t else None,
                "lpn_age_s": (now - self._lpn_t) if self._lpn_t else None,
                "hb": dict(self._hb) if self._hb else None,
                "lpn": dict(self._lpn) if self._lpn else None,
            }


def main(args=None):
    rclpy.init(args=args)
    node = Px4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
