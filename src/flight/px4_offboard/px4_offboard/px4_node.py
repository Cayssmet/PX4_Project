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
import os
import sys
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
        read_errors = 0

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
            except Exception as exc:  # noqa: BLE001
                # 写失败才是真掉线
                self.get_logger().warn(f"串口写失败，重连：{exc}")
                try:
                    ser.close()
                except Exception:  # noqa: BLE001
                    pass
                ser = None
                with self._lock:
                    self._serial_ok = False
                time.sleep(self._reconnect_s)
                continue

            # ---- 接收 ----
            # ★ pyserial 在「select 说可读但 read 返回 0 字节」时抛
            # SerialException("device reports readiness to read but returned no data ...")。
            # 这在 CH340 + Linux 上偶发，**不是掉线**；早期版本一见异常就 close/reopen，
            # 结果每秒重连几十次、每次丢帧，状态永远判不新鲜。现在按瞬时错误处理：
            # 少量连续出现只睡一下继续读，连续 25 次以上才认定真掉线。
            try:
                chunk = ser.read(4096)
                read_errors = 0
            except Exception as exc:  # noqa: BLE001
                read_errors += 1
                msg = str(exc)
                transient = ("returned no data" in msg) or ("device disconnected" in msg)
                if transient and read_errors < 25:
                    if read_errors == 1:
                        self.get_logger().debug(f"串口空读（瞬时，忽略）：{exc}")
                    time.sleep(0.004)
                    continue
                self.get_logger().warn(
                    f"串口连续读失败 {read_errors} 次，重连：{exc}")
                try:
                    ser.close()
                except Exception:  # noqa: BLE001
                    pass
                ser = None
                read_errors = 0
                with self._lock:
                    self._serial_ok = False
                time.sleep(0.2)
                continue

            if chunk:
                with self._lock:
                    self._bytes_in += len(chunk)
                for msgid, payload, crc_ok in parser.feed(chunk):
                    if crc_ok is False:
                        continue
                    self._dispatch(msgid, payload)

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
                # ★ 每次点名（REQUEST_MESSAGE=512）飞控都会回一条 ACK，
                # 25 Hz 位置 + 10 Hz 姿态就是每秒 40+ 条，全打出来会把日志淹掉。
                # 规则：512 的 ACCEPTED 不打；其它命令与非 ACCEPTED 一律打。
                routine = (ack["command"] == 512 and ack["result"] == 0)
                if self._verbose and not routine:
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

        # ★ 心跳是 2 Hz 点名的，周期 500 ms，若还用 500 ms 判新鲜会长期在边界抖动
        #   （正好差几毫秒就判死）。给它 3 个周期的宽限。
        hb_tol = max(self._stale_s, 3.0 * self._heartbeat_interval) \
            if self._heartbeat_interval > 0 else self._stale_s
        hb_fresh = hb_age < hb_tol
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


def _acquire_single_instance_lock(dev: str):
    """同一个串口只允许一个 px4_node 打开。

    ★ 为什么必须做这个：现场踩过两次同类事故（v3.0.0 时两个 MicroXRCEAgent、
    v3.1.0 时两个 px4_node）。两个进程同时读同一个 tty，字节流被劈成两半、
    写方向又互相交错，现象是「串口能打开、ACK 偶尔能收到、但状态永远不新鲜」，
    极难从日志看出根因。用 flock 在进程启动时就把它挡掉。

    返回 (fd, path)；fd 为 None 表示拿不到锁（已在别处运行）。
    """
    try:
        import fcntl
    except ImportError:
        return None, None            # 非 Linux，跳过（Windows 侧只跑单元测试）

    slug = dev.strip("/").replace("/", "_") or "default"
    path = f"/tmp/px4_offboard_{slug}.lock"
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        return None, path
    os.truncate(fd, 0)
    os.write(fd, str(os.getpid()).encode())
    return fd, path


def main(args=None):
    rclpy.init(args=args)

    # 先拿单实例锁，再建节点（建节点就会开串口）
    dev = "/dev/flight_controller"
    for i, a in enumerate(sys.argv):
        if a.startswith("--ros-args"):
            break
        if a in ("--dev", "-d") and i + 1 < len(sys.argv):
            dev = sys.argv[i + 1]
    lock_fd, lock_path = _acquire_single_instance_lock(dev)

    node = Px4Node()
    if lock_fd is None and lock_path:
        node.get_logger().fatal(
            f"已有一个 px4_node 在占用 {dev}（锁文件 {lock_path}）。"
            f" 两个进程同时读同一串口会让数据流劈半、现象极难排查 —— 本进程退出。"
            f" 清理：pkill -f lib/px4_offboard/px4_node")
        node.destroy_node()
        rclpy.try_shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        if lock_fd is not None:
            try:
                os.close(lock_fd)
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    main()
