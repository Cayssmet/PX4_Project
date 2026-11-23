"""伪串口闭环：CAR_STATUS、START、C点减速完成、MISSION_SYNC和500ms超时。"""
import os
import pty
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from uav_interfaces.msg import CarStatus, MissionSync

MASTER_FD = None
SLAVE_FD = None


def make_frame(message_type, sequence, session, payload):
    frame = bytearray([0xDA, 0x7E, 0x01, message_type, sequence, len(payload)])
    frame += int(session).to_bytes(2, "big")
    frame += payload
    frame.append(sum(frame[2:]) & 0xFF)
    frame += b"\x0D\x0A"
    return bytes(frame)


def extract_frame(buffer, message_type):
    while len(buffer) >= 2:
        start = buffer.find(b"\xDA\x7E")
        if start < 0:
            return None, buffer[-1:]
        buffer = buffer[start:]
        if len(buffer) < 8:
            return None, buffer
        total = 11 + buffer[5]
        if len(buffer) < total:
            return None, buffer
        frame, buffer = buffer[:total], buffer[total:]
        if frame[-2:] != b"\x0D\x0A" or frame[-3] != (sum(frame[2:-3]) & 0xFF):
            buffer = frame[1:] + buffer
            continue
        if frame[3] == message_type:
            return frame, buffer
    return None, buffer


def car_payload(phase, state, checkpoint, flags, countdown=0, speed=200,
                target_speed=150, distance=0):
    payload = bytearray([1, phase, state, checkpoint, 0])
    payload += int(flags).to_bytes(2, "big")
    payload += int(countdown).to_bytes(2, "big")
    payload += int(speed).to_bytes(2, "big")
    payload += int(target_speed).to_bytes(2, "big")
    payload += int(distance).to_bytes(4, "big")
    return payload


@pytest.mark.launch_test
def generate_test_description():
    global MASTER_FD, SLAVE_FD
    MASTER_FD, SLAVE_FD = pty.openpty()
    os.set_blocking(MASTER_FD, False)
    node = launch_ros.actions.Node(
        package="car_serial_bridge",
        executable="car_serial_bridge_node",
        name="car_serial_bridge",
        output="screen",
        parameters=[{
            "serial_device": os.ttyname(SLAVE_FD),
            "baud_rate": 115200,
            "sync_rate_hz": 20.0,
            "status_timeout_ms": 500,
            "reconnect_interval_ms": 50,
            "flight_status_timeout_ms": 500,
            "print_status": False,
        }],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {"bridge": node}


class TestCarBridge(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("car_bridge_integration_test")
        retained = QoSProfile(depth=1)
        retained.reliability = ReliabilityPolicy.RELIABLE
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cls.phase_pub = cls.node.create_publisher(String, "/competition/phase", retained)
        cls.status = None
        cls.sync = None
        cls.start = None
        cls.drop_window = None
        cls.link = None
        cls.subscriptions = [
            cls.node.create_subscription(
                CarStatus, "/car/status", lambda m: setattr(cls, "status", m), retained),
            cls.node.create_subscription(
                MissionSync, "/car/mission_sync", lambda m: setattr(cls, "sync", m), retained),
            cls.node.create_subscription(
                Bool, "/competition/start", lambda m: setattr(cls, "start", m.data), 10),
            cls.node.create_subscription(
                Bool, "/car/drop_window", lambda m: setattr(cls, "drop_window", m.data), 10),
            cls.node.create_subscription(
                Bool, "/car/link_ok", lambda m: setattr(cls, "link", m.data), retained),
        ]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        os.close(MASTER_FD)
        os.close(SLAVE_FD)

    def spin_until(self, condition, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if condition():
                return
        self.fail("等待测试条件超时")

    def test_car_start_c_anchor_sync_and_timeout(self):
        # 倒计时帧选择task1，但不得提前发布START。
        os.write(MASTER_FD, make_frame(
            0x10, 1, 1, car_payload(0x01, 0x01, 1, 0, countdown=2500, speed=0)))
        self.spin_until(lambda: self.status is not None and self.status.sequence == 1)
        self.assertTrue(self.status.communication_ok)
        self.assertEqual(self.status.task_mode, 1)
        self.assertNotEqual(self.start, True)

        # C点已过且减速完成，START与动作窗口都应成立。
        flags = 0x0001 | 0x0002 | 0x0040 | 0x0080
        c_frame = make_frame(
            0x10, 2, 1, car_payload(0x07, 0x04, 3, flags, distance=320))
        os.write(MASTER_FD, c_frame)
        self.spin_until(
            lambda: self.status is not None and self.status.sequence == 2
            and self.start is True and self.drop_window is True)
        self.assertEqual(self.status.last_checkpoint, 3)
        self.assertEqual(self.status.distance_since_checkpoint_mm, 320)

        phase = String()
        phase.data = "TASK1_ALIGN_DROP"
        # ROS阶段和CAR_STATUS在实机上都会周期播报；测试同样重复发送，
        # 避免伪串口/调度抖动被误判成协议失败。重复sequence按协议允许。
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.phase_pub.publish(phase)
            os.write(MASTER_FD, c_frame)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (self.sync is not None and self.sync.mission_phase == 0x10
                    and (self.sync.coordination_flags & 0x0003) == 0x0003):
                break
        else:
            self.fail("等待MISSION_SYNC阶段同步超时")
        self.assertEqual(self.sync.session_id, 1)
        self.assertEqual(self.sync.task_mode, 1)

        serial_buffer = b""
        wire_sync = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and wire_sync is None:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            try:
                serial_buffer += os.read(MASTER_FD, 4096)
            except BlockingIOError:
                pass
            wire_sync, serial_buffer = extract_frame(serial_buffer, 0x11)
            if wire_sync is not None and wire_sync[9] != 0x10:
                wire_sync = None
        self.assertIsNotNone(wire_sync)
        self.assertEqual(len(wire_sync), 23)
        self.assertEqual(int.from_bytes(wire_sync[6:8], "big"), 1)
        self.assertEqual(int.from_bytes(wire_sync[14:16], "big") & 0x0003, 0x0003)

        self.spin_until(lambda: self.link is False and self.drop_window is False, timeout=1.5)
        self.assertFalse(self.link)
        self.assertFalse(self.drop_window)

        # 简化task2在车顶稳定5秒后直接完成：仍应报告“落在车上”，
        # 并同时锁存车顶稳定完成和任务完成，不能误报为H点降落。
        phase.data = "TASK2_COMPLETE"
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.phase_pub.publish(phase)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.sync is not None and self.sync.mission_phase == 0x27:
                break
        else:
            self.fail("等待TASK2_COMPLETE同步超时")
        self.assertEqual(self.sync.uav_state, 0x06)
        self.assertNotEqual(self.sync.sync_flags & MissionSync.SYNC_TASK_COMPLETE, 0)
        self.assertNotEqual(
            self.sync.coordination_flags & MissionSync.LANDED_ON_CAR, 0)
        self.assertNotEqual(
            self.sync.coordination_flags & MissionSync.HOLD_5S_COMPLETE, 0)
        self.assertNotEqual(
            self.sync.coordination_flags & MissionSync.TASK_COMPLETE, 0)


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
