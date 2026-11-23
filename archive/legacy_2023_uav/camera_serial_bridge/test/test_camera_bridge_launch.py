"""伪串口闭环：协议命令、毫米转米、标志位和200ms失效。"""
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
from geometry_msgs.msg import Vector3Stamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8

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
        if (frame[-2:] != b"\x0D\x0A" or
                frame[-3] != (sum(frame[2:-3]) & 0xFF)):
            buffer = frame[1:] + buffer
            continue
        if frame[3] == message_type:
            return frame, buffer
    return None, buffer


@pytest.mark.launch_test
def generate_test_description():
    global MASTER_FD, SLAVE_FD
    MASTER_FD, SLAVE_FD = pty.openpty()
    os.set_blocking(MASTER_FD, False)
    node = launch_ros.actions.Node(
        package="camera_serial_bridge",
        executable="camera_serial_bridge_node",
        name="camera_serial_bridge",
        output="screen",
        parameters=[{
            "serial_device": os.ttyname(SLAVE_FD),
            "baud_rate": 115200,
            "command_rate_hz": 20.0,
            "requested_camera_rate_hz": 20,
            "data_timeout_ms": 200,
            "reconnect_interval_ms": 50,
            "print_status": False,
        }],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {"bridge": node}


class TestCameraBridge(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("camera_bridge_integration_test")
        retained = QoSProfile(depth=1)
        retained.reliability = ReliabilityPolicy.RELIABLE
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cls.phase_pub = cls.node.create_publisher(String, "/competition/phase", retained)
        cls.task_pub = cls.node.create_publisher(UInt8, "/competition/active_task_mode", retained)
        cls.valid = None
        cls.link = None
        cls.error = None
        cls.drop_in_progress = None
        cls.drop_completed = None
        cls.drop_failed = None
        cls.subscriptions = [
            cls.node.create_subscription(Bool, "/vision/target_valid", lambda m: setattr(cls, "valid", m.data), 10),
            cls.node.create_subscription(Bool, "/vision/link_ok", lambda m: setattr(cls, "link", m.data), 10),
            cls.node.create_subscription(Vector3Stamped, "/vision/relative_error", lambda m: setattr(cls, "error", m), 10),
            cls.node.create_subscription(Bool, "/payload/drop_in_progress", lambda m: setattr(cls, "drop_in_progress", m.data), 10),
            cls.node.create_subscription(Bool, "/payload/drop_completed", lambda m: setattr(cls, "drop_completed", m.data), 10),
            cls.node.create_subscription(Bool, "/payload/drop_failed", lambda m: setattr(cls, "drop_failed", m.data), 10),
        ]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        os.close(MASTER_FD)
        os.close(SLAVE_FD)

    def test_command_data_and_timeout(self):
        task = UInt8()
        task.data = 1
        phase = String()
        phase.data = "FORWARD_CHASE"
        command = None
        serial_buffer = b""
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and command is None:
            self.task_pub.publish(task)
            self.phase_pub.publish(phase)
            rclpy.spin_once(self.node, timeout_sec=0.03)
            try:
                serial_buffer += os.read(MASTER_FD, 4096)
            except BlockingIOError:
                pass
            command, serial_buffer = extract_frame(serial_buffer, 0x20)
            if command is not None and (int.from_bytes(command[6:8], "big") != 1 or command[8] != 1):
                command = None
        self.assertIsNotNone(command, "未收到VISION_COMMAND")
        self.assertEqual(len(command), 19)
        self.assertEqual(int.from_bytes(command[6:8], "big"), 1)
        self.assertEqual(command[8], 1)       # enable
        self.assertEqual(command[9], 1)       # SEARCH
        self.assertEqual(command[10], 1)      # task1
        self.assertEqual(command[11], 0x05)   # SEARCH_CAR
        self.assertEqual(command[12], 1)      # 小车平台
        self.assertEqual(int.from_bytes(command[14:16], "big"), 20)

        payload = bytearray([0x05, 0x03, 0x01, 0x1B])
        payload += (1250).to_bytes(4, "big", signed=True)
        payload += (-350).to_bytes(4, "big", signed=True)
        os.write(MASTER_FD, make_frame(0x21, 7, 1, payload))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.valid is True and self.error is not None:
                break
        self.assertTrue(self.valid)
        self.assertTrue(self.link)
        self.assertEqual(self.error.header.frame_id, "base_link")
        self.assertAlmostEqual(self.error.vector.x, 1.25, places=6)
        self.assertAlmostEqual(self.error.vector.y, -0.35, places=6)
        self.assertEqual(self.error.vector.z, 0.0)

        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.03)
            if (self.valid is False and self.link is False and self.error is not None and
                    abs(self.error.vector.x) < 1e-9 and abs(self.error.vector.y) < 1e-9):
                break
        self.assertFalse(self.valid)
        self.assertFalse(self.link)
        self.assertAlmostEqual(self.error.vector.x, 0.0, places=6)
        self.assertAlmostEqual(self.error.vector.y, 0.0, places=6)

        phase.data = "TASK1_DROP"
        command = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and command is None:
            self.phase_pub.publish(phase)
            rclpy.spin_once(self.node, timeout_sec=0.03)
            try:
                serial_buffer += os.read(MASTER_FD, 4096)
            except BlockingIOError:
                pass
            command, serial_buffer = extract_frame(serial_buffer, 0x20)
            if command is not None and command[11] != 0x11:
                command = None
        self.assertIsNotNone(command, "未收到TASK1_DROP命令")

        executing = bytearray([0x11, 0x07, 0x03, 0x17])
        executing += (0).to_bytes(4, "big", signed=True) * 2
        os.write(MASTER_FD, make_frame(0x21, 8, 1, executing))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and self.drop_in_progress is not True:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(self.drop_in_progress)
        self.assertFalse(self.drop_completed)
        self.assertFalse(self.drop_failed)

        completed = bytearray([0x11, 0x08, 0x03, 0x10])
        completed += (0).to_bytes(4, "big", signed=True) * 2
        os.write(MASTER_FD, make_frame(0x21, 9, 1, completed))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and self.drop_completed is not True:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(self.drop_completed)
        self.assertFalse(self.drop_failed)

        failed = bytearray([0x11, 0x09, 0x03, 0x30])
        failed += (0).to_bytes(4, "big", signed=True) * 2
        os.write(MASTER_FD, make_frame(0x21, 10, 1, failed))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and self.drop_failed is not True:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(self.drop_failed)
        self.assertFalse(self.drop_completed)


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
