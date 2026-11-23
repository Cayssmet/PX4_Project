"""V2伪终端集成测试：STANDBY建session，再发送一次TAKEOFF动作。"""

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
from uav_interfaces.msg import FlightCommand, FlightDirective, FlightStatus


MASTER_FD = None
SLAVE_FD = None


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


def make_v2_status(
    sequence,
    mode,
    flags,
    session_id,
    action_id=0,
    height_mm=0,
    vertical_velocity_mm_s=0,
    height_age_ms=10,
    command_age_ms=10,
    action_result=0,
    error_code=0,
):
    payload = bytearray(24)
    payload[0] = 0x02
    payload[1] = 0x02
    payload[2] = sequence
    payload[3] = mode
    payload[4] = flags
    payload[5] = error_code
    payload[6:10] = session_id.to_bytes(4, "big")
    payload[10:14] = action_id.to_bytes(4, "big")
    payload[14:16] = height_mm.to_bytes(2, "big")
    payload[16:18] = int(vertical_velocity_mm_s).to_bytes(2, "big", signed=True)
    payload[18:20] = height_age_ms.to_bytes(2, "big")
    payload[20:22] = command_age_ms.to_bytes(2, "big")
    payload[22] = action_result
    payload[23] = 0
    crc = crc16_ccitt_false(payload)
    return bytes([0x5A, 0xA5]) + bytes(payload) + crc.to_bytes(2, "big") + b"\x0D\x0A"


@pytest.mark.launch_test
def generate_test_description():
    global MASTER_FD, SLAVE_FD
    MASTER_FD, SLAVE_FD = pty.openpty()
    os.set_blocking(MASTER_FD, False)
    serial_path = os.ttyname(SLAVE_FD)

    bridge = launch_ros.actions.Node(
        package="flight_controller_bridge",
        executable="flight_controller_bridge_node",
        name="flight_controller_bridge",
        output="screen",
        parameters=[
            {
                "protocol_version": 2,
                "serial_device": serial_path,
                "baud_rate": 115200,
                "send_rate_hz": 50.0,
                "flight_feedback_timeout_ms": 300,
                "directive_timeout_ms": 500,
                "print_status": False,
            }
        ],
    )
    return launch.LaunchDescription(
        [bridge, launch_testing.actions.ReadyToTest()]
    ), {"bridge": bridge}


class TestBridgeV2Path(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("flight_controller_bridge_v2_test")
        retained = QoSProfile(depth=1)
        retained.reliability = ReliabilityPolicy.RELIABLE
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cls.directive_pub = cls.node.create_publisher(
            FlightDirective, "/flight/directive", retained
        )
        cls.status = None
        cls.command = None
        cls.node.create_subscription(
            FlightStatus, "/flight/status", cls._status_callback, 10
        )
        cls.node.create_subscription(
            FlightCommand, "/flight/command", cls._command_callback, 10
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        if MASTER_FD is not None:
            os.close(MASTER_FD)
        if SLAVE_FD is not None:
            os.close(SLAVE_FD)

    @classmethod
    def _status_callback(cls, message):
        cls.status = message

    @classmethod
    def _command_callback(cls, message):
        cls.command = message

    @staticmethod
    def extract_v2_frame(buffer, required_mode=None, require_valid=True):
        while len(buffer) >= 2:
            start = buffer.find(b"\xA5\x5A")
            if start < 0:
                return None, buffer[-1:]
            buffer = buffer[start:]
            if len(buffer) < 30:
                return None, buffer
            frame = buffer[:30]
            buffer = buffer[30:]
            expected = crc16_ccitt_false(frame[2:26])
            received = int.from_bytes(frame[26:28], "big")
            if frame[28:30] != b"\x0D\x0A" or expected != received:
                buffer = frame[1:] + buffer
                continue
            if required_mode is not None and frame[5] != required_mode:
                continue
            if require_valid and not frame[6] & 0x01:
                continue
            return frame, buffer
        return None, buffer

    def publish_directive(self, mode, session, action, height=0.0):
        directive = FlightDirective()
        directive.header.stamp = self.node.get_clock().now().to_msg()
        directive.control_mode = mode
        directive.session_id = session
        directive.action_id = action
        directive.vertical_velocity = 0.0
        directive.target_height = height
        directive.control_valid = True
        self.directive_pub.publish(directive)

    def read_matching_frame(self, mode, timeout=3.0):
        buffer = b""
        frame = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and frame is None:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            try:
                buffer += os.read(MASTER_FD, 4096)
            except BlockingIOError:
                pass
            frame, buffer = self.extract_v2_frame(buffer, required_mode=mode)
        return frame

    def test_session_handshake_and_takeoff_status(self):
        session = 0x12345678

        standby = None
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and standby is None:
            self.publish_directive(FlightDirective.STANDBY, session, 0)
            standby = self.read_matching_frame(FlightDirective.STANDBY, timeout=0.15)
        self.assertIsNotNone(standby, "没有读到V2 STANDBY有效帧")
        self.assertEqual(int.from_bytes(standby[8:12], "big"), session)
        self.assertEqual(int.from_bytes(standby[12:16], "big"), 0)

        os.write(
            MASTER_FD,
            make_v2_status(standby[4], FlightStatus.STANDBY, 0x01, session),
        )
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.status is not None and self.status.communication_ok:
                break
        self.assertIsNotNone(self.status)
        self.assertTrue(self.status.communication_ok)
        self.assertEqual(self.status.protocol_version, 2)
        self.assertEqual(self.status.active_session_id, session)

        takeoff = None
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and takeoff is None:
            self.publish_directive(FlightDirective.TAKEOFF, session, 1, 1.5)
            takeoff = self.read_matching_frame(FlightDirective.TAKEOFF, timeout=0.15)
        self.assertIsNotNone(takeoff, "STANDBY回显后没有读到TAKEOFF有效帧")
        self.assertEqual(int.from_bytes(takeoff[12:16], "big"), 1)
        self.assertEqual(takeoff[16:24], b"\x00" * 8)
        self.assertEqual(int.from_bytes(takeoff[24:26], "big"), 1500)

        # 高度有效+armed+airborne+动作接受，结果PENDING。
        os.write(
            MASTER_FD,
            make_v2_status(
                takeoff[4], FlightStatus.TAKEOFF, 0x2D, session,
                action_id=1, height_mm=700, action_result=FlightStatus.ACTION_PENDING,
            ),
        )
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.status is not None and self.status.action_accepted:
                break
        self.assertTrue(self.status.action_accepted)
        self.assertFalse(self.status.action_completed)
        self.assertEqual(self.status.action_result, FlightStatus.ACTION_PENDING)
        self.assertAlmostEqual(self.status.current_height, 0.7, places=5)

        # 使用最近的新TAKEOFF序号报告动作完成并切到HEIGHT_HOLD。
        latest_takeoff = self.read_matching_frame(FlightDirective.TAKEOFF)
        self.assertIsNotNone(latest_takeoff)
        os.write(
            MASTER_FD,
            make_v2_status(
                latest_takeoff[4], FlightStatus.HEIGHT_HOLD, 0x6F, session,
                action_id=1, height_mm=1500,
                action_result=FlightStatus.ACTION_SUCCEEDED,
            ),
        )
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.status is not None and self.status.action_completed:
                break
        self.assertTrue(self.status.action_completed)
        self.assertEqual(self.status.current_mode, FlightStatus.HEIGHT_HOLD)
        self.assertEqual(self.status.active_action_id, 1)
        self.assertEqual(self.status.action_result, FlightStatus.ACTION_SUCCEEDED)

        # 定高阶段即使定位/PID暂时无效，也必须继续发送有效的本地定高帧，
        # 但水平和偏航速度全部归零，不能让错误地图坐标驱动飞机抽动。
        height_hold = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and height_hold is None:
            self.publish_directive(FlightDirective.HEIGHT_HOLD, session, 0, 1.5)
            height_hold = self.read_matching_frame(
                FlightDirective.HEIGHT_HOLD, timeout=0.15
            )
        self.assertIsNotNone(height_hold, "PID缺失时没有继续发送有效HEIGHT_HOLD帧")
        self.assertEqual(height_hold[16:24], b"\x00" * 8)
        self.assertEqual(int.from_bytes(height_hold[24:26], "big"), 1500)


@launch_testing.post_shutdown_test()
class TestV2ProcessExit(unittest.TestCase):
    def test_node_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
