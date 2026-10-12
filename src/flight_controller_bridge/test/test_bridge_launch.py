"""用 Linux 伪终端模拟飞控串口，测试 ROS 话题到字节帧的完整通路。"""

import math
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
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from uav_interfaces.msg import FlightCommand, FlightStatus, MissionStatus, Waypoint


MASTER_FD = None
SLAVE_FD = None


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_status_frame(sequence, flags):
    payload = bytes([sequence, flags])
    crc = crc16_ccitt_false(payload)
    return bytes([0x55, 0xAA, sequence, flags, crc >> 8, crc & 0xFF, 0x0D, 0x0A])


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
                "serial_device": serial_path,
                "baud_rate": 115200,
                "send_rate_hz": 50.0,
                "require_flight_feedback_before_control": False,
                "pid_command_timeout_ms": 1000,
                "pose_timeout_ms": 1000,
                "mission_status_timeout_ms": 1000,
                "land_request_timeout_ms": 1000,
                "print_status": False,
            }
        ],
    )

    return (
        launch.LaunchDescription([bridge, launch_testing.actions.ReadyToTest()]),
        {"bridge": bridge},
    )


class TestBridgeSerialPath(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("flight_controller_bridge_test")

        retained = QoSProfile(depth=1)
        retained.reliability = ReliabilityPolicy.RELIABLE
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL

        cls.pid_pub = cls.node.create_publisher(TwistStamped, "/pid/cmd_vel", 10)
        cls.pid_valid_pub = cls.node.create_publisher(Bool, "/pid/control_valid", 10)
        cls.pose_pub = cls.node.create_publisher(PoseStamped, "/map/current_pose", 10)
        cls.pose_valid_pub = cls.node.create_publisher(Bool, "/map/current_pose_valid", 10)
        cls.waypoint_pub = cls.node.create_publisher(Waypoint, "/waypoint/current", retained)
        cls.mission_pub = cls.node.create_publisher(MissionStatus, "/mission/status", retained)
        cls.land_pub = cls.node.create_publisher(Bool, "/mission/land_request", retained)

        cls.flight_status = None
        cls.flight_command = None
        cls.status_sub = cls.node.create_subscription(
            FlightStatus, "/flight/status", cls._status_callback, 10
        )
        cls.command_sub = cls.node.create_subscription(
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
        cls.flight_status = message

    @classmethod
    def _command_callback(cls, message):
        cls.flight_command = message

    def publish_valid_inputs(self):
        now = self.node.get_clock().now().to_msg()

        command = TwistStamped()
        command.header.stamp = now
        command.header.frame_id = "map"
        command.twist.linear.x = 1.0
        command.twist.linear.y = 0.0
        command.twist.angular.z = 0.25
        self.pid_pub.publish(command)
        self.pid_valid_pub.publish(Bool(data=True))

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.orientation.z = math.sin(math.pi / 4.0)
        pose.pose.orientation.w = math.cos(math.pi / 4.0)
        self.pose_pub.publish(pose)
        self.pose_valid_pub.publish(Bool(data=True))

        waypoint = Waypoint()
        waypoint.id = 7
        waypoint.target_height = 1.2
        self.waypoint_pub.publish(waypoint)

        mission = MissionStatus()
        mission.header.stamp = now
        mission.header.frame_id = "map"
        mission.route_id = 9
        mission.current_waypoint_id = 7
        mission.waypoint_count = 1
        mission.state = MissionStatus.TRACKING
        mission.route_valid = True
        mission.target_sent = True
        mission.mission_completed = False
        self.mission_pub.publish(mission)
        self.land_pub.publish(Bool(data=False))

    @staticmethod
    def extract_valid_command(buffer):
        while len(buffer) >= 2:
            try:
                start = buffer.index(b"\xAA\x55")
            except ValueError:
                return None, buffer[-1:]
            buffer = buffer[start:]
            if len(buffer) < 16:
                return None, buffer
            frame = buffer[:16]
            buffer = buffer[16:]
            if frame[14:16] != b"\x0D\x0A":
                buffer = frame[1:] + buffer
                continue
            expected = crc16_ccitt_false(frame[2:12])
            received = (frame[12] << 8) | frame[13]
            if expected == received and frame[3] & 0x01:
                return frame, buffer
        return None, buffer

    def test_map_to_body_command_and_status_return(self):
        serial_bytes = b""
        valid_frame = None
        deadline = time.monotonic() + 8.0

        while time.monotonic() < deadline and valid_frame is None:
            self.publish_valid_inputs()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            try:
                serial_bytes += os.read(MASTER_FD, 4096)
            except BlockingIOError:
                pass
            valid_frame, serial_bytes = self.extract_valid_command(serial_bytes)

        self.assertIsNotNone(valid_frame, "没有从伪串口读到 control_valid=true 的命令帧")

        def read_i16_be(data):
            raw = (data[0] << 8) | data[1]
            return raw - 0x10000 if raw & 0x8000 else raw

        # yaw=+90°：map +x 应变成 body y=-1 m/s，而不是直接原样发送。
        self.assertAlmostEqual(read_i16_be(valid_frame[4:6]), 0, delta=1)
        self.assertAlmostEqual(read_i16_be(valid_frame[6:8]), -1000, delta=1)
        self.assertEqual(read_i16_be(valid_frame[8:10]), 250)
        self.assertEqual((valid_frame[10] << 8) | valid_frame[11], 1200)

        # 先发送 CRC 正确但位于序号半区之外的 ACK，不能错误恢复通信状态。
        wrong_sequence = (valid_frame[2] + 128) & 0xFF
        os.write(MASTER_FD, make_status_frame(wrong_sequence, 0x01))
        reject_deadline = time.monotonic() + 0.12
        while time.monotonic() < reject_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(
            self.flight_status is None or not self.flight_status.communication_ok,
            "错误会话的 ACK 不应令 communication_ok=true",
        )

        # 模拟飞控确认同一序号并报告“目标高度已到达”。0.12 秒约落后 6 帧，
        # 仍在默认 10 帧窗口内。
        os.write(MASTER_FD, make_status_frame(valid_frame[2], 0x01))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                self.flight_status is not None
                and self.flight_status.communication_ok
                and self.flight_status.height_reached
            ):
                break

        self.assertIsNotNone(self.flight_status)
        self.assertTrue(self.flight_status.communication_ok)
        self.assertTrue(self.flight_status.height_reached)
        self.assertFalse(self.flight_status.failsafe)
        # 航点身份变化后，旧 PID 缓存必须立即失效，先输出无效零速命令。
        next_waypoint = Waypoint()
        next_waypoint.id = 8
        next_waypoint.target_height = 1.3
        next_mission = MissionStatus()
        next_mission.header.stamp = self.node.get_clock().now().to_msg()
        next_mission.header.frame_id = "map"
        next_mission.route_id = 9
        next_mission.current_waypoint_id = 8
        next_mission.waypoint_count = 2
        next_mission.state = MissionStatus.TRACKING
        next_mission.route_valid = True
        next_mission.target_sent = True
        type(self).flight_command = None
        self.waypoint_pub.publish(next_waypoint)
        self.mission_pub.publish(next_mission)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (
                self.flight_command is not None
                and abs(self.flight_command.target_height - 1.3) < 1.0e-6
                and not self.flight_command.control_valid
            ):
                break
        self.assertIsNotNone(self.flight_command)
        self.assertFalse(self.flight_command.control_valid)

        def publish_new_pid_and_pose():
            now = self.node.get_clock().now().to_msg()
            command = TwistStamped()
            command.header.stamp = now
            command.header.frame_id = "map"
            command.twist.linear.x = 0.5
            self.pid_pub.publish(command)
            self.pid_valid_pub.publish(Bool(data=True))

            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "map"
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)
            self.pose_valid_pub.publish(Bool(data=True))

        # 保护时间结束且收到新 PID 输出后，普通控制应恢复。
        type(self).flight_command = None
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            publish_new_pid_and_pose()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.flight_command is not None and self.flight_command.control_valid:
                break
        self.assertIsNotNone(self.flight_command)
        self.assertTrue(self.flight_command.control_valid)
        self.assertAlmostEqual(self.flight_command.target_height, 1.3, places=6)

        # 找到新航点的一帧近期命令，用它模拟“降落请求前误报完成”。
        tracking_frame = None
        serial_bytes = b""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and tracking_frame is None:
            publish_new_pid_and_pose()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            try:
                serial_bytes += os.read(MASTER_FD, 4096)
            except BlockingIOError:
                pass
            candidate, serial_bytes = self.extract_valid_command(serial_bytes)
            if candidate is not None and ((candidate[10] << 8) | candidate[11]) == 1300:
                tracking_frame = candidate
        self.assertIsNotNone(tracking_frame)

        type(self).flight_status = None
        os.write(MASTER_FD, make_status_frame(tracking_frame[2], 0x04))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.flight_status is not None and self.flight_status.land_completed:
                break
        self.assertIsNotNone(self.flight_status)
        self.assertTrue(self.flight_status.land_completed)

        # 之后合法进入降落流程时，之前的完成位不能抑制第一帧降落请求。
        next_mission.state = MissionStatus.COMPLETED
        next_mission.mission_completed = True
        type(self).flight_command = None
        self.mission_pub.publish(next_mission)
        self.land_pub.publish(Bool(data=True))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.flight_command is not None and self.flight_command.land_request:
                break
        self.assertIsNotNone(self.flight_command)
        self.assertTrue(self.flight_command.land_request)

        # 合法进入后收到的新完成反馈应被接受，并停止重复发送 land_request。
        land_frame = None
        serial_bytes = b""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and land_frame is None:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            try:
                serial_bytes += os.read(MASTER_FD, 4096)
            except BlockingIOError:
                pass
            candidate, serial_bytes = self.extract_valid_command(serial_bytes)
            if candidate is not None and candidate[3] & 0x02:
                land_frame = candidate
        self.assertIsNotNone(land_frame)

        type(self).flight_command = None
        os.write(MASTER_FD, make_status_frame(land_frame[2], 0x06))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.flight_command is not None and not self.flight_command.land_request:
                break
        self.assertIsNotNone(self.flight_command)
        self.assertFalse(self.flight_command.land_request)


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_node_exits_cleanly(self, proc_info):
        """测试结束关闭 launch 时，桥接节点与串口线程应正常退出。"""
        launch_testing.asserts.assertExitCodes(proc_info)
