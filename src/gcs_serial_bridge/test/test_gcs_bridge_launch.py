"""伪串口闭环：UAV_STATUS发送、GCS_STATUS心跳与离线不影响持续发送。"""

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
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8
from uav_interfaces.msg import CarStatus, FlightStatus, GcsStatus, UavStatus

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


@pytest.mark.launch_test
def generate_test_description():
    global MASTER_FD, SLAVE_FD
    MASTER_FD, SLAVE_FD = pty.openpty()
    os.set_blocking(MASTER_FD, False)
    node = launch_ros.actions.Node(
        package="gcs_serial_bridge",
        executable="gcs_serial_bridge_node",
        name="gcs_serial_bridge",
        output="screen",
        parameters=[{
            "serial_device": os.ttyname(SLAVE_FD),
            "baud_rate": 115200,
            "send_rate_hz": 20.0,
            "heartbeat_timeout_ms": 300,
            "input_timeout_ms": 1000,
            "vision_timeout_ms": 1000,
            "reconnect_interval_ms": 50,
            "print_status": False,
        }],
    )
    return launch.LaunchDescription(
        [node, launch_testing.actions.ReadyToTest()]
    ), {"bridge": node}


class TestGcsBridge(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("gcs_bridge_integration_test")
        retained = QoSProfile(depth=1)
        retained.reliability = ReliabilityPolicy.RELIABLE
        retained.durability = DurabilityPolicy.TRANSIENT_LOCAL

        cls.phase_pub = cls.node.create_publisher(String, "/competition/phase", retained)
        cls.task_pub = cls.node.create_publisher(
            UInt8, "/competition/active_task_mode", retained)
        cls.car_pub = cls.node.create_publisher(CarStatus, "/car/status", retained)
        cls.car_link_pub = cls.node.create_publisher(Bool, "/car/link_ok", retained)
        cls.pose_pub = cls.node.create_publisher(PoseStamped, "/map/current_pose", 10)
        cls.pose_valid_pub = cls.node.create_publisher(
            Bool, "/map/current_pose_valid", 10)
        cls.flight_pub = cls.node.create_publisher(FlightStatus, "/flight/status", 10)
        cls.camera_link_pub = cls.node.create_publisher(
            Bool, "/vision/link_ok", retained)
        cls.visual_pub = cls.node.create_publisher(
            Bool, "/pid/visual_control_active", 10)
        cls.drop_pub = cls.node.create_publisher(Bool, "/payload/drop_completed", 10)

        cls.uav_status = None
        cls.gcs_status = None
        cls.gcs_link = None
        cls.subscriptions = [
            cls.node.create_subscription(
                UavStatus, "/gcs/uav_status",
                lambda message: setattr(cls, "uav_status", message), retained),
            cls.node.create_subscription(
                GcsStatus, "/gcs/status",
                lambda message: setattr(cls, "gcs_status", message), retained),
            cls.node.create_subscription(
                Bool, "/gcs/link_ok",
                lambda message: setattr(cls, "gcs_link", message.data), retained),
        ]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        os.close(MASTER_FD)
        os.close(SLAVE_FD)

    def spin_until(self, condition, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if condition():
                return
        self.fail("等待测试条件超时")

    def publish_bool(self, publisher, value):
        message = Bool()
        message.data = value
        publisher.publish(message)

    def publish_ready_inputs(self):
        phase = String()
        phase.data = "WAIT_START"
        self.phase_pub.publish(phase)

        task = UInt8()
        task.data = 0
        self.task_pub.publish(task)

        car = CarStatus()
        car.communication_ok = True
        car.session_id = 0
        car.task_mode = 0
        car.mission_phase = 0
        car.car_state = CarStatus.CAR_READY_A
        car.last_checkpoint = CarStatus.CHECKPOINT_A
        self.car_pub.publish(car)
        self.publish_bool(self.car_link_pub, True)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 10.0
        pose.pose.position.y = 20.0
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        self.publish_bool(self.pose_valid_pub, True)

        flight = FlightStatus()
        flight.protocol_version = FlightStatus.PROTOCOL_V2
        flight.communication_ok = True
        flight.height_valid = True
        flight.current_height = 0.06
        flight.landed = True
        self.flight_pub.publish(flight)

        self.publish_bool(self.camera_link_pub, True)
        self.publish_bool(self.visual_pub, False)
        self.publish_bool(self.drop_pub, False)

    def read_uav_frame(self, timeout=2.0):
        serial_buffer = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            try:
                serial_buffer += os.read(MASTER_FD, 4096)
            except BlockingIOError:
                pass
            frame, serial_buffer = extract_frame(serial_buffer, 0x30)
            if frame is not None:
                return frame
        self.fail("未收到UAV_STATUS线缆帧")

    def test_status_heartbeat_timeout_and_continued_transmit(self):
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self.publish_ready_inputs()
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.spin_until(
            lambda: self.uav_status is not None
            and self.uav_status.mission_phase == UavStatus.PHASE_READY
            and (self.uav_status.flags & UavStatus.FLAG_POSE_VALID) != 0)

        ready_frame = self.read_uav_frame()
        self.assertEqual(len(ready_frame), 29)
        self.assertEqual(ready_frame[5], 18)
        self.assertEqual(int.from_bytes(ready_frame[6:8], "big"), 0)

        car = CarStatus()
        car.communication_ok = True
        car.session_id = 1
        car.task_mode = 1
        car.mission_phase = UavStatus.PHASE_MISSION_START
        car.car_state = CarStatus.CAR_RUN_AB
        car.last_checkpoint = CarStatus.CHECKPOINT_A
        car.flags = CarStatus.FLAG_START_LATCHED | CarStatus.FLAG_MOVING
        self.car_pub.publish(car)

        task = UInt8()
        task.data = 1
        self.task_pub.publish(task)
        phase = String()
        phase.data = "WAIT_READY"
        self.phase_pub.publish(phase)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 10.375
        pose.pose.position.y = 19.750
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        self.publish_bool(self.pose_valid_pub, True)
        self.publish_bool(self.car_link_pub, True)

        flight = FlightStatus()
        flight.protocol_version = FlightStatus.PROTOCOL_V2
        flight.communication_ok = True
        flight.height_valid = True
        flight.current_height = 1.37
        flight.armed = True
        flight.airborne = True
        self.flight_pub.publish(flight)

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.car_pub.publish(car)
            self.task_pub.publish(task)
            self.phase_pub.publish(phase)
            self.pose_pub.publish(pose)
            self.publish_bool(self.pose_valid_pub, True)
            self.publish_bool(self.car_link_pub, True)
            self.flight_pub.publish(flight)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (
                self.uav_status is not None
                and self.uav_status.session_id == 1
                and self.uav_status.mission_phase == UavStatus.PHASE_MISSION_START
                and self.uav_status.map_x_mm == 375
                and self.uav_status.map_y_mm == -250
            ):
                break
        else:
            self.fail("地面站未形成START后的H点相对坐标")

        heartbeat_payload = bytes([
            GcsStatus.GCS_DISPLAYING,
            UavStatus.PHASE_MISSION_START,
            GcsStatus.FLAG_DISPLAY_READY | GcsStatus.FLAG_RECORDING,
            0,
        ])
        os.write(MASTER_FD, make_frame(0x31, 7, 1, heartbeat_payload))
        self.spin_until(
            lambda: self.gcs_link is True
            and self.gcs_status is not None
            and self.gcs_status.communication_ok
            and self.gcs_status.sequence == 7)

        self.spin_until(
            lambda: self.gcs_link is False
            and self.gcs_status is not None
            and not self.gcs_status.communication_ok,
            timeout=1.5,
        )
        self.assertFalse(self.gcs_status.communication_ok)

        later_frame = self.read_uav_frame()
        self.assertEqual(len(later_frame), 29)
        self.assertEqual(int.from_bytes(later_frame[6:8], "big"), 1)

        # task2落车确认和完成都必须显示为落在车上，不能误标为落在H点。
        task.data = 2
        self.task_pub.publish(task)
        car.task_mode = 2
        car.session_id = 2
        self.car_pub.publish(car)
        flight.armed = False
        flight.airborne = False
        flight.landed = True
        self.flight_pub.publish(flight)
        phase.data = "TASK2_HOLD_5S"
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.phase_pub.publish(phase)
            self.car_pub.publish(car)
            self.flight_pub.publish(flight)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (
                self.uav_status is not None
                and self.uav_status.mission_phase == UavStatus.PHASE_TASK2_HOLD_5S
            ):
                break
        else:
            self.fail("地面站未显示task2落车稳定确认阶段")
        self.assertEqual(self.uav_status.uav_state, UavStatus.UAV_LANDED_ON_CAR)
        self.assertNotEqual(
            self.uav_status.flags & UavStatus.FLAG_LANDED_ON_CAR, 0)
        self.assertEqual(self.uav_status.flags & UavStatus.FLAG_LANDED_AT_H, 0)

        phase.data = "TASK2_COMPLETE"
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.phase_pub.publish(phase)
            self.car_pub.publish(car)
            self.flight_pub.publish(flight)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (
                self.uav_status is not None
                and self.uav_status.mission_phase == UavStatus.PHASE_TASK2_COMPLETE
            ):
                break
        else:
            self.fail("地面站未显示task2已稳定完成阶段")
        self.assertEqual(self.uav_status.uav_state, UavStatus.UAV_LANDED_ON_CAR)
        self.assertNotEqual(
            self.uav_status.flags & UavStatus.FLAG_LANDED_ON_CAR, 0)
        self.assertEqual(self.uav_status.flags & UavStatus.FLAG_LANDED_AT_H, 0)
        self.assertNotEqual(
            self.uav_status.flags & UavStatus.FLAG_TASK_COMPLETE, 0)


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

