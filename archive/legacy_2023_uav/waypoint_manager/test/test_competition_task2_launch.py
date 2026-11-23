"""task2选择测试：由小车选择task2，不等待task1规定的3秒悬停。"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8
from uav_interfaces.msg import FlightStatus


@pytest.mark.launch_test
def generate_test_description():
    params_file = os.path.join(
        get_package_share_directory("waypoint_manager"),
        "config",
        "competition_mission_params.yaml",
    )
    process = launch_ros.actions.Node(
        package="waypoint_manager",
        executable="competition_mission_node",
        name="competition_mission_manager",
        output="screen",
        parameters=[
            params_file,
            {
                "auto_start_for_test": False,
                "manager_rate_hz": 50.0,
                "task1_takeoff_hover_time_sec": 5.0,
                "task2_takeoff_hover_time_sec": 0.0,
                "lateral_hold_time_sec": 0.03,
                "arrival_confirm_time_sec": 0.03,
                "reach_lost_grace_time_sec": 0.02,
                "new_target_guard_time_sec": 0.02,
                "vision_confirm_time_sec": 0.03,
                "pose_timeout_ms": 300,
                "pid_feedback_timeout_ms": 300,
                "flight_status_timeout_ms": 300,
                "task2_landed_stability_grace_sec": 0.05,
                "vision_timeout_ms": 300,
                "task2_land_align_time_sec": 0.05,
                "task2_min_descent_time_sec": 0.05,
                "task2_on_car_hold_time_sec": 0.20,
                "print_status": False,
            },
        ],
        remappings=[
            ("/competition/task_mode", "/test/task2/task_mode"),
            ("/competition/start", "/test/task2/start"),
            ("/car/drop_window", "/test/task2/car/drop_window"),
            ("/pid/visual_control_active", "/test/task2/pid/visual_control_active"),
            ("/vision/target_valid", "/test/task2/vision/target_valid"),
            ("/map/current_pose", "/test/task2/map/current_pose"),
            ("/map/current_pose_valid", "/test/task2/map/current_pose_valid"),
            ("/pid/target_reached", "/test/task2/pid/target_reached"),
            ("/flight/status", "/test/task2/flight/status"),
            ("/pid/target_pose", "/test/task2/pid/target_pose"),
            ("/competition/phase", "/test/task2/phase"),
            ("/competition/vision_assist_requested", "/test/task2/vision_requested"),
            ("/payload/drop_request", "/test/task2/drop_request"),
            ("/mission/land_request", "/test/task2/land_request"),
        ],
    )
    return launch.LaunchDescription(
        [process, launch_testing.actions.ReadyToTest()]
    ), {"process": process}


class TestTask2Selection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("task2_selection_test")
        retained = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.phase = None
        cls.phases = []
        cls.targets = []
        cls.vision_requested = None
        cls.drop_request = None
        cls.land_request = None

        cls.node.create_subscription(String, "/test/task2/phase", cls._phase, 10)
        cls.node.create_subscription(
            PoseStamped, "/test/task2/pid/target_pose", cls._target, 10
        )
        cls.node.create_subscription(Bool, "/test/task2/vision_requested", cls._vision, 10)
        cls.node.create_subscription(Bool, "/test/task2/drop_request", cls._drop, 10)
        cls.node.create_subscription(Bool, "/test/task2/land_request", cls._land, 10)

        cls.task_mode_pub = cls.node.create_publisher(
            UInt8, "/test/task2/task_mode", retained
        )
        cls.start_pub = cls.node.create_publisher(Bool, "/test/task2/start", 10)
        cls.drop_window_pub = cls.node.create_publisher(
            Bool, "/test/task2/car/drop_window", 10
        )
        cls.visual_active_pub = cls.node.create_publisher(
            Bool, "/test/task2/pid/visual_control_active", 10
        )
        cls.vision_pub = cls.node.create_publisher(
            Bool, "/test/task2/vision/target_valid", 10
        )
        cls.pose_pub = cls.node.create_publisher(
            PoseStamped, "/test/task2/map/current_pose", 10
        )
        cls.pose_valid_pub = cls.node.create_publisher(
            Bool, "/test/task2/map/current_pose_valid", 10
        )
        cls.pid_pub = cls.node.create_publisher(
            Bool, "/test/task2/pid/target_reached", 10
        )
        cls.flight_pub = cls.node.create_publisher(
            FlightStatus, "/test/task2/flight/status", 10
        )
        cls._wait_for(
            lambda: cls.task_mode_pub.get_subscription_count() > 0
            and cls.start_pub.get_subscription_count() > 0
            and cls.drop_window_pub.get_subscription_count() > 0
            and cls.visual_active_pub.get_subscription_count() > 0
            and cls.vision_pub.get_subscription_count() > 0
            and cls.pose_pub.get_subscription_count() > 0
            and cls.pose_valid_pub.get_subscription_count() > 0
            and cls.pid_pub.get_subscription_count() > 0
            and cls.flight_pub.get_subscription_count() > 0,
            5.0,
        )
        cls._wait_for(lambda: cls.phase is not None, 5.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _phase(cls, msg):
        cls.phase = msg.data
        cls.phases.append(msg.data)

    @classmethod
    def _target(cls, msg):
        cls.targets.append(msg)

    @classmethod
    def _vision(cls, msg):
        cls.vision_requested = msg.data

    @classmethod
    def _drop(cls, msg):
        cls.drop_request = msg.data

    @classmethod
    def _land(cls, msg):
        cls.land_request = msg.data

    @classmethod
    def _wait_for(cls, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError("等待task2测试条件超时")

    @classmethod
    def _publish(
        cls, duration, *, pid_reached=False, takeoff_succeeded=False, vision=False,
        visual_active=False, drop_window=False, landed=False,
        final_land_succeeded=False, pose_valid=True):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            pose = PoseStamped()
            pose.header.stamp = cls.node.get_clock().now().to_msg()
            pose.header.frame_id = "odom"
            pose.pose.orientation.w = 1.0
            cls.pose_pub.publish(pose)
            cls.pose_valid_pub.publish(Bool(data=pose_valid))
            cls.pid_pub.publish(Bool(data=pid_reached))
            cls.vision_pub.publish(Bool(data=vision))
            cls.visual_active_pub.publish(Bool(data=visual_active))
            cls.drop_window_pub.publish(Bool(data=drop_window))
            status = FlightStatus()
            status.protocol_version = FlightStatus.PROTOCOL_V2
            status.communication_ok = True
            status.height_valid = True
            status.height_reached = False
            status.active_session_id = 0x22334455
            status.landed = landed
            if takeoff_succeeded:
                status.current_mode = FlightStatus.HEIGHT_HOLD
                status.active_action_id = 1
                status.action_result = FlightStatus.ACTION_SUCCEEDED
                status.action_accepted = True
            if final_land_succeeded:
                status.current_mode = FlightStatus.STANDBY
                status.active_action_id = 2
                status.action_result = FlightStatus.ACTION_SUCCEEDED
                status.action_accepted = True
                status.action_completed = True
            cls.flight_pub.publish(status)
            rclpy.spin_once(cls.node, timeout_sec=0.02)

    def test_task2_skips_task1_hover_and_uses_right_shift(self):
        self.task_mode_pub.publish(UInt8(data=2))
        self.start_pub.publish(Bool(data=True))
        self._publish(0.16)
        self._wait_for(lambda: len(self.targets) >= 1)

        # task2只需A=1/2，不等待task1的悬停时间，也不等待PID/HR到达。
        self._publish(
            0.10, pid_reached=False, takeoff_succeeded=True)
        self._wait_for(lambda: self.phase == "RIGHT_SHIFT")
        # 阶段已经进入右移后，再用PID到达反馈完成横移并进入前追。
        self._publish(0.30, pid_reached=True, takeoff_succeeded=True)
        self._wait_for(lambda: self.phase == "FORWARD_CHASE")
        self._wait_for(lambda: len(self.targets) >= 3)
        self.assertAlmostEqual(self.targets[1].pose.position.y, -0.375, places=6)
        self.assertAlmostEqual(self.targets[2].pose.position.y, -0.375, places=6)

        # 持续刷新飞控/定位状态，避免在视觉确认期间被300 ms新鲜度门误判过期。
        self._publish(0.50, pid_reached=False, vision=True)
        self._wait_for(lambda: self.phase == "TASK2_VISUAL_APPROACH")
        self._wait_for(lambda: self.vision_requested is True)
        self.assertFalse(self.drop_request)
        self.assertFalse(self.land_request)

        # C点减速完成且视觉持续对准后进入精密下降。
        self._publish(
            0.15, pid_reached=True, takeoff_succeeded=True, vision=True,
            visual_active=True, drop_window=True, pose_valid=False)
        self._wait_for(lambda: self.phase == "TASK2_DESCEND")

        # 视觉接管后即使Cartographer无效，task2仍能对准并进入下降。
        # 只有同session的FINAL_LAND action=2成功且landed，才开始落车稳定确认。
        self._publish(
            0.15, pid_reached=True, takeoff_succeeded=True, vision=True,
            visual_active=True, drop_window=True, landed=True,
            final_land_succeeded=True, pose_valid=False)
        self._wait_for(lambda: self.phase == "TASK2_HOLD_5S")
        self.assertIn("TASK2_ON_CAR", self.phases)
        self.assertNotIn("TASK2_COMPLETE", self.phases)
        self._publish(
            0.25, vision=True, visual_active=True, drop_window=True,
            landed=True, final_land_succeeded=True, pose_valid=False)
        self._wait_for(lambda: self.phase == "TASK2_COMPLETE")
        self.assertIn("TASK2_HOLD_5S", self.phases)
        self.assertNotIn("TASK2_RETAKEOFF", self.phases)
        self.assertNotIn("TASK2_RETURN_H", self.phases)
        self.assertNotIn("TASK2_LAND_H", self.phases)
        self.assertFalse(self.land_request)


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_node_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
