"""task1主线集成测试：小车选任务、右移、伴飞、投放、返航和H点降落。"""

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
from uav_interfaces.msg import FlightStatus, MissionStatus, Waypoint


TEST_SESSION = 0x12345678


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
                "require_task_mode_from_car": True,
                "manager_rate_hz": 50.0,
                "task1_takeoff_hover_time_sec": 0.06,
                "takeoff_completion_timeout_sec": 0.30,
                "task2_takeoff_hover_time_sec": 0.0,
                "lateral_hold_time_sec": 0.04,
                "arrival_confirm_time_sec": 0.04,
                "reach_lost_grace_time_sec": 0.02,
                "new_target_guard_time_sec": 0.02,
                "vision_confirm_time_sec": 0.04,
                "drop_alignment_time_sec": 0.04,
                "drop_action_timeout_sec": 0.8,
                "home_hold_time_sec": 0.04,
                "pose_timeout_ms": 1000,
                "pid_feedback_timeout_ms": 300,
                "flight_status_timeout_ms": 1000,
                "enable_safe_hold_latch": True,
                "flight_recovery_confirm_time_sec": 0.08,
                "flight_safe_hold_timeout_sec": 0.50,
                "vision_timeout_ms": 300,
                "drop_window_timeout_ms": 300,
                "visual_control_timeout_ms": 300,
                "drop_feedback_timeout_ms": 300,
                "print_status": False,
            },
        ],
        remappings=[
            ("/competition/task_mode", "/test/task1/task_mode"),
            ("/competition/start", "/test/task1/start"),
            ("/car/drop_window", "/test/task1/car/drop_window"),
            ("/car/lap_complete", "/test/task1/car/lap_complete"),
            ("/payload/drop_completed", "/test/task1/payload/drop_completed"),
            ("/payload/drop_failed", "/test/task1/payload/drop_failed"),
            ("/vision/target_valid", "/test/task1/vision/target_valid"),
            ("/pid/visual_control_active", "/test/task1/pid/visual_control_active"),
            ("/map/current_pose", "/test/task1/map/current_pose"),
            ("/map/current_pose_valid", "/test/task1/map/current_pose_valid"),
            ("/pid/target_reached", "/test/task1/pid/target_reached"),
            ("/flight/status", "/test/task1/flight/status"),
            ("/pid/target_pose", "/test/task1/pid/target_pose"),
            ("/waypoint/current", "/test/task1/waypoint/current"),
            ("/mission/status", "/test/task1/mission/status"),
            ("/mission/land_request", "/test/task1/mission/land_request"),
            ("/competition/phase", "/test/task1/phase"),
            ("/competition/vision_assist_requested", "/test/task1/vision_requested"),
            ("/payload/drop_request", "/test/task1/payload/drop_request"),
            ("/competition/active_task_mode", "/test/task1/active_task_mode"),
        ],
    )

    return launch.LaunchDescription(
        [process, launch_testing.actions.ReadyToTest()]
    ), {"process": process}


class TestTask1MissionNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("task1_mission_integration_test")
        cls.retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        cls.targets = []
        cls.current_waypoint = None
        cls.mission_status = None
        cls.land_request = None
        cls.phase = None
        cls.vision_requested = None
        cls.drop_request = None
        cls.active_task_mode = None

        cls.node.create_subscription(
            PoseStamped, "/test/task1/pid/target_pose", cls._target_callback, 10
        )
        cls.node.create_subscription(
            Waypoint, "/test/task1/waypoint/current", cls._waypoint_callback, 10
        )
        cls.node.create_subscription(
            MissionStatus, "/test/task1/mission/status", cls._status_callback, 10
        )
        cls.node.create_subscription(
            Bool, "/test/task1/mission/land_request", cls._land_callback, 10
        )
        cls.node.create_subscription(String, "/test/task1/phase", cls._phase_callback, 10)
        cls.node.create_subscription(
            Bool, "/test/task1/vision_requested", cls._vision_callback, 10
        )
        cls.node.create_subscription(
            Bool, "/test/task1/payload/drop_request", cls._drop_request_callback, 10
        )
        cls.node.create_subscription(
            UInt8, "/test/task1/active_task_mode", cls._active_task_callback, 10
        )

        cls.task_mode_publisher = cls.node.create_publisher(
            UInt8, "/test/task1/task_mode", cls.retained_qos
        )
        cls.start_publisher = cls.node.create_publisher(
            Bool, "/test/task1/start", cls.retained_qos
        )
        cls.lap_complete_publisher = cls.node.create_publisher(
            Bool, "/test/task1/car/lap_complete", 10
        )
        cls.drop_window_publisher = cls.node.create_publisher(
            Bool, "/test/task1/car/drop_window", 10
        )
        cls.drop_completed_publisher = cls.node.create_publisher(
            Bool, "/test/task1/payload/drop_completed", 10
        )
        cls.drop_failed_publisher = cls.node.create_publisher(
            Bool, "/test/task1/payload/drop_failed", 10
        )
        cls.vision_valid_publisher = cls.node.create_publisher(
            Bool, "/test/task1/vision/target_valid", 10
        )
        cls.visual_active_publisher = cls.node.create_publisher(
            Bool, "/test/task1/pid/visual_control_active", 10
        )
        cls.pose_publisher = cls.node.create_publisher(
            PoseStamped, "/test/task1/map/current_pose", 10
        )
        cls.pose_valid_publisher = cls.node.create_publisher(
            Bool, "/test/task1/map/current_pose_valid", 10
        )
        cls.pid_reached_publisher = cls.node.create_publisher(
            Bool, "/test/task1/pid/target_reached", 10
        )
        cls.flight_status_publisher = cls.node.create_publisher(
            FlightStatus, "/test/task1/flight/status", 10
        )

        cls._wait_for(
            lambda: cls.task_mode_publisher.get_subscription_count() > 0
            and cls.start_publisher.get_subscription_count() > 0
            and cls.drop_window_publisher.get_subscription_count() > 0
            and cls.drop_completed_publisher.get_subscription_count() > 0
            and cls.drop_failed_publisher.get_subscription_count() > 0
            and cls.vision_valid_publisher.get_subscription_count() > 0
            and cls.visual_active_publisher.get_subscription_count() > 0
            and cls.pose_publisher.get_subscription_count() > 0
            and cls.pose_valid_publisher.get_subscription_count() > 0
            and cls.pid_reached_publisher.get_subscription_count() > 0
            and cls.flight_status_publisher.get_subscription_count() > 0,
            timeout=5.0,
        )
        cls._wait_for(
            lambda: cls.phase is not None
            and cls.mission_status is not None
            and cls.land_request is not None
            and cls.drop_request is not None,
            timeout=5.0,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _target_callback(cls, message):
        cls.targets.append(message)

    @classmethod
    def _waypoint_callback(cls, message):
        cls.current_waypoint = message

    @classmethod
    def _status_callback(cls, message):
        cls.mission_status = message

    @classmethod
    def _land_callback(cls, message):
        cls.land_request = message.data

    @classmethod
    def _phase_callback(cls, message):
        cls.phase = message.data

    @classmethod
    def _vision_callback(cls, message):
        cls.vision_requested = message.data

    @classmethod
    def _drop_request_callback(cls, message):
        cls.drop_request = message.data

    @classmethod
    def _active_task_callback(cls, message):
        cls.active_task_mode = message.data

    @classmethod
    def _wait_for(cls, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError("等待task1状态机测试条件超时")

    @classmethod
    def _has_target(cls, x, y, start_index=0):
        return any(
            abs(target.pose.position.x - x) < 1e-6
            and abs(target.pose.position.y - y) < 1e-6
            for target in cls.targets[start_index:]
        )

    @classmethod
    def _publish_inputs(
        cls,
        duration,
        *,
        pid_reached=False,
        height_reached=False,
        takeoff_succeeded=None,
        vision_valid=False,
        visual_active=False,
        drop_window=False,
        drop_completed=False,
        drop_failed=False,
        lap_complete=False,
        land_completed=False,
        pose_valid=True,
        failsafe=False,
        error_code=0,
        armed=False,
        airborne=False,
        landed=False,
    ):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            pose = PoseStamped()
            pose.header.stamp = cls.node.get_clock().now().to_msg()
            pose.header.frame_id = "odom"
            pose.pose.orientation.w = 1.0
            cls.pose_publisher.publish(pose)
            cls.pose_valid_publisher.publish(Bool(data=pose_valid))
            cls.pid_reached_publisher.publish(Bool(data=pid_reached))
            cls.vision_valid_publisher.publish(Bool(data=vision_valid))
            cls.visual_active_publisher.publish(Bool(data=visual_active))
            cls.drop_window_publisher.publish(Bool(data=drop_window))
            cls.drop_completed_publisher.publish(Bool(data=drop_completed))
            cls.drop_failed_publisher.publish(Bool(data=drop_failed))
            cls.lap_complete_publisher.publish(Bool(data=lap_complete))

            status = FlightStatus()
            status.header.stamp = cls.node.get_clock().now().to_msg()
            status.protocol_version = FlightStatus.PROTOCOL_V2
            status.communication_ok = True
            status.height_valid = True
            status.height_reached = height_reached
            status.land_completed = land_completed
            status.failsafe = failsafe
            status.error_code = error_code
            status.armed = armed
            status.airborne = airborne
            status.landed = landed
            status.active_session_id = TEST_SESSION
            action_succeeded = (
                height_reached if takeoff_succeeded is None else takeoff_succeeded
            )
            if action_succeeded:
                status.current_mode = FlightStatus.HEIGHT_HOLD
                status.active_action_id = 1
                status.action_result = FlightStatus.ACTION_SUCCEEDED
                status.action_accepted = True
                status.action_completed = True
            cls.flight_status_publisher.publish(status)
            rclpy.spin_once(cls.node, timeout_sec=0.02)

    def test_task1_right_shift_drop_return_and_land(self):
        self._publish_inputs(0.10)
        self.assertEqual(self.phase, "WAIT_START")
        self.assertEqual(len(self.targets), 0)
        self.assertFalse(self.mission_status.route_valid)

        # START先到但没有task_mode时必须继续等待，不能自行选择任务。
        for _ in range(3):
            self.start_publisher.publish(Bool(data=True))
            self._publish_inputs(0.05)
        self._wait_for(lambda: self.phase == "WAIT_READY")
        self.assertEqual(self.phase, "WAIT_READY")
        self.assertEqual(len(self.targets), 0)

        self.task_mode_publisher.publish(UInt8(data=1))
        self._publish_inputs(0.15)
        self._wait_for(lambda: len(self.targets) >= 1)
        self._wait_for(lambda: self.active_task_mode == 1)
        self.assertEqual(self.active_task_mode, 1)
        self.assertEqual(self.phase, "TAKEOFF_HOVER")
        self.assertAlmostEqual(self.current_waypoint.target_height, 1.23, places=6)

        # V2正式报告TAKEOFF成功后，task1仍必须重新完整等待缩短后的0.06秒。
        # HR、PID到达均为false，且计时前半段定位无效，也不得重置悬停计时。
        self._publish_inputs(
            0.03, pid_reached=False, height_reached=False,
            takeoff_succeeded=True, pose_valid=False
        )
        self.assertEqual(self.phase, "TAKEOFF_HOVER")
        self._publish_inputs(
            0.10, pid_reached=False, height_reached=False, takeoff_succeeded=True
        )
        self._wait_for(lambda: self.phase == "RIGHT_SHIFT")
        # 进入右移不需要PID到达；真正完成37.5厘米横移仍需要定位PID反馈。
        self._publish_inputs(
            0.70, pid_reached=True, height_reached=False, takeoff_succeeded=True
        )
        self._wait_for(
            lambda: self.phase == "FORWARD_CHASE"
            and self._has_target(0.0, -0.375)
            and self._has_target(2.75, -0.375)
        )
        self.assertTrue(self._has_target(0.0, -0.375))
        self.assertTrue(self._has_target(2.75, -0.375))

        self._publish_inputs(0.14, height_reached=True, vision_valid=True)
        self._wait_for(lambda: self.phase == "TASK1_FOLLOW")
        self.assertTrue(self.vision_requested)

        # 进入C-D窗口但视觉控制未确认对准时不能投放。
        self._publish_inputs(
            0.12,
            pid_reached=True,
            height_reached=True,
            vision_valid=True,
            visual_active=False,
            drop_window=True,
        )
        self._wait_for(lambda: self.phase == "TASK1_ALIGN_DROP")
        self.assertEqual(self.phase, "TASK1_ALIGN_DROP")
        self.assertFalse(self.drop_request)

        # 所有投放条件连续满足后，只进入一次投放状态。
        self._publish_inputs(
            0.16,
            pid_reached=True,
            height_reached=True,
            vision_valid=True,
            visual_active=True,
            drop_window=True,
        )
        self._wait_for(lambda: self.phase == "TASK1_DROP" and self.drop_request)

        return_target_start = len(self.targets)
        self._publish_inputs(
            0.12,
            height_reached=True,
            vision_valid=True,
            visual_active=True,
            drop_window=True,
            drop_failed=True,
        )
        self._wait_for(
            lambda: self.phase == "TASK1_RETURN_HOME" and not self.drop_request
        )
        self._wait_for(lambda: self._has_target(0.0, 0.0, return_target_start))
        self.assertFalse(self.vision_requested)
        self.assertFalse(self.drop_request)
        self.assertNotEqual(self.phase, "ERROR")
        self.assertTrue(self._has_target(0.0, 0.0, return_target_start))

        self._publish_inputs(0.24, pid_reached=True, height_reached=True)
        self._wait_for(lambda: self.phase == "TASK1_LAND_HOME")
        self.assertTrue(self.land_request)
        self.assertTrue(self.mission_status.mission_completed)
        self.assertTrue(self.mission_status.route_valid)

        self._publish_inputs(
            0.10, land_completed=True, lap_complete=True, landed=True
        )
        self._wait_for(lambda: self.phase == "TASK1_COMPLETE")
        self.assertEqual(self.mission_status.state, MissionStatus.COMPLETED)
        self.assertFalse(self.drop_request)

        # v1.7 READY(session=0/task=0)必须清除上一轮并允许固定session再次启动。
        self.task_mode_publisher.publish(UInt8(data=0))
        self.start_publisher.publish(Bool(data=False))
        self._publish_inputs(0.12, landed=True)
        self._wait_for(lambda: self.phase == "WAIT_START")
        self.assertEqual(self.active_task_mode, 0)
        self.assertFalse(self.mission_status.route_valid)
        self.assertFalse(self.land_request)
        self.assertFalse(self.drop_request)

        self.task_mode_publisher.publish(UInt8(data=1))
        self.start_publisher.publish(Bool(data=True))
        self._publish_inputs(0.15, landed=True)
        self._wait_for(lambda: self.phase == "TAKEOFF_HOVER")
        self._wait_for(lambda: self.active_task_mode == 1)
        self.assertEqual(self.active_task_mode, 1)

        # 当前HR已经抖回false时，只要同一session仍正式回报A=1/2，
        # 树莓派就必须独立锁存成功并在固定悬停时间后右移，不再等待PID/HR。
        self._publish_inputs(
            0.20, height_reached=False, takeoff_succeeded=True
        )
        self._wait_for(lambda: self.phase == "RIGHT_SHIFT")

        # 飞行中收到READY不得清空任务。
        self._publish_inputs(0.08, armed=True, airborne=True)
        self.task_mode_publisher.publish(UInt8(data=0))
        self.start_publisher.publish(Bool(data=False))
        self._publish_inputs(0.10, armed=True, airborne=True)
        self.assertEqual(self.phase, "RIGHT_SHIFT")
        self.assertEqual(self.active_task_mode, 1)

        # 单帧/短时failsafe必须立即SAFE_HOLD，但恢复确认后返回原阶段。
        self._publish_inputs(0.06, failsafe=True, armed=True, airborne=True)
        self._wait_for(lambda: self.phase == "SAFE_HOLD")
        self.assertFalse(self.mission_status.route_valid)
        self._publish_inputs(0.18, armed=True, airborne=True)
        self._wait_for(lambda: self.phase == "RIGHT_SHIFT")

        # 单帧/短时定位无效也必须SAFE_HOLD，定位连续恢复后返回原阶段。
        self._publish_inputs(
            0.06, pose_valid=False, armed=True, airborne=True
        )
        self._wait_for(lambda: self.phase == "SAFE_HOLD")
        self.assertFalse(self.mission_status.route_valid)
        self._publish_inputs(0.18, armed=True, airborne=True)
        self._wait_for(lambda: self.phase == "RIGHT_SHIFT")

        # 持续异常超过SAFE_HOLD上限才永久进入ERROR。
        self._publish_inputs(0.65, failsafe=True, armed=True, airborne=True)
        self._wait_for(lambda: self.phase == "ERROR")

        # ERROR后只有飞控重新报告安全着地，READY才可复位。CH6撤权造成的
        # COMMAND_TIMEOUT/failsafe不得阻止已经上锁且明确着地的任务清理。
        self._publish_inputs(
            0.08, landed=True, failsafe=True, error_code=2
        )
        self.task_mode_publisher.publish(UInt8(data=0))
        self.start_publisher.publish(Bool(data=False))
        self._publish_inputs(
            0.12, landed=True, failsafe=True, error_code=2
        )
        self._wait_for(lambda: self.phase == "WAIT_START")
        self.assertEqual(self.active_task_mode, 0)
        self.assertFalse(self.mission_status.route_valid)


        # 持续定位异常超过SAFE_HOLD上限仍必须进入ERROR。
        self.task_mode_publisher.publish(UInt8(data=1))
        self.start_publisher.publish(Bool(data=True))
        self._publish_inputs(0.15, landed=True)
        self._wait_for(lambda: self.phase == "TAKEOFF_HOVER")
        self._publish_inputs(0.20, height_reached=True)
        self._wait_for(lambda: self.phase == "RIGHT_SHIFT")
        self._publish_inputs(
            0.65, pose_valid=False, armed=True, airborne=True
        )
        self._wait_for(lambda: self.phase == "ERROR")


        # 先从上一轮ERROR安全复位，准备验证起飞完成反馈总看门狗。
        self._publish_inputs(0.08, pose_valid=True, landed=True)
        self.task_mode_publisher.publish(UInt8(data=0))
        self.start_publisher.publish(Bool(data=False))
        self._publish_inputs(0.12, pose_valid=True, landed=True)
        self._wait_for(lambda: self.phase == "WAIT_START")

        # 新任务始终没有到高反馈，超过0.30秒后必须锁存SAFE_HOLD。
        self.task_mode_publisher.publish(UInt8(data=1))
        self.start_publisher.publish(Bool(data=True))
        self._publish_inputs(0.15, landed=True)
        self._wait_for(lambda: self.phase == "TAKEOFF_HOVER")
        self._publish_inputs(0.38)
        self._wait_for(lambda: self.phase == "SAFE_HOLD")
        self.assertFalse(self.mission_status.route_valid)

        # 即使健康反馈继续到来，也不能自动恢复并突然进入RIGHT_SHIFT。
        self._publish_inputs(0.70)
        self.assertEqual(self.phase, "SAFE_HOLD")

@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_node_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
