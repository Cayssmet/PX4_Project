"""V2任务阶段到飞控指令的真实节点测试。"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from uav_interfaces.msg import FlightDirective, FlightStatus, Waypoint


TEST_SESSION = 0x10203040


@pytest.mark.launch_test
def generate_test_description():
    process = launch_ros.actions.Node(
        package="waypoint_manager",
        executable="competition_flight_directive_node",
        name="competition_flight_directive",
        output="screen",
        parameters=[
            {
                "flight_session_id_override": TEST_SESSION,
                "publish_rate_hz": 50.0,
                "status_timeout_ms": 300,
                "print_status": False,
            }
        ],
        remappings=[
            ("/competition/phase", "/test/directive/phase"),
            ("/waypoint/current", "/test/directive/waypoint"),
            ("/flight/status", "/test/directive/status"),
            ("/flight/directive", "/test/directive/output"),
        ],
    )
    return launch.LaunchDescription(
        [process, launch_testing.actions.ReadyToTest()]
    ), {"process": process}


class TestCompetitionFlightDirective(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("competition_flight_directive_test")
        retained = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.directive = None
        cls.node.create_subscription(
            FlightDirective,
            "/test/directive/output",
            cls._directive_callback,
            retained,
        )
        cls.phase_publisher = cls.node.create_publisher(
            String, "/test/directive/phase", retained
        )
        cls.waypoint_publisher = cls.node.create_publisher(
            Waypoint, "/test/directive/waypoint", retained
        )
        cls.status_publisher = cls.node.create_publisher(
            FlightStatus, "/test/directive/status", 10
        )
        cls._wait_for(
            lambda: cls.phase_publisher.get_subscription_count() > 0
            and cls.waypoint_publisher.get_subscription_count() > 0
            and cls.status_publisher.get_subscription_count() > 0,
            5.0,
        )
        cls._wait_for(lambda: cls.directive is not None, 5.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _directive_callback(cls, message):
        cls.directive = message

    @classmethod
    def _wait_for(cls, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError("等待V2任务指令测试条件超时")

    @classmethod
    def _publish_safe_status_until(cls, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = FlightStatus()
            status.protocol_version = FlightStatus.PROTOCOL_V2
            status.communication_ok = True
            status.active_session_id = TEST_SESSION
            status.error_code = 0
            status.failsafe = False
            cls.status_publisher.publish(status)
            rclpy.spin_once(cls.node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError("飞控安全状态下没有生成预期V2指令")

    @classmethod
    def _set_phase_and_wait(cls, phase, predicate):
        cls.phase_publisher.publish(String(data=phase))
        cls._publish_safe_status_until(predicate)

    def test_takeoff_right_shift_and_forward_mapping(self):
        self.assertEqual(self.directive.session_id, TEST_SESSION)
        self.assertEqual(self.directive.control_mode, FlightDirective.STANDBY)
        self.assertEqual(self.directive.action_id, 0)
        self.assertTrue(self.directive.control_valid)

        self._publish_safe_status_until(lambda: self.directive.control_valid)

        waypoint = Waypoint()
        waypoint.id = 1001
        waypoint.target_height = 1.5
        self.waypoint_publisher.publish(waypoint)

        self._set_phase_and_wait(
            "TAKEOFF_HOVER",
            lambda: self.directive.control_mode == FlightDirective.TAKEOFF
            and self.directive.action_id == 1
            and self.directive.control_valid,
        )
        self.assertAlmostEqual(self.directive.target_height, 1.5, places=5)

        # TAKEOFF尚为PENDING时若任务因定位抖动进入SAFE_HOLD，必须继续重发
        # 完全相同的TAKEOFF，不能改发会触发ACTION_NOT_ALLOWED的STANDBY。
        pending_status = FlightStatus()
        pending_status.protocol_version = FlightStatus.PROTOCOL_V2
        pending_status.communication_ok = True
        pending_status.active_session_id = TEST_SESSION
        pending_status.active_action_id = 1
        pending_status.action_result = FlightStatus.ACTION_PENDING
        pending_status.error_code = 0
        pending_status.failsafe = False
        self.phase_publisher.publish(String(data="SAFE_HOLD"))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.status_publisher.publish(pending_status)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (
                self.directive.control_mode == FlightDirective.TAKEOFF
                and self.directive.action_id == 1
                and self.directive.control_valid
            ):
                break
        else:
            self.fail("TAKEOFF PENDING进入SAFE_HOLD后没有保持原离散动作")
        self.assertAlmostEqual(self.directive.target_height, 1.5, places=5)
        self.phase_publisher.publish(String(data="TAKEOFF_HOVER"))

        # 飞控只有在内部到高成立后才可能把同一TAKEOFF action标为成功。
        # 树莓派收到A=1/2时即使当前HR已抖回false，也必须独立锁存成功，
        # 并在全局TAKEOFF_HOVER阶段切到连续HEIGHT_HOLD转发PID横向保持。
        def takeoff_succeeded_status():
            status = FlightStatus()
            status.protocol_version = FlightStatus.PROTOCOL_V2
            status.communication_ok = True
            status.active_session_id = TEST_SESSION
            status.active_action_id = 1
            status.action_result = FlightStatus.ACTION_SUCCEEDED
            status.height_reached = False
            status.error_code = 0
            status.failsafe = False
            return status

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.status_publisher.publish(takeoff_succeeded_status())
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (
                self.directive.control_mode == FlightDirective.HEIGHT_HOLD
                and self.directive.action_id == 0
                and self.directive.control_valid
            ):
                break
        else:
            self.fail("TAKEOFF成功后未在TAKEOFF_HOVER阶段切换HEIGHT_HOLD")
        self.assertAlmostEqual(self.directive.target_height, 1.5, places=5)

        # 飞控的离散动作结果会保留，但个别状态帧可能暂时缺少动作/到高标志。
        # 一旦当前session的TAKEOFF成功，指令节点不得重新退回TAKEOFF。
        transient_status = FlightStatus()
        transient_status.protocol_version = FlightStatus.PROTOCOL_V2
        transient_status.communication_ok = True
        transient_status.active_session_id = TEST_SESSION
        transient_status.active_action_id = 0
        transient_status.action_result = FlightStatus.ACTION_NONE
        transient_status.height_reached = False
        transient_status.error_code = 0
        transient_status.failsafe = False
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            self.status_publisher.publish(transient_status)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(self.directive.control_mode, FlightDirective.HEIGHT_HOLD)
        self.assertEqual(self.directive.action_id, 0)
        self.assertTrue(self.directive.control_valid)

        # 同一session/action的TAKEOFF摘要必须保持不变。即使航点消息被刷新，
        # 起飞后定高同样使用首次锁存的1.5m，避免高度目标突然变化。
        changed_waypoint = Waypoint()
        changed_waypoint.id = 1001
        changed_waypoint.target_height = 1.8
        self.waypoint_publisher.publish(changed_waypoint)
        time.sleep(0.15)
        rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertAlmostEqual(self.directive.target_height, 1.5, places=5)

        self.waypoint_publisher.publish(waypoint)

        self._set_phase_and_wait(
            "RIGHT_SHIFT",
            lambda: self.directive.control_mode == FlightDirective.HEIGHT_HOLD
            and self.directive.action_id == 0
            and self.directive.control_valid,
        )
        self.assertAlmostEqual(self.directive.target_height, 1.5, places=5)

        self._set_phase_and_wait(
            "FORWARD_CHASE",
            lambda: self.directive.control_mode == FlightDirective.HEIGHT_HOLD
            and self.directive.action_id == 0
            and self.directive.control_valid,
        )

        # 短时链路异常由任务状态机切入SAFE_HOLD。指令节点必须立即撤销
        # 水平/偏航控制；反馈稳定恢复后，原任务阶段可以重新接管。
        self._set_phase_and_wait(
            "SAFE_HOLD",
            lambda: self.directive.control_mode == FlightDirective.HEIGHT_HOLD
            and self.directive.action_id == 0
            and self.directive.control_valid,
        )
        self.assertAlmostEqual(self.directive.target_height, 1.5, places=5)
        self._set_phase_and_wait(
            "FORWARD_CHASE",
            lambda: self.directive.control_mode == FlightDirective.HEIGHT_HOLD
            and self.directive.action_id == 0
            and self.directive.control_valid,
        )

        # task2在20cm以上保持固定12cm/s视觉精密下降；到达20cm后锁存FINAL_LAND。
        descent_status = FlightStatus()
        descent_status.protocol_version = FlightStatus.PROTOCOL_V2
        descent_status.communication_ok = True
        descent_status.active_session_id = TEST_SESSION
        descent_status.error_code = 0
        descent_status.failsafe = False
        descent_status.height_valid = True
        descent_status.current_height = 0.50
        self.phase_publisher.publish(String(data="TASK2_DESCEND"))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.status_publisher.publish(descent_status)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.directive.control_mode == FlightDirective.PRECISION_DESCENT:
                break
        else:
            self.fail("task2在交接高度以上没有生成PRECISION_DESCENT")
        self.assertEqual(self.directive.action_id, 0)
        self.assertAlmostEqual(self.directive.vertical_velocity, -0.12, places=5)
        self.assertAlmostEqual(self.directive.target_height, 0.20, places=5)

        # 下降中视觉丢失会退回对准阶段，但必须保持丢失瞬间的0.50m，
        # 不能重新使用普通航点的1.50m巡航高度。
        self.phase_publisher.publish(String(data="TASK2_ALIGN_LAND"))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.status_publisher.publish(descent_status)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (
                self.directive.control_mode == FlightDirective.HEIGHT_HOLD
                and abs(self.directive.target_height - 0.50) < 1.0e-5
            ):
                break
        else:
            self.fail("task2下降中视觉丢失后没有保持当前低空高度")

        self.phase_publisher.publish(String(data="TASK2_DESCEND"))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.status_publisher.publish(descent_status)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.directive.control_mode == FlightDirective.PRECISION_DESCENT:
                break
        else:
            self.fail("task2重新对准后没有恢复精密下降")

        descent_status.current_height = 0.20
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.status_publisher.publish(descent_status)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (
                self.directive.control_mode == FlightDirective.FINAL_LAND
                and self.directive.action_id == 2
                and self.directive.control_valid
            ):
                break
        else:
            self.fail("task2到达交接高度后没有生成FINAL_LAND")

        # 锁存后即使测距反弹到20cm以上，也不能恢复精密下降。
        descent_status.current_height = 0.45
        deadline = time.monotonic() + 0.30
        while time.monotonic() < deadline:
            self.status_publisher.publish(descent_status)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(self.directive.control_mode, FlightDirective.FINAL_LAND)
        self.assertEqual(self.directive.action_id, 2)

        self.phase_publisher.publish(String(data="TASK2_HOLD_5S"))
        self._wait_for(
            lambda: self.directive.control_mode == FlightDirective.STANDBY
            and self.directive.control_valid
        )

        self.phase_publisher.publish(String(data="ERROR"))
        self._wait_for(
            lambda: self.directive.control_mode == FlightDirective.STANDBY
            and not self.directive.control_valid
        )


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_node_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
