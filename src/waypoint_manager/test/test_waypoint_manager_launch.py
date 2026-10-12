"""航点管理器真实 ROS 2 接口集成测试。"""

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
from std_msgs.msg import Bool
from uav_interfaces.msg import FlightStatus, MissionStatus, Waypoint, WaypointList
from uav_interfaces.srv import SetWaypointList


@pytest.mark.launch_test
def generate_test_description():
    params_file = os.path.join(
        get_package_share_directory("waypoint_manager"),
        "config",
        "waypoint_manager_params.yaml",
    )

    manager_process = launch_ros.actions.Node(
        package="waypoint_manager",
        executable="waypoint_manager_node",
        name="waypoint_manager",
        output="screen",
        parameters=[
            params_file,
            {
                "require_ground_station_route": True,
                "allow_local_route_for_test": False,
                "arrival_confirm_time": 0.10,
                "minimum_hold_time": 0.10,
                "reach_lost_grace_time": 0.05,
                "new_target_guard_time": 0.05,
                "pid_feedback_timeout_ms": 300,
                "flight_status_timeout_ms": 300,
                "manager_rate_hz": 50.0,
                "print_status": False,
            },
        ],
        remappings=[
            ("/pid/target_pose", "/test/waypoint_manager/pid/target_pose"),
            ("/waypoint/current", "/test/waypoint_manager/waypoint/current"),
            ("/mission/status", "/test/waypoint_manager/mission/status"),
            ("/mission/land_request", "/test/waypoint_manager/mission/land_request"),
            ("/pid/target_reached", "/test/waypoint_manager/pid/target_reached"),
            ("/flight/status", "/test/waypoint_manager/flight/status"),
            ("/waypoint/set_list", "/test/waypoint_manager/waypoint/set_list"),
        ],
    )

    return launch.LaunchDescription(
        [manager_process, launch_testing.actions.ReadyToTest()]
    ), {"manager_process": manager_process}


class TestWaypointManagerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("waypoint_manager_integration_test")

        cls.target_messages = []
        cls.current_waypoint = None
        cls.mission_status = None
        cls.land_request = None

        # 该订阅者在测试中扮演 position_pid_controller。
        cls.node.create_subscription(
            PoseStamped, "/test/waypoint_manager/pid/target_pose", cls._target_callback, 10
        )
        cls.node.create_subscription(
            Waypoint, "/test/waypoint_manager/waypoint/current", cls._waypoint_callback, 10
        )
        cls.node.create_subscription(
            MissionStatus, "/test/waypoint_manager/mission/status", cls._status_callback, 10
        )
        cls.node.create_subscription(
            Bool, "/test/waypoint_manager/mission/land_request", cls._land_callback, 10
        )

        cls.pid_reached_publisher = cls.node.create_publisher(
            Bool, "/test/waypoint_manager/pid/target_reached", 10
        )
        cls.flight_status_publisher = cls.node.create_publisher(
            FlightStatus, "/test/waypoint_manager/flight/status", 10
        )
        cls.route_client = cls.node.create_client(
            SetWaypointList, "/test/waypoint_manager/waypoint/set_list"
        )

        assert cls.route_client.wait_for_service(timeout_sec=5.0)
        cls._wait_for(
            lambda: cls.mission_status is not None and cls.land_request is not None,
            timeout=5.0,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _target_callback(cls, message):
        cls.target_messages.append(message)

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
    def _wait_for(cls, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError("等待航点管理器测试条件超时")

    @classmethod
    def _call_set_route(cls, route):
        request = SetWaypointList.Request()
        request.waypoint_list = route
        future = cls.route_client.call_async(request)
        cls._wait_for(lambda: future.done(), timeout=3.0)
        return future.result()

    @classmethod
    def _publish_feedback(
        cls, duration, *, pid_reached, height_reached,
        communication_ok=True, failsafe=False
    ):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            cls.pid_reached_publisher.publish(Bool(data=pid_reached))

            flight_message = FlightStatus()
            flight_message.header.stamp = cls.node.get_clock().now().to_msg()
            flight_message.communication_ok = communication_ok
            flight_message.height_valid = True
            flight_message.height_reached = height_reached
            flight_message.failsafe = failsafe
            cls.flight_status_publisher.publish(flight_message)
            rclpy.spin_once(cls.node, timeout_sec=0.02)

    @classmethod
    def _publish_reached_feedback(cls, duration):
        cls._publish_feedback(
            duration, pid_reached=True, height_reached=False
        )

    @staticmethod
    def _valid_single_waypoint_route():
        route = WaypointList()
        route.header.frame_id = "map"
        route.route_id = 42
        point = Waypoint()
        point.id = 7
        point.x = 2.0
        point.y = -1.0
        point.yaw = 0.5
        point.target_height = 1.2
        point.hold_time = 0.1
        route.waypoints.append(point)
        return route

    def test_wait_route_execute_and_reject_invalid_replacement(self):
        # 即使“PID”订阅者已连接，比赛模式在地面站下发前也不能发送本地目标。
        time.sleep(0.15)
        rclpy.spin_once(self.node, timeout_sec=0.05)
        assert len(self.target_messages) == 0
        assert self.mission_status.state == MissionStatus.WAITING_FOR_ROUTE
        assert self.mission_status.route_valid is False
        assert self.land_request is False

        response = self._call_set_route(self._valid_single_waypoint_route())
        assert response.accepted is True
        self._wait_for(lambda: len(self.target_messages) == 1)
        self._wait_for(lambda: self.current_waypoint is not None)
        assert self.current_waypoint.id == 7
        assert abs(self.target_messages[0].pose.position.x - 2.0) < 1.0e-9

        # 继续运行一段时间，目标数量仍应为 1，证明没有周期性重发并重置 PID。
        time.sleep(0.15)
        rclpy.spin_once(self.node, timeout_sec=0.05)
        assert len(self.target_messages) == 1

        # HR不参与推进；PID未到达时即使HR=true也不能推进航点。
        self._publish_feedback(0.15, pid_reached=False, height_reached=True)
        self.assertEqual(self.mission_status.current_index, 0)
        self.assertEqual(self.mission_status.state, MissionStatus.TRACKING)
        self.assertFalse(self.mission_status.mission_completed)

        # 飞控 failsafe 必须暂停推进并进入 ERROR，恢复后允许继续跟踪。
        self._publish_feedback(
            0.10, pid_reached=True, height_reached=True, failsafe=True
        )
        self._wait_for(lambda: self.mission_status.state == MissionStatus.ERROR)
        self.assertEqual(self.mission_status.current_index, 0)
        self._publish_feedback(0.10, pid_reached=False, height_reached=False)
        self._wait_for(lambda: self.mission_status.state == MissionStatus.TRACKING)

        # PID持续到达且高度数据有效时，即使HR=false也应完成任务并请求降落。
        self._publish_reached_feedback(0.40)
        self._wait_for(
            lambda: self.mission_status is not None
            and self.mission_status.state == MissionStatus.COMPLETED
            and self.land_request is True,
            timeout=2.0,
        )

        # 非法新航线必须整体拒绝，并保留已完成的 route_id=42 航线。
        invalid = self._valid_single_waypoint_route()
        invalid.route_id = 43
        invalid.header.frame_id = "odom"
        rejected = self._call_set_route(invalid)
        assert rejected.accepted is False
        assert self.mission_status.route_id == 42
        assert self.mission_status.mission_completed is True


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_node_exits_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
