"""位置 PID 节点的 ROS 2 接口与输入失效保护集成测试。"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from std_msgs.msg import Bool, String, UInt8


@pytest.mark.launch_test
def generate_test_description():
    params_file = os.path.join(
        get_package_share_directory("position_pid_controller"),
        "config",
        "position_pid_params.yaml",
    )

    # 加载真正的 YAML，再只覆盖测试所需参数；这也能发现 YAML 结构或参数名错误。
    pid_process = launch_ros.actions.Node(
        package="position_pid_controller",
        executable="position_pid_node",
        name="position_pid_controller",
        output="screen",
        parameters=[
            params_file,
            {
                "target_x": 1.0,
                "target_y": 0.0,
                "target_yaw": 0.0,
                "enable_global_yaw_control": False,
                "x_kp": 0.01,
                "y_kp": 0.01,
                "yaw_kp": 0.01,
                "pose_timeout_ms": 300,
                "valid_timeout_ms": 300,
                "vision_timeout_ms": 300,
                "min_xy_speed": 0.15,
                "max_xy_speed": 0.25,
                "right_shift_max_xy_speed": 0.19,
                "forward_chase_max_xy_speed": 0.23,
                "task2_max_xy_speed": 0.28,
                "task2_right_shift_max_xy_speed": 0.23,
                "task2_forward_chase_max_xy_speed": 0.25,
                "vision_forward_kp": 0.01,
                "vision_left_kp": 0.01,
                "vision_forward_ki": 0.0,
                "vision_left_ki": 0.0,
                "vision_forward_kd": 0.0,
                "vision_left_kd": 0.0,
                "vision_min_xy_speed": 0.10,
                "print_status": False,
            },
        ],
        remappings=[
            ("/map/current_pose", "/test/position_pid/current_pose"),
            ("/map/current_pose_valid", "/test/position_pid/current_pose_valid"),
            ("/pid/target_pose", "/test/position_pid/target_pose"),
            ("/pid/cmd_vel", "/test/position_pid/cmd_vel"),
            ("/pid/control_valid", "/test/position_pid/control_valid"),
            ("/pid/target_reached", "/test/position_pid/target_reached"),
            ("/competition/phase", "/test/position_pid/phase"),
            (
                "/competition/active_task_mode",
                "/test/position_pid/active_task_mode",
            ),
            (
                "/competition/vision_assist_requested",
                "/test/position_pid/vision_assist_requested",
            ),
            ("/vision/target_valid", "/test/position_pid/vision_target_valid"),
            ("/vision/relative_error", "/test/position_pid/vision_relative_error"),
            (
                "/pid/visual_control_active",
                "/test/position_pid/visual_control_active",
            ),
        ],
    )

    return launch.LaunchDescription(
        [pid_process, launch_testing.actions.ReadyToTest()]
    ), {"pid_process": pid_process}


class TestPositionPidNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("position_pid_integration_test")

        cls.pose_publisher = cls.node.create_publisher(
            PoseStamped, "/test/position_pid/current_pose", 10
        )
        cls.valid_publisher = cls.node.create_publisher(
            Bool, "/test/position_pid/current_pose_valid", 10
        )
        cls.target_publisher = cls.node.create_publisher(
            PoseStamped, "/test/position_pid/target_pose", 10
        )
        cls.phase_publisher = cls.node.create_publisher(
            String, "/test/position_pid/phase", 10
        )
        cls.task_mode_publisher = cls.node.create_publisher(
            UInt8, "/test/position_pid/active_task_mode", 10
        )
        cls.vision_request_publisher = cls.node.create_publisher(
            Bool, "/test/position_pid/vision_assist_requested", 10
        )
        cls.vision_valid_publisher = cls.node.create_publisher(
            Bool, "/test/position_pid/vision_target_valid", 10
        )
        cls.vision_error_publisher = cls.node.create_publisher(
            Vector3Stamped, "/test/position_pid/vision_relative_error", 10
        )

        cls.latest_command = None
        cls.latest_control_valid = None
        cls.latest_target_reached = None
        cls.latest_visual_control_active = None
        cls.command_count = 0
        cls.valid_count = 0
        cls.reached_count = 0
        cls.visual_active_count = 0

        cls.node.create_subscription(
            TwistStamped, "/test/position_pid/cmd_vel", cls._command_callback, 10
        )
        cls.node.create_subscription(
            Bool, "/test/position_pid/control_valid", cls._valid_callback, 10
        )
        cls.node.create_subscription(
            Bool, "/test/position_pid/target_reached", cls._reached_callback, 10
        )
        cls.node.create_subscription(
            Bool,
            "/test/position_pid/visual_control_active",
            cls._visual_active_callback,
            10,
        )

        # 等待 DDS 发现完成，避免第一批测试消息在订阅匹配前丢失。
        cls._wait_for(
            lambda: cls.pose_publisher.get_subscription_count() > 0
            and cls.valid_publisher.get_subscription_count() > 0
            and cls.target_publisher.get_subscription_count() > 0
            and cls.phase_publisher.get_subscription_count() > 0,
            # 视觉接口也必须已经完成发现，避免切换消息在测试开始时丢失。
            timeout=5.0,
        )
        cls._wait_for(
            lambda: cls.task_mode_publisher.get_subscription_count() > 0,
            timeout=5.0,
        )
        cls._wait_for(
            lambda: cls.vision_request_publisher.get_subscription_count() > 0
            and cls.vision_valid_publisher.get_subscription_count() > 0
            and cls.vision_error_publisher.get_subscription_count() > 0,
            timeout=5.0,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _command_callback(cls, message):
        cls.latest_command = message
        cls.command_count += 1

    @classmethod
    def _valid_callback(cls, message):
        cls.latest_control_valid = message.data
        cls.valid_count += 1

    @classmethod
    def _reached_callback(cls, message):
        cls.latest_target_reached = message.data
        cls.reached_count += 1

    @classmethod
    def _visual_active_callback(cls, message):
        cls.latest_visual_control_active = message.data
        cls.visual_active_count += 1

    @classmethod
    def _wait_for(cls, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError("等待 ROS 2 测试条件超时")

    @classmethod
    def _pose(cls, x=0.0, y=0.0, frame="odom", valid_quaternion=True):
        message = PoseStamped()
        message.header.stamp = cls.node.get_clock().now().to_msg()
        message.header.frame_id = frame
        message.pose.position.x = x
        message.pose.position.y = y
        if valid_quaternion:
            message.pose.orientation.w = 1.0
        # valid_quaternion=false 时保持全零四元数，用来验证非法输入保护。
        return message

    @classmethod
    def _publish_state(cls, pose, valid=True):
        cls.pose_publisher.publish(pose)
        valid_message = Bool()
        valid_message.data = valid
        cls.valid_publisher.publish(valid_message)

    @classmethod
    def _publish_visual(cls, requested, valid, forward=0.0, left=0.0, frame="base_link"):
        request_message = Bool()
        request_message.data = requested
        cls.vision_request_publisher.publish(request_message)

        valid_message = Bool()
        valid_message.data = valid
        cls.vision_valid_publisher.publish(valid_message)

        error = Vector3Stamped()
        # 故意不填 header.stamp：节点应按本机接收时间判断新鲜度，降低相机联调复杂度。
        error.header.frame_id = frame
        error.vector.x = forward
        error.vector.y = left
        cls.vision_error_publisher.publish(error)

    @classmethod
    def _wait_for_new_output(cls, predicate, timeout=2.0):
        start_command_count = cls.command_count
        start_valid_count = cls.valid_count
        start_reached_count = cls.reached_count
        start_visual_active_count = cls.visual_active_count
        cls._wait_for(
            lambda: cls.command_count > start_command_count
            and cls.valid_count > start_valid_count
            and cls.reached_count > start_reached_count
            and cls.visual_active_count > start_visual_active_count
            and predicate(),
            timeout=timeout,
        )

    def test_complete_control_and_safety_sequence(self):
        # 未提供坐标时，节点仍持续输出零命令，但 control_valid 必须为 false。
        self._wait_for_new_output(
            lambda: self.latest_control_valid is False
            and self.latest_command.twist.linear.x == 0.0
            and self.latest_command.twist.linear.y == 0.0
            and self.latest_command.twist.angular.z == 0.0
        )

        # YAML 目标为 x=1；当前位置为 0，较小 Kp 产生的输出应被提升到最小速度。
        self._publish_state(self._pose(0.0, 0.0), valid=True)
        self._wait_for_new_output(
            lambda: self.latest_control_valid is True
            and abs(self.latest_command.twist.linear.x - 0.15) < 1.0e-6
            and abs(self.latest_command.twist.linear.y) < 1.0e-12
            and self.latest_target_reached is False
        )

        # topic 新目标覆盖 YAML 目标；当前位姿正好位于新目标，三轴应进入保持。
        self.target_publisher.publish(self._pose(0.0, 0.0))
        self._wait_for_new_output(
            lambda: self.latest_control_valid is True
            and self.latest_target_reached is True
            and self.latest_command.twist.linear.x == 0.0
            and self.latest_command.twist.linear.y == 0.0
            and self.latest_command.twist.angular.z == 0.0
        )

        # 比赛配置禁用全局yaw控制：即使目标yaw与当前相差90度，
        # 也必须保持yaw_rate=0，且不能阻塞平面目标到达。
        yaw_target = self._pose(0.0, 0.0)
        yaw_target.pose.orientation.z = 2.0 ** -0.5
        yaw_target.pose.orientation.w = 2.0 ** -0.5
        self.target_publisher.publish(yaw_target)
        self._publish_state(self._pose(0.0, 0.0), valid=True)
        self._wait_for_new_output(
            lambda: self.latest_control_valid is True
            and self.latest_target_reached is True
            and self.latest_command.twist.linear.x == 0.0
            and self.latest_command.twist.linear.y == 0.0
            and self.latest_command.twist.angular.z == 0.0
        )

        # TAKEOFF_HOVER只放宽前后X：X偏差25cm、Y偏差8cm时应保持且判定到达。
        # 这不能改变后续右移、前追和返航使用的普通二维死区。
        self.phase_publisher.publish(String(data="TAKEOFF_HOVER"))
        time.sleep(0.05)
        rclpy.spin_once(self.node, timeout_sec=0.05)

        self._publish_state(self._pose(-0.25, -0.08), valid=True)
        self._wait_for_new_output(
            lambda: self.latest_control_valid is True
            and self.latest_target_reached is True
            and self.latest_command.twist.linear.x == 0.0
            and self.latest_command.twist.linear.y == 0.0
        )

        # Y仍使用原0.10/0.15m迟滞区；超过15cm后必须恢复Y修正，X仍为零。
        self._publish_state(self._pose(-0.25, -0.16), valid=True)
        self._wait_for_new_output(
            lambda: self.latest_target_reached is False
            and self.latest_command.twist.linear.x == 0.0
            and self.latest_command.twist.linear.y > 0.0
        )

        # 离开悬停阶段后，25cm的X偏差必须重新按普通二维死区跟踪。
        self.phase_publisher.publish(String(data="RIGHT_SHIFT"))
        time.sleep(0.05)
        rclpy.spin_once(self.node, timeout_sec=0.05)
        self._publish_state(self._pose(-0.25, 0.0), valid=True)
        self._wait_for_new_output(
            lambda: self.latest_target_reached is False
            and self.latest_command.twist.linear.x > 0.0
        )

        # 三个地图阶段使用各自的最大合速度；只改变限速，不改变航点或协议。
        self.task_mode_publisher.publish(UInt8(data=1))
        far_target = self._pose(100.0, 0.0)
        self.target_publisher.publish(far_target)
        self._publish_state(self._pose(0.0, 0.0), valid=True)
        self._wait_for_new_output(
            lambda: abs(self.latest_command.twist.linear.x - 0.19) < 1.0e-6
            and abs(self.latest_command.twist.linear.y) < 1.0e-12
        )

        self.phase_publisher.publish(String(data="FORWARD_CHASE"))
        time.sleep(0.05)
        rclpy.spin_once(self.node, timeout_sec=0.05)
        self._publish_state(self._pose(0.0, 0.0), valid=True)
        self._wait_for_new_output(
            lambda: abs(self.latest_command.twist.linear.x - 0.23) < 1.0e-6
            and abs(self.latest_command.twist.linear.y) < 1.0e-12
        )

        self.phase_publisher.publish(String(data="TASK1_RETURN_H"))
        time.sleep(0.05)
        rclpy.spin_once(self.node, timeout_sec=0.05)
        self._publish_state(self._pose(0.0, 0.0), valid=True)
        self._wait_for_new_output(
            lambda: abs(self.latest_command.twist.linear.x - 0.25) < 1.0e-6
            and abs(self.latest_command.twist.linear.y) < 1.0e-12
        )

        # task2切换到独立限速，不能反向改变上面已经验证的task1数值。
        self.task_mode_publisher.publish(UInt8(data=2))
        self.phase_publisher.publish(String(data="RIGHT_SHIFT"))
        time.sleep(0.05)
        rclpy.spin_once(self.node, timeout_sec=0.05)
        self._publish_state(self._pose(0.0, 0.0), valid=True)
        self._wait_for_new_output(
            lambda: abs(self.latest_command.twist.linear.x - 0.23) < 1.0e-6
        )

        self.phase_publisher.publish(String(data="FORWARD_CHASE"))
        time.sleep(0.05)
        rclpy.spin_once(self.node, timeout_sec=0.05)
        self._publish_state(self._pose(0.0, 0.0), valid=True)
        self._wait_for_new_output(
            lambda: abs(self.latest_command.twist.linear.x - 0.25) < 1.0e-6
        )

        self.phase_publisher.publish(String(data="TASK2_ALIGN_LAND"))
        time.sleep(0.05)
        rclpy.spin_once(self.node, timeout_sec=0.05)
        self._publish_state(self._pose(0.0, 0.0), valid=True)
        self._wait_for_new_output(
            lambda: abs(self.latest_command.twist.linear.x - 0.28) < 1.0e-6
        )

        # 恢复后续测试使用的原目标和当前位置。
        self.target_publisher.publish(self._pose(0.0, 0.0))
        self._publish_state(self._pose(0.0, 0.0), valid=True)
        self._wait_for_new_output(
            lambda: self.latest_target_reached is True
            and self.latest_command.twist.linear.x == 0.0
        )

        # 错误 frame 必须立即使当前位姿无效，不能继续使用上一帧。
        self._publish_state(self._pose(frame="odom"), valid=True)
        self._wait_for_new_output(
            lambda: self.latest_control_valid is False
            and self.latest_command.twist.linear.x == 0.0
        )

        # 全零四元数同样属于非法位姿。
        self._publish_state(
            self._pose(valid_quaternion=False), valid=True
        )
        self._wait_for_new_output(
            lambda: self.latest_control_valid is False
            and self.latest_command.twist.angular.z == 0.0
        )

        # 即使位姿本身合法，valid=false 也必须关闭控制。
        self._publish_state(self._pose(), valid=False)
        self._wait_for_new_output(
            lambda: self.latest_control_valid is False
            and self.latest_target_reached is False
        )

        # 恢复合法输入后控制恢复；停止发送后，300 ms 超时保护再次输出无效零速度。
        self._publish_state(self._pose(), valid=True)
        self._wait_for_new_output(
            lambda: self.latest_control_valid is True
            and self.latest_target_reached is True
        )

        # 比赛任务请求视觉模式后，相对位移 x>0 应生成向前速度，且 yaw_rate 必须为 0。
        self._publish_visual(requested=True, valid=True, forward=1.0, left=0.0)
        self._wait_for_new_output(
            lambda: self.latest_control_valid is True
            and self.latest_visual_control_active is True
            and abs(self.latest_command.twist.linear.x - 0.10) < 1.0e-6
            and abs(self.latest_command.twist.linear.y) < 1.0e-12
            and self.latest_command.twist.angular.z == 0.0
        )

        # 视觉目标丢失时不能退回旧的固定航点继续飞，必须立即输出无效零速度。
        self._publish_state(self._pose(), valid=True)
        self._publish_visual(requested=True, valid=False, forward=1.0, left=0.0)
        self._wait_for_new_output(
            lambda: self.latest_control_valid is False
            and self.latest_visual_control_active is False
            and self.latest_command.twist.linear.x == 0.0
            and self.latest_command.twist.angular.z == 0.0
        )

        # 任务管理器明确撤销视觉模式后，才恢复地图固定目标控制。
        self._publish_visual(requested=False, valid=False)
        self._publish_state(self._pose(), valid=True)
        self._wait_for_new_output(
            lambda: self.latest_control_valid is True
            and self.latest_visual_control_active is False
            and self.latest_target_reached is True
        )

        # 停止发送定位输入后，300 ms 超时保护再次输出无效零速度。
        self._wait_for_new_output(
            lambda: self.latest_control_valid is False
            and self.latest_command.twist.linear.x == 0.0,
            timeout=1.5,
        )


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    def test_node_exits_cleanly(self, proc_info):
        """测试结束关闭 launch 时，节点不应因异常崩溃。"""
        launch_testing.asserts.assertExitCodes(proc_info)
