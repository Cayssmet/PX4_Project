"""pos_link 的 launch 入口。

    ros2 launch pos_link pos_link.launch.py
    ros2 launch pos_link pos_link.launch.py input_topic:=/mission/target port:=5000

不在这里拉起坐标源——坐标从哪来是上层的事，本包只负责把话题里的坐标
推给 ESP32。没有坐标源时节点会稳定地打印「还没有坐标」，不会崩。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    port = LaunchConfiguration("port")
    input_topic = LaunchConfiguration("input_topic")
    period_ms = LaunchConfiguration("send_period_ms")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port", default_value="5000", description="TCP 监听端口（ESP32 连这个端口）"
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value="/pos_link/target",
                description="坐标来源话题（geometry_msgs/PointStamped）",
            ),
            DeclareLaunchArgument(
                "send_period_ms", default_value="1000", description="下发周期，毫秒"
            ),
            Node(
                package="pos_link",
                executable="bridge",
                name="pos_link_bridge",
                output="screen",
                parameters=[
                    {
                        "listen_port": ParameterValue(port, value_type=int),
                        "input_topic": ParameterValue(input_topic, value_type=str),
                        "send_period_ms": ParameterValue(period_ms, value_type=int),
                    }
                ],
            ),
        ]
    )
