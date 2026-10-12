"""启动D题树莓派—地面站统一协议串口桥。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("gcs_serial_bridge"),
        "config",
        "gcs_serial_bridge_params.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gcs_serial_device",
                default_value="/dev/gcs_bluetooth",
                description="地面站蓝牙串口的持久别名",
            ),
            Node(
                package="gcs_serial_bridge",
                executable="gcs_serial_bridge_node",
                name="gcs_serial_bridge",
                output="screen",
                parameters=[
                    params,
                    {"serial_device": LaunchConfiguration("gcs_serial_device")},
                ],
            ),
        ]
    )

