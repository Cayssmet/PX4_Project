"""使用中文 YAML 参数启动树莓派—飞控串口桥。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("flight_controller_bridge"),
        "config",
        "flight_controller_bridge_params.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="flight_controller_bridge",
                executable="flight_controller_bridge_node",
                name="flight_controller_bridge",
                output="screen",
                parameters=[params_file],
            )
        ]
    )
