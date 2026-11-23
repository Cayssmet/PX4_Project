"""使用中文 YAML 参数启动航点管理器。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("waypoint_manager"),
        "config",
        "waypoint_manager_params.yaml",
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="航点管理器 YAML 参数文件路径",
            ),
            Node(
                package="waypoint_manager",
                executable="waypoint_manager_node",
                name="waypoint_manager",
                output="screen",
                parameters=[params_file],
            )
        ]
    )
