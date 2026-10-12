"""启动D题摄像头统一协议串口桥。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("camera_serial_bridge"),
        "config", "camera_serial_bridge_params.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("camera_serial_device", default_value="/dev/ttyAMA0"),
        Node(
            package="camera_serial_bridge",
            executable="camera_serial_bridge_node",
            name="camera_serial_bridge",
            output="screen",
            parameters=[params, {"serial_device": LaunchConfiguration("camera_serial_device")}],
        ),
    ])
