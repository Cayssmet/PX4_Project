# -*- coding: utf-8 -*-
"""启动 px4_node（只读）。

用法：
    ros2 launch px4_offboard px4_offboard.launch.py
    ros2 launch px4_offboard px4_offboard.launch.py serial_device:=/dev/ttyUSB0 baud:=115200

注意：运行前先确认 micro-xrce-agent 没占着这个串口
    sudo systemctl stop micro-xrce-agent
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("px4_offboard")
    default_params = os.path.join(pkg_share, "config", "px4_offboard.yaml")

    serial_device = LaunchConfiguration("serial_device")
    baud = LaunchConfiguration("baud")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_device", default_value="/dev/flight_controller",
            description="飞控串口设备（udev 固定名）"),
        DeclareLaunchArgument(
            "baud", default_value="115200",
            description="波特率，必须等于飞控 SER_TEL3_BAUD"),
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="参数文件"),

        LogInfo(msg=["px4_offboard 只读模式：不发任何控制量。设备=",
                     serial_device, " 波特率=", baud]),

        Node(
            package="px4_offboard",
            executable="px4_node",
            name="px4_node",
            output="screen",
            parameters=[
                params_file,
                {"serial_device": serial_device, "baud": baud},
            ],
        ),
    ])
