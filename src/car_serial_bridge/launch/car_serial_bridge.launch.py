"""启动D题小车统一协议串口桥。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params = os.path.join(get_package_share_directory("car_serial_bridge"),
                          "config", "car_serial_bridge_params.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("car_serial_device", default_value="/dev/car_bluetooth",
                              description="小车蓝牙串口的持久别名"),
        Node(package="car_serial_bridge", executable="car_serial_bridge_node",
             name="car_serial_bridge", output="screen",
             parameters=[params, {"serial_device": LaunchConfiguration("car_serial_device")}]),
    ])
