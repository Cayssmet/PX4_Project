"""启动新版完整地图位姿获取节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 默认读取安装目录中的中文参数文件，也支持启动时传入另一份文件。
    default_params_file = os.path.join(
        get_package_share_directory("map_coordinate_provider"),
        "config",
        "coordinate_params.yaml",
    )

    params_file_argument = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="完整地图位姿获取节点的参数文件路径",
    )

    coordinate_node = Node(
        package="map_coordinate_provider",
        executable="map_coordinate_node",
        name="map_coordinate_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_file_argument, coordinate_node])
