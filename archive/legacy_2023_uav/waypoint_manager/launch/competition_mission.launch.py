"""启动D题预追逐任务状态机；默认等待小车无线启动指令。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("waypoint_manager"),
        "config",
        "competition_mission_params.yaml",
    )

    params_file = LaunchConfiguration("params_file")
    auto_start_for_test = LaunchConfiguration("auto_start_for_test")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="D题比赛任务参数文件",
            ),
            DeclareLaunchArgument(
                "auto_start_for_test",
                default_value="false",
                description="仅无桨地面测试可设true；比赛必须等待小车启动信号",
            ),
            Node(
                package="waypoint_manager",
                executable="competition_mission_node",
                name="competition_mission_manager",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "auto_start_for_test": ParameterValue(
                            auto_start_for_test, value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
