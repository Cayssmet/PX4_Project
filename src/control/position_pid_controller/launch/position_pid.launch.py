"""启动 x/y/yaw 位置 PID 控制器，并默认加载包内中文参数文件。"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # 安装后参数文件位于 share/position_pid_controller/config/ 下。
    default_params_file = os.path.join(
        get_package_share_directory("position_pid_controller"),
        "config",
        "position_pid_params.yaml",
    )

    params_file_argument = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="位置 PID 参数文件的完整路径",
    )

    pid_node = Node(
        package="position_pid_controller",
        executable="position_pid_node",
        name="position_pid_controller",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_file_argument, pid_node])
