"""启动 pos_link 桥接节点。

    ros2 launch pos_link pos_link.launch.py
    ros2 launch pos_link pos_link.launch.py port:=5000 frame_id:=esp32
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    port = LaunchConfiguration("port")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="5000",
                description="TCP 监听端口（要和 ESP32 固件里的 HOST_PORT 一致）",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="esp32",
                description="PointStamped 的坐标系名；本系统不做 TF 变换，只是个标签",
            ),
            Node(
                package="pos_link",
                executable="bridge",
                name="wifi_bridge",
                output="screen",
                parameters=[{"listen_port": port, "frame_id": frame_id}],
            ),
        ]
    )
