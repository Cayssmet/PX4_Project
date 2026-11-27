"""2023 电赛 G 题《空地协同智能消防》启动文件（v3.0.0 重构后）。

这一版只拉起「当前代码库里真实存在、且已实测跑过」的节点：
    雷达 -> Cartographer -> TF(map/odom/base_link) -> 坐标提供 -> PID -> [/pid/cmd_vel]

还没写好的三块（飞控适配、K230 视觉桥、赛题任务层、小车链路）在文件末尾
以注释形式占位，节点写好后取消注释即可接入。

设计原则（与计划书 §9.4 一致）：
  * 位置环留在树莓派（position_pid_controller），PX4 只做速度执行器
  * Cartographer 是 odom->base_link 的唯一发布者，任何其他节点都不得再发这条 TF
  * /pid/cmd_vel 是 map(ENU) 世界系速度，转 NED 只需 (vx,vy,vz)->(vy,vx,-vz)

用法：
    ros2 launch uav_bringup competition_g.launch.py
    ros2 launch uav_bringup competition_g.launch.py use_rviz:=true
    ros2 launch uav_bringup competition_g.launch.py lidar_serial_device:=/dev/ttyUSB0
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lidar_share = get_package_share_directory("lslidar_driver")
    cartographer_share = get_package_share_directory("cartographer_config")
    coordinate_share = get_package_share_directory("map_coordinate_provider")
    pid_share = get_package_share_directory("position_pid_controller")

    # ---- launch 参数 ----
    lidar_serial_device = LaunchConfiguration("lidar_serial_device")
    use_rviz = LaunchConfiguration("use_rviz")
    enable_recorder = LaunchConfiguration("enable_recorder")
    flight_log_root = LaunchConfiguration("flight_log_root")
    configuration_basename = LaunchConfiguration("configuration_basename")

    declare_args = [
        DeclareLaunchArgument(
            "lidar_serial_device",
            default_value="/dev/wheeltec_laser",
            description="镭神 N10 串口设备（udev 固定名）",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="是否启动 rviz2（无桌面/走 ssh 时保持 false）",
        ),
        DeclareLaunchArgument(
            "enable_recorder",
            default_value="true",
            description="是否启动飞行数据记录器",
        ),
        DeclareLaunchArgument(
            "flight_log_root",
            default_value=os.path.expanduser("~/ros2_ws/logs/flights"),
            description="飞行日志根目录",
        ),
        DeclareLaunchArgument(
            "configuration_basename",
            default_value="n10_2d.lua",
            description="Cartographer 配置文件名",
        ),
    ]

    # ---- 参数文件 ----
    lidar_params = os.path.join(lidar_share, "params", "lidar_uart_ros2", "lsn10.yaml")
    cartographer_config_dir = os.path.join(
        cartographer_share, "configuration_files"
    )
    coordinate_params = os.path.join(
        coordinate_share, "config", "coordinate_params.yaml"
    )
    pid_params = os.path.join(pid_share, "config", "position_pid_params.yaml")

    # ---- 节点 ----
    # 1) 雷达驱动：发布 /scan（sensor_msgs/LaserScan），不发任何 TF
    lidar = Node(
        package="lslidar_driver",
        executable="lslidar_driver_node",
        name="lslidar_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[lidar_params, {"serial_port_": lidar_serial_device}],
    )

    # 2) base_link -> laser 静态 TF（雷达相对机体的安装位置，目前全 0）
    base_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_static_transform",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "laser",
        ],
    )

    # 3) Cartographer：唯一的 map->odom 与 odom->base_link 发布者
    cartographer = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        arguments=[
            "-configuration_directory", cartographer_config_dir,
            "-configuration_basename", configuration_basename,
        ],
        remappings=[("scan", "/scan")],
    )

    # 4) 栅格地图（只给 rviz / 需要地图的节点用，不参与飞行）
    occupancy_grid = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"resolution": 0.05}],
    )

    # 5) 坐标提供：把 TF 变成 /map/current_pose + /map/current_pose_valid
    coordinate = Node(
        package="map_coordinate_provider",
        executable="map_coordinate_node",
        name="map_coordinate_node",
        output="screen",
        parameters=[coordinate_params],
    )

    # 6) 位置环 PID：输出 /pid/cmd_vel（map 系速度）与 /pid/control_valid
    pid = Node(
        package="position_pid_controller",
        executable="position_pid_node",
        name="position_pid_controller",
        output="screen",
        parameters=[pid_params],
    )

    # 7) 飞行数据记录（可选）
    flight_recorder = Node(
        condition=IfCondition(enable_recorder),
        package="flight_data_recorder",
        executable="flight_data_recorder_node",
        name="flight_data_recorder",
        output="screen",
        parameters=[{"output_root": flight_log_root}],
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    # ---- 分时启动 ----
    # 雷达先跑 3 秒再起 Cartographer，避免刚上电时 /scan 还没稳定就建图
    delayed_mapping = TimerAction(
        period=3.0,
        actions=[base_to_laser, cartographer, occupancy_grid, rviz],
    )
    # 建图起来 3 秒后再起坐标与 PID，保证第一帧 TF 已经可用
    delayed_control = TimerAction(
        period=6.0,
        actions=[coordinate, pid],
    )

    # ---- 待接入（节点写好后取消注释）----
    # px4_bridge = Node(
    #     package="px4_offboard",
    #     executable="px4_mavlink_bridge_node",
    #     name="px4_mavlink_bridge",
    #     output="screen",
    #     parameters=[{"serial_device": "/dev/flight_controller", "baud_rate": 115200}],
    # )
    # k230_bridge = Node(
    #     package="k230_bridge",
    #     executable="k230_bridge_node",
    #     name="k230_bridge",
    #     output="screen",
    #     parameters=[{"serial_device": "/dev/k230", "baud_rate": 115200}],
    # )
    # mission = Node(
    #     package="competition_g",
    #     executable="mission_node",
    #     name="competition_g_mission",
    #     output="screen",
    # )
    # pos_link 不在这里起：它要独占 WiFi 热点与 5000 端口，
    # 单独用 `ros2 launch pos_link pos_link.launch.py` 起更容易看清谁在连。

    return LaunchDescription(
        declare_args
        + [
            lidar,
            flight_recorder,
            delayed_mapping,
            delayed_control,
        ]
    )
