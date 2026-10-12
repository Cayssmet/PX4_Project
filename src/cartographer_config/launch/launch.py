import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    cartographer_config_dir = os.path.join(
        get_package_share_directory('cartographer_config'), 'configuration_files')
    configuration_basename = LaunchConfiguration('configuration_basename')

    cartographer_prefix = get_package_share_directory('cartographer_ros')

    return LaunchDescription([
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='n10_2d.lua',
            description='Lua配置文件名'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否启动RViz2'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            remappings=[('scan', '/scan')]),
            
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05}]),
            
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
    ])
