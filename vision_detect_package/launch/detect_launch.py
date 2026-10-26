import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Default parameter file installed with this package
    default_config = os.path.join(
        get_package_share_directory('vision_detect_package'), 'config', 'detect_node.yaml')

    return LaunchDescription([
        # Let the user swap in their own config file
        DeclareLaunchArgument('config', default_value=default_config,
                              description='Path to the parameter YAML file'),
        # Optional: start the ZED publisher too (with_camera:=true)
        DeclareLaunchArgument('with_camera', default_value='false',
                              description='Also launch orca_zed_publisher'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('orca_zed_publisher'),
                'launch', 'zed_publisher_launch.py')),
            condition=IfCondition(LaunchConfiguration('with_camera')),
        ),
        Node(
            package='vision_detect_package',
            executable='detect_node',
            name='vision_detect',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
