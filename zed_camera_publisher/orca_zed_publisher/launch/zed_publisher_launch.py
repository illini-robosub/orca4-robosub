import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Default parameter file installed with this package
    default_config = os.path.join(
        get_package_share_directory('orca_zed_publisher'), 'config', 'zed_publisher.yaml')

    return LaunchDescription([
        # Let the user swap in their own config file
        DeclareLaunchArgument('config', default_value=default_config,
                              description='Path to the parameter YAML file'),
        # Optional: play a recording instead of the live camera
        DeclareLaunchArgument('svo_file', default_value='',
                              description='Path to a .svo file (empty = live camera)'),
        Node(
            package='orca_zed_publisher',
            executable='zed_publisher_node',
            name='zed_publisher',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'svo_file': LaunchConfiguration('svo_file')},  # overrides the YAML value
            ],
        ),
    ])
