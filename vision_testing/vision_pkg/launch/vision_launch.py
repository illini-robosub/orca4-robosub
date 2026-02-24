from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('vision_pkg')
    rviz_config = os.path.join(pkg_share, 'rviz', 'vision_config.rviz')

    return LaunchDescription([
        Node(
            package='vision_pkg',
            executable='vision_node',
            name='vision_node',
            output='screen'
        ), 
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
