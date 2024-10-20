import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('waypoint_navigation'),
        'config',
        'waypoint_navigation_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='waypoint_navigation',
            executable='waypoint_nav_node',
            name='waypoint_nav_node',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='waypoint_navigation',
            executable='waypoint_nav_py_node',
            name='waypoint_nav_py_node',
            output='screen',
            parameters=[config],
        ),
    ])
