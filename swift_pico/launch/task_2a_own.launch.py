import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pico_server = Node(
        package='swift_pico',
        executable='pico_server.py',
        name='pico_server'

    )
    pico_cilent = Node(
        package='swift_pico',
        executable='pico_client.py',
        name='pico_client'

    )
    waypoint_service = Node(
        package='swift_pico',
        executable='waypoint_service.py',
        name='waypoint_service'

    )

    return LaunchDescription([
        # pico_server, 
        waypoint_service,
        pico_cilent,
    ])
