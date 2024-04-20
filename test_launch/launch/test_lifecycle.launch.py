import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='',
            remappings=[('/tf', 'tf'), ('/scan', 'scan')],
            parameters=[{'use_sim_time': True}],
            arguments=['--ros-args', '--log-level', 'info'],
        )
    ])