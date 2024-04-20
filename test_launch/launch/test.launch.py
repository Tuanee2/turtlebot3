import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    package_share_directory = get_package_share_directory("test_launch")
    param_file = os.path.join(package_share_directory,"params","amcl.yaml")

    nav2_amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        parameters=[param_file],
    )

    ld.add_action(nav2_amcl_node)
    return ld
