import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    namespace = LaunchConfiguration('namespace')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    launch_file_dir = get_package_share_directory('my_launch')
    param_file_dir = os.path.join(launch_file_dir,'params','my_robot.yaml')

    lifecycle_nodes =['map_server','amcl']

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack',
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    start_localization = GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                output = 'screen',
                respawn = use_respawn,
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[param_file_dir],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                output='screen',
                respawn = use_respawn,
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[param_file_dir],
                
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            )
        ]
    )



    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(start_localization)

    return ld