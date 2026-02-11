import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('isaac_manipulator_pose_server')
    default_config = os.path.join(package_share, 'params', 'pose_server.yaml')

    launch_args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to the workflow configuration file.'
        ),
    ]

    server = Node(
        package='isaac_manipulator_pose_server',
        executable='multi_object_pose_server_direct',
        name='multi_object_pose_server_direct',
        output='screen',
        arguments=[
            '--config-file', LaunchConfiguration('config_file'),
        ],
    )

    return LaunchDescription(launch_args + [server])
