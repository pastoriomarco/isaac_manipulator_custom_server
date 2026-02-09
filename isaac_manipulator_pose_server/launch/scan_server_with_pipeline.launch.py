import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('isaac_manipulator_pose_server')
    bringup_share = get_package_share_directory('isaac_manipulator_bringup')

    default_bt_config = os.path.join(package_share, 'params', 'pose_server.yaml')
    default_workflow_config = os.path.join(
        bringup_share, 'params', 'ur5e_robotiq_85_soup_can.yaml')

    launch_args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=default_bt_config,
            description='Path to the custom pose server configuration file.'
        ),
        DeclareLaunchArgument(
            'launch_pipeline',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to launch isaac_manipulator pipeline bringup.'
        ),
        DeclareLaunchArgument(
            'manipulator_workflow_config',
            default_value=default_workflow_config,
            description='Isaac Manipulator workflow config YAML passed to workflows.launch.py.'
        ),
    ]

    workflow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'workflows.launch.py')
        ),
        launch_arguments={
            'manipulator_workflow_config': LaunchConfiguration('manipulator_workflow_config')
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_pipeline')),
    )

    server = Node(
        package='isaac_manipulator_pose_server',
        executable='multi_object_pose_server',
        name='multi_object_pose_server',
        output='screen',
        arguments=[
            '--config-file', LaunchConfiguration('config_file'),
        ],
    )

    return LaunchDescription(launch_args + [workflow_launch, server])
