"""Attach the new client to an existing stack without switching controllers."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('jaco_vla'))/'config/client.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('client_config', default_value=config),
        DeclareLaunchArgument('instruction', default_value=''),
        DeclareLaunchArgument('dry_run', default_value='true'),
        Node(package='jaco_vla', executable='vla_client', name='vla_client', output='screen',
             parameters=[LaunchConfiguration('client_config'), {
                 'instruction': ParameterValue(LaunchConfiguration('instruction'), value_type=str),
                 'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}]),
    ])
