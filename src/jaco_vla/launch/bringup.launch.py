"""One clean controller configuration; no Servo or legacy spawner races."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription,
                            ExecuteProcess, RegisterEventHandler, EmitEvent)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def boolean(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError(f'{name} must be true or false')
    return value == 'true'


def configure(context):
    own = Path(get_package_share_directory('jaco_vla'))
    ada = Path(get_package_share_directory('ada_moveit'))
    profile_path = Path(LaunchConfiguration('hardware_config').perform(context))
    with profile_path.open() as stream:
        hardware = yaml.safe_load(stream)
    if not isinstance(hardware, dict):
        raise ValueError('hardware_config must contain a YAML mapping')
    for key in ('overview_serial', 'wrist_serial', 'imu_serial_port'):
        if not isinstance(hardware.get(key), str) or not hardware[key].strip():
            raise ValueError(f'hardware_config needs a nonempty string {key}')
    if not hardware['imu_serial_port'].startswith('/dev/'):
        raise ValueError('imu_serial_port must be a /dev/ device path')
    sim = LaunchConfiguration('sim').perform(context)
    if sim not in ('mock', 'real'):
        raise ValueError('sim must be mock or real')
    config = (MoveItConfigsBuilder('ada', package_name='ada_moveit')
              .robot_description(mappings={'sim': sim, 'initial_positions_file': str(ada/'config/initial_positions.yaml')})
              .robot_description_kinematics(file_path=str(own/'config/kinematics.yaml'))
              .to_moveit_configs())
    # The old octomap input depends on feeding perception, which is not launched
    # here. Do not claim it works: this stack uses the externally maintained scene.
    config.sensors_3d = {}
    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[config.to_dict(), {'allow_trajectory_execution': False,
            'publish_robot_description_semantic': True, 'publish_planning_scene': True,
            'publish_geometry_updates': True, 'publish_state_updates': True,
            'publish_transforms_updates': True}])
    actions = list(generate_static_virtual_joint_tfs_launch(config).entities)
    actions += [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[config.robot_description]),
        move_group,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('ada_imu'))/'launch/ada_imu.launch.py')),
            launch_arguments={'sim': sim, 'serial_port': hardware['imu_serial_port']}.items()),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(str(
            Path(get_package_share_directory('ada_calibrate_camera'))/'launch/publish_camera_extrinsics_launch.xml')),
            launch_arguments={'calibration_file_name': LaunchConfiguration('calib')}.items()),
    ]
    control = Node(package='controller_manager', executable='ros2_control_node',
        output='screen', parameters=[config.robot_description, str(own/'config/controllers.yaml')])
    spawner = Node(package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', 'jaco_arm_position_controller', 'hand_controller',
                   '--controller-manager', '/controller_manager', '--controller-manager-timeout', '30',
                   '--activate-as-group'])
    client = Node(package='jaco_vla', executable='vla_client', name='vla_client', output='screen',
        parameters=[LaunchConfiguration('client_config'), {
            'instruction': ParameterValue(LaunchConfiguration('instruction'), value_type=str),
            'dry_run': boolean(context, 'dry_run')}])
    # A failed spawner shuts down instead of leaving an apparently ready stack.
    def spawned(event, context):
        return [client] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Controller startup failed'))]
    actions += [RegisterEventHandler(OnProcessExit(target_action=move_group,
                    on_exit=[EmitEvent(event=Shutdown(reason='MoveIt exited'))])),
                RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=spawned)),
                RegisterEventHandler(OnProcessExit(target_action=control,
                    on_exit=[EmitEvent(event=Shutdown(reason='Controller manager exited'))])),
                RegisterEventHandler(OnProcessExit(target_action=client,
                    on_exit=[EmitEvent(event=Shutdown(reason='VLA client exited'))])), control, spawner]
    if boolean(context, 'cameras'):
        camera_launch = str(Path(get_package_share_directory('realsense2_camera'))/'launch/rs_launch.py')
        for name, serial in (('camera', hardware['overview_serial']), ('wrist', hardware['wrist_serial'])):
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={'camera_name': name, 'camera_namespace': 'camera',
                    'serial_no': '_'+serial, 'rgb_camera.profile': '640x480x30',
                    'rgb_camera.enable_auto_exposure': 'false', 'rgb_camera.exposure': '166',
                    'rgb_camera.gain': '32', 'rgb_camera.enable_auto_white_balance': 'false',
                    'rgb_camera.white_balance': '4600.0'}.items()))
    if boolean(context, 'view_images'):
        for topic in ('/camera/camera/color/image_raw', '/camera/wrist/color/image_raw'):
            actions.append(Node(package='image_tools', executable='showimage',
                                arguments=['--ros-args', '-r', 'image:='+topic]))
    if boolean(context, 'rviz'):
        actions.append(Node(package='rviz2', executable='rviz2',
            arguments=['-d', str(ada/'config/moveit.rviz')], parameters=[config.to_dict()]))
    remote = LaunchConfiguration('ssh_target').perform(context)
    if remote:
        if remote.startswith('-') or any(c.isspace() for c in remote):
            raise ValueError('ssh_target must be a single SSH host alias or user@host')
        tunnel = ExecuteProcess(cmd=['ssh', '-N', '-L', '127.0.0.1:8010:127.0.0.1:8000',
            '-o', 'ExitOnForwardFailure=yes', '-o', 'BatchMode=yes', '-o', 'ServerAliveInterval=15',
            '-o', 'ServerAliveCountMax=3', remote], output='screen')
        actions += [RegisterEventHandler(OnProcessExit(target_action=tunnel,
            on_exit=[EmitEvent(event=Shutdown(reason='SSH tunnel exited'))])), tunnel]
    return actions


def generate_launch_description():
    own = Path(get_package_share_directory('jaco_vla'))
    defaults = {'sim': 'mock', 'cameras': 'false', 'view_images': 'false', 'rviz': 'false',
                'calib': 'auto', 'ssh_target': '', 'instruction': '', 'dry_run': 'true',
                'client_config': str(own/'config/client.yaml'),
                'hardware_config': str(own/'config/hardware.yaml')}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=configure)])
