import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from bluerov_waypoint_nav.fleet_config import (build_fleet, load_config,
                                               remove_drone_files, write_drone_files)


def _setup(context, *args, **kwargs):
    cfg = load_config(LaunchConfiguration('config').perform(context))
    fleet = build_fleet(cfg)
    share = get_package_share_directory('dave_robot_models')
    dave_launch = os.path.join(get_package_share_directory('dave_demos'),
                               'launch', 'dave_robot.launch.py')
    delay = float(cfg['timing']['spawn_delay'])

    for d in fleet[1:]:
        write_drone_files(share, d)
        print(f"[fleet] generated files for {d['model']} (ArduSub instance "
              f"{d['instance']}, system id {d['sysid']}) in {share}")

    def cleanup(ctx, *a, **k):
        for d in fleet[1:]:
            remove_drone_files(share, d)
        print('[fleet] removed generated drone files')

    actions = [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]

    # Drone 1: DAVE's stock launch (starts Gazebo, teleop, joystick).
    first = fleet[0]
    sp = first['spawn']
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dave_launch),
        launch_arguments={
            'gui': 'true',
            'world_name': cfg['world_name'],
            'namespace': first['model'],
            'x': str(sp['x']), 'y': str(sp['y']), 'z': str(sp['z']),
            'yaw': str(sp['yaw']),
            'use_teleop': 'true' if cfg['drone1_teleop'] else 'false',
            'use_web_joystick': 'true' if cfg['drone1_teleop'] else 'false',
        }.items()))

    # Drones 2..N: DAVE's upload_robot.launch.py reads 'namespace' from the shared
    # launch context when its exit handler fires, so a second drone ends up with
    # drone 1's config. Do the same two steps here with literal values instead.
    for d in fleet[1:]:
        sp = d['spawn']
        model = d['model']
        spawner = Node(
            package='ros_gz_sim', executable='create', output='both',
            arguments=['-name', model,
                       '-file', os.path.join(share, 'description', model, 'model.sdf'),
                       '-x', str(sp['x']), '-y', str(sp['y']), '-z', str(sp['z']),
                       '-R', '0.0', '-P', '0.0', '-Y', str(sp['yaw'])],
            parameters=[{'use_sim_time': True}])
        robot_config = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'config', model, 'robot_config.py')),
            launch_arguments={
                'namespace': model,
                'use_teleop': 'false', 'use_web_joystick': 'false',
            }.items())
        handler = RegisterEventHandler(OnProcessExit(
            target_action=spawner,
            on_exit=[LogInfo(msg=f'{model} spawned, starting its ArduSub and mavros'),
                     robot_config]))
        actions.append(TimerAction(period=(d['index'] - 1) * delay,
                                   actions=[spawner, handler]))
    return actions


def generate_launch_description():
    default = os.path.join(get_package_share_directory('bluerov_waypoint_nav'),
                           'config', 'fleet.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default,
                              description='path to fleet.yaml'),
        OpaqueFunction(function=_setup)])
