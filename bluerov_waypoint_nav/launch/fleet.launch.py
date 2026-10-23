import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from bluerov_waypoint_nav.fleet_config import load_config


def _setup(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context)
    cfg = load_config(config)
    pkg = get_package_share_directory('bluerov_waypoint_nav')
    t = cfg['timing']
    wait = (cfg['num_drones'] - 1) * float(t['spawn_delay']) + float(t['survey_delay'])
    print(f"[fleet] {cfg['num_drones']} drone(s); survey starts in {wait:.0f} s")
    args = {'config': config}.items()
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'fleet_sim.launch.py')),
        launch_arguments=args)
    survey = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'fleet_survey.launch.py')),
        launch_arguments=args)
    return [sim, TimerAction(period=wait, actions=[survey])]


def generate_launch_description():
    default = os.path.join(get_package_share_directory('bluerov_waypoint_nav'),
                           'config', 'fleet.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default,
                              description='path to fleet.yaml'),
        OpaqueFunction(function=_setup)])
