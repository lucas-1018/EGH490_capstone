import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from bluerov_waypoint_nav.fleet_config import build_fleet, load_config, survey_strips
from bluerov_waypoint_nav.fleet_nodes import drone_nodes


def _setup(context, *args, **kwargs):
    cfg = load_config(LaunchConfiguration('config').perform(context))
    fleet = build_fleet(cfg)
    shared = cfg['fleet_mode'] == 'shared_edges' and len(fleet) > 1
    if shared:
        # every drone plans over the whole polygon and follows its neighbour's live edge
        strips = [(cfg['polygon_x'], cfg['polygon_y'])] * len(fleet)
        print(f"[fleet] shared-edge mode: {len(fleet)} drones sweep the whole polygon side by side")
    else:
        strips = survey_strips(cfg, fleet)
    actions = []
    for d, (sx, sy) in zip(fleet, strips):
        if not shared:
            print(f"[fleet] {d['name']}: strip x {min(sx):.1f}..{max(sx):.1f}  "
                  f"(model {d['model']}, mavros {d['mavros']}, offset {d['offset']})")
        actions += drone_nodes(d, sx, sy, cfg, fleet_size=len(fleet))
    return actions


def generate_launch_description():
    default = os.path.join(get_package_share_directory('bluerov_waypoint_nav'),
                           'config', 'fleet.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default,
                              description='path to fleet.yaml'),
        OpaqueFunction(function=_setup)])
