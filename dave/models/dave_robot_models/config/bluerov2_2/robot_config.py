"""
robot_config.py for the SECOND BlueROV2 ("bluerov2_2").

Same job as config/bluerov2/robot_config.py, but everything that would clash
with the first vehicle is separate:
  * Gazebo <-> ROS bridge only for /model/bluerov2_2/... (no second /clock)
  * ArduSub SITL instance 1 (-I1): its ports are the first instance's + 10
        TCP 5770 (mavros connects here), JSON physics port 9012
    and its own working directory, so its eeprom.bin is not shared/wiped
  * mavros targets system 1 (what ArduSub reports) but uses its own internal
    channel /uas2, so it does not mix with the first vehicle's /uas1
  * mavros runs in the ROS namespace "drone2" -> topics under /drone2/mavros/

Teleop / QGC / joystick are not started for this vehicle.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node

INSTANCE = 1                      # ArduSub SITL instance number
MAVROS_NAMESPACE = 'drone2'       # -> /drone2/mavros/...
SYSTEM_ID = 1                     # ArduSub reports system id 1 (it ignores SYSID_THISMAV here)
UAS_URL = '/uas2'                 # mavros' internal channel; must differ from drone 1's /uas1
ARDUSUB_HOME = '35.074823,129.084798,0.0,270.0'


def _mavros_params(share):
    """Parameters from DAVE's mavros.yaml, flattened so they apply whatever
    the node is named, with this vehicle's connection settings on top."""
    params = {}
    path = os.path.join(share, 'config', 'mavros', 'mavros.yaml')
    try:
        with open(path) as f:
            doc = yaml.safe_load(f) or {}
        for section in doc.values():
            params.update((section or {}).get('ros__parameters', {}))
    except Exception as e:  # keep going with defaults if the file is odd
        print(f'[bluerov2_2 robot_config] could not read {path}: {e}')
    params['fcu_url'] = f'tcp://127.0.0.1:{5760 + 10 * INSTANCE}'
    params['tgt_system'] = SYSTEM_ID
    params['uas_url'] = UAS_URL
    params['use_sim_time'] = True
    return params


def launch_setup(context, *args, **kwargs):
    namespace = context.launch_configurations.get('namespace', 'bluerov2_2')
    share = get_package_share_directory('dave_robot_models')
    parm = os.path.join(share, 'config', 'bluerov2_2', 'ardusub.parm')
    workdir = f'/tmp/ardusub_{namespace}'
    os.makedirs(workdir, exist_ok=True)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            f'/model/{namespace}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ('/model/' + namespace + '/odometry_with_covariance@'
             'nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance'),
            f'/model/{namespace}/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            f'/model/{namespace}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'/model/{namespace}/magnetometer@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer',
        ],
    )

    wait_for_imu = ExecuteProcess(
        cmd=['/usr/bin/env', 'bash', '-lc',
             'while true; do '
             f'if gz topic -l | grep -q "/model/{namespace}/imu"; then exit 0; fi; '
             'sleep 1; done'],
        output='screen')

    ardusub = ExecuteProcess(
        cmd=['ardusub', '-w', '--model', 'JSON:127.0.0.1', '--defaults', parm,
             f'-I{INSTANCE}', '--home', ARDUSUB_HOME],
        cwd=workdir, output='screen')

    mavros = Node(
        package='mavros', executable='mavros_node', namespace=MAVROS_NAMESPACE,
        output='screen', parameters=[_mavros_params(share)])

    return [
        bridge,
        RegisterEventHandler(OnProcessStart(
            target_action=bridge, on_start=[wait_for_imu])),
        RegisterEventHandler(OnProcessExit(
            target_action=wait_for_imu,
            on_exit=[LogInfo(msg='bluerov2_2: IMU topic found, starting ArduSub '
                                 f'instance {INSTANCE}'), ardusub])),
        RegisterEventHandler(OnProcessStart(
            target_action=ardusub,
            on_start=[LogInfo(msg=f'bluerov2_2: ArduSub started, starting mavros '
                                  f'in /{MAVROS_NAMESPACE}'), mavros])),
    ]


def generate_launch_description():
    # upload_robot.launch.py passes these to every robot_config.py
    names = ['namespace', 'use_teleop', 'use_web_joystick', 'joystick_ws_host',
             'joystick_ws_port', 'zoom_camera', 'zoom_camera_delay', 'open_qgc',
             'open_virtual_joystick', 'virtual_joystick_url', 'ui_launch_delay']
    defaults = {'namespace': 'bluerov2_2'}
    args = [DeclareLaunchArgument(n, default_value=defaults.get(n, ''))
            for n in names]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
