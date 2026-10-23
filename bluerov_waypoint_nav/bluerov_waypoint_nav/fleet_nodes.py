from launch_ros.actions import Node


def drone_nodes(d, strip_x, strip_y, cfg, fleet_size=1):
    s = cfg['survey']
    name, model, mav = d['name'], d['model'], d['mavros']
    ox, oy = d['offset']
    # the strip is in drone 1's frame; this drone's odometry starts at its own spawn
    local_x = [v - ox for v in strip_x]
    local_y = [v - oy for v in strip_y]

    odom = f'{mav}/local_position/odom'
    waypoints = f'/{name}/waypoints'
    reached = f'/{name}/waypoint_reached'
    altitude = f'/{name}/altitude'
    cloud = f'/model/{model}/multibeam_sonar/point_cloud'
    imaging = f'/{name}/imaging_active'
    complete = f'/{name}/survey_complete'
    # topics that are hard-coded as /bluerov2/... inside the nodes
    remap = [('/bluerov2/' + t, f'/{name}/' + t) for t in (
        'imaging_active', 'survey_complete', 'waypoint_reached',
        'sonar_left_lateral', 'sonar_left_down',
        'sonar_right_lateral', 'sonar_right_down')]
    out_dir = f"{s['output_root']}/{name}"
    k = d['index']
    shared = cfg.get('fleet_mode') == 'shared_edges' and fleet_size > 1
    prev = (k - 2) % fleet_size + 1                 # the drone whose lane edge I follow
    fleet_params = {
        'fleet_size': fleet_size if shared else 1, 'fleet_index': k - 1,
        'edge_out_topic': f'/{name}/lane_edge', 'edge_in_topic': f'/drone{prev}/lane_edge',
        'frame_offset_x': float(ox), 'frame_offset_y': float(oy),
        'provisional_jump': True} if shared else {}

    def node(pkg, exe, params=None, args=None, rm=True):
        return Node(package=pkg, executable=exe, namespace=name, output='screen',
                    parameters=[params] if params else [],
                    arguments=args or [], remappings=remap if rm else [])

    return [
        node('bluerov_waypoint_nav', 'vision_pose_bridge', {
            'gt_odom_topic': f'/model/{model}/odometry',
            'vision_pose_topic': f'{mav}/vision_pose/pose',
            'publish_rate_hz': 30.0, 'frame_id': 'map'}),
        node('bluerov_waypoint_nav', 'reactive_planner', {
            'polygon_x': local_x, 'polygon_y': local_y,
            'camera_fov_deg': s['fov_deg'], 'nominal_altitude': s['altitude'],
            'step_length': s['step_length'], 'lane_overlap': s['lane_overlap'],
            'lateral_deadband': 0.15, 'max_lateral_correction': 3.0, 'max_lanes': 60,
            'sonar_max_range': s['sonar_max_range'], 'point_cloud_topic': cloud,
            'odom_topic': odom, 'waypoints_topic': waypoints,
            'waypoint_reached_topic': reached, 'frame_id': 'map',
            'boresight_axis': 'axis_x', 'lateral_axis': 'axis_y',
            'lateral_sign': 1.0, 'edge_tol_deg': 2.0, 'settle_delay': 1.0,
            'coverage_log_dir': out_dir, **fleet_params}),
        node('bluerov_waypoint_nav', 'pid_controller', {
            'odom_topic': odom, 'waypoints_topic': waypoints,
            'cmd_vel_topic': f'{mav}/setpoint_velocity/cmd_vel_unstamped',
            'state_topic': f'{mav}/state',
            'set_mode_service': f'{mav}/set_mode',
            'arming_service': f'{mav}/cmd/arming',
            'guided_mode_name': 'GUIDED', 'auto_arm': True,
            'service_retry_period': 2.0, 'control_rate_hz': 20.0,
            'waypoint_tolerance': 0.5, 'control_yaw': True, 'yaw_tolerance': 0.1,
            'use_waypoint_yaw': True,
            'kp_xy': 0.6, 'ki_xy': 0.0, 'kd_xy': 0.15, 'max_lin_vel': 0.5,
            'kp_z': 0.8, 'ki_z': 0.05, 'kd_z': 0.2, 'max_z_vel': s['max_z_vel'],
            'use_sonar_altitude': True, 'altitude_topic': altitude,
            'target_altitude': s['altitude'], 'altitude_hold_tolerance': 0.5,
            'pre_descend': True, 'search_descend_rate': s['descent_rate'],
            'kp_yaw': 1.0, 'ki_yaw': 0.0, 'kd_yaw': 0.1, 'max_yaw_rate': 0.5}),
        node('ros_gz_bridge', 'parameter_bridge',
             args=[f'/model/{model}/camera@sensor_msgs/msg/Image[gz.msgs.Image'], rm=False),
        node('ros_gz_bridge', 'parameter_bridge',
             args=[f'{cloud}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'], rm=False),
        node('bluerov_waypoint_nav', 'sonar_altitude', {
            'point_cloud_topic': cloud, 'altitude_topic': altitude,
            'min_valid_range': 0.4, 'max_valid_range': s['sonar_max_range']}),
        node('bluerov_waypoint_nav', 'waypoint_camera_capture', {
            'image_topic': f'/model/{model}/camera', 'odom_topic': odom,
            'altitude_topic': altitude, 'imaging_active_topic': imaging,
            'survey_complete_topic': complete, 'output_dir': out_dir,
            'max_fps': 10.0, 'image_format': 'png', 'use_imaging_gate': True}),
        node('bluerov_waypoint_nav', 'path_tracker', {
            'odom_topic': f'/model/{model}/pose', 'trail_topic': f'/{name}/trail',
            'frame_id': 'map', 'pose_frame_name': model, 'pose_index': 0,
            'min_spacing': 0.1}),
    ]
