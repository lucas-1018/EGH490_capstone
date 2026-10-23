#!/usr/bin/env python3
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32

from bluerov_waypoint_nav.survey_core import (SurveyPlanner, WAIT, lane_yaw,
                                              cloud_to_xyz, extract_edge,
                                              describe_cloud)


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class ReactivePlanner(Node):
    def __init__(self):
        super().__init__('reactive_planner')

        self.declare_parameter('polygon_x', [0.0, 40.0, 40.0, 0.0])
        self.declare_parameter('polygon_y', [0.0, 0.0, 30.0, 30.0])
        self.declare_parameter('camera_fov_deg', 90.0)
        self.declare_parameter('nominal_altitude', 5.0)
        self.declare_parameter('step_length', 3.0)
        self.declare_parameter('lane_overlap', 0.5)
        self.declare_parameter('lateral_deadband', 0.25)
        self.declare_parameter('max_lateral_correction', 3.0)
        self.declare_parameter('max_lanes', 60)
        # sonar range (m): a beam with no return means the floor is beyond this,
        # so the footprint edge is at least range*sin(fov/2) out
        self.declare_parameter('sonar_max_range', 10.0)

        self.declare_parameter('point_cloud_topic', '/model/bluerov2/multibeam_sonar/point_cloud')
        self.declare_parameter('odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('waypoints_topic', '/bluerov2/waypoints')
        self.declare_parameter('waypoint_reached_topic', '/bluerov2/waypoint_reached')
        self.declare_parameter('frame_id', 'map')

        # sonar frame mapping -- verify with the logged diagnostic line!
        self.declare_parameter('boresight_axis', 'axis_x')   # axis pointing out of the sonar (down)
        self.declare_parameter('lateral_axis', 'axis_y')     # axis across the fan
        self.declare_parameter('lateral_sign', 1.0)     # +1 if +lateral_axis is vehicle-left
        self.declare_parameter('edge_tol_deg', 2.0)
        self.declare_parameter('min_edge_points', 3)

        # every sonar frame is logged with the pose + the two edge readings so
        # plot_coverage.py can draw the coverage map afterwards
        self.declare_parameter('coverage_log_dir', '/tmp/bluerov_survey_images')

        # --- shared-edge fleet mode (fleet_size > 1) ------------------------------
        # every drone plans over the whole polygon; neighbours exchange the east
        # edge of the lane they are sweeping (Float32MultiArray
        # [lane, complete, done, y0, e0, y1, e1, ...], in drone 1's frame)
        self.declare_parameter('fleet_size', 1)
        self.declare_parameter('fleet_index', 0)
        self.declare_parameter('edge_out_topic', '/bluerov2/lane_edge')
        self.declare_parameter('edge_in_topic', '')
        self.declare_parameter('frame_offset_x', 0.0)    # this drone's spawn minus drone 1's
        self.declare_parameter('frame_offset_y', 0.0)
        self.declare_parameter('provisional_jump', True)

        self.declare_parameter('settle_delay', 1.0)
        self.declare_parameter('sonar_timeout', 5.0)

        g = self.get_parameter
        self.planner = SurveyPlanner(
            g('polygon_x').value, g('polygon_y').value,
            camera_fov_deg=float(g('camera_fov_deg').value),
            nominal_altitude=float(g('nominal_altitude').value),
            step_length=float(g('step_length').value),
            lane_overlap=float(g('lane_overlap').value),
            lateral_deadband=float(g('lateral_deadband').value),
            max_lateral_correction=float(g('max_lateral_correction').value),
            max_lanes=int(g('max_lanes').value),
            fleet_size=int(g('fleet_size').value), fleet_index=int(g('fleet_index').value),
            provisional_jump=bool(g('provisional_jump').value),
            missing_lateral=float(g('sonar_max_range').value) *
            math.sin(math.radians(float(g('camera_fov_deg').value) / 2.0)))
        self.edge_angle = float(g('camera_fov_deg').value) / 2.0
        self.frame_id = g('frame_id').value
        # launch params go through YAML, where a bare y/n means a boolean, so
        # axes are written 'axis_x' / 'axis_y' / 'axis_z' (bare x/y/z also OK)
        self.boresight = str(g('boresight_axis').value).lower().replace('axis_', '')
        self.lateral = str(g('lateral_axis').value).lower().replace('axis_', '')
        self.lat_sign = float(g('lateral_sign').value)
        self.edge_tol = float(g('edge_tol_deg').value)
        self.min_pts = int(g('min_edge_points').value)
        self.settle_delay = float(g('settle_delay').value)
        self.sonar_timeout = float(g('sonar_timeout').value)

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        best_effort = QoSProfile(depth=10)
        best_effort.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.path_pub = self.create_publisher(Path, g('waypoints_topic').value, latched)
        self.active_pub = self.create_publisher(Bool, '/bluerov2/imaging_active', latched)
        self.done_pub = self.create_publisher(Bool, '/bluerov2/survey_complete', latched)
        # live edge-beam readings for graphing (NaN = beam saw nothing)
        self.edge_pubs = {
            name: self.create_publisher(Float32, f'/bluerov2/sonar_{name}', 10)
            for name in ('left_lateral', 'left_down', 'right_lateral', 'right_down')}

        self.shared = self.planner.shared
        self.off = (float(g('frame_offset_x').value), float(g('frame_offset_y').value))
        if self.shared:
            edge_qos = QoSProfile(depth=200)
            edge_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            edge_qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.edge_out = self.create_publisher(Float32MultiArray,
                                                  g('edge_out_topic').value, edge_qos)
            self.create_subscription(Float32MultiArray, g('edge_in_topic').value,
                                     self.on_edge_in, edge_qos)
            self.get_logger().info(
                f"fleet mode: drone {self.planner.fleet_index + 1} of {self.planner.fleet_size}, "
                f"listening to {g('edge_in_topic').value}, publishing {g('edge_out_topic').value}")

        self.create_subscription(Odometry, g('odom_topic').value, self.on_odom, best_effort)
        self.create_subscription(PointCloud2, g('point_cloud_topic').value,
                                 self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Int32, g('waypoint_reached_topic').value,
                                 self.on_reached, 10)

        self.pose = None            # (x, y, yaw)
        self.cloud_xyz = None
        self.cloud_time = None      # node-clock seconds when the cloud arrived
        self._logged_diag = False
        self.started = False
        self.done = False
        self.pending_since = None   # time of the waypoint_reached event
        self.imaging = False

        log_dir = g('coverage_log_dir').value
        os.makedirs(log_dir, exist_ok=True)
        self.cov_path = os.path.join(log_dir, time.strftime('coverage_%Y%m%d_%H%M%S.csv'))
        self.cov_file = open(self.cov_path, 'w', newline='')
        self.cov = csv.writer(self.cov_file)
        self.cov.writerow(['t', 'x', 'y', 'yaw', 'left_lateral', 'right_lateral',
                           'lane', 'imaging'])
        self.get_logger().info(f'coverage log -> {self.cov_path}')

        self.active_pub.publish(Bool(data=False))
        self.done_pub.publish(Bool(data=False))
        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info(
            'reactive_planner up: waiting for odometry, then publishing the first waypoint.')

    # ------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw)

    def on_cloud(self, msg: PointCloud2):
        offs = {f.name: f.offset for f in msg.fields}
        if not all(k in offs for k in ('x', 'y', 'z')):
            return
        self.cloud_xyz = cloud_to_xyz(bytes(msg.data), msg.point_step,
                                      (offs['x'], offs['y'], offs['z']),
                                      msg.width * msg.height)
        self.cloud_time = self.now()
        lats = self.publish_edges(self.cloud_xyz)
        if self.started and self.pose is not None:
            px, py, yaw = self.pose
            self.cov.writerow([f'{self.now():.3f}', f'{px:.3f}', f'{py:.3f}', f'{yaw:.4f}',
                               f"{lats['left']:.3f}", f"{lats['right']:.3f}",
                               self.planner.lane, int(self.imaging)])
            self.cov_file.flush()
        if not self._logged_diag:
            self._logged_diag = True
            self.get_logger().info(
                'FIRST SONAR CLOUD -> ' + describe_cloud(
                    self.cloud_xyz, self.boresight, self.lateral, self.lat_sign) +
                '   (azimuth should span roughly +/-65 deg; if not, change '
                'boresight_axis/lateral_axis)')

    def publish_edges(self, xyz):
        kw = dict(tol_deg=self.edge_tol, boresight_axis=self.boresight,
                  lateral_axis=self.lateral, lateral_sign=self.lat_sign,
                  min_points=self.min_pts)
        lats = {}
        for side, name in ((+1, 'left'), (-1, 'right')):
            e = extract_edge(xyz, side, self.edge_angle, **kw)
            lat, down = (e[0], e[1]) if e else (float('nan'), float('nan'))
            lats[name] = lat
            self.edge_pubs[f'{name}_lateral'].publish(Float32(data=float(lat)))
            self.edge_pubs[f'{name}_down'].publish(Float32(data=float(down)))
        return lats

    def on_edge_in(self, msg: Float32MultiArray):
        d = list(msg.data)
        if len(d) < 3:
            return
        pts = d[3:]
        ys = [pts[i] - self.off[1] for i in range(0, len(pts) - 1, 2)]
        es = [pts[i + 1] - self.off[0] for i in range(0, len(pts) - 1, 2)]
        self.planner.offer_ref(int(d[0]), d[1] > 0.5, d[2] > 0.5, ys, es)

    def flush_edges(self):
        """Publish what the planner queued for its neighbour (drone-1 frame)."""
        if not self.shared:
            return
        for lane, complete, done, ys, es in self.planner.outbox:
            m = Float32MultiArray()
            data = [float(lane), 1.0 if complete else 0.0, 1.0 if done else 0.0]
            for y, e in zip(ys, es):
                data += [float(y) + self.off[1], float(e) + self.off[0]]
            m.data = data
            self.edge_out.publish(m)
        self.planner.outbox.clear()

    def on_reached(self, msg: Int32):
        if not self.started or self.done:
            return
        self.pending_since = self.now()

    # ------------------------------------------------------------------
    def publish_waypoint(self, wp):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = float(wp.x)
        ps.pose.position.y = float(wp.y)
        ps.pose.position.z = 0.0           # altitude comes from the sonar
        qx, qy, qz, qw = yaw_to_quat(lane_yaw(wp.lane_dir))
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        path.poses.append(ps)
        self.path_pub.publish(path)

    def finish(self):
        self.done = True
        self.imaging = False
        self.active_pub.publish(Bool(data=False))
        self.done_pub.publish(Bool(data=True))
        self.get_logger().info('SURVEY COMPLETE -- ' + self.planner.last_info)

    def tick(self):
        if self.done:
            return

        if not self.started:
            if self.pose is None:
                return
            if not self.planner.start_ready():
                self.get_logger().info('waiting for the drone ahead to measure its first lane...',
                                       throttle_duration_sec=5.0)
                return
            wp = self.planner.start()
            self.started = True
            self.get_logger().info('Start: ' + self.planner.last_info)
            self.publish_waypoint(wp)
            return

        if self.pending_since is None or self.pose is None:
            return
        t = self.now()
        waited = t - self.pending_since
        if waited < self.settle_delay:
            return
        fresh = self.cloud_time is not None and \
            self.cloud_time >= self.pending_since + self.settle_delay
        if not fresh and waited < self.sonar_timeout:
            return
        if not fresh:
            self.get_logger().warn('No fresh sonar cloud -- planning with nominal footprint.')

        px, py, yaw = self.pose
        left = right = None
        if fresh and self.cloud_xyz is not None:
            kw = dict(tol_deg=self.edge_tol, boresight_axis=self.boresight,
                      lateral_axis=self.lateral, lateral_sign=self.lat_sign,
                      min_points=self.min_pts)
            left = extract_edge(self.cloud_xyz, +1, self.edge_angle, **kw)
            right = extract_edge(self.cloud_xyz, -1, self.edge_angle, **kw)

        lane_before = self.planner.lane
        wp = self.planner.next_waypoint(
            px, py, yaw,
            left[0] if left else None, right[0] if right else None)

        self.flush_edges()
        if wp is WAIT:
            self.get_logger().info('holding: waiting for the neighbouring drone\'s edge data',
                                   throttle_duration_sec=5.0)
            return                      # pending_since stays set: retried on the next tick
        self.pending_since = None

        def fmt(e):
            return f'{e[0]:.2f}m(down {e[1]:.2f}, n={e[2]})' if e else 'none'
        self.get_logger().info(
            f'edges L={fmt(left)} R={fmt(right)} | {self.planner.last_info}')

        if wp is None:
            self.finish()
            return
        # False while moving to the next lane (images there are not useful)
        self.imaging = (self.planner.lane == lane_before)
        self.active_pub.publish(Bool(data=self.imaging))
        self.publish_waypoint(wp)
        self.get_logger().info(
            f'-> next waypoint ({wp.x:.2f}, {wp.y:.2f}) lane {self.planner.lane} '
            f'heading {"+Y" if wp.lane_dir > 0 else "-Y"}')


def main(args=None):
    rclpy.init(args=args)
    node = ReactivePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
