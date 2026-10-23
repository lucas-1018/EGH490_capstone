#!/usr/bin/env python3
import csv
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class WaypointCameraCapture(Node):
    def __init__(self):
        super().__init__('waypoint_camera_capture')

        self.declare_parameter('image_topic', '/model/bluerov2/camera')
        self.declare_parameter('odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('altitude_topic', '/bluerov2/altitude')
        self.declare_parameter('imaging_active_topic', '/bluerov2/imaging_active')
        self.declare_parameter('survey_complete_topic', '/bluerov2/survey_complete')
        self.declare_parameter('output_dir', '/tmp/bluerov_survey_images')
        self.declare_parameter('max_fps', 10.0)
        self.declare_parameter('image_format', 'png')      # 'png' or 'jpg'
        self.declare_parameter('use_imaging_gate', True)
        self.declare_parameter('queue_size', 60)

        g = self.get_parameter
        self.max_period = 1.0 / max(float(g('max_fps').value), 0.1)
        self.fmt = g('image_format').value.lower()
        if self.fmt not in ('png', 'jpg'):
            self.fmt = 'png'
        self.use_gate = bool(g('use_imaging_gate').value)

        run = time.strftime('run_%Y%m%d_%H%M%S')
        self.run_dir = os.path.join(g('output_dir').value, run)
        os.makedirs(self.run_dir, exist_ok=True)
        self.csv_file = open(os.path.join(self.run_dir, 'poses.csv'), 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['file', 'stamp_sec', 'x', 'y', 'z', 'yaw_rad', 'altitude_m',
                           'left_m', 'right_m'])

        self.active = not self.use_gate
        self.complete = False
        self.pose = (float('nan'),) * 4
        self.altitude = float('nan')
        self.left_m = float('nan')     # sonar edge beams (sideways distance)
        self.right_m = float('nan')
        self.last_save_t = -1e9
        self.count = 0
        self.dropped = 0
        self.queue = queue.Queue(maxsize=int(g('queue_size').value))
        self.stop_evt = threading.Event()
        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        best_effort = QoSProfile(depth=10)
        best_effort.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(Image, g('image_topic').value, self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_topic').value, self.on_odom, best_effort)
        self.create_subscription(Float32, g('altitude_topic').value, self.on_alt, 10)
        self.create_subscription(Float32, '/bluerov2/sonar_left_lateral', self.on_left, 10)
        self.create_subscription(Float32, '/bluerov2/sonar_right_lateral', self.on_right, 10)
        self.create_subscription(Bool, g('imaging_active_topic').value,
                                 self.on_active, latched)
        self.create_subscription(Bool, g('survey_complete_topic').value,
                                 self.on_complete, latched)
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f'Continuous capture -> {self.run_dir} (max {1.0 / self.max_period:.0f} fps, '
            f'{self.fmt}, gated by imaging_active={self.use_gate})')

    # ------------------------------------------------------------------
    def on_odom(self, msg: Odometry):
        import math
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, p.z, yaw)

    def on_alt(self, msg: Float32):
        self.altitude = msg.data

    def on_left(self, msg: Float32):
        self.left_m = msg.data

    def on_right(self, msg: Float32):
        self.right_m = msg.data

    def on_active(self, msg: Bool):
        if self.use_gate and msg.data != self.active:
            self.get_logger().info(
                'Imaging ON (sweep)' if msg.data else 'Imaging OFF (repositioning)')
        if self.use_gate:
            self.active = msg.data

    def on_complete(self, msg: Bool):
        if msg.data and not self.complete:
            self.complete = True
            self.get_logger().info(
                f'Survey complete -- stopped capture. {self.count} frames in {self.run_dir}')

    def on_image(self, msg: Image):
        if self.complete or not self.active:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if t - self.last_save_t < self.max_period * 0.9:
            return
        self.last_save_t = t
        name = f'img_{self.count:06d}.{self.fmt}'
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp == 0.0:
            stamp = t
        try:
            self.queue.put_nowait((name, msg, stamp, self.pose, self.altitude, self.left_m, self.right_m))
            self.count += 1
        except queue.Full:
            self.dropped += 1

    def report(self):
        if self.active and not self.complete:
            self.get_logger().info(
                f'captured {self.count} frames (queue {self.queue.qsize()}, '
                f'dropped {self.dropped})')

    # ------------------------------------------------------------------
    def worker_loop(self):
        try:
            from PIL import Image as PILImage
        except ImportError:
            self.get_logger().error('Pillow not installed -- cannot save images')
            return
        while not self.stop_evt.is_set():
            try:
                name, img, stamp, pose, alt, left_m, right_m = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if img.encoding in ('rgb8', 'bgr8'):
                mode = 'RGB'
            elif img.encoding == 'mono8':
                mode = 'L'
            else:
                self.get_logger().error(f'Unsupported encoding: {img.encoding}')
                continue
            pil = PILImage.frombuffer(mode, (img.width, img.height), bytes(img.data),
                                      'raw', mode, img.step, 1)
            if img.encoding == 'bgr8':
                b, g_, r = pil.split()
                pil = PILImage.merge('RGB', (r, g_, b))
            path = os.path.join(self.run_dir, name)
            if self.fmt == 'png':
                pil.save(path, compress_level=1)
            else:
                pil.save(path, quality=95)
            self.csv.writerow([name, f'{stamp:.4f}', f'{pose[0]:.3f}', f'{pose[1]:.3f}',
                               f'{pose[2]:.3f}', f'{pose[3]:.4f}', f'{alt:.3f}',
                               f'{left_m:.3f}', f'{right_m:.3f}'])
            self.csv_file.flush()

    def shutdown(self):
        self.stop_evt.set()
        self.worker.join(timeout=2.0)
        try:
            self.csv_file.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WaypointCameraCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
