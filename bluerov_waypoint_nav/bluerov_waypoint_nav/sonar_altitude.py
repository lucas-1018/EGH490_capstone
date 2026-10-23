#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Float32


class SonarAltitude(Node):
    def __init__(self):
        super().__init__('sonar_altitude')

        self.declare_parameter('point_cloud_topic', '/model/bluerov2/multibeam_sonar/point_cloud')
        self.declare_parameter('altitude_topic', '/bluerov2/altitude')
        self.declare_parameter('min_valid_range', 0.4)
        self.declare_parameter('max_valid_range', 10.0)

        self.min_r = float(self.get_parameter('min_valid_range').value)
        self.max_r = float(self.get_parameter('max_valid_range').value)

        self.pub = self.create_publisher(Float32, self.get_parameter('altitude_topic').value, 10)
        self.create_subscription(
            PointCloud2, self.get_parameter('point_cloud_topic').value, self.on_cloud, 5)

        self.get_logger().info(
            f"sonar_altitude: {self.get_parameter('point_cloud_topic').value} -> "
            f"{self.get_parameter('altitude_topic').value}")

    def on_cloud(self, msg: PointCloud2):
        best = None
        for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            r = math.sqrt(x * x + y * y + z * z)
            if self.min_r <= r <= self.max_r and (best is None or r < best):
                best = r
        if best is not None:
            self.pub.publish(Float32(data=float(best)))


def main(args=None):
    rclpy.init(args=args)
    node = SonarAltitude()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
