#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage


class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')
        self.declare_parameter('odom_topic', '/model/bluerov2/pose')
        self.declare_parameter('trail_topic', '/bluerov2/trail')
        self.declare_parameter('min_spacing', 0.1)      # m between stored points
        self.declare_parameter('max_points', 20000)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('pose_frame_name', 'bluerov2')  # for TFMessage
        # for a PoseArray (a list with no names): which entry is the vehicle.
        # Entry 0 is normally the model itself, the rest are its links.
        self.declare_parameter('pose_index', 0)

        g = self.get_parameter
        self.topic = g('odom_topic').value
        self.min_spacing = float(g('min_spacing').value)
        self.max_points = int(g('max_points').value)
        self.frame_id = g('frame_id').value
        self.frame_name = g('pose_frame_name').value
        self.pose_index = int(g('pose_index').value)

        self.pub = self.create_publisher(Path, g('trail_topic').value, 10)
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last = None
        self.sub = None

        # type of the pose topic is looked up once it exists
        self.finder = self.create_timer(1.0, self.find_type)
        self.get_logger().info(f'path_tracker: waiting for {self.topic} ...')

    # ------------------------------------------------------------------
    def find_type(self):
        types = dict(self.get_topic_names_and_types()).get(self.topic)
        if not types:
            return
        t = types[0]
        table = {
            'nav_msgs/msg/Odometry': (Odometry, lambda m: m.pose.pose),
            'geometry_msgs/msg/PoseStamped': (PoseStamped, lambda m: m.pose),
            'geometry_msgs/msg/Pose': (Pose, lambda m: m),
            'geometry_msgs/msg/PoseWithCovarianceStamped':
                (PoseWithCovarianceStamped, lambda m: m.pose.pose),
            'geometry_msgs/msg/PoseArray': (PoseArray, self.pose_from_array),
            'tf2_msgs/msg/TFMessage': (TFMessage, self.pose_from_tf),
        }
        if t not in table:
            self.get_logger().error(
                f'{self.topic} has unsupported type {t}. Supported: {list(table)}')
            self.finder.cancel()
            return
        msg_cls, getter = table[t]
        self.getter = getter
        best_effort = QoSProfile(depth=10)
        best_effort.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(msg_cls, self.topic, self.on_msg, best_effort)
        self.finder.cancel()
        self.get_logger().info(f'path_tracker: {self.topic} ({t}) -> trail, true world pose')

    def pose_from_array(self, msg: PoseArray):
        if self.pose_index >= len(msg.poses):
            return None
        return msg.poses[self.pose_index]

    def pose_from_tf(self, msg: TFMessage):
        pick = None
        for tf in msg.transforms:
            name = tf.child_frame_id.split('/')[-1]
            if tf.child_frame_id == self.frame_name or name == self.frame_name:
                pick = tf
                break
        if pick is None:
            for tf in msg.transforms:
                if self.frame_name in tf.child_frame_id:
                    pick = tf
                    break
        if pick is None:
            return None
        p = Pose()
        p.position.x = pick.transform.translation.x
        p.position.y = pick.transform.translation.y
        p.position.z = pick.transform.translation.z
        p.orientation = pick.transform.rotation
        return p

    def on_msg(self, msg):
        pose = self.getter(msg)
        if pose is None:
            return
        p = pose.position
        if self.last is not None:
            dx, dy, dz = p.x - self.last[0], p.y - self.last[1], p.z - self.last[2]
            if (dx * dx + dy * dy + dz * dz) ** 0.5 < self.min_spacing:
                return
        self.last = (p.x, p.y, p.z)

        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose
        self.path.poses.append(ps)
        if len(self.path.poses) > self.max_points:
            del self.path.poses[:len(self.path.poses) - self.max_points]
        self.path.header.stamp = ps.header.stamp
        self.pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
