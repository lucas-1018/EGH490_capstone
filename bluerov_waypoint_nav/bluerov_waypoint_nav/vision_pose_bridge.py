import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class VisionPoseBridge(Node):
    def __init__(self):
        super().__init__('vision_pose_bridge')

        self.declare_parameter('gt_odom_topic', '/model/bluerov2/odometry')
        self.declare_parameter('vision_pose_topic', '/mavros/vision_pose/pose')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('frame_id', 'map')

        self.frame_id = self.get_parameter('frame_id').value
        self.latest_pose = None

        self.create_subscription(
            Odometry, self.get_parameter('gt_odom_topic').value, self.on_odom, 10)
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter('vision_pose_topic').value, 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.publish_pose)

        self.get_logger().info(
            f"Bridging {self.get_parameter('gt_odom_topic').value} -> "
            f"{self.get_parameter('vision_pose_topic').value} at {rate} Hz")

    def on_odom(self, msg: Odometry):
        self.latest_pose = msg.pose.pose

    def publish_pose(self):
        if self.latest_pose is None:
            return
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id
        ps.pose = self.latest_pose
        self.pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = VisionPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
