# zed_to_mavros_bridge.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class ZedToMavrosBridge(Node):
    def __init__(self):
        super().__init__('zed_to_mavros_bridge')

        self.subscription = self.create_subscription(
            Odometry,
            '/zedx/zed_node/odom',
            self.odom_callback,
            10)

        self.publisher = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10)

    def odom_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZedToMavrosBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
