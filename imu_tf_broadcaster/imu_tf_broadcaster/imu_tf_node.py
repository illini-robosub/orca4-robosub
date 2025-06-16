#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from rclpy.qos import qos_profile_sensor_data

class IMUTFBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile_sensor_data
        )

    def imu_callback(self, msg):
        # print("imu callback triggered")
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = IMUTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
