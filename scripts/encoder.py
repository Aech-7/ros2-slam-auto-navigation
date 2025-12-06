#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class EncoderOdom(Node):
    def __init__(self):
        super().__init__('encoder_odom')

        # PARAMETERS
        self.declare_parameter('wheel_radius', 0.033)  # meters
        self.declare_parameter('wheel_separation', 0.18)
        self.declare_parameter('ticks_per_rev', 360)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value

        # Encoder counts received externally
        self.sub_enc_left = self.create_subscription(
            Odometry, '/encoder_left', self.cb_left, 10)
        self.sub_enc_right = self.create_subscription(
            Odometry, '/encoder_right', self.cb_right, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # State
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.left_last = 0
        self.right_last = 0

    def cb_left(self, msg):
        self.left_last = msg.pose.pose.position.x  # expecting encoder count here

    def cb_right(self, msg):
        self.right_last = msg.pose.pose.position.x  # expecting encoder count here
        self.update_odom()

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Convert ticks to distance
        left_dist = (2 * math.pi * self.wheel_radius) * (self.left_last / self.ticks_per_rev)
        right_dist = (2 * math.pi * self.wheel_radius) * (self.right_last / self.ticks_per_rev)

        d = (right_dist + left_dist) / 2
        th = (right_dist - left_dist) / self.wheel_sep

        # Update pose
        self.x += d * math.cos(self.th + th/2)
        self.y += d * math.sin(self.th + th/2)
        self.th += th

        # Publish odom msg
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th/2)
        odom.pose.pose.orientation.w = math.cos(self.th/2)

        odom.child_frame_id = "base_link"

        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th/2)
        t.transform.rotation.w = math.cos(self.th/2)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdom()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
