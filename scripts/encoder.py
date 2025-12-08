#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import math
import tf2_ros

class EncoderOdom(Node):
    def __init__(self):
        super().__init__('encoder_odometry')

        self.declare_parameter('wheel_separation', 0.30)
        self.declare_parameter('wheel_radius', 0.05)

        self.L = self.get_parameter('wheel_separation').value
        self.R = self.get_parameter('wheel_radius').value

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.last_time = self.get_clock().now()

        # encoder topic example: custom interface giving left & right distance
        self.sub = self.create_subscription(
            Odometry, '/wheel/encoder', self.encoder_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def encoder_callback(self, msg):

        # distance traveled by wheels in this time step
        dl = msg.twist.twist.linear.x       # left  wheel distance (m)
        dr = msg.twist.twist.linear.y       # right wheel distance (m)

        dt = (self.get_clock().now() - self.last_time).nanoseconds * 1e-9
        self.last_time = self.get_clock().now()

        # velocities
        v = (dr + dl) / 2.0
        w = (dr - dl) / self.L

        # integrate pose
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.th += w * dt

        # publish odom message
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.header.stamp = self.get_clock().now().to_msg()

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th/2)
        odom.pose.pose.orientation.w = math.cos(self.th/2)

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        # broadcast TF transform
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id  = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdom()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
