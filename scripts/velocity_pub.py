#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelToWheels(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_wheels')

        # Robot parameters
        self.wheel_radius = 0.05   # meters
        self.wheel_base   = 0.30   # distance between wheels (m)

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

    def cmd_callback(self, msg):
        v = msg.linear.x      # Forward speed (m/s)
        w = msg.angular.z     # Angular speed (rad/s)

        L = self.wheel_base
        R = self.wheel_radius

        # Differential drive equations
        v_left  = v - w * (L / 2.0)
        v_right = v + w * (L / 2.0)

        # Convert to rad/s
        wl = v_left / R
        wr = v_right / R

        # Convert to RPM
        rpm_left  = wl * 60 / (2 * 3.14159)
        rpm_right = wr * 60 / (2 * 3.14159)

        self.get_logger().info(
            f"Left wheel: {rpm_left:.2f} RPM | Right wheel: {rpm_right:.2f} RPM"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWheels()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
