#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control your robot!
---------------------------
w : forward
s : backward
a : turn left
d : turn right

q : quit
---------------------------
"""

settings = termios.tcgetattr(sys.stdin)

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed_linear = 0.5
        self.speed_angular = 1.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        print(msg)
        twist = Twist()

        try:
            while rclpy.ok():
                key = self.get_key()
                print("Pressed:", repr(key))


                if key == 'w':          # forward
                    twist.linear.x = self.speed_linear
                    twist.angular.z = 0.0

                elif key == 's':        # backward
                    twist.linear.x = -self.speed_linear
                    twist.angular.z = 0.0

                elif key == 'a':        # turn left
                    twist.linear.x = 0.0
                    twist.angular.z = self.speed_angular

                elif key == 'd':        # turn right
                    twist.linear.x = 0.0
                    twist.angular.z = -self.speed_angular

                elif key == 'q':        # quit
                    print("Exiting teleop...")
                    break

                else:
                    # If no key pressed, stop robot
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.pub.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            self.pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopKeyboard()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
