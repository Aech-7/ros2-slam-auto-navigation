#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class SimpleNav2Client(Node):
    def __init__(self):
        super().__init__('simple_nav2_client')

        # Create action client for NavigateToPose
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw_deg):
        goal_msg = NavigateToPose.Goal()

        # Fill in target pose in MAP frame
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        # yaw → quaternion
        import math
        yaw = math.radians(yaw_deg)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal_msg.pose = pose

        # Wait for Nav2
        self.get_logger().info('Waiting for Nav2 action server...')
        self._client.wait_for_server()
        self.get_logger().info('Nav2 action server available! Sending goal...')

        # Send goal
        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted!')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation finished with result: {result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        p = feedback_msg.feedback.current_pose.pose.position
        self.get_logger().info(f'Current position: ({p.x:.2f}, {p.y:.2f})')


def main(args=None):
    rclpy.init(args=args)

    node = SimpleNav2Client()

    # ---------------------------------------
    # Set your goal here in MAP FRAME:
    # x, y, yaw_deg
    # ---------------------------------------
    GOAL_X = 1.3       # meters
    GOAL_Y = 0
    GOAL_YAW = 0       # degrees

    node.send_goal(GOAL_X, GOAL_Y, GOAL_YAW)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
