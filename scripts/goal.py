#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math


class MultiGoalNav2Client(Node):
    def __init__(self):
        super().__init__('multi_goal_nav2_client')

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --------------------------
        # Define your list of goals
        # Each goal: (x, y, yaw_deg)
        # --------------------------
        self.goals = [
            (1.3, 0.0, 0),
            (2.0, 0.5, 90),
            (3.0, 0.0, 180),
            (2.0, -0.5, -90)
        ]

        self.current_goal_index = 0


    # ------------------------------------
    # Convert a goal into a PoseStamped
    # ------------------------------------
    def create_pose(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        yaw = math.radians(yaw_deg)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose


    # ------------------------------------
    # Main goal sending function
    # ------------------------------------
    def send_next_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info("🎉 All goals completed!")
            rclpy.shutdown()
            return

        x, y, yaw = self.goals[self.current_goal_index]
        pose = self.create_pose(x, y, yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f"➡️ Sending goal {self.current_goal_index + 1}/{len(self.goals)}:"
            f" (x={x}, y={y}, yaw={yaw})"
        )

        self._client.wait_for_server()

        future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected!")
            return

        self.get_logger().info("✔️ Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)


    def result_callback(self, future):
        self.get_logger().info("🏁 Goal reached!")
        
        # Move to the next goal
        self.current_goal_index += 1
        self.send_next_goal()


    def feedback_callback(self, feedback_msg):
        p = feedback_msg.feedback.current_pose.pose.position
        self.get_logger().info(f"   📍 Current robot position: ({p.x:.2f}, {p.y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalNav2Client()
    node.send_next_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
