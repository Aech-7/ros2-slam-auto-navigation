#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
import math

class WaypointClient(Node):
    def __init__(self):
        super().__init__('waypoint_client')

        # Create action client
        self._client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # Define your waypoints (x, y, yaw_deg)
        self.waypoints = [
            (1.3, 0.0, 0),
            (2.0, -0.5, 90),
            (3.0, 0.0, 180),
            (2.0, -0.5, -90)
        ]

    # Convert yaw (deg) → quaternion
    def quaternion_from_yaw(self, yaw_deg):
        yaw = math.radians(yaw_deg)
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    # Convert (x, y, yaw_deg) into PoseStamped
    def pose_from_xyyaw(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        qx, qy, qz, qw = self.quaternion_from_yaw(yaw_deg)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def send_waypoints(self):
        # Prepare FollowWaypoints goal
        goal_msg = FollowWaypoints.Goal()

        # Convert each (x,y,yaw) into PoseStamped
        for wp in self.waypoints:
            x, y, yaw = wp
            goal_msg.poses.append(self.pose_from_xyyaw(x, y, yaw))

        self.get_logger().info("Sending waypoint list to Nav2...")

        # Wait for Nav2
        self._client.wait_for_server()

        # Send goal
        self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(
            f"Robot reached waypoint index: {feedback.feedback.current_waypoint}"
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Waypoints rejected by Nav2!")
            return

        self.get_logger().info("Waypoints accepted.")

        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info("All waypoints completed!")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointClient()
    node.send_waypoints()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
