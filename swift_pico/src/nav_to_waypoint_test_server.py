#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseStamped
from waypoint_navigation.action import NavToWaypoint
import time

class NavToWaypointActionServer(Node):

    def __init__(self):
        super().__init__('nav_to_waypoint_action_server')

        # Initialize the action server
        self._action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            self.execute_callback
        )

        self.get_logger().info("NavToWaypoint action server has been started.")

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Received a goal request.")
        result = NavToWaypoint.Result()

        # Dummy feedback
        feedback_msg = NavToWaypoint.Feedback()
        feedback_msg.current_waypoint = PoseStamped()
        feedback_msg.current_waypoint.pose = Pose()

        start_time = time.time()

        # Publish feedback every 0.5 seconds, for 3 seconds
        while (time.time() - start_time) < 3.0:
            self.get_logger().info("Publishing feedback...")

            # Fill feedback with dummy data
            feedback_msg.current_waypoint.pose.position.x += 0.1
            feedback_msg.current_waypoint.pose.position.y += 0.1
            feedback_msg.current_waypoint.pose.position.z += 0.1

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        # Return the result
        result.hov_time = 3  # Example hover time
        self.get_logger().info("Goal succeeded.")

        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = NavToWaypointActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass

    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
