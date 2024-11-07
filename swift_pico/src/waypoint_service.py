#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from waypoint_navigation.srv import GetWaypoints

def coords_list_to_pose(coords):
    pose_array = PoseArray()
    for x, y, z in coords:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0  # Default orientation
        pose_array.poses.append(pose)
    return pose_array

class WaypointsServer(Node):
    def __init__(self):
        super().__init__('waypoints_server')
        self.srv = self.create_service(GetWaypoints, 'GetWaypoints', self.get_waypoints_callback)
        self.get_logger().info("WaypointsServer node has been started.")

    def get_waypoints_callback(self, request:GetWaypoints.Request, response:GetWaypoints.Response):
        self.get_logger().info("Request received.")
        if request.get_waypoints:
            response.waypoints = self.get_waypoints()
            self.get_logger().info("Waypoints have been successfully sent.")
        else:
            self.get_logger().error("Request received but 'get_waypoints' flag is False.")
        return response

    def get_waypoints(self):
        pose_array = PoseArray()
        # Populate the waypoints
        coordinates = [
            [0.0, 0.0, 25.0],
            [0.0, 0.0, 20.0],
            # [2.0, 2.0, 27.0],
            # [2.0, -2.0, 27.0],
            # [-2.0, -2.0, 27.0],
            # [-2.0, 2.0, 27.0],
            # [1.0, 1.0, 27.0]
        ]

        pose_array = coords_list_to_pose(coordinates)

        return pose_array

def main(args=None):
    rclpy.init(args=args)
    waypoints_server = WaypointsServer()

    try:
        rclpy.spin(waypoints_server)
    except KeyboardInterrupt:
        waypoints_server.get_logger().info("WaypointsServer node has been interrupted.")
    finally:
        waypoints_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
