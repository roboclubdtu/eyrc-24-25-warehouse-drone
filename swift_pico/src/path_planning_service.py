#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from waypoint_navigation.srv import GetPath

from swift_pico.scripts import path_planning

from pico_utils import PREDEFINED_WP_HEIGHT_M, coords_list_to_pose_array

def dummy_path():
    # Straight line path along Y-axis
    return coords_list_to_pose_array([[0.0, y * 0.1, 27.0] for y in range(21)])
    
class PathPlanningServer(Node):
    def __init__(self):
        super().__init__('path_planning_server')
        self.ros_interfaces_init()


        self.get_logger().info(f"'{self.get_name()}' node has been started.")

    def ros_interfaces_init(self):

        self.srv = self.create_service(GetPath, 'GetPath', self.get_path_cb)


    def get_path_cb(self, request:GetPath.Request, response:GetPath.Response):
        self.get_logger().info("Path planning request received")
        self.get_logger().info(F"Wp1: ({request.waypoints.poses[0].position.x}, {request.waypoints.poses[0].position.y}), Wp2: ({request.waypoints.poses[1].position.x}, {request.waypoints.poses[1].position.y})")

        if self.waypoint_valid(request.waypoints):
            response.path = self.plan_path(request.waypoints)
            self.get_logger().info("Waypoints have been successfully sent.")
        else:
            self.get_logger().error(f"waypoints are invalid. waypoints: {request.waypoints}")
        return response

    def get_waypoints(self):
        pose_array = PoseArray()
        # Populate the waypoints
        coordinates = [
            [0.0, 0.0, 27.0],
            # [0.0, 0.0, 20.0],
            # [2.0, 2.0, 27.0],
            # [2.0, -2.0, 27.0],
            # [-2.0, -2.0, 27.0],
            # [-2.0, 2.0, 27.0],
            # [1.0, 1.0, 27.0]
        ]

        pose_array = coords_list_to_pose_array(coordinates)

        return pose_array

    def waypoint_valid(self, waypoints:PoseArray):
            if len(waypoints.poses) < 2:
                self.get_logger().error("There should be at least two waypoints")
                return False
            wp1 = waypoints.poses[0].position
            wp2 = waypoints.poses[1].position
            if (wp1.x, wp1.y, wp1.z) == (wp2.x, wp2.y, wp2.z):
                self.get_logger().error("Waypoints should not be the same coordinates")
                return False
            return True

    def plan_path(self, waypoints:PoseArray):

        return dummy_path()
    
def main(args=None):
    rclpy.init(args=args)
    waypoints_server = PathPlanningServer()

    try:
        rclpy.spin(waypoints_server)
    except KeyboardInterrupt:
        waypoints_server.get_logger().info("WaypointsServer node has been interrupted.")
    finally:
        waypoints_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
