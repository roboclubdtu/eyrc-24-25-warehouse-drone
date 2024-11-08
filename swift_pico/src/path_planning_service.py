#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image
from waypoint_navigation.srv import GetPath

import numpy as np
from cv_bridge import CvBridge

# from swift_pico.scripts import bit_map
from swift_pico.scripts import path_planning
# from swift_pico.scripts

from pico_utils import BITMAP_PATH, PREDEFINED_WP_HEIGHT_M, coords_list_to_pose_array

def dummy_path():
    # Straight line path along Y-axis
    return coords_list_to_pose_array([[0.0, y * 0.1, 27.0] for y in range(21)])

class PathPlanningServer(Node):
    def __init__(self):
        super().__init__('path_planning_server')
        
        self.path_planner_init()
        self.ros_interfaces_init()

        self.get_logger().info(f"'{self.get_name()}' node has been started.")

    def path_planner_init(self):        
        x_min, x_max, y_min, y_max, map_width, map_height, grid_size, robot_radius = \
            path_planning.get_planner_init_params(x_max=1000, y_max=1000, robot_radius=10.0, grid_size=10.0)
        
        # set obstacle positions
        ox, oy = [], []
        # map surroundings
        ox, oy = path_planning.place_wall_positions(ox, oy, x_min, x_max, y_min, y_max, grid_size)
        # place obstacles
        ox, oy = path_planning.place_bitmap_obstacles(ox, oy, bit_map=np.load(BITMAP_PATH), grid_size=grid_size)

        self.path_planner = path_planning.AStarPlanner(ox, oy, grid_size, robot_radius)

        self.get_logger().info("Path planner initialized")

    def ros_interfaces_init(self):
        self.bit_map = None
        self.cv_bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, "/arena_display/output", self.image_callback, 10
        )

        self.srv = self.create_service(GetPath, 'GetPath', self.get_path_cb)

    def image_callback(self, msg):
        if not self.bit_map:
            try:
                self.get_logger().info("Received an image!")
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                self.bit_map = bit_map.create_2d_bitmap(cv_image)
                self.generate_map()
                self.get_logger().info("bitmap created")

            except Exception as e:
                self.get_logger().error(f"Could not convert image: {e}")

    def get_path_cb(self, request:GetPath.Request, response:GetPath.Response):
        self.get_logger().info("Path planning request received")
        self.get_logger().info(F"Wp1: ({request.waypoints.poses[0].position.x}, {request.waypoints.poses[0].position.y}), Wp2: ({request.waypoints.poses[1].position.x}, {request.waypoints.poses[1].position.y})")

        if self.waypoint_valid(request.waypoints):
            response.path = self.plan_path(request.waypoints)
            self.get_logger().info("Waypoints have been successfully sent.")
        else:
            self.get_logger().error(f"waypoints are invalid. waypoints: {request.waypoints}")
        return response

    def generate_map(self):
        self.get_logger().info("Generating map")
        # Generate the map
        self.map = path_planning.generate_map(self.bit_map)
        self.get_logger().info("Map generated")

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
