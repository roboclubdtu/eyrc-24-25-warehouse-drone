#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image
from waypoint_navigation.srv import GetPath

from typing import Tuple
import numpy as np
from cv_bridge import CvBridge

# from swift_pico.scripts import bit_map
from swift_pico.scripts import path_planning, whycon_mapper
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
        self.get_logger().info("Path planner initializing...")   
        x_min, x_max, y_min, y_max, map_width, map_height, grid_size, robot_radius = \
            path_planning.get_planner_init_params(
                x_min = 0, x_max=1000, y_min=0, y_max=1000, robot_radius=10.0, grid_size=10.0)
        
        # set obstacle positions
        ox, oy = [], []
        # map surroundings
        ox, oy = path_planning.place_wall_positions(ox, oy, x_min, x_max, y_min, y_max, grid_size)
        # place obstacles
        ox, oy = path_planning.place_bitmap_obstacles(ox, oy, bit_map=np.load(BITMAP_PATH), grid_size=grid_size)

        self.path_planner = path_planning.AStarPlanner(ox, oy, grid_size, robot_radius)

        self.get_logger().info("Path planner initialized")

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
            return True #BUG something wrong with this function
            if len(waypoints.poses) >= 2:
                self.get_logger().error(f"Waypoints len should be 2 not {len(waypoints.poses)}")
                return False
            
            if (waypoints.poses[0].position.x, waypoints.poses[0].position.y) == (waypoints.poses[1].position.x, waypoints.poses[1].position.y):
                self.get_logger().error("Waypoints should not be the same coordinates")
                return False
            
            if not (self.path_planner.min_x < waypoints.poses[0].position.x < self.path_planner.max_x) and \
                (self.path_planner.min_y < waypoints.poses[0].position.y < self.path_planner.max_y ) and \
                (self.path_planner.min_x < waypoints.poses[1].position.x < self.path_planner.max_x) and \
                (self.path_planner.min_y < waypoints.poses[1].position.y < self.path_planner.max_y):
                
                self.get_logger().error("Waypoints should be within the map boundaries")
                return False
            
            return True

    def plan_path(self, waypoints:PoseArray) -> PoseArray:
        # NOTE: The plan is in image coordinates, needs to be converted to world coordinates
        wp1 = waypoints.poses[0].position
        wp2 = waypoints.poses[1].position

        path_i, path_j = self.path_planner.planning(
            sx=wp1.x, sy=wp1.y,
            gx=wp2.x, gy=wp2.y)
        
        self.get_logger().info(f"Path found. Length: {len(path_i)}")
        
        path_x, path_y = whycon_mapper.pixel_to_whycon(np.array(path_i), np.array(path_j))

        return coords_list_to_pose_array(list(zip(path_x, path_y)), constant_z=PREDEFINED_WP_HEIGHT_M)
    
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
