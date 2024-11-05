#!/usr/bin/env python3
# WD_4122

import time
from enum import Enum
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints

TIMER_INTERVAL_S = 0.5

class State(Enum):
    IDLE = 0
    GETTING_PATH = 1
    NAVIGATING = 2


class WayPointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')
        self.state = State.IDLE
        self.goals = []
        self.goal_index = 0

        self.ros_interfaces_init()
        self.get_logger().info(f"{self.get_name()} node has been started.")
    
    def ros_interfaces_init(self):
        #create an action client for the action 'NavToWaypoint'. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        #action name should 'waypoint_navigation'.

        
        #create a client for the service 'GetWaypoints'. Refer to Writing a simple service and client (Python) in ROS 2 tutorials
        #service name should be 'waypoints'
        self.get_waypoints_client = self.create_client(GetWaypoints, 'GetWaypoints')

        self.main_timer = self.create_timer(TIMER_INTERVAL_S, self.main_timer_callback)

    ### State machine functions

    def main_timer_callback(self):
        state_fn = {
            State.IDLE: self.idle_state,
            State.GETTING_PATH: self.getting_path_state,
            State.NAVIGATING: self.navigating_state
        }

        state_fn[self.state]()
    
    def idle_state(self):
        # check if service is available
        if not self.get_waypoints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available')
            return
        
        self.change_state(State.GETTING_PATH)

    def getting_path_state(self):
        pass

    def navigating_state(self):
        pass

    def change_state(self, state: State):
        self.get_logger().info(f'State changed from {self.state} to {state}')
        self.state = state
    
    ### action client functions

    def send_goal(self, waypoint):

        #create a NavToWaypoint goal object.

        goal_msg.waypoint.position.x = waypoint[0]
        goal_msg.waypoint.position.y = waypoint[1]
        goal_msg.waypoint.position.z = waypoint[2]

        #create a method waits for the action server to be available.
        

        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)    
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        #complete the goal_response_callback. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        pass

    def get_result_callback(self, future):
        return 
        #complete the missing line
        # result = 
        self.get_logger().info('Result: {0}'.format(result.hov_time))

        self.goal_index += 1

        if self.goal_index < len(self.goals):
            self.send_goal(self.goals[self.goal_index])
        else:
            self.get_logger().info('All waypoints have been reached successfully')      

    def feedback_callback(self, feedback_msg):
        return 
        #complete the missing line
        # feedback = 
        x = feedback.current_waypoint.pose.position.x
        y = feedback.current_waypoint.pose.position.y
        z = feedback.current_waypoint.pose.position.z
        t = feedback.current_waypoint.header.stamp.sec
        self.get_logger().info(f'Received feedback! The current whycon position is: {x}, {y}, {z}')
        self.get_logger().info(f'Max time inside sphere: {t}')

    ### service client functions

    def send_request(self):
        request = GetWaypoints.Request()
        future = self.get_waypoints_client.call_async(request)
        return future
    
    def receive_goals(self):
        future = self.send_request()
        rclpy.spin_until_future_complete(self, future)
        
        response:GetWaypoints.Response = future.result()
        self.get_logger().info('Waypoints received by the action client')

        for pose in response.waypoints.poses:
            waypoints = [pose.position.x, pose.position.y, pose.position.z]
            self.goals.append(waypoints)
            self.get_logger().info(f'Waypoints: {waypoints}')

        self.send_goal(self.goals[0])
    

def main(args=None):
    rclpy.init(args=args)

    waypoint_client = WayPointClient()
    waypoint_client.receive_goals()

    try:
        rclpy.spin(waypoint_client)
    except KeyboardInterrupt:
        waypoint_client.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_client.destroy_node()
        rclpy.shutdown()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
