#!/usr/bin/env python3
# WD_4122

import rclpy
import rclpy.guard_condition
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.action import ActionClient

from utils import State

from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints

TIMER_INTERVAL_S = 1.0

class WayPointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')
        self.state = State.IDLE
        self.goals = []
        self.goal_index = 0

        self.ros_interfaces_init()
        self.get_logger().info(f"{self.get_name()} node has been started.")
    
    def ros_interfaces_init(self):
        # srv_cb_group = MutuallyExclusiveCallbackGroup()
        srv_cb_group = ReentrantCallbackGroup()
        # main_cb_group = MutuallyExclusiveCallbackGroup() # default callback group
        main_cb_group = None # default callback group
        
        #create a client for the service 'GetWaypoints'. Refer to Writing a simple service and client (Python) in ROS 2 tutorials
        #service name should be 'waypoints'
        self.get_waypoints_client = self.create_client(GetWaypoints, 'GetWaypoints', callback_group=srv_cb_group)

        #create an action client for the action 'NavToWaypoint'. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        #action name should 'waypoint_navigation'.


        self.main_timer = self.create_timer(TIMER_INTERVAL_S, self.main_timer_callback, callback_group=main_cb_group)

    ### State machine functions

    def main_timer_callback(self):
        self.get_logger().info(f'State: {self.state}')
        
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
        
        self.get_logger().info(f"'{self.get_waypoints_client.srv_name}' service available")
        self.change_state(State.GETTING_PATH)

    def getting_path_state(self):
        if not self.goals:
            self.fetch_waypoints()
        
        self.change_state(State.NAVIGATING)

    def navigating_state(self):
        self.get_logger().info('Navigation started')
        pass

    def change_state(self, state: State):
        self.get_logger().info(f'State changed from {self.state} to {state}')
        self.state = state
    

    ### service client functions
    def send_request(self):
        try:
            request = GetWaypoints.Request()
            request.get_waypoints = True
            future = self.get_waypoints_client.call_async(request)
            future
        finally:
            return future
    
    def fetch_waypoints(self):
        self.get_logger().info('Fetching waypoints')
        # future = self.send_request()
        request = GetWaypoints.Request()
        request.get_waypoints = True
        future = self.get_waypoints_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        result = future.result()
        self.get_logger().info('Waypoints received')
        self.get_logger().info(f'response: {result}')


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

def main(args=None):
    rclpy.init(args=args)

    node = WayPointClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # while rclpy.ok():
    #     rclpy.spin_once(node, executor=executor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
