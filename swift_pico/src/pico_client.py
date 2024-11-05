#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import PoseArray

from pico_utils import State

from waypoint_navigation.srv import GetWaypoints

TIMER_INTERVAL_S = 1.0

class CallbackGroupDemo(Node):
    def __init__(self):
        super().__init__('waypoint_client')
        self.state = State.IDLE
        self.goals:PoseArray = None
        self.goal_index = 0

        self.ros_interfaces_init()

        self.get_logger().info(f"{self.get_name()} node has been started.")

    def ros_interfaces_init(self):
        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = None
        self.get_waypoints_client = self.create_client(GetWaypoints, 'GetWaypoints', callback_group=client_cb_group)
        
        self.call_timer = self.create_timer(TIMER_INTERVAL_S, self._timer_cb, callback_group=timer_cb_group)

    # State machine functions
    def _timer_cb(self):
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
        self.get_waypoints()

    def navigating_state(self):
        self.get_logger().info(f'Navigating to waypoints. # of waypoints: {len(self.goals.poses)}')
        # self.change_state(State.IDLE)

    def change_state(self, state: State):
        self.get_logger().info(f'State changed from {self.state} to {state}')
        self.state = state

    def get_waypoints(self):
        self.get_logger().info('Fetching waypoints')
        req = GetWaypoints.Request()
        req.get_waypoints = True
        future:GetWaypoints.Response = self.get_waypoints_client.call(req)
        self.get_logger().info(f'Received response. Future: {future}')

        if future.waypoints:
            self.goals = future.waypoints
            self.change_state(State.NAVIGATING)

if __name__ == '__main__':
    rclpy.init()
    node = CallbackGroupDemo()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()