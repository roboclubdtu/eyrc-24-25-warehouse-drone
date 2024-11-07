#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseArray, Pose

from pico_utils import ClientStates
from waypoint_navigation.srv import GetWaypoints
from waypoint_navigation.action import NavToWaypoint

TIMER_INTERVAL_S = 0.25

class CallbackGroupDemo(Node):
    def __init__(self):
        super().__init__('waypoint_client')
        self.state = ClientStates.IDLE
        self.goals:PoseArray = None
        self.current_pose = Pose()
        self.goal_index = 0

        self.ros_interfaces_init()

        self.get_logger().info(f"{self.get_name()} node has been started.")

    def ros_interfaces_init(self):
        client_cb_group = ReentrantCallbackGroup()
        timer_cb_group = None
        
        # service client
        self._get_waypoints_client = self.create_client(GetWaypoints, "GetWaypoints", callback_group=client_cb_group)
        
        # action client
        self._nav_client = ActionClient(self, NavToWaypoint, "waypoint_navigation", callback_group=client_cb_group)
        self.executing_action = False
        
        # timer
        self.call_timer = self.create_timer(TIMER_INTERVAL_S, self.timer_cb, callback_group=timer_cb_group)

    # State machine functions
    def timer_cb(self):
        self.get_logger().debug(f'State: {self.state}')
        state_fn = {
            ClientStates.IDLE: self.idle_state,
            ClientStates.GETTING_PATH: self.getting_path_state,
            ClientStates.NAVIGATING: self.navigating_state,
            ClientStates.DONE: self.shutdown_proc
        }

        state_fn[self.state]()
    
    def idle_state(self):
        # check if service is available
        if not self._get_waypoints_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available')
            return
        
        self.get_logger().info(f"'{self._get_waypoints_client.srv_name}' service available")
        self.change_state(ClientStates.GETTING_PATH)
    
    def getting_path_state(self):
        self.call_get_waypoints()
        if self.goals:
            self.change_state(ClientStates.NAVIGATING)

    def navigating_state(self):
        if self.goal_index >= len(self.goals.poses):
            self.get_logger().info('Reached end of path')
            self.get_logger().info('Task done')
            self.change_state(ClientStates.DONE)
            return
        
        if self.executing_action:
            return
        self.send_goal()

    def change_state(self, state: ClientStates):
        self.get_logger().warn(f'State changed from {self.state} to {state}')
        self.state = state

    def next_waypoint(self):
        next_waypoint = self.goals.poses[self.goal_index]
        return next_waypoint
    
    # service fns
    def call_get_waypoints(self):
        self.get_logger().info('Fetching waypoints')
        req = GetWaypoints.Request()
        req.get_waypoints = True
        future:GetWaypoints.Response = self._get_waypoints_client.call(req)

        if hasattr(future, 'waypoints') and future.waypoints:
            self.get_logger().info(f'Received waypoints. {len(future.waypoints.poses)} waypoints')
            self.goals = future.waypoints

    # action fns
    def send_goal(self):
        # inspired from https://foxglove.dev/blog/creating-ros2-actions
        self.get_logger().info('Sending goal...')

        goal_msg = NavToWaypoint.Goal()
        goal_msg.waypoint = self.next_waypoint()
        self._nav_client.wait_for_server()

        # Returns future to goal handle; client runs feedback_callback after sending the goal
        self._send_goal_future = self._nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        # Register a callback for when future is complete (i.e. server accepts or rejects goal request)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg: NavToWaypoint.Feedback):
        self.current_pose = feedback_msg.feedback.current_waypoint.pose
    
    def goal_response_callback(self, future):
        # Get handle for the goal we just sent
        goal_handle = future.result()

        # Return early if goal is rejected
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.executing_action = goal_handle.accepted
        self.get_logger().info('Goal accepted :)')
        self.get_logger().info(f'Navigating to wp {self.goal_index + 1} of {len(self.goals.poses)}')

        # Use goal handle to request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        # Log result and shut down ROS 2 cleanly
        self.get_logger().info('Result: hov_time:{0}'.format(result.hov_time))

        self.executing_action = False
        self.goal_index += 1

    # shutdown
    def shutdown_proc(self):
        pass
        # self.call_timer.cancel()
        # self._nav_client.destroy()
        # self.destroy_node()

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
    # node.destroy_node()
    rclpy.shutdown()