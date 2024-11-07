#!/usr/bin/env python3
# WD_4122

# Importing the required libraries

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pid_msg.msg import PIDError
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray, Pose, PoseStamped

from pico_utils import ServerStates, PID, timestamp_pose, coords_from_pose
from waypoint_navigation.action import NavToWaypoint

SAMPLE_TIME_S = 0.060 # QUESTION: When running `ros2 topic hz /whycon/poses` the rate is 30 Hz. That means a period of 0.0333 seconds. Why is the sample time set to 0.060 seconds?

PID_VALS = {
    'roll': {'P': 10.0, 'I': 0.0, 'D': 35.0},
    'pitch': {'P': 10.0, 'I': 0.0, 'D': 35.0},
    'throttle': {'P': 18.0, 'I': 0.0, 'D': 36.0}
}

X_TOLERANCE = 0.15
Y_TOLERANCE = 0.15
Z_TOLERANCE = 0.15

LOG_LEVEL = rclpy.logging.LoggingSeverity.INFO

def get_dummy_pose():
    pose = Pose()
    pose.position.x = 2.0
    pose.position.y = 2.0
    pose.position.z = 19.0
    return pose

HOVER_TIME_S = 3

class PicoServerNode(Node):
    def __init__(self):
        super().__init__('pico_server')
        self.state = ServerStates.IDLE
        self.get_logger().set_level(LOG_LEVEL)
        self.ros_interfaces_init()
        self.controllers_init()
        self.timers_reset()

        self.arm()

        self.get_logger().info(f"{self.get_name()} initialized")
    
    def ros_params_init(self):
        # TODO
        pass

    def ros_interfaces_init(self):
        main_callback_group = None
        action_callback_group = ReentrantCallbackGroup()
        
        # Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, /roll_pid
        self.current_pose:Pose = None
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1, callback_group=main_callback_group)

        # Publishing /drone_command, /pid_error
        self.cmd = SwiftMsgs()
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

        # diagnostics
        self.diag_status = DiagnosticStatus()
        self.diag_status.name = "PicoServerNode"
        self.diagnostics_pub = self.create_publisher(DiagnosticStatus, '/diagnostics', 10)
        self.create_timer(1.0, self.publish_diagnostics, callback_group=main_callback_group)

        # Action stuff
        self.action_started = False
        self.action_completed:bool = False
        self.goal_pose:Pose = None
        self.goalpose_arrived:bool = False
        self.navigating_start_time = None
        self.hover_start_time = None
        self.hover_total_time = 0

        self.feedback_msg = NavToWaypoint.Feedback()

        self._action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            callback_group=action_callback_group,
            execute_callback=self.nav_action_callback,
            goal_callback=self.nav_goal_callback,
        )

        # Creating a timer to run the pid function periodically
        self.create_timer(SAMPLE_TIME_S, self.main, callback_group=main_callback_group)

    def change_state(self, new_state:ServerStates):
        self.state = new_state
        self.get_logger().warn(f"State changed to {self.state}")
    
    def reset(self):
        self.get_logger().info(f"{self.get_name()} general reset")
        self.action_reset()
        self.controllers_reset()
        self.timers_reset()
        self.change_state(ServerStates.IDLE)

    def timers_reset(self):
        self.hover_start_time = None
        self.navigating_start_time = None
        self.get_logger().debug("Timers reset")

    # Callback functions
    def whycon_callback(self, msg:PoseArray):
        self.current_pose = msg.poses[0]

    def publish_diagnostics(self):
        # self.diag_msg = DiagnosticArray()
        # self.diag_msg.header.stamp = self.get_clock().now().to_msg()

        self.diag_status.level = DiagnosticStatus.OK
        msg = f"Current state: {self.state.name}. "

        if self.state == ServerStates.NAVIGATING:
            coords = coords_from_pose(self.goal_pose)
            msg += f"Navigating to x:{coords[0]}, y:{coords[1]}, z:{coords[2]}."

        # self.diag_msg.status.append(status)
        self.diag_status.message = msg
        self.diagnostics_pub.publish(self.diag_status)

    # control functions
    def pid(self):
        # Skip PID calculation until valid position data is available
        if self.goal_pose is None or self.current_pose is None:
            return

        # refactor code 
        _current_x = self.current_pose.position.x
        _current_y = self.current_pose.position.y
        _current_z = self.current_pose.position.z

        self.cmd.rc_roll =  int(self.roll_controller.compute(self.goal_pose.position.x, _current_x))
        self.cmd.rc_pitch =  int(self.pitch_controller.compute(self.goal_pose.position.y, _current_y))
        self.cmd.rc_throttle =  int(self.throttle_controller.compute(self.goal_pose.position.z, _current_z))
        # /refactor code 

        self.command_pub.publish(self.cmd)

        # Publish error values
        # refactor code
        pid_error_msg = PIDError()

        pid_error_msg.roll_error = self.roll_controller.error
        pid_error_msg.pitch_error = self.pitch_controller.error
        pid_error_msg.throttle_error = self.throttle_controller.error
        # /refactor code

        self.pid_error_pub.publish(pid_error_msg)
   
    def controllers_init(self):
        self.roll_controller = PID(sample_time=SAMPLE_TIME_S, offset=1500)
        self.roll_controller.set_gains(PID_VALS['roll']['P'], PID_VALS['roll']['I'], PID_VALS['roll']['D'])
        
        self.pitch_controller = PID(sample_time=SAMPLE_TIME_S, offset=-1500)
        self.pitch_controller.set_gains(PID_VALS['pitch']['P'], PID_VALS['pitch']['I'], PID_VALS['pitch']['D'])
        
        self.throttle_controller = PID(sample_time=SAMPLE_TIME_S, offset=-1528)
        self.throttle_controller.set_gains(PID_VALS['throttle']['P'], PID_VALS['throttle']['I'], PID_VALS['throttle']['D'])
    
    def controllers_reset(self):
        self.roll_controller.reset()
        self.pitch_controller.reset()
        self.throttle_controller.reset()

        self.get_logger().debug("Controllers reset")

    # drone functions
    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

        self.get_logger().info("Drone disarmed")

    def arm(self):
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

        self.get_logger().info("Drone armed")

    def main(self):
        state_fns = {
            ServerStates.IDLE: self.idle,
            ServerStates.NAVIGATING: self.navigate,
            ServerStates.HOVER: self.hover
        }

        state_fns[self.state]()

        self.pid() # maybe add it to its own thread?

    def idle(self):
        pass
        # if self.goal_pose is not None:
        #     self.get_logger().info("New goal pose received. Changing state to NAVIGATING",throttle_duration_sec=1)

    def navigate(self):
        if not self.navigating_start_time:
            self.navigating_start_time = time.time()

        if self.goal_reached():
            self.goalpose_arrived = True
            self.get_logger().info("Goal pose reached")
            self.change_state(ServerStates.HOVER)

    def hover(self):
        # Only if the action has started we care about hover time, if not, we simply hover
        if self.action_started:
            if self.hover_start_time is None:
                self.hover_start_time = time.time()
            
            self.hover_total_time = time.time() - self.hover_start_time
            if self.hover_total_time > HOVER_TIME_S:
                self.get_logger().info(f"Hover time reached {self.hover_total_time}. Action completed.")
                self.action_completed = True
                # self.timers_reset()

    def goal_reached(self):
        if self.current_pose is None or self.goal_pose is None:
            return False

        x_within_tolerance = abs(self.current_pose.position.x - self.goal_pose.position.x) < X_TOLERANCE
        y_within_tolerance = abs(self.current_pose.position.y - self.goal_pose.position.y) < Y_TOLERANCE
        z_within_tolerance = abs(self.current_pose.position.z - self.goal_pose.position.z) < Z_TOLERANCE

        return x_within_tolerance and y_within_tolerance and z_within_tolerance
  
    # action functions
    def action_reset(self):
        self.goalpose_arrived = False
        self.action_started = False
        self.action_completed = False
        self.navigating_start_time = None
        self.hover_start_time = None
        self.hover_total_time = 0

        self.get_logger().debug("Action reset")

    def set_goalpose(self, pose:Pose):
        self.goal_pose = pose
        self.get_logger().info(f"Goal pose set to: x:{pose.position.x}, y:{pose.position.y}, z:{pose.position.z}")
        self.change_state(ServerStates.NAVIGATING)

    def nav_goal_callback(self, goal_request:NavToWaypoint.Goal):
        """Callback function for the goal request. To accept or reject the goal request"""

        coords = coords_from_pose(goal_request.waypoint)
        msg = f"New goal request. x:{coords[0]}, y:{coords[1]}, z:{coords[2]}"
        self.get_logger().debug(f"{msg}")
        return GoalResponse.ACCEPT

    def nav_action_callback(self, goal_handle):
        """Callback function for the execution of the action with an accepted goal request"""
        self.get_logger().info("Received a goal handle.")

        self.set_goalpose(goal_handle.request.waypoint)
        self.action_started = True

        while not self.action_completed:
            pass
            # self.feedback_msg.current_waypoint = timestamp_pose(self.current_pose, self.get_clock().now())
            # goal_handle.publish_feedback(self.feedback_msg)

        result = NavToWaypoint.Result()
        total_nav_time = time.time() - self.navigating_start_time
        self.get_logger().info(f"Total navigation time: {total_nav_time}")
        result.hov_time = int(total_nav_time)

        goal_handle.succeed()
        self.reset()
        return result

def main(args=None):
    rclpy.init(args=args)

    pico_server_node = PicoServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(pico_server_node)
    
    try:
         executor.spin()
    except KeyboardInterrupt:
        pico_server_node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
         pico_server_node.destroy_node()
         rclpy.shutdown()


if __name__ == '__main__':
    main()