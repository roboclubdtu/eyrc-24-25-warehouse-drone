#!/usr/bin/env python3
# WD_4122

# Importing the required libraries

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from pid_msg.msg import PIDError

from waypoint_navigation.action import NavToWaypoint
from pico_utils import PID, timestamp_pose
import time

SAMPLE_TIME_S = 0.060 # QUESTION: When running `ros2 topic hz /whycon/poses` the rate is 30 Hz. That means a period of 0.0333 seconds. Why is the sample time set to 0.060 seconds?

PID_VALS = {
    'roll': {'P': 10.0, 'I': 0.0, 'D': 35.0},
    'pitch': {'P': 10.0, 'I': 0.0, 'D': 35.0},
    'throttle': {'P': 18.0, 'I': 0.0, 'D': 36.0}
}
def get_dummy_pose():
    pose = Pose()
    pose.position.x = 2.0
    pose.position.y = 2.0
    pose.position.z = 19.0
    return pose

DUMMY_NAV_TIME = 5

class PicoControllerNode(Node):
    def __init__(self):
        super().__init__('pico_controller')
        self.ros_interfaces_init()
        self.pid_init()

        self.arm()

        self.get_logger().info(f"{self.get_name()} initialized")
    
    def ros_params_init(self):
        # TODO
        pass

    def ros_interfaces_init(self):
        self.current_pose:Pose = None
        self.setpoint_pose = None
        
        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = SwiftMsgs()

        # Publishing /drone_command, /pid_error
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

        # Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, /roll_pid
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)

        self._action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            self.nav_action_callback
        )

        # Creating a timer to run the pid function periodically
        self.create_timer(SAMPLE_TIME_S, self.pid)

    # drone functions
    def pid_init(self):
        self.roll_controller = PID(sample_time=SAMPLE_TIME_S, offset=1500)
        self.roll_controller.set_gains(PID_VALS['roll']['P'], PID_VALS['roll']['I'], PID_VALS['roll']['D'])
        
        self.pitch_controller = PID(sample_time=SAMPLE_TIME_S, offset=-1500)
        self.pitch_controller.set_gains(PID_VALS['pitch']['P'], PID_VALS['pitch']['I'], PID_VALS['pitch']['D'])
        
        self.throttle_controller = PID(sample_time=SAMPLE_TIME_S, offset=-1528)
        self.throttle_controller.set_gains(PID_VALS['throttle']['P'], PID_VALS['throttle']['I'], PID_VALS['throttle']['D'])
    
    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

        self.get_logger().info("Drone disarmed")

    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

        self.get_logger().info("Drone armed")

    def pid(self):
        # Skip PID calculation until valid position data is available
        if self.current_pose is None or self.setpoint_pose is None:
            return

        # refactor code 
        _current_x = self.current_pose.position.x
        _current_y = self.current_pose.position.y
        _current_z = self.current_pose.position.z

        self.cmd.rc_roll =  int(self.roll_controller.compute(self.setpoint_pose.position.x, _current_x))
        self.cmd.rc_pitch =  int(self.pitch_controller.compute(self.setpoint_pose.position.y, _current_y))
        self.cmd.rc_throttle =  int(self.throttle_controller.compute(self.setpoint_pose.position.z, _current_z))
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

    # Callback functions
    def whycon_callback(self, msg:PoseArray):
        self.current_pose = msg.poses[0]

    # action functions
    def set_goalpose(self, pose:Pose):
        self.setpoint_pose = pose

        self.get_logger().warn(f"Goal pose set to: {pose.position.x}, {pose.position.y}, {pose.position.z}")

    def nav_action_callback(self, goal_handle):
        self.get_logger().info("Received a goal request.")
        result = NavToWaypoint.Result()

        # Dummy navigation, input process
        self.get_logger().info(f"Navigating to {goal_handle.request.waypoint}")
        self.set_goalpose(goal_handle.request.waypoint)

        # Dummy feedback
        feedback_msg = NavToWaypoint.Feedback()
        feedback_msg.current_waypoint = timestamp_pose(self.current_pose, self.get_clock().now())
        
        start_time = time.time()

        # Publish feedback every 0.5 seconds, for 3 seconds
        while (time.time() - start_time) < DUMMY_NAV_TIME:
            self.get_logger().info("Publishing feedback...")
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        # Return the result
        result.hov_time = 3

def main(args=None):
    rclpy.init(args=args)
    swift_pico =    PicoControllerNode()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()