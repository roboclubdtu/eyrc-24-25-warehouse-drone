#!/usr/bin/env python3
# WD_4122

# Importing the required libraries

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray, Pose
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node

from pico_utils import PID

SAMPLE_TIME_S = 0.060 # QUESTION: When running `ros2 topic hz /whycon/poses` the rate is 30 Hz. That means a period of 0.0333 seconds. Why is the sample time set to 0.060 seconds?
PID_VALS = {
    'roll': {'P': 10.0, 'I': 0.0, 'D': 35.0},
    'pitch': {'P': 10.0, 'I': 0.0, 'D': 35.0},
    'throttle': {'P': 18.0, 'I': 0.0, 'D': 36.0}
}

class PicoControllerNode(Node):
    def __init__(self):
        super().__init__('pico_controller')
        self.ros_interfaces_init()
        self.pid_init()

        self.drone_position = [0.0, 0.0, 0.0]


        self.get_logger().info(f"{self.get_name()} initialized")
    
    def ros_params_init(self):
        # TODO
        pass

    def ros_interfaces_init(self):
        # goal pose
        self.setpoint_pose = Pose()
        self.set_goalpose(2,2,19)
        
        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = SwiftMsgs()

        # Publishing /drone_command, /pid_error
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

        # Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, /roll_pid
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

        # Arming the drone
        self.arm()

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

    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)  # Publishing /drone_command

    def pid(self):
        # Skip PID calculation until valid position data is available
        if self.drone_position == [0.0, 0.0, 0.0]:
            return           

        # refactor code 
        _current_x = self.drone_position[0]
        _current_y = self.drone_position[1]
        _current_z = self.drone_position[2]

        self.cmd.rc_roll =  int(self.roll_controller.compute(self.setpoint_pose.position.x, _current_x))
        self.cmd.rc_pitch =  int(self.pitch_controller.compute(self.setpoint_pose.position.y, _current_y))
        self.cmd.rc_throttle =  int(self.throttle_controller.compute(self.setpoint_pose.position.z, _current_z))
        # /refactor code 

        # constant yaw val
        self.cmd.rc_yaw = 1500
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
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp
        self.Ki[2] = alt.ki
        self.Kd[2] = alt.kd

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.kp
        self.Ki[1] = pitch.ki
        self.Kd[1] = pitch.kd

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.kp 
        self.Ki[0] = roll.ki 
        self.Kd[0] = roll.kd

    # action functions
    def set_goalpose(self, x,y,z):
        self.setpoint_pose.position.x = x
        self.setpoint_pose.position.y = y
        self.setpoint_pose.position.z = z

        self.get_logger().warn(f"Goal pose set to: {x}, {y}, {z}")


def main(args=None):
    rclpy.init(args=args)
    swift_pico =    PicoControllerNode()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()