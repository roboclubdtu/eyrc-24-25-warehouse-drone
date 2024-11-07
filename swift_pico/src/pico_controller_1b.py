#!/usr/bin/env python3
# WD_4122

# Importing the required libraries

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray, Pose
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node

SAMPLE_TIME_S = 0.060

class PID:
    def __init__(self, sample_time=SAMPLE_TIME_S, Kp=0.0, Ki=0.0, Kd=0.0, max_output=2000.0, min_output=1000.0, offset=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sample_time = sample_time
        self.max_output = max_output
        self.min_output = min_output

        self.error = 0
        self.offset = offset
        
        self.reset()
    
    def set_gains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def compute(self, setpoint, current_value, clip_output=False):
        error = setpoint - current_value
        self.integral += error * self.sample_time
        derivative = (error - self.prev_error) / self.sample_time

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        output += self.offset
        
        if clip_output:
            output = max(min(output, self.max_output), self.min_output)

        output = abs(output) # to account for negative offsets

        self.prev_error = error
        
        # for debugging
        self.error = error
        return output

    def reset(self):
        self.prev_error = 0
        self.integral = 0

class PicoControllerNode(Node):
    def __init__(self):
        super().__init__('pico_controller')  # initializing ros node with name pico_controller

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = [0.0, 0.0, 0.0]

        # [x_setpoint, y_setpoint, z_setpoint]
        self.setpoint = [2, 2, 19]  # whycon marker at the position of the dummy given in the scene
        self.setpoint_pose = Pose()
        self.setpoint_pose.position.x = float(self.setpoint[0])
        self.setpoint_pose.position.y = float(self.setpoint[1])
        self.setpoint_pose.position.z = float(self.setpoint[2])

        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        # Initial setting of Kp, Ki, Kd for [roll, pitch, throttle]
        self.roll_controller = PID(offset=1500)
        self.pitch_controller = PID(offset=-1500)
        self.throttle_controller = PID(offset=-1528)

        XY_P = 10
        XY_I = 0
        XY_D = 35
        Z_constant = 18
        ratio = 2
        self.Kp = [XY_P, XY_P, Z_constant]
        self.Ki = [0, 0, 0]
        self.Kd = [XY_D, XY_D, Z_constant * ratio]
        
        # ---
        self.roll_controller.set_gains(XY_P, XY_I, XY_D)
        self.pitch_controller.set_gains(XY_P, XY_I, XY_D)
        self.throttle_controller.set_gains(Z_constant, 0, Z_constant * ratio)

        # Additional PID variables
        self.prev_error = [0, 0, 0]  # Previous errors for [roll, pitch, throttle]
        self.error_sum = [0, 0, 0]  # Sum of errors for integral term
        self.max_values = [2000, 2000, 2000]  # Upper limit for [roll, pitch, throttle]
        self.min_values = [1000, 1000, 1000]  # Lower limit for [roll, pitch, throttle]

        # NOTE: max_output not used
        self.max_output = 10  # Maximum allowable control signal to prevent sudden large changes

        # Sample time for running the PID algorithm
        self.sample_time = SAMPLE_TIME_S

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
        self.create_timer(self.sample_time, self.pid)

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

    # PID control function
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

        self.command_pub.publish(self.cmd)

        # Publish error values
        # refactor code
        pid_error_msg = PIDError()

        pid_error_msg.roll_error = self.roll_controller.error
        pid_error_msg.pitch_error = self.pitch_controller.error
        pid_error_msg.throttle_error = self.throttle_controller.error
        # /refactor code

        self.pid_error_pub.publish(pid_error_msg)


def main(args=None):
    rclpy.init(args=args)
    swift_pico =    PicoControllerNode()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()