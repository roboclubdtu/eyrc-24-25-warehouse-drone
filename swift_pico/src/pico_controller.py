#!/usr/bin/env python3
# WD_4122

# Importing the required libraries

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node


class Swift_Pico(Node):
    def __init__(self):
        super().__init__('pico_controller')  # initializing ros node with name pico_controller

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = [0.0, 0.0, 0.0]

        # [x_setpoint, y_setpoint, z_setpoint]
        self.setpoint = [2, 2, 19]  # whycon marker at the position of the dummy given in the scene

        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        # Initial setting of Kp, Ki, Kd for [roll, pitch, throttle]
        XY_P = 10
        XY_D = 35
        Z_constant = 18
        ratio = 2
        self.Kp = [XY_P, XY_P, Z_constant]
        self.Ki = [0, 0, 0]
        self.Kd = [XY_D, XY_D, Z_constant * ratio]

        # Additional PID variables
        self.prev_error = [0, 0, 0]  # Previous errors for [roll, pitch, throttle]
        self.error_sum = [0, 0, 0]  # Sum of errors for integral term
        self.max_values = [2000, 2000, 2000]  # Upper limit for [roll, pitch, throttle]
        self.min_values = [1000, 1000, 1000]  # Lower limit for [roll, pitch, throttle]
        self.max_output = 10  # Maximum allowable control signal to prevent sudden large changes

        # Sample time for running the PID algorithm
        self.sample_time = 0.060  # in seconds

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

    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    # Callback function for /throttle_pid
    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp
        self.Ki[2] = alt.ki
        self.Kd[2] = alt.kd

    # Callback function for /pitch_pid
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.kp
        self.Ki[1] = pitch.ki
        self.Kd[1] = pitch.kd

    # Callback function for /roll_pid
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.kp 
        self.Ki[0] = roll.ki 
        self.Kd[0] = roll.kd

    # PID control function
    def pid(self):
        # Skip PID calculation until valid position data is available
        if self.drone_position == [0.0, 0.0, 0.0]:
            return

        # Step 1: Compute error in each axis [roll (x), pitch (y), throttle (z)]
        error = [self.setpoint[i] - self.drone_position[i] for i in range(3)]

        self.get_logger().info(f"height setpoint: {self.setpoint[2]}")
        self.get_logger().info(f"Drone position: {self.drone_position[2]}")
        self.get_logger().info(f"throttle error: {error[2]}")

        # Step 2: Compute P, I, and D terms
        for i in range(3):
            P_term = self.Kp[i] * error[i]
            self.error_sum[i] += error[i] * self.sample_time
            I_term = self.Ki[i] * self.error_sum[i]
            D_term = self.Kd[i] * (error[i] - self.prev_error[i]) / self.sample_time

            # Step 3: Calculate output
            output = P_term + I_term + D_term
            # Step 4: Adjust command value (1550 is base for steady throttle)
            if i == 0:  # Roll
                self.cmd.rc_roll = int(1500 + output)
                self.cmd.rc_roll = max(min(self.cmd.rc_roll, self.max_values[0]), self.min_values[0])
            elif i == 1:  # Pitch
                self.cmd.rc_pitch = int(1500 - output)
                self.cmd.rc_pitch = max(min(self.cmd.rc_pitch, self.max_values[1]), self.min_values[1])
            else:  # Throttle (z-axis)
                # Base of 1550 for maintaining steady position
                self.cmd.rc_throttle = int(1528 - output)
                self.cmd.rc_throttle = max(min(self.cmd.rc_throttle, self.max_values[2]), self.min_values[2])

            # Step 7: Update previous error
            self.prev_error[i] = error[i]

            # Log the control signal output
            self.get_logger().info(f"Drone position: {self.drone_position[2]}, throttle error: {error[2]}, Throttle command: {self.cmd.rc_throttle}, Control Signal: {output}")

        # Step 6: Publish command after all adjustments
        self.command_pub.publish(self.cmd)

        # Step 8: Publish error values
        pid_error_msg = PIDError()
        pid_error_msg.roll_error = error[0]
        pid_error_msg.pitch_error = error[1]
        pid_error_msg.throttle_error = error[2]
        self.pid_error_pub.publish(pid_error_msg)


def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()