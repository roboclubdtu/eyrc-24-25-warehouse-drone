from rclpy.time import Time
from geometry_msgs.msg import Pose, PoseStamped
from enum import Enum, auto

class PID:
    def __init__(self, sample_time, Kp=0.0, Ki=0.0, Kd=0.0, max_output=2000.0, min_output=1000.0, offset=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sample_time = sample_time
        self.max_output = max_output
        self.min_output = min_output
        self.error = 0
        self.offset = offset
        self.d_filter_alpha = 0.95  # Filter coefficient (0.0-1.0)
        self.filtered_derivative = 0.0
        
        # Moving average window
        self.output_window = []
        self.window_size = 3
        
        self.reset()
        
    def set_gains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
    def compute(self, setpoint, current_value, clip_output=False):
        error = setpoint - current_value
        
        # Calculate integral term
        self.integral += error * self.sample_time
        
        # Calculate and filter derivative term
        derivative = (error - self.prev_error) / self.sample_time
        self.filtered_derivative = (self.d_filter_alpha * self.filtered_derivative + 
                                  (1.0 - self.d_filter_alpha) * derivative)
        
        # Calculate output using filtered derivative
        output = (self.Kp * error + 
                 self.Ki * self.integral + 
                 self.Kd * self.filtered_derivative)
        
        output += self.offset
        
        if clip_output:
            output = max(min(output, self.max_output), self.min_output)
        output = abs(output)  # to account for negative offsets
        
        # Apply moving average filter
        self.output_window.append(output)
        if len(self.output_window) > self.window_size:
            self.output_window.pop(0)
        output = sum(self.output_window) / len(self.output_window)
        
        self.prev_error = error
        self.error = error  # for debugging
        return output
        
    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.output_window = []  # Clear the window

class ClientStates(Enum):
    IDLE = auto()
    NAVIGATING = auto()
    DONE = auto()
    GETTING_PATH = auto()

class ServerStates(Enum):
    IDLE = auto()
    NAVIGATING = auto()
    HOVER = auto()
    DONE = auto()

def timestamp_pose(pose:Pose, stamp: Time):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.stamp = stamp.to_msg()
    return pose_stamped

def coords_from_pose(pose:Pose):
    return (pose.position.x, pose.position.y, pose.position.z)