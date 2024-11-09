
from rclpy.time import Time

from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from enum import Enum, auto

PREDEFINED_WP_HEIGHT_M = 27.0
BITMAP_PATH = '/home/dtu_dev/ws/pico_ws/src/waypoint_navigation/map/2D_bit_map.npy'

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

def coords_list_to_pose_array(coords:list, constant_z=None):
    pose_array = PoseArray()
    for coord in coords:
        pose = Pose()
        pose.position.x = coord[0]
        pose.position.y = coord[1]
        if constant_z:
            pose.position.z = constant_z
        else:
            pose.position.z = coord[2]
        pose.orientation.w = 1.0  # Default orientation
        pose_array.poses.append(pose)
    return pose_array