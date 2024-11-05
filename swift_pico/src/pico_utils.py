
from enum import Enum

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

class State(Enum):
    IDLE = 0
    GETTING_PATH = 1
    NAVIGATING = 2
    DONE = 3