#!/usr/bin/env python3
# WD_4122

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from typing import List
from enum import Enum

# Import action and service messages
from swift_pico.action import GoToPoint, NavigatePath, Hover
from swift_pico.srv import GenPath
from std_msgs.msg import String, Int32MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point


class State(Enum):
    IDLE = 0
    GENERATING_PATH = 1
    MOVING = 2
    HOVERING = 3

class picoClient(Node):
    def __init__(self):
        super().__init__('pico_client_2b')

        self.state: State = State.IDLE

        self.waypoints: List[Point] = []

        self.ros_interfaces_init()

        self.get_logger().info(f"{self.get_name()} node has been started.")

    def ros_interfaces_init(self):
        self.go_to_point_client = ActionClient(self, GoToPoint, 'go_to_point')
        self.navigate_path_client = ActionClient(self, NavigatePath, 'navigate_path')
        self.hover_client = ActionClient(self, Hover, 'hover')

        self.generate_path_client = self.create_client(GenPath, 'GeneratePath')

        qos_profile = QoSProfile(depth=10)

        self.random_points_subscriber = self.create_subscription(
            Int32MultiArray,
            '/random_points',
            self.random_points_callback,
            qos_profile
        )

        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', qos_profile)

        self.diagnostics_timer = self.create_timer(1.0, self.publish_diagnostics)  # Publish diagnostics every second
        self.main_timer = self.create_timer(0.1, self.main_timer_callback)  # Main loop timer
    
    def main_timer_callback(self):
        state_fn = {
            State.IDLE: self.idle_state,
            State.GENERATING_PATH: self.generating_path_state,
            State.MOVING: self.moving_state,
            State.HOVERING: self.hovering_state
        }

        state_fn[self.state]()
    
    def idle_state(self):
        if self.waypoints:
            self.change_state(State.GENERATING_PATH)
    
    def generating_path_state(self):
        
        pass
    
    def moving_state(self):
        pass

    def hovering_state(self):
        pass

    def random_points_callback(self, msg:Int32MultiArray):
        assert len(msg.data) % 2 == 0, "Message data length should be even."
        
        if not self.waypoints:
            for i in range(0, len(msg.data), 2):
                point = Point()
                point.x = float(msg.data[i])
                point.y = float(msg.data[i+1])
                self.waypoints.append(point)
            
            self.get_logger().info(f"Waypoints: {self.waypoints}")
        
    def call_generate_path_service(self, waypoints):
        if not self.generate_path_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'GeneratePath' not available.")
            return

        request = GenPath.Request()
        request.waypoints = waypoints

        future = self.generate_path_client.call_async(request)
        future.add_done_callback(self.generate_path_response_callback)

    def generate_path_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received path: {response.paths}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def publish_diagnostics(self):
        diagnostics_msg = DiagnosticArray()
        diagnostics_msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = "picoClient Diagnostics"
        status.message = "System is running smoothly"
        status.values = [KeyValue(key="Status", value="Operational")]

        diagnostics_msg.status.append(status)
        self.diagnostics_publisher.publish(diagnostics_msg)

    def change_state(self, state: State):
        self.state = state
        self.get_logger().info(f"State changed to: {self.state}")

def main(args=None):
    rclpy.init(args=args)
    pico_client_node = picoClient()
    rclpy.spin(pico_client_node)

    # Shutdown
    pico_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
