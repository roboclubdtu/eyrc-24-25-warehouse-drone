#!/usr/bin/env python3
# WD_4122

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

# Import action and service messages
from swift_pico.action import GoToPoint, NavigatePath, Hover
from swift_pico.srv import GenPath
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point

class picoClient(Node):
    def __init__(self):
        super().__init__('pico_client_2b')

        self.ros_interfaces_init()

        self.get_logger().info(f"{self.get_name()} node has been started.")

    def ros_interfaces_init(self):
        self.go_to_point_client = ActionClient(self, GoToPoint, 'go_to_point')
        self.navigate_path_client = ActionClient(self, NavigatePath, 'navigate_path')
        self.hover_client = ActionClient(self, Hover, 'hover')

        self.generate_path_client = self.create_client(GenPath, 'GeneratePath')

        qos_profile = QoSProfile(depth=10)
        self.random_points_subscriber = self.create_subscription(
            String,
            '/random_points',
            self.random_points_callback,
            qos_profile
        )

        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', qos_profile)
        self.timer = self.create_timer(1.0, self.publish_diagnostics)  # Publish diagnostics every second

    def random_points_callback(self, msg):
        self.get_logger().info(f"Received random points message: {msg.data}")
        # Handle the received message as needed

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

def main(args=None):
    rclpy.init(args=args)
    pico_client_node = picoClient()
    rclpy.spin(pico_client_node)

    # Shutdown
    pico_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
