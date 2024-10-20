#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class WaypointNavPyNode(Node):

    def __init__(self):
        super().__init__("waypoint_nav_py_node")
        example_param = self.declare_parameter("example_param", "default_value").value
        self.get_logger().info(f"Declared parameter 'example_param'. Value: {example_param}")
        self.get_logger().info("Hello world from the Python node waypoint_nav_py_node")


def main(args=None):
    rclpy.init(args=args)

    waypoint_nav_py_node = WaypointNavPyNode()

    try:
        rclpy.spin(waypoint_nav_py_node)
    except KeyboardInterrupt:
        pass

    waypoint_nav_py_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
