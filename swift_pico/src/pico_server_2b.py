#!/usr/bin/env python3
# Made by: Jonathan Mikler
# Creation date: 2024-11-03
# WD_4122


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer

# Import your action messages
from swift_pico.action import GoToPoint, NavigatePath, Hover

class PicoServerNode(Node):
    def __init__(self):
        super().__init__('pico_server_2b')
        
        # Configure the logger
        self.configure_logger()

        # Initialize the action client for 'goToPoint'
        self.go_to_point_client = ActionClient(self, GoToPoint, 'go_to_point')
        
        # Initialize the action server for 'navigatePath'
        self.navigate_path_server = ActionServer(
            self,
            NavigatePath,
            'navigate_path',
            self.handle_navigate_path
        )
        
        # Initialize the action server for 'hover'
        self.hover_server = ActionServer(
            self,
            Hover,
            'hover',
            self.handle_hover
        )


        self.get_logger().info(f"{self.get_name()} node has been started.")

    def configure_logger(self):
        # Custom logger configuration can be added here
        self.get_logger().debug("Logger has been configured.")

    # Callback for the 'navigatePath' action server
    def handle_navigate_path(self, goal_handle):
        self.get_logger().info("Received a navigatePath action request.")
        # Implement your logic to handle the goal
        goal_handle.succeed()
        
        result = NavigatePath.Result()
        # Fill in result fields as necessary
        return result

    # Callback for the 'hover' action server
    def handle_hover(self, goal_handle):
        self.get_logger().info("Received a hover action request.")
        # Implement your logic to handle the goal
        goal_handle.succeed()
        
        result = Hover.Result()
        # Fill in result fields as necessary
        return result

def main(args=None):
    rclpy.init(args=args)
    custom_action_node = PicoServerNode()
    rclpy.spin(custom_action_node)

    # Shutdown
    custom_action_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
