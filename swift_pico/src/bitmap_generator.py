#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from bitmap import bit_map


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")

        self.subscription = self.create_subscription(
            Image, "/arena_display/output", self.image_callback, 10
        )

        self.bridge = CvBridge()
        self.image_received = False

    def image_callback(self, msg):
        if not self.image_received:
            try:
                self.image_received = True
                self.get_logger().info("Received an image!")
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                bit_map.create_2d_bitmap(cv_image)

                # Clean up
                self.get_logger().info("Shutting down after receiving one image.")
                self.destroy_node()
                rclpy.shutdown()
            except Exception as e:
                self.get_logger().error(f"Could not convert image: {e}")


def main(args=None):
    rclpy.init(args=None)
    node = ImageSubscriber()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
