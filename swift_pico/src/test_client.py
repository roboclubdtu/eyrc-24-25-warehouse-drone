import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty


class CallbackGroupDemo(Node):
    def __init__(self):
        super().__init__('client_node')

        client_cb_group = None
        timer_cb_group = None
        self.client = self.create_client(Empty, 'test_service', callback_group=client_cb_group)
        self.call_timer = self.create_timer(1, self._timer_cb, callback_group=timer_cb_group)

    def _timer_cb(self):
        self.get_logger().info('Sending request')
        _ = self.client.call(Empty.Request())
        self.get_logger().info('Received response')


if __name__ == '__main__':
    rclpy.init()
    node = CallbackGroupDemo()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()