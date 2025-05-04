# node which subscribes to topics for testing purposes
# not intended for final product

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class StateSubscriber(Node):

    def __init__(self):
        super().__init__('state_subscriber')
        self.subscription = self.create_subscription(
            String,
            'state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    state_subscriber = StateSubscriber()

    rclpy.spin(state_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
