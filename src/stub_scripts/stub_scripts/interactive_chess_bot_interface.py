#!/usr/bin/env python3

import sys

from control_logic.srv import RequestEngineMove
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(RequestEngineMove, 'request_engine_move')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RequestEngineMove.Request()

    def send_request(self, fen):
        self.req.fen = fen
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(sys.argv[1])
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
            'Engine move: from square: %d, to square: %d' %
        (response.from_square, response.to_square))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
