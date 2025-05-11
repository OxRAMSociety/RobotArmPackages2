
from control_logic.srv import RequestEngineMove

import rclpy
from rclpy.node import Node

import chess
import chess.engine


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(RequestEngineMove, 'request_engine_move', self.request_engine_move_callback)
        self.engine = chess.engine.SimpleEngine.popen_uci('stockfish')

    def request_engine_move_callback(self, request, response):
        self.get_logger().info('Incoming request\na: %s' % (request.fen))
        
        board = chess.Board(request.fen)

        if not board.is_valid():
            self.get_logger().warn('Invalid board!');

        result = engine.play(board, chess.engine.Limit(time=0.1))
        self.get_logger().info('Engine played: %s' % result.move.uci())

        response.from_square = result.move.from_square
        response.to_square = result.move.to_square
    
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

