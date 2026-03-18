from rclpy.node import Node

from task_planning_interfaces.msg import PieceName, Square
from task_planning_interfaces.srv import ChessEngine as ChessEngineServer


class ChessEngine(Node):
    def __init__(self):
        super().__init__("chess_engine")
        self._chess_engine = self.create_service(
            ChessEngineServer,
            "chess_engine",
            self.execute_callback)

    def execute_callback(self, request, response):
        # TODO: Use request.current_state

        response.piece = PieceName()
        response.piece.piece_name = PieceName.W_PAWN_0  # TODO
        response.destination = Square()
        response.destination.x = 0  # TODO
        response.destination.y = 0  # TODO

        return response
