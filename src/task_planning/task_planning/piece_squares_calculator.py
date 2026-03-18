from rclpy.node import Node

from task_planning_interfaces.msg import PiecePoses, BoardCorners, PieceSquares, Square


class PieceSquaresCalculator(Node):
    def __init__(self):
        super().__init__("piece_squares_calculator")
        self._board_corners_value = None
        self._board_corners = self.create_subscription(
            BoardCorners,
            "board_corners",
            self.board_corners_listener_callback,
            10)
        self._piece_poses_value = None
        self._piece_poses = self.create_subscription(
            PiecePoses,
            "piece_poses",
            self.piece_poses_listener_callback,
            10)
        self._piece_squares = self.create_publisher(
            PieceSquares,
            "piece_squares",
            10)

    def board_corners_listener_callback(self, msg):
        self._board_corners_value = msg.data
        if (self._piece_poses_value is not None):
            self.listener_callback()

    def piece_poses_listener_callback(self, msg):
        self._piece_poses_value = msg.data
        if (self._board_corners_value is not None):
            self.listener_callback()

    def listener_callback(self):
        board_corners = self._board_corners_value
        piece_poses = self._piece_poses_value
        self.get_logger().info(
            f"Poses changed: Board {board_corners}; Pieces {piece_poses}"
        )
        # TODO: Convert to PieceSquares
        result = PieceSquares()
        for color in ["w", "b"]:
            for pawn_i in range(8):
                piece_name = color + "_pawn_" + pawn_i
                result[piece_name] = Square()
                result[piece_name].x = pawn_i % 7  # TODO
                result[piece_name].y = pawn_i % 7  # TODO
            for rook_i in range(2):
                piece_name = color + "_rook_" + rook_i
                result[piece_name] = Square()
                result[piece_name].x = rook_i % 7  # TODO
                result[piece_name].y = rook_i % 7  # TODO
            for knight_i in range(2):
                piece_name = color + "_knight_" + knight_i
                result[piece_name] = Square()
                result[piece_name].x = knight_i % 7  # TODO
                result[piece_name].y = knight_i % 7  # TODO
            for bishop_i in range(2):
                piece_name = color + "_bishop_" + bishop_i
                result[piece_name] = Square()
                result[piece_name].x = bishop_i % 7  # TODO
                result[piece_name].y = bishop_i % 7  # TODO

            piece_name = color + "_queen"
            result[piece_name] = Square()
            result[piece_name].x = 0  # TODO
            result[piece_name].y = 0  # TODO

            piece_name = color + "_king"
            result[piece_name] = Square()
            result[piece_name].x = 0  # TODO
            result[piece_name].y = 0  # TODO
        self._piece_squares.publish(result)
