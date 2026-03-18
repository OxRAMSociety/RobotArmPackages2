from rclpy.node import Node

from task_planning_interfaces.msg import PieceName, BoardCorners, PieceSquares
from geometry_msgs.msg import Pose, Point, Quaternion


class TwoToThreeDimensions(Node):
    def __init__(self):
        super().__init__("two_to_three_dimensions")
        self._piece_to_move_value = None
        self._piece_to_move = self.create_subscription(
            PieceName,
            "piece_to_move",
            self.piece_to_move_listener_callback,
            10)
        self._board_corners_value = None
        self._board_corners = self.create_subscription(
            BoardCorners,
            "board_corners",
            self.board_corners_listener_callback,
            10)
        self._piece_squares_value = None
        self._piece_squares = self.create_subscription(
            PieceSquares,
            "piece_squares",
            self.piece_squares_listener_callback,
            10)
        self._arm_destination = self.create_publisher(
            Pose,
            "arm_destination",
            10)

    def piece_to_move_listener_callback(self, msg):
        self._piece_to_move_value = msg.data
        if (self._board_corners_value is not None
                and self._piece_squares_value is not None):
            self.listener_callback()

    def board_corners_listener_callback(self, msg):
        self._board_corners_value = msg.data
        if (self._piece_to_move_value is not None
                and self._piece_squares_value is not None):
            self.listener_callback()

    def piece_squares_listener_callback(self, msg):
        self._piece_squares_value = msg.data
        if (self._board_corners_value is not None
                and self._piece_squares_value is not None):
            self.listener_callback()

    def listener_callback(self, msg):
        piece_to_move = self._piece_to_move_value
        board_corners = self._board_corners_value
        piece_squares = self._piece_squares_value
        self.get_logger().info(
            f"2D to 3D: Piece to move {piece_to_move}; Board corners {board_corners}; Piece squares {piece_squares}"
        )
        result = Pose()
        result.position = Point()
        result.orientation = Quaternion()

        # TODO: Calculate position
        result.position.x = 0.0
        result.position.y = 0.0
        result.position.z = 0.0
        result.quaternion.x = 0.0
        result.quaternion.y = 0.0
        result.quaternion.z = 0.0
        result.quaternion.w = 1.0
        self._piece_squares.publish(result)
