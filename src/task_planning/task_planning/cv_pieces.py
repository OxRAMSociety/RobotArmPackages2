from rclpy.node import Node

from sensor_msgs.msg import Image
from task_planning_interfaces.msg import PiecePoses
from geometry_msgs.msg import Point


class CVPieces(Node):
    def __init__(self):
        super().__init__("cv_pieces")
        self._camera = self.create_subscription(
            Image,
            "camera",
            self.listener_callback,
            10)
        self._piece_poses = self.create_publisher(
            PiecePoses,
            "piece_poses",
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Camera changed: '{msg.data}'")
        # TODO: Convert to PiecePoses
        result = PiecePoses()
        for color in ["w", "b"]:
            for pawn_i in range(8):
                piece_name = color + "_pawn_" + pawn_i
                result[piece_name] = Point()
                result[piece_name].x = pawn_i  # TODO
                result[piece_name].y = pawn_i  # TODO
                result[piece_name].z = pawn_i  # TODO
            for rook_i in range(2):
                piece_name = color + "_rook_" + rook_i
                result[piece_name] = Point()
                result[piece_name].x = rook_i  # TODO
                result[piece_name].y = rook_i  # TODO
                result[piece_name].z = rook_i  # TODO
            for knight_i in range(2):
                piece_name = color + "_knight_" + knight_i
                result[piece_name] = Point()
                result[piece_name].x = knight_i  # TODO
                result[piece_name].y = knight_i  # TODO
                result[piece_name].z = knight_i  # TODO
            for bishop_i in range(2):
                piece_name = color + "_bishop_" + bishop_i
                result[piece_name] = Point()
                result[piece_name].x = bishop_i  # TODO
                result[piece_name].y = bishop_i  # TODO
                result[piece_name].z = bishop_i  # TODO

            piece_name = color + "_queen"
            result[piece_name] = Point()
            result[piece_name].x = 0  # TODO
            result[piece_name].y = 0  # TODO
            result[piece_name].z = 0  # TODO

            piece_name = color + "_king"
            result[piece_name] = Point()
            result[piece_name].x = 0  # TODO
            result[piece_name].y = 0  # TODO
            result[piece_name].z = 0  # TODO
        self._piece_poses.publish(result)
