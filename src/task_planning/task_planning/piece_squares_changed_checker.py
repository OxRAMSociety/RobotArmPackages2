from rclpy.action import ActionServer
from rclpy.node import Node

from task_planning_interfaces.msg import PieceSquares
from task_planning_interfaces.action import PieceSquaresChangedChecker as PieceSquaresChangedCheckerAction

from time import sleep


class PieceSquaresChangedChecker(Node):
    def __init__(self):
        super().__init__("piece_squares_changed_checker")
        self._piece_squares_value = None
        self._piece_squares_changed = False
        self._piece_squares = self.create_subscription(
            PieceSquares,
            "piece_squares",
            self.listener_callback,
            10)
        self._piece_squares_changed_checker = ActionServer(
            self,
            PieceSquaresChangedCheckerAction,
            "piece_squares_changed_checker",
            self.execute_callback)

    def listener_callback(self, msg):
        if (self._piece_squares_value is None):
            # PieceSquares hasn't changed; just hasn't been seen yet.
            self._piece_squares_value = msg.data

        if (msg.data != self._piece_squares_value):
            # PieceSquares has changed.
            self._piece_squares_value = msg.data
            self._piece_squares_changed = True

    def execute_callback(self, goal_handle):
        self._piece_squares_changed = False
        while (not self._piece_squares_changed):
            sleep(0.5)

        goal_handle.succeed()
        result = PieceSquaresChangedChecker.Result()
        result.piece_squares = self._piece_squares_value
        return result
