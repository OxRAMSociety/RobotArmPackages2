import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from enum import Enum

from task_planning.arm import Arm
from task_planning.chess_engine import ChessEngine
from task_planning.cv_board import CVBoard
from task_planning.cv_pieces import CVPieces
from task_planning.piece_squares_calculator import PieceSquaresCalculator
from task_planning.piece_squares_changed_checker import PieceSquaresChangedChecker
from task_planning.two_to_three_dimensions import TwoToThreeDimensions

# TODO: import interfaces

from threading import Thread


class State(Enum):
    WAITING = 0
    THINKING = 1
    MOVING = 2


class StateMachine(Node):
    def __init__(self):
        # TODO: Move to launch file; prevent unnecessary calculations
        super().__init__("state_machine")
        nodes = []
        for NodeClass in [
                    Arm,
                    ChessEngine,
                    CVBoard,
                    CVPieces,
                    PieceSquaresCalculator,
                    PieceSquaresChangedChecker,
                    TwoToThreeDimensions
                ]:
            print("spin", NodeClass)
            nodes.append(NodeClass())
            Thread(target = lambda: rclpy.spin(nodes[-1])).start()

        print(nodes)

        # TODO: Make data pass correctly between nodes, with correct timing - in client classes
        # Asynchronous code
        while True:
            # Waiting
            waiting_client = WaitingClient()
            future = waiting_client.send_goal()
            rclpy.spin_until_future_complete(waiting_client, future)

            # Thinking
            thinking_client = ThinkingClient()
            future = thinking_client.send_request(PieceSquares())
            rclpy.spin_until_future_complete(thinking_client, future)

            # Moving
            moving_client = MovingClient()
            future = moving_client.send_goal(Pose(), Pose())
            rclpy.spin_until_future_complete(moving_client, future)

class WaitingClient(Node):
    def __init__(self):
        super().__init__("waiting_client")
        self._piece_squares_changed_checker = ActionClient(
            self,
            PieceSquaresChangedChecker,
            "piece_squares_changed_checker")

    def send_goal(self) -> PieceSquaresChangedChecker.Goal:
        goal_msg = PieceSquaresChangedChecker.Goal()

        self._piece_squares_changed_checker.wait_for_server()

        return self._piece_squares_changed_checker.send_goal_async(goal_msg)

class ThinkingClient(Node):
    def __init__(self):
        super().__init__("thinking_client")
        self._chess_engine = self.create_client(
            ChessEngine,
            "chess_engine")

    def send_request(self, current_state: PieceSquares) -> ChessEngine.Response:
        req = ChessEngine.Request()
        req.current_state = current_state

        self._chess_engine.wait_for_server()

        return self._chess_engine.call_async(req)

class MovingClient(Node):
    def __init__(self):
        super().__init__("moving_client")
        self._arm = ActionClient(
            self,
            Arm,
            "arm")

    def send_goal(self, piece_from: Pose, piece_to: Pose) -> Arm.Result:
        goal_msg = Arm.Goal()
        goal_msg.piece_from = piece_from
        goal_msg.piece_to = piece_to

        self._chess_engine.wait_for_server()

        return self._chess_engine.call_async(req)


def main(args=None):
    rclpy.init(args=args)

    state_machine = StateMachine()
    rclpy.spin(state_machine)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
