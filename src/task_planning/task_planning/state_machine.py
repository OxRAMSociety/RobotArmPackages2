from enum import Enum
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class State(Enum):
    WAITING = 0
    THINKING = 1
    MOVING = 2

class StateMachine(Node):
    def __init__(self):
        pass

    # Waiting
    def start_waiting(self):
        pass
    # Thinking
    def start_thinking(self):
        pass
    # Moving
    def start_moving(self):
        pass

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
