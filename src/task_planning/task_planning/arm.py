from rclpy.action import ActionServer
from rclpy.node import Node

from task_planning_interfaces.action import Arm as ArmAction


class Arm(Node):
    def __init__(self):
        super().__init__("arm")
        self._arm = ActionServer(
            self,
            ArmAction,
            "arm",
            self.execute_callback)

    def execute_callback(self, goal_handle):
        result = ArmAction.result()
        # TODO: Use goal_handle
        return result
