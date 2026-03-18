from rclpy.node import Node

from sensor_msgs.msg import Image
from task_planning_interfaces.msg import BoardCorners
from geometry_msgs.msg import Point


class CVBoard(Node):
    def __init__(self):
        super().__init__("cv_board")
        self._camera = self.create_subscription(
            Image,
            "camera",
            self.listener_callback,
            10)
        self._board_corners = self.create_publisher(
            BoardCorners,
            "board_corners",
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Camera changed: '{msg.data}'")
        # TODO: Convert to BoardCorners
        result = BoardCorners()
        result.a1_corner = Point()
        result.a1_corner.x = 0  # TODO
        result.a1_corner.y = 0  # TODO
        result.a1_corner.z = 0  # TODO
        result.a8_corner = Point()
        result.a8_corner.x = 0  # TODO
        result.a8_corner.y = 0  # TODO
        result.a8_corner.z = 0  # TODO
        result.h1_corner = Point()
        result.h1_corner.x = 0  # TODO
        result.h1_corner.y = 0  # TODO
        result.h1_corner.z = 0  # TODO
        result.h8_corner = Point()
        result.h8_corner.x = 0  # TODO
        result.h8_corner.y = 0  # TODO
        result.h8_corner.z = 0  # TODO
        self._board_corners.publish(result)
