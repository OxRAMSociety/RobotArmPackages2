import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.publisher_ = self.create_publisher(String, 'state', 10)
        self.state = String() # three states: 'waiting', 'moving', 'thinking'
        self.state.data = 'waiting' # the initial state is 'waiting'
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.publish_state()
        self.i += 1
    
    def publish_state(self):
        self.publisher_.publish(self.state)
        self.get_logger().info(f'Publishing: "{self.state.data}"')


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
