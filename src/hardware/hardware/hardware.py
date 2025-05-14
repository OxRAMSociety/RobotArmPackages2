# /usr/bin/python3
# Code modified from https://github.com/AnrdyShmrdy/ros2_serial_interface
import rclpy
from rclpy.node import Node
import serial
import json
import time


class SerialServer(Node):
    def __init__(self):
        super().__init__("serial_server")
        self.device_name = "/dev/ttyACM0"
        self.ser = serial.Serial(self.device_name, 9600, timeout=0.1)
        # self.subscriber = self.create_subscription(
        #     Num, "topic", self.serial_listener_callback, 10
        # )
        # # self.subscriber

        # Solves a bug where writing directly after creating the serial
        # somehow stops all reads from occuring
        time.sleep(1)
        self.ser.reset_input_buffer()

        self.num_messages_sent = 0
        self.timer = self.create_timer(10.0 / 10, self.toggle_led)

    def send_cmd(self, cmd):
        cmd = json.dumps(cmd)
        print("> "+cmd)
        self.ser.write(bytes(cmd + "\n", "utf-8"))
        self.ser.flush()

    def receive_cmd(self, max_wait=0.1, poll_delay=0.01):

        # Wait for a message to come in
        start_time = time.time()
        while not self.ser.in_waiting and time.time() - start_time <= max_wait:
            time.sleep(poll_delay)

        while self.ser.in_waiting:
            line = self.ser.readline().decode("utf-8").rstrip()
            print("\033[95mReply from Arduino:\033[0m")
            print("< "+line)

    def toggle_led(self):
        print()
        self.num_messages_sent += 1  
        print(f"\033[96mROS: Sending message #{self.num_messages_sent}\033[0m")
        self.send_cmd({"turn_on_led": self.num_messages_sent % 2})
        self.receive_cmd()
        print()
        print("========================")


def main(args=None):
    rclpy.init(args=args)
    serial_server = SerialServer()
    rclpy.spin(serial_server)


if __name__ == "__main__":
    main()
