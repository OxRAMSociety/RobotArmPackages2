# /usr/bin/python3
# Code modified from https://github.com/AnrdyShmrdy/ros2_serial_interface
import json
import time
import signal

import rclpy
import serial
from rclpy.node import Node


class SerialServer(Node):
    def __init__(self):
        super().__init__("serial_server")
        self.device_name = "/dev/ttyACM0"
        self.ser = serial.Serial(self.device_name, 9600, timeout=0.1)
        self.state = 'idle'
        # self.subscriber = self.create_subscription(
        #     Num, "topic", self.serial_listener_callback, 10
        # )
        # # self.subscriber

        # Solves a bug where writing directly after creating the serial
        # somehow stops all reads from occuring
        time.sleep(1)
        self.ser.reset_input_buffer()

        self.num_messages_sent = 0
        self.position = 0
        self.timer = self.create_timer(10.0 / 10, self.move_motor)

    def send_cmd(self, cmd):
        cmd = json.dumps(cmd)
        print("> " + cmd)
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
            print("< " + line)

    def move_motor(self):
        print()
        # if self.num_messages_sent % 10 == 0:
        #     self.position += 3 * 8 * 200 # rotations * microsteps * steps for 3 rotations
        # elif self.num_messages_sent % 10 == 5:
        #     self.position = 0
        self.num_messages_sent += 1

        # self.send_cmd({"motor_0": {"position": self.position}})
        # self.send_cmd({"motor_1": {"position": self.position}})
        # self.send_cmd({"motor_2": {"position": self.position}})
        # self.send_cmd({"motor_3": {"position": -self.position}})
        # self.receive_cmd()

        if self.num_messages_sent < 10:
            self.state = 'idle'
        elif self.num_messages_sent < 20:
            self.state = '1'
        elif self.num_messages_sent < 30:
            self.state = '2'
        elif self.num_messages_sent < 40:
            self.state = '3'


        match self.state:
            case 'idle':
                time.sleep(1)
            case '1':
                self.send_cmd({"motor_0": 100})
                self.send_cmd({"motor_1": 300})
            case '2':
                self.send_cmd({"motor_1": 200})
            case '3':
                self.send_cmd({"motor_0": 0})
                self.send_cmd({"motor_1": 0})


        self.receive_cmd()
        print()
        print("========================")


def main(args=None):
    rclpy.init(args=args)
    serial_server = SerialServer()

    def interrupt_handler(sig, frame):
        print("Aborting")
        serial_server.send_cmd({"killswitch": True})

        import sys
        sys.exit()

    signal.signal(signal.SIGINT, interrupt_handler)
    rclpy.spin(serial_server)

if __name__ == "__main__":
    main()
