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
        self.state = '0'
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
        self.timer = self.create_timer(10.0 / 10, self.move_motors)
    
    def move_motor(self,motor, position):
        motor_key = f"motor_{motor}"
        multiplier = 1
        match motor:
            case 0:
                factor = 10
                self.send_cmd({"motor_0": {"position": (position  * factor * multiplier)}})
                self.send_cmd({"motor_1": {"position": (position  * factor * multiplier)}})
                return
            case 1:
                factor = 10
                return
            case 2:
                factor = 2
            case 3:
                factor = 4
            case 4:
                factor = 1.5
            case 5:
                factor = 2
            case _:
                factor = 30
        self.send_cmd({motor_key: {"position": int(position  * factor * multiplier)}})


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

    def move_motors(self):
        print()
        self.num_messages_sent += 1

        if self.num_messages_sent < 1:
            self.state = '0'
        elif self.num_messages_sent < 5:
            self.state = '1'
        elif self.num_messages_sent < 9:
            self.state = '2'
        elif self.num_messages_sent < 12:
            self.state = '3'
        elif self.num_messages_sent < 15:
            self.state = '4'
        elif self.num_messages_sent < 18:
            self.state = '5'
        elif self.num_messages_sent < 21:
            self.state = '6'
        elif self.num_messages_sent < 24:
            self.state = '7'
        else:
            self.state = '0'

        variable = 30
        match self.state:
            case '0':
                print("idling")
                time.sleep(0.1)
            case '1':
                print("state 1")
                self.move_motor(0, 150)
                # self.move_motor(1, 200)
                self.move_motor(2, 600)
                self.move_motor(3, -250)
                self.move_motor(4, 0)
                self.move_motor(5, 100)
                # time.sleep(0.1)
            case '2':
                print("state 2")
                # self.move_motor(0, 450)
                # self.move_motor(1, 200)
                # self.move_motor(2, 200)
                self.move_motor(3, -100)
                # self.move_motor(4, 200)
                # self.move_motor(5, 200)
                # time.sleep(0.1)
            case '3':
                print("state 3")
                # self.move_motor(0, 150)
                # self.move_motor(1, 200)
                # self.move_motor(2, 200)
                self.move_motor(3, -250-variable)
                # self.move_motor(4, 200)
                # self.move_motor(5, 200)
                # time.sleep(0.1)
            case '4':
                print("state 4")
                # self.move_motor(0, 200)
                # self.move_motor(1, 200)
                self.move_motor(2, -600)
                # self.move_motor(3, 200)
                # self.move_motor(4, 200)
                # self.move_motor(5, 200)
                # time.sleep(0.1)
            case '5':
                print("state 5")
                self.move_motor(3, -100-variable)
                # self.move_motor(1, 200)
                # self.move_motor(2, 200)
                # self.move_motor(3, 200)
                # self.move_motor(4, 200)
                # self.move_motor(5, 200)
            case '6':
                print("state 6")
                self.move_motor(3, -250-2*variable)
            case '7':
                print("state 7")
                self.move_motor(0, 0)
                self.move_motor(1, 0)
                self.move_motor(2, 0)
                self.move_motor(3, -2*variable)
                self.move_motor(4, 0)
                self.move_motor(5, 0)


        self.receive_cmd()
        print()
        print("========================")


def main(args=None):
    rclpy.init(args=args)
    serial_server = SerialServer()

    def interrupt_handler(sig, frame):
        print("\nPausing..")
        serial_server.send_cmd({"killswitch": True})

        exiting = input("Press ENTER to continue...")
        if (exiting):
            import sys
            sys.exit()
        serial_server.send_cmd({"killswitch": False})

    signal.signal(signal.SIGINT, interrupt_handler)
    rclpy.spin(serial_server)

if __name__ == "__main__":
    main()
