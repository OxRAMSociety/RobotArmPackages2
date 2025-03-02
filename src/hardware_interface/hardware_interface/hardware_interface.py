#/usr/bin/python3
# Code modified from https://github.com/AnrdyShmrdy/ros2_serial_interface
import rclpy
from rclpy.node import Node
import serial
from interfaces.msg import Num

class SerialServer(Node):
    def __init__(self):
        super().__init__('serial_server')
        self.device_name = '/dev/ttyACM0'
        self.ser = serial.Serial(self.device_name,9600,timeout=.1)
        self.subscriber = self.create_subscription(Num,
                                                   'topic',
                                                   self.serial_listener_callback,
                                                   10)
        #self.subscriber
        self.ser.reset_input_buffer()
    def send_cmd(self, cmd):
        print("Sending to arduino: " + cmd)
        self.ser.write(bytes(cmd,'utf-8'))
    def receive_cmd(self):
        try:
            line = self.ser.readline().decode('utf-8').rstrip()
            line1 = self.ser.readline().decode('utf-8').rstrip()
            line2 = self.ser.readline().decode('utf-8').rstrip()
        except:
            line = str(self.ser.readline())
            line1 = str(self.ser.readline())
            line2 = str(self.ser.readline())
        print("Received from arduino: " + str(line1))
    def serial_listener_callback(self,msg):
        print("message received from rostalker: ", msg.num)
        self.send_cmd("<Text," + str(msg.num) + ",19.2>")
        self.receive_cmd()



def main(args=None):
    rclpy.init(args=args)
    serial_server = SerialServer()
    rclpy.spin(serial_server)

if __name__ == '__main__':
    main()

