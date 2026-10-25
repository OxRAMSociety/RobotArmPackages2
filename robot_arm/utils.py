
import serial


def parse_parial_json(buffer):
    # a "possible json object" is either a complete json object, an unfinished json object, or malformed
    # the buffer may be:
    # - 
    # - complete json object
    # - complete json object with the next 



# Michael - detect arduino usb port (linux)
def get_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "ttyACM0" in p.description or "Arduino" in p.description:
            return p.device


