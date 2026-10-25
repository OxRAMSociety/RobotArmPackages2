import asyncio
import serial
import logging

logger = logging.getLogger(__name__)


class HardwareBridge:
    def __init__(self, port='COM4', baudrate=115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=.1)
        self.serial_lock = asyncio.Lock()
        self.logger = logging.getLogger("HardwareBridge")

    async def write(self, command):
        async with self.serial_lock:
            self.ser.write(command.encode(encoding='UTF-8'))
            self.ser.flush()

    async def read(self):
        async with self.serial_lock:
            return self.ser.readline().decode(encoding='UTF-8').strip()

    async def test(self):
        self.logger.info("Testing hardware bridge...")
        start_time = asyncio.get_event_loop().time()
        await self.write("test\n")
        response = await self.read()
        end_time = asyncio.get_event_loop().time()
        self.logger.info(f"Received response, test duration: {end_time - start_time:.2f} seconds")

    async def close(self):
        async with self.serial_lock:
            self.ser.close()


class RobotArmBridge:
    def __init__(self, hardware_bridge):
        self.hardware_bridge = hardware_bridge

    async def test_connection(self):
        ...

    async def send_command(self, command):
        ...

    async def move_motor(self, motor_id, position):
        ...
