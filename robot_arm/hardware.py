import asyncio
import serial
import logging
import json

logger = logging.getLogger(__name__)


class HardwareBridge:
    def __init__(self, port, baudrate):
        if port is None:
            # TODO: detect arduino automatically
            pass
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=.1)
        self.serial_lock = asyncio.Lock()
        # Note: possible separate lock for ser.flush()?
        # self.flush_lock = asyncio.Lock()
        self.logger = logging.getLogger("HardwareBridge")
        self._read_proccess_task = None

    async def start_read_process(self):
        if self._read_proccess_task is None:
            self._read_proccess_task = asyncio.get_event_loop().create_task(self._read_process())

    async def _read_process(self):
        self._read_buffer = ""
        while True:
            await asyncio.sleep(0.1)
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting).decode(encoding='UTF-8')
                self._read_buffer += data
                

    async def write(self, command):
        async with self.serial_lock:
            self.ser.write(command.encode(encoding='UTF-8'))
            self.ser.flush()

    async def read(self):
        async with self.serial_lock:
            return self.ser.readline().decode(encoding='UTF-8').strip()

    async def test(self):
        ...

    async def close(self):
        async with self.serial_lock:
            self.ser.close()


class RobotArmBridge:
    def __init__(self, hardware_bridge, port='COM3', baudrate=115200):
        self.hardware_bridge = HardwareBridge(port=port, baudrate=baudrate)
        self.logger = logging.getLogger("RobotArmBridge")

    async def test_connection(self):
        self.logger.info("Testing bridge...")
        start_time = asyncio.get_event_loop().time()
        response = await self.test_command()
        end_time = asyncio.get_event_loop().time()
        self.logger.info(f"Received response: {response}")
        self.logger.info(f"Test succeeded, latency={end_time - start_time:.2f} seconds")

    async def test_command(self):
        await self.send_command({"test": True})
        response = await self.receive_response()
        assert response == {"test": True}

    async def send_command(self, command):
        # TODO: change to json.dump for streaming the data
        data = json.dumps(command)
        self.logger.info("> " + data)
        await self.hardware_bridge.write(data + "\n")

    async def receive_response(self, max_wait=0.1, poll_delay=0.01):
        with self.serial_lock:
            # Wait for a message to come in
            self.logger.debug("Waiting for response...")
            start_time = asyncio.get_event_loop().time()
            while not self.ser.in_waiting and asyncio.get_event_loop().time() - start_time <= max_wait:
                await asyncio.sleep(poll_delay)

            # Wait while receiving message
            self.logger.debug("Receiving message...")
            response_lines = []
            while self.hardware_bridge.ser.in_waiting:
                line = await self.hardware_bridge.read()
                self.logger.info("\033[95mReply from Arduino:\033[0m")
                self.logger.info("< " + line)
                response_lines.append(line)
            return response_lines
        

    async def move_motor(self, motor_id, position):
        ...
