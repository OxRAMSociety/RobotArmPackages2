import asyncio
import logging

from robot_arm import HardwareBridge, RobotArmBridge

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


async def main():
    bridge = RobotArmBridge(port='COM3', baudrate=115200)
    await bridge.test_connection()


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
