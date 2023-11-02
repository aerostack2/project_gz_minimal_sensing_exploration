#!/bin/python3
"""explore.py"""

import sys
from time import sleep
import rclpy
from rclpy.task import Future
from rclpy import logging
from as2_python_api.drone_interface import DroneInterface
from std_srvs.srv import SetBool


class Explorer(DroneInterface):
    """An UAV born to explore the world"""

    def __init__(self, drone_id: str = "drone0", verbose: bool = False,
                 use_sim_time: bool = False) -> None:
        super().__init__(drone_id, verbose, use_sim_time)

        self.explore_client = self.create_client(SetBool, "start_exploration")

    def explore(self) -> Future:
        """Call exploration service asynchronously and return a future"""
        request = SetBool.Request()
        request.data = True

        return self.explore_client.call_async(request)


if __name__ == '__main__':
    rclpy.init()

    scouts: list[Explorer] = []
    scouts.append(Explorer(drone_id="drone0", verbose=False))
    scouts.append(Explorer(drone_id="drone1", verbose=False))
    scouts.append(Explorer(drone_id="drone2", verbose=False))

    for scout in scouts:
        scout.offboard()
        scout.arm()

    logging.get_logger("rclpy").info("Taking off")
    for scout in scouts:
        wait = scout == scouts[-1]
        scout.takeoff(1.0, wait=wait)

    futures: list[Future] = [scout.explore() for scout in scouts]
    logging.get_logger("rclpy").info("Exploring")
    while not all((fut.done() for fut in futures)):
        sleep(0.5)

    logging.get_logger("rclpy").info("Landing")
    for scout in scouts:
        wait = scout == scouts[-1]
        scout.land(wait=wait)

    for scout in scouts:
        scout.disarm()
        scout.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    sys.exit(0)
