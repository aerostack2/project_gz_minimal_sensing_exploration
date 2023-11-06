"""Exploring evaluation script"""

import sys
import logging
import argparse
import threading
from datetime import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.time import Time
from rclpy.parameter import Parameter
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
import matplotlib.pyplot as plt
from matplotlib import animation


class Visualizer:
    """Matplotlib visualizer"""

    def __init__(self, x_lim: int):
        self.xlim = x_lim

        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], lw=2)
        self.ax.set(xlabel='time (s)', ylabel='area (%)',
                    title='Exploration Progress')
        self.ax.grid()
        self.xdata, self.ydata = [], []

    def init_plot(self):
        """Initialize the plot"""
        self.ax.set_ylim(0, 100)
        self.ax.set_xlim(0, self.xlim)
        self.line.set_data(self.xdata, self.ydata)
        return self.line

    def update_plot(self, frame):
        """Update plot data"""
        t, y = frame
        self.xdata.append(t)
        self.ydata.append(y)

        self.line.set_data(self.xdata, self.ydata)
        return self.line


class Evaluator(Node):
    """Evaluator node"""

    def __init__(self, use_sim_time: bool, log_file: str, verbose: bool) -> None:
        super().__init__("evaluator")

        self.param_use_sim_time = Parameter(
            'use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        self.logger = logging.Logger("evaluator")
        self.logger.setLevel(logging.INFO)
        if verbose:
            self.logger.addHandler(logging.StreamHandler(sys.stdout))
        self.logger.addHandler(logging.FileHandler(log_file, mode="w"))

        self.explored_percent = 0.0
        self.last_time = 0.0
        self.last_occ_grid: OccupancyGrid
        self.start_timestamp: Time

        self.create_subscription(
            msg_type=OccupancyGrid,
            topic="/map_server/map",
            callback=self.occ_grid_cbk,
            qos_profile=1,
        )

        self._timer: Timer
        self.create_service(Trigger, "/evaluator/start", self.start_cbk)

    def yield_viz(self):
        """Yield last time and percentage explored. Used for plotting"""
        yield self.last_time, self.explored_percent

    def occ_grid_cbk(self, msg: OccupancyGrid) -> None:
        """Callback for occupancy grid"""
        self.last_occ_grid = msg

    def start_cbk(self, request: Trigger.Request, response: Trigger.Response) -> None:
        """Start the evaluation"""
        _ = (request,)
        self._timer = self.create_timer(2.0, self.evaluate)

        timestamp = self.last_occ_grid.header.stamp
        width = self.last_occ_grid.info.width
        height = self.last_occ_grid.info.height
        resolution = self.last_occ_grid.info.resolution
        self.logger.info(
            "%d.%d %d %d %f", timestamp.sec, timestamp.nanosec, width, height, resolution)
        self.start_timestamp = timestamp.sec + timestamp.nanosec * 1e-9
        self.evaluate()

        response.success = True
        return response

    def evaluate(self) -> None:
        """Evaluate the exploration"""
        timestamp = self.last_occ_grid.header.stamp
        unique, counts = np.unique(self.last_occ_grid.data, return_counts=True)
        counter = dict(zip(unique, counts))
        self.logger.info("%d.%d %s", timestamp.sec, timestamp.nanosec, counter)

        size = self.last_occ_grid.info.width * self.last_occ_grid.info.height
        self.explored_percent = 100 * (size - counter[-1]) / size
        self.last_time = (
            timestamp.sec + timestamp.nanosec * 1e-9) - self.start_timestamp

        # TODO: ROS log landmarks each X seconds


if __name__ == "__main__":
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file_default = f"exploration_{now}.log"

    parser = argparse.ArgumentParser(
        prog="evaluate.py", description="Evaluate exploration", add_help=True)
    parser.add_argument("-s", "--use-sim-time", action="store_true",
                        help="Use simulation time")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Verbose mode")
    parser.add_argument("-p", "--plot-data", action="store_true",
                        help="Plot real time exploration data")
    parser.add_argument("-l", "--log-file", type=str,
                        default=log_file_default, help="Log file")
    args = parser.parse_args()

    rclpy.init()

    print("Saving log at", args.log_file)
    evaluator = Evaluator(args.use_sim_time, args.log_file, args.verbose)

    if args.plot_data:
        vis = Visualizer(x_lim=600)  # 10 minutes
        ani = animation.FuncAnimation(vis.fig, vis.update_plot, evaluator.yield_viz,
                                      interval=1000, init_func=vis.init_plot)

        threading.Thread(target=rclpy.spin, args=(evaluator,)).start()
        plt.show()
    else:
        rclpy.spin(evaluator)

    rclpy.shutdown()
    sys.exit(0)
