"""Exploring evaluation script"""

import sys
import logging
import argparse
import threading
from datetime import datetime
from dataclasses import dataclass
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


@dataclass
class PlottingData:
    """Data used for plotting"""
    occ_grid: OccupancyGrid

    def __post_init__(self):
        self._first_stamp = self.occ_grid.header.stamp.sec + \
            self.occ_grid.header.stamp.nanosec * 1e-9

    def log(self) -> str:
        """Return a log string"""
        return f"{self.timestamp} {self.explored_area} {self.counter}"

    def log_header(self) -> str:
        """Log header"""
        return f"{self._first_stamp} {self.occ_grid.info.width} {self.occ_grid.info.height} {self.occ_grid.info.resolution}"

    @property
    def timestamp(self) -> str:
        """Data timestamp (s)"""
        return (self.occ_grid.header.stamp.sec +
                self.occ_grid.header.stamp.nanosec * 1e-9) - self._first_stamp

    @property
    def counter(self) -> dict[int, int]:
        """Dictionary of counter cells in occ_grid"""
        unique, counts = np.unique(self.occ_grid.data, return_counts=True)
        return dict(zip(unique, counts))

    @property
    def explored_area(self) -> float:
        """Explored area (m^2)"""
        size = self.occ_grid.info.width * self.occ_grid.info.height
        return (size - self.counter[-1]) * \
            self.occ_grid.info.resolution * self.occ_grid.info.resolution

    @property
    def explored_percent(self) -> float:
        """Explored area (%)"""
        size = self.occ_grid.info.width * self.occ_grid.info.height
        return 100 * (size - self.counter[-1]) / size


class Visualizer:
    """Matplotlib visualizer"""

    def __init__(self, x_lim: int, y_lim: int):
        self.xlim = x_lim
        self.ylim = y_lim

        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], lw=2)
        self.ax.set(xlabel='time (s)', ylabel='area (%)',
                    title='Exploration Progress')
        self.ax.grid()
        self.ax2 = self.ax.twinx()
        self.line2, = self.ax2.plot([], [], lw=2)
        self.ax2.set_ylabel('area (m^2)')
        self.xdata, self.ydata, self.ydata2 = [], [], []

        self.fig.tight_layout()

    def init_plot(self):
        """Initialize the plot"""
        self.ax.set_ylim(0, 100)
        self.ax.set_xlim(0, self.xlim)
        self.ax2.set_ylim(0, self.ylim)
        self.line.set_data(self.xdata, self.ydata)
        self.line2.set_data(self.xdata, self.ydata2)
        return self.line

    def update_plot(self, frame: PlottingData):
        """Update plot data"""
        if frame is None:
            return self.line
        self.xdata.append(frame.timestamp)
        self.ydata.append(frame.explored_percent)
        self.ydata2.append(frame.explored_area)

        self.line.set_data(self.xdata, self.ydata)
        self.line2.set_data(self.xdata, self.ydata2)
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

        self.plotting_data: PlottingData = None
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
        yield self.plotting_data

    def occ_grid_cbk(self, msg: OccupancyGrid) -> None:
        """Callback for occupancy grid"""
        self.last_occ_grid = msg

    def start_cbk(self, request: Trigger.Request, response: Trigger.Response) -> None:
        """Start the evaluation"""
        _ = (request,)
        self._timer = self.create_timer(2.0, self.evaluate)

        self.plotting_data = PlottingData(self.last_occ_grid)
        self.logger.info(self.plotting_data.log_header())
        self.evaluate()

        response.success = True
        return response

    def evaluate(self) -> None:
        """Evaluate the exploration"""
        self.plotting_data.occ_grid = self.last_occ_grid
        self.logger.info(self.plotting_data.log())

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
        vis = Visualizer(x_lim=600, y_lim=400)  # 10 minutes
        ani = animation.FuncAnimation(vis.fig, vis.update_plot, evaluator.yield_viz,
                                      interval=1000, init_func=vis.init_plot)

        threading.Thread(target=rclpy.spin, args=(evaluator,)).start()
        plt.show()
    else:
        rclpy.spin(evaluator)

    rclpy.shutdown()
    sys.exit(0)
