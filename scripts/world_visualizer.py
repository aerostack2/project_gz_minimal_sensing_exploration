"""
world_visualizer.py
"""
import json
from typing import Any
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from viz_evaluation import LogData

Pose2D = tuple[float, float]


def parse_json(filename: str) -> tuple[str, dict[str, Pose2D]]:
    """Parse json file and return XML world name and drone positions"""
    with open(filename, 'r', encoding='utf-8') as f:
        environment = json.loads(f.read())
    world_name = environment['world_name']
    drones = {}
    for drone in environment['drones']:
        drones[drone['model_name']] = drone['xyz'][:-1]
    return world_name, drones


def parse_xml(filename: str) -> dict[str, Pose2D]:
    """Parse XML file and return pole positions"""
    world_tree = ET.parse(filename).getroot()
    models = {}
    for model in world_tree.iter('include'):
        if model.find('uri').text == 'model://pole':
            x, y, *_ = model.find('pose').text.split(' ')
            models[model.find('name').text] = (float(x), float(y))

    return models


class WorldFigure:
    """World matplotlib figure"""

    def __init__(self, name: str, plot_boundaries: bool = False) -> None:
        self.fig: Figure = plt.figure(name)
        self.name = name
        side_length: float = 10.0

        self.main_plot = self.fig.add_subplot(1, 1, 1)
        self.main_plot.axis('equal')
        # self.main_plot.set_xlabel('X-axis')
        # self.main_plot.set_ylabel('Y-axis')
        # self.main_plot.set_title(f"{name}")

        # Draw world boundaries
        if plot_boundaries:
            self.main_plot.plot([side_length, side_length, -side_length, -side_length, side_length],
                                [side_length, -side_length, -side_length,
                                    side_length, side_length],
                                'k-')

    def draw_drones(self, drones: dict[str, Pose2D], color: str = 'rD') -> None:
        """Draw drones on plot"""
        drone_xs = [item[0] for item in drones.values()]
        drone_ys = [item[1] for item in drones.values()]
        labels = [item for item in drones.keys()]
        if len(labels) == 1:
            labels = labels[0]
        self.main_plot.plot(drone_xs, drone_ys, color, label=labels)

    def draw_obstacles(self, obstacles: dict[str, Pose2D], color: str = 'o') -> None:
        """Draw obstacles on plot"""
        xpoints = [item[0] for item in obstacles.values()]
        ypoints = [item[1] for item in obstacles.values()]
        self.main_plot.plot(xpoints, ypoints, color)

    def draw_paths(self, paths: dict[str, Pose2D], color: str = 'b') -> None:
        """Draw paths on plot"""
        for path in paths.values():
            x, y = [], []
            for msg in path:
                x.append(msg[0])
                y.append(msg[1])
            self.main_plot.plot(x, y, color)

    def draw_grid_maps(self, grid_d: dict[tuple[np.ndarray, Any]], colors: list[str] | str = 'g'):
        """Draw grid map layers on plot"""
        colors = colors if isinstance(colors, list) else [
            colors] * len(grid_d.keys())
        for grid_t, color in zip(grid_d.values(), colors):
            grid, info = grid_t
            x, y = [], []
            for i in range(grid.shape[0]):
                for j in range(grid.shape[1]):
                    if grid[i, j] != 128:
                        x.append(i*info.resolution - info.length_x/2)
                        y.append(j*info.resolution - info.length_y/2)
            self.main_plot.scatter(x, y, color=color, alpha=0.025)

    def show(self) -> None:
        """Show plot"""
        self.fig.savefig(f"/tmp/{self.name}.png")
        plt.show()


def visualize_world(world_name, drones, obstacles):
    """Visualize world"""
    fig = WorldFigure(world_name)
    fig.draw_obstacles(obstacles)
    fig.draw_drones(drones)
    fig.show()


def main(json_filename: str):
    """Main function"""
    if not Path(json_filename).exists():
        raise FileNotFoundError(f"{json_filename} not found")
    world_name, drones = parse_json(json_filename)
    colors = ['r', 'm', 'b', 'y', 'g', 'c', 'k', 'w']

    xml_filename = Path(json_filename).parent / f"{world_name}.sdf"
    poles = parse_xml(xml_filename)

    data = LogData.from_rosbag(Path("rosbags/exploration_20240408_005107"))

    fig = WorldFigure("World")
    fig.draw_obstacles(poles)
    for drone, pose, path, c in zip(drones.keys(), drones.values(), data.poses.values(), colors):
        fig.draw_drones({drone: pose}, c + 'D')
        fig.draw_paths({drone: path}, c)
    fig.draw_grid_maps(data.grids, colors=['#96ff89', '#89cfff', '#ff8989'])
    fig.main_plot.legend()
    fig.show()


if __name__ == "__main__":
    main("assets/worlds/low5.json")
