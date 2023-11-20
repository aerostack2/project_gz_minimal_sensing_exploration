"""
world_visualizer.py
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt

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


def visualize_world(name: str, drones: dict[str, Pose2D],
                    objects: dict[str, Pose2D]) -> None:
    """Visualize world in matplotlib"""

    drone_xs = [item[0] for item in drones.values()]
    drone_ys = [item[1] for item in drones.values()]
    xpoints = [item[0] for item in objects.values()]
    ypoints = [item[1] for item in objects.values()]
    plt.plot([10, 10, -10, -10, 10], [10, -10, -10, 10, 10], 'k-')
    plt.plot(xpoints, ypoints, 'o')
    plt.plot(drone_xs, drone_ys, 'rD')
    plt.axis('equal')  # Equal scaling for x and y axes
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title(f"{name}")
    plt.show()


def main(json_filename: str):
    """Main function"""
    if not Path(json_filename).exists():
        raise FileNotFoundError(f"{json_filename} not found")
    world_name, drones = parse_json(json_filename)

    xml_filename = Path(json_filename).parent / f"{world_name}.sdf"
    poles = parse_xml(xml_filename)
    visualize_world(world_name, drones, poles)


if __name__ == "__main__":
    # main("assets/worlds/world_multi_ranger1.json")
    # main("assets/worlds/world_two_multi_ranger1.json")
    main("assets/worlds/world_three_multi_ranger1.json")
