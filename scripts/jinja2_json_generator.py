from jinja2 import Environment, FileSystemLoader
import random
import pprint
import math
import matplotlib.pyplot as plt
import numpy as np
import typing
import argparse


def distance(point1: list, point2: list):
    """
    Calculate the distance between 2 points

    point = (x,y)
    """
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def randomize_drone_pose():
    """
    Generate a random drone position within a -5 to +5 square
    """
    x = round(random.uniform(-5, 5), 2)
    y = round(random.uniform(-5, 5), 2)
    yaw = round(random.uniform(0, 2 * math.pi), 2)

    drone = [{
        "xyz": [x, y, 0],
        "rpy": [0, 0, yaw],
    }]

    return drone, x, y


def generate_objects(num_objects: int, drone_coords: list, min_distance: int):
    """
    Given drone coordinates, generate random objects within the same grid

    drone_coords = (x, y)
    min_distance = minimum distance away from drone
    """

    objects = []
    drones = [drone_coords]
    x_ls = []
    y_ls = []
    for i in range(num_objects):
        while True:
            x = round(random.uniform(-5, 5), 2)
            y = round(random.uniform(-5, 5), 2)

            too_close = any(distance((x, y), drone) <
                            min_distance for drone in drones)

            if not too_close:
                break
        object = {
            "model_name": f"pole{i + 1}",
            "xyz": [x, y, 0],
        }
        objects.append(object)
        x_ls.append(x)
        y_ls.append(y)

    return objects, x_ls, y_ls


def generate_json_world(world_name: str, num_world: int, num_object: int, safety_margin: int, visualize: bool):
    environment = Environment(loader=FileSystemLoader("../assets/templates/"))

    world_template = environment.get_template("world.json.jinja")

    for i in range(num_world):
        random_drone_pose, x_drone, y_drone = randomize_drone_pose()

        drone_xy = (x_drone, y_drone)

        generated_objects, xpoints, ypoints = generate_objects(
            num_object, drone_xy, safety_margin)

        world_data = {"world_name": f"{world_name}{i+1}",
                      "objects": generated_objects, "drones": random_drone_pose}

        world_output = world_template.render(world_data)

        # print(world_output)

        if visualize:
            # pprint.pprint(world_output)

            angles = np.linspace(0, 2 * np.pi, 100)
            x_coords_O = x_drone + safety_margin * np.cos(angles)
            y_coords_O = y_drone + safety_margin * np.sin(angles)

            # plt.figure(figsize=(6, 6))
            plt.plot(xpoints, ypoints, 'o')
            plt.plot(x_drone, y_drone, 'D:r')
            plt.plot(x_coords_O, y_coords_O, color='purple')
            plt.axis('equal')  # Equal scaling for x and y axes
            plt.xlim(-5, 5)
            plt.ylim(-5, 5)
            plt.xlabel('X-axis')
            plt.ylabel('Y-axis')
            plt.title(f"World {i+1}")
            plt.grid()
            plt.show()

        json_file_path = f"../assets/worlds/world_{i + 1}.json"

        # Write the JSON string to the file
        with open(json_file_path, "w") as json_file:
            json_file.write(world_output)

        print(f"World {i+1} has been saved")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Generate a randomize world as a json file')

    parser.add_argument('-viz', '--visualize',
                        action='store_true', help='visualize generated worlds')

    parser.add_argument('number_of_objects', metavar='objects', type=int,
                        help='number of objects to generate')

    parser.add_argument('number_of_worlds', metavar='worlds', type=int, nargs='?',
                        default=1, help='number of worlds to generate')

    parser.add_argument('margin_of_safety', metavar='safety', type=int, nargs='?',
                        default=2, help='margin of safety for the drone (m)')

    parser.add_argument('-name', '--world_name', metavar='name', type=str, nargs='?',
                        default=None, help='generic name of generated worlds')

    args = parser.parse_args()

    num_obj = args.number_of_objects            # Number of Objects

    mos = args.margin_of_safety                 # Margin of Safety for the drone

    num_world = args.number_of_worlds           # Number of worlds to generate

    # Visualize the generated world in matplotlib
    visualizing = args.visualize

    world_name = args.world_name

    generate_json_world(world_name, num_world, num_obj, mos, visualizing)
