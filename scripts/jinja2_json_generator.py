from jinja2 import Environment, FileSystemLoader
import random
import pprint
import math
import matplotlib.pyplot as plt
import numpy as np

# ----------------------------------------------------------------------------------------
#                                                               Parameters:
num = 10            # Number of Objects

r = 2               # Margin of Safety for the drone

num_world = 10      # Number of worlds to generate

visualizing = False
# ----------------------------------------------------------------------------------------


# ----------------------------------------------------------------------------------------
#                                                               Functions:
def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def randomize_drone_pose():
    x = round(random.uniform(-5, 5), 2)
    y = round(random.uniform(-5, 5), 2)
    yaw = round(random.uniform(0, 2 * math.pi), 2)
    
    drone = [{
            "xyz": [x, y, 0],
            "rpy": [0, 0, yaw],
        }]
    
    return drone, x, y

def generate_objects(num_objects, drone_coords, min_distance):
    objects = []
    generated_points = [drone_coords]
    x_ls = []
    y_ls = []
    for i in range(num_objects):
        while True:
            x = round(random.uniform(-5, 5), 2)
            y = round(random.uniform(-5, 5), 2)
            
            too_close = any(distance((x, y), existing_point) < min_distance for existing_point in generated_points)
            
            if not too_close:
                generated_points.append((x, y))
                break
        object = {
            "model_name": f"pole{i + 1}",
            "xyz": [x, y, 0],
        }
        objects.append(object)
        x_ls.append(x)
        y_ls.append(y)
    
    return objects, x_ls, y_ls
# ----------------------------------------------------------------------------------------




# ----------------------------------------------------------------------------------------
#                                                              Main Code:
environment = Environment(loader=FileSystemLoader("../assets/templates/"))

world_template = environment.get_template("world.json.jinja")

for i in range(num_world):
    random_drone_pose, x_drone, y_drone = randomize_drone_pose()

    drone_xy = (x_drone, y_drone)

    generated_objects, xpoints, ypoints = generate_objects(num, drone_xy, r)

    coordinates = list(zip(xpoints, ypoints))

    world_data = {"world_name": f"world_{i + 1}", "objects": generated_objects, "drones": random_drone_pose}

    world_output = world_template.render(world_data)

    # print(world_output)

    if visualizing:
        pprint.pprint(world_output)

        angles = np.linspace(0, 2 * np.pi, 100)
        x_coords_O = x_drone + r * np.cos(angles)
        y_coords_O = y_drone + r * np.sin(angles)

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
    
# ----------------------------------------------------------------------------------------