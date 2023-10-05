"""get frontiers from map"""
from __future__ import annotations
import cv2
import numpy as np

SAFETY_DISTANCE = 0.3
RESOLUTION = 0.1


def random_color() -> tuple:
    """radom color tuple"""
    return tuple(np.random.randint(0, 255, 3).tolist())


def get_frontiers(img: np.array, area_thresh: int = 10) -> tuple[list, list]:
    """Get map frontiers"""
    unk_obs = np.copy(img)
    unk_obs[unk_obs == 128] = 0
    it = int(SAFETY_DISTANCE / RESOLUTION)
    unk_obs = cv2.erode(unk_obs, np.ones([3*it, 3*it]))

    # https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga04723e007ed888ddf11d9ba04e2232de
    edges = cv2.Canny(unk_obs, 250, 250)

    obstacles = np.copy(img)
    obstacles[obstacles == 128] = 255
    drone_size = np.array([it] * 2).astype(int)
    # cv2.rectangle(obstacles, [69, 99]-drone_size,
    #               [69, 99] + drone_size, 0, -1)  # drone pose
    # one extra iteration to filter borders of map
    obstacles = cv2.erode(obstacles, np.ones([3*(it+1), 3*(it+1)]))

    frontiers = cv2.bitwise_and(obstacles, edges)

    # findContours + moments dont work well for 1 pixel lines (polygons)
    # Using connectedComponents instead
    output = cv2.connectedComponentsWithStats(frontiers)
    (num_labels, labels, stats, centroids) = output

    filtered_masks = []
    filtered_centroids = []
    # item labeled 0 represents the background label, skip background
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < area_thresh:
            continue
        i_mask = (labels == i).astype("uint8") * 255

        filtered_masks.append(i_mask)
        filtered_centroids.append(centroids[i])
    return filtered_centroids, filtered_masks


def paint_frontiers(img: np.array, frontiers: list, centroids: list) -> np.array:
    """Returns a copy of img with frontiers painted"""
    rgb_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    for i, centroid in enumerate(centroids):
        mask = frontiers[i]

        color = random_color()
        rgb_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        rgb_img[(rgb_mask == 255).all(-1)] = color

        (cX, cY) = centroid
        cv2.circle(rgb_img, (int(cX), int(cY)), 2, color, -1)

        # cv2.imshow(f'mask{i}', rgb_mask)

    return rgb_img


def main(file_name: str):
    """get frontiers"""
    img = cv2.imread(file_name, cv2.IMREAD_GRAYSCALE)
    # img = cv2.resize(img, (500, 500))

    centroids, frontiers = get_frontiers(img)
    new_img = paint_frontiers(img, frontiers, centroids)
    cv2.imshow('frontiers', new_img)
    cv2.waitKey(0)


if __name__ == "__main__":
    # input img: 200x200 with free space = 255, obstacles = 0, unknown = 128
    main('map.png')
