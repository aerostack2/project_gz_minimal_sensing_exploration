"""get frontiers from map"""
from __future__ import annotations
import cv2
import numpy as np

SAFETY_DISTANCE = 0.3
RESOLUTION = 0.1


def random_color() -> tuple:
    """radom color tuple"""
    return tuple(np.random.randint(0, 255, 3).tolist())


def get_drift_from_x_axis(vx: float, vy: float) -> float:
    """get angle between x axis and vector defined by vx, vy"""
    # unit vector in the same direction as the x axis
    x_axis = np.array([1, 0])

    # unit vector in the same direction as your line
    vector = np.array([vx, vy])
    dot_product = np.dot(x_axis, vector)
    angle_2_x = np.arccos(dot_product)
    return angle_2_x.item()


def rotate_image(image, angle):
    """
    https://stackoverflow.com/questions/9041681/opencv-python-rotate-image-by-x-degrees-around-specific-point
    """
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(
        image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result


def calc_centroid(arr: np.array) -> tuple:
    """calc centroid of array"""
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return int(sum_y/length), int(sum_x/length)


def rotate_around(pts: np.array, angle: float, origin: np.array) -> np.array:
    """
    rotate array around origin
        return R * (pts - o) + o
    """
    R = np.array([(np.cos(angle), -np.sin(angle)),
                  (np.sin(angle),  np.cos(angle))])
    o = np.array(origin)
    w = np.zeros_like(pts)
    for i, v in enumerate(pts):
        w[i] = np.squeeze((R @ (v.T-o.T) + o.T).T)
    return w


def split_frontier(frontier: np.array, n: int) -> list[np.array]:
    """Split frontier in n parts"""
    # get list of point in frontier
    pts = np.argwhere(frontier == 255)
    # get orientation of frontier from point list
    [vx, vy, x, y] = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
    angle = get_drift_from_x_axis(vx, vy)  # rad

    # rotate frontier to align with x axis
    o = np.array(frontier.shape[1::-1]) / 2
    rotated = rotate_around(pts, -angle, o)
    # sort by x
    rotated = rotated[np.argsort(rotated[:, 0])]

    # split rotated frontier in n parts
    tokens = np.array_split(rotated, n)

    frontiers = []
    centroids = []
    for token in tokens:
        # rotate back
        tok = rotate_around(token, angle, o)
        frontiers.append(tok)
        centroids.append(calc_centroid(tok))
    return frontiers, centroids


def get_frontiers(img: np.array, min_thresh: int = 10, max_thresh: int = 30) -> tuple[list, list]:
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
        # x = stats[i, cv2.CC_STAT_LEFT]
        # y = stats[i, cv2.CC_STAT_TOP]
        # w = stats[i, cv2.CC_STAT_WIDTH]
        # h = stats[i, cv2.CC_STAT_HEIGHT]
        if area < min_thresh:
            continue
        i_mask = (labels == i).astype("uint8") * 255
        # cv2.imshow(f'mask{i}', i_mask)

        n = area // max_thresh
        if n <= 1:
            filtered_masks.append(i_mask)
            filtered_centroids.append(centroids[i])
            continue

        front, centr = split_frontier(i_mask, n)
        for i, f in enumerate(front):
            myimg = np.zeros_like(img)
            myimg[f[:, 0], f[:, 1]] = 255
            filtered_masks.append(myimg)
            filtered_centroids.append(centr[i])
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

    src = np.array([[0, 0], [0, 1], [0, 2]])
    # angle = -np.pi/2
    # origin = (0, 0)
    # rotated = rotate_around(src, angle, origin)
    # assert np.flip(src, 1).tolist() == rotated.tolist()
    # exit()

    main('map.png')
