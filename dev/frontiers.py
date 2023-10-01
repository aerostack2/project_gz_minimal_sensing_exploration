"""get frontiers from map"""
import cv2
import numpy as np


def random_color() -> tuple:
    """radom color tuple"""
    return tuple(np.random.randint(0, 255, 3).tolist())


# input img: 200x200 with free space = 255, obstacles = 0, unknown = 128
img = cv2.imread('map.png', cv2.IMREAD_GRAYSCALE)
# img = cv2.resize(img, (500, 500))

unk_obs = np.copy(img)
unk_obs[unk_obs == 128] = 0
unk_obs = cv2.erode(unk_obs, np.ones((3, 3)))

# https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html#ga04723e007ed888ddf11d9ba04e2232de
edges = cv2.Canny(unk_obs, 250, 250)

obstacles = np.copy(img)
obstacles[obstacles == 128] = 255
drone_size = np.array([0.3/0.1] * 2).astype(int)
cv2.rectangle(obstacles, [69, 99]-drone_size,
              [69, 99] + drone_size, 0, -1)  # drone pose
obstacles = cv2.erode(obstacles, np.ones((3, 3)))

frontiers = cv2.bitwise_and(obstacles, edges)

# findContours + moments dont work well for 1 pixel lines (polygons)
# Using connectedComponents instead
output = cv2.connectedComponentsWithStats(frontiers)
(numLabels, labels, stats, centroids) = output

rgb_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

# item labeled 0 represents the background label, skip background
for i in range(1, numLabels):
    area = stats[i, cv2.CC_STAT_AREA]
    if area < 10:
        continue
    mask = np.zeros(img.shape, dtype="uint8")
    i_mask = (labels == i).astype("uint8") * 255
    mask = cv2.bitwise_or(mask, i_mask)

    color = random_color()
    rgb_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    rgb_img[(rgb_mask == 255).all(-1)] = color

    (cX, cY) = centroids[i]
    cv2.circle(rgb_img, (int(cX), int(cY)), 2, color, -1)
    # cv2.imshow(f'mask{i}', rgb_mask)

rgb_img = cv2.resize(rgb_img, (400, 400))
cv2.imshow('centroids', rgb_img)
cv2.imwrite('centroids_big.png', rgb_img)

cv2.waitKey(0)
