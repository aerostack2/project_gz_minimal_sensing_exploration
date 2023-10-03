"""parse_occ_grid_msg.py"""

import numpy as np
import cv2

from nav_msgs.msg import OccupancyGrid

from viz_utils import viz_img


def csv_to_occ_grid(file: str) -> OccupancyGrid:
    """CSV file to OccupancyGrid"""
    with open(file, encoding='utf-8') as fd:
        text = fd.readlines()[0]

    stamp_s, stamp_ns, frame, *tokens = text.split(',')
    load_s, load_ns, *tokens = tokens
    res, w, h, x, y, z, qx, qy, qz, qw, *data = tokens

    occ_grid = OccupancyGrid()
    occ_grid.header.stamp.sec = int(stamp_s)
    occ_grid.header.stamp.nanosec = int(stamp_ns)
    occ_grid.header.frame_id = frame
    occ_grid.info.map_load_time.sec = int(load_s)
    occ_grid.info.map_load_time.nanosec = int(load_ns)
    occ_grid.info.resolution = float(res)
    occ_grid.info.width = int(w)
    occ_grid.info.height = int(h)
    occ_grid.info.origin.position.x = float(x)
    occ_grid.info.origin.position.y = float(y)
    occ_grid.info.origin.position.z = float(z)
    occ_grid.info.origin.orientation.x = float(qx)
    occ_grid.info.origin.orientation.y = float(qy)
    occ_grid.info.origin.orientation.z = float(qz)
    occ_grid.info.origin.orientation.w = float(qw)
    occ_grid.data = list(map(int, data))
    return occ_grid


def occ_grid_to_map(occ_grid: OccupancyGrid) -> np.array:
    """OccupancyGrid to map (np.array)"""
    img = np.array(occ_grid.data, dtype=np.int8)
    img = img.reshape((occ_grid.info.width, occ_grid.info.height))
    img[img == 0] = 127
    img[img == 100] = 0
    return img


def map_to_img(img: np.array, obs_value: int = 128) -> np.array:
    """Map to img, basically from int8 to uint8"""
    # Method 1: int8 to uint8
    # img1 = np.zeros(img.shape, dtype=np.uint8)
    # img1[img == 127] = 255
    # img1[img == -1] = obs_value
    # # img1[img == 0] = 0

    # Method 2: int8 to uint8
    aux = cv2.convertScaleAbs(img, alpha=100).astype(np.uint8)
    aux[aux == 100] = obs_value
    return aux


def main():
    csv_file = 'test2.csv'

    occ_grid = csv_to_occ_grid(csv_file)
    map_ = occ_grid_to_map(occ_grid)
    img = map_to_img(map_, 128)
    viz_img(img, 3)


if __name__ == "__main__":
    main()
