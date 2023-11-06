"""filter.py"""
import cv2
import numpy as np

from parse_occ_grid_msg import csv_to_occ_grid, occ_grid_to_map, map_to_img
from viz_utils import viz_img


def myfilter(img: np.array) -> np.array:
    """ OpenCV function to return a skeletonized version of img, a Mat object"""
    aux = img.copy()

    aux = cv2.morphologyEx(aux, cv2.MORPH_CLOSE, np.ones([3, 3]))
    aux[img == 0] = 0

    # Otra opcion menos agresiva
    # Algo asi: convolucion 3x3 y compruebo si el pixel central es mayor 8*scaled_value

    # # endpoints should only have one neighbor,
    # # then endpoint value will be 100*2 (neighbors + itself)
    # f_scaled = np.copy(frontier)
    # f_scaled[f_scaled == 255] = 100
    # filtered_img = cv2.filter2D(f_scaled, -1, np.ones([3, 3]))
    # # masking to only pixels in frontier
    # filtered_img = cv2.bitwise_and(filtered_img, frontier, frontier)

    return aux


def main():
    csv_file = 'dev/map2filter.csv'

    occ_grid = csv_to_occ_grid(csv_file)
    map_ = occ_grid_to_map(occ_grid)
    img = map_to_img(map_)
    viz_img(img, "original", 3, wait=False)

    filtered = myfilter(img)
    viz_img(filtered, "filtered", 3)


if __name__ == "__main__":
    main()
