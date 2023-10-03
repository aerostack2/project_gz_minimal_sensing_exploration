"""Testing..."""
import numpy as np
import cv2

from parse_occ_grid_msg import viz_img, csv_to_occ_grid, occ_grid_to_map, map_to_img
from frontiers import get_frontiers, paint_frontiers
from viz_utils import resize
from skeletonize import skeletonize


def main():
    csv_file = 'test2.csv'

    occ_grid = csv_to_occ_grid(csv_file)
    map_ = occ_grid_to_map(occ_grid)
    img = map_to_img(map_)

    img1 = img.copy()
    img1[img1 == 128] = 0
    skel = skeletonize(img1)
    viz_img(skel, "skel", 3, False)

    centroids, frontiers = get_frontiers(img)
    aux = paint_frontiers(img, frontiers, centroids)
    viz_img(aux, "frontiers", 3)


if __name__ == "__main__":
    main()
