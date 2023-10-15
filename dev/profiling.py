"""Testing..."""
import numpy as np
import cv2

from parse_occ_grid_msg import viz_img, csv_to_occ_grid, occ_grid_to_map, map_to_img
from frontiers import get_frontiers, paint_frontiers


def front():
    csv_file = 'test.csv'

    occ_grid = csv_to_occ_grid(csv_file)
    map_ = occ_grid_to_map(occ_grid)
    img = map_to_img(map_)

    centroids, frontiers = get_frontiers(img, 20)
    aux = paint_frontiers(img, frontiers, centroids)
    cv2.imwrite('/tmp/full_frontiers.png', aux)

    # viz_img(aux, "frontiers", 3)


def main():
    import cProfile
    import pstats

    with cProfile.Profile() as pr:
        front()

    stats = pstats.Stats(pr)
    stats.sort_stats(pstats.SortKey.TIME)
    # stats.print_stats()
    stats.dump_stats(filename='needs_profiling.prof')

    # visualization:
    # snakeviz program.prof


if __name__ == "__main__":
    main()
