"""Testing..."""

from parse_occ_grid_msg import viz_img, csv_to_occ_grid, occ_grid_to_map, map_to_img
from frontiers import get_frontiers, paint_frontiers
from viz_utils import resize
from skeletonize import skeletonize
from filter import myfilter


def main():
    csv_file = 'dev/map2filter.csv'
    csv_file = 'dev/big_map_1.csv'

    occ_grid = csv_to_occ_grid(csv_file)
    map_ = occ_grid_to_map(occ_grid)
    img = map_to_img(map_)
    viz_img(img, "img", 3, wait=False)

    # img1 = img.copy()
    # img1[img1 == 128] = 0
    # skel = skeletonize(img1)
    # viz_img(skel, "skel", 3, False)

    centroids, frontiers = get_frontiers(img, 15, 30)
    aux = paint_frontiers(img, frontiers, centroids)
    viz_img(aux, "frontiers", 3, wait=False)

    centroids, frontiers = get_frontiers(myfilter(img), 15, 30)
    aux = paint_frontiers(myfilter(img), frontiers, centroids)
    viz_img(aux, "frontiers_prefiltered", 3)


if __name__ == "__main__":
    main()
