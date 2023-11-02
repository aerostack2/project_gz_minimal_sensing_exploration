"""Testing..."""
import numpy as np
import cv2

from parse_occ_grid_msg import viz_img, csv_to_occ_grid, occ_grid_to_map, map_to_img
from frontiers import get_frontiers, paint_frontiers, random_color
from viz_utils import resize


def main():
    csv_file = 'test2.csv'

    occ_grid = csv_to_occ_grid(csv_file)
    map_ = occ_grid_to_map(occ_grid)
    img = map_to_img(map_)

    centroids, frontiers = get_frontiers(img, 20)
    aux = paint_frontiers(img, frontiers, centroids)
    viz_img(aux, "frontiers", 3, False)

    rgb_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    for i, centroids in enumerate(centroids):
        mask = frontiers[i]
        # (x, y), (MA, ma), angle = cv2.fitEllipse(mask)
        # print(angle)

        # print(f"mask{i}", cv2.countNonZero(mask))
        if cv2.countNonZero(mask) > 50:
            # test = cv2.erode(mask, np.ones([1, 3]))
            # viz_img(mask, f'mask{i}', 3, False)
            pass
        else:
            continue

        # https://docs.opencv.org/4.x/d6/d10/tutorial_py_houghlines.html
        lines = cv2.HoughLinesP(mask, rho=1, theta=np.pi/360,
                                threshold=10, minLineLength=10, maxLineGap=10)

        if lines is None:
            continue

        line_image = np.zeros_like(mask)
        for line in lines:
            # (x, y), (MA, ma), angle = cv2.fitEllipse(line)
            # print(angle)

            color = random_color()
            x1, y1, x2, y2 = line[0]
            px = (x2 - x1) / 2 + x1
            py = (y2 - y1) / 2 + y1
            cv2.line(rgb_img, (x1, y1), (x2, y2), color, 1)
            cv2.circle(rgb_img, (int(px), int(py)), 1, color, -1)

        # (cX, cY) = centroids[i]
        # cv2.circle(rgb_img, (int(cX), int(cY)), 2, color, -1)

        # cv2.rectangle(rgb_img, (x, y), (x+w, y+h), color, 1)
        # cv2.imshow(f'mask{i}', rgb_mask)

    viz_img(rgb_img, "lines", 3)
    cv2.imwrite("/tmp/lines.png", rgb_img)


if __name__ == "__main__":
    main()
