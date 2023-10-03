"""skeleton"""
import cv2
import numpy as np

from viz_utils import viz_img


def skeletonize(img: np.array) -> np.array:
    """ OpenCV function to return a skeletonized version of img, a Mat object"""
    aux = img.copy()

    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    thin = np.zeros(img.shape, dtype='uint8')

    while cv2.countNonZero(aux) != 0:
        eroded = cv2.erode(aux, kernel)

        opening = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, kernel)

        subset = eroded - opening
        thin = cv2.bitwise_or(subset, thin)

        aux = eroded.copy()
    return thin


def skeletonize2(img):
    """ OpenCV function to return a skeletonized version of img, a Mat object"""

    #  hat tip to http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/

    img = img.copy()  # don't clobber original
    skel = img.copy()

    skel[:, :] = 0
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

    while True:
        eroded = cv2.morphologyEx(img, cv2.MORPH_ERODE, kernel)
        temp = cv2.morphologyEx(eroded, cv2.MORPH_DILATE, kernel)
        temp = cv2.subtract(img, temp)
        skel = cv2.bitwise_or(skel, temp)
        img[:, :] = eroded[:, :]
        if cv2.countNonZero(img) == 0:
            break

    return skel


def main():
    # input img: 200x200 with free space = 255, obstacles = 0, unknown = 128
    img = cv2.imread('map.png', cv2.IMREAD_GRAYSCALE)
    img[img == 128] = 0

    skel = skeletonize(img)
    viz_img(skel, "skel", 3, False)

    skel2 = skeletonize2(img)
    viz_img(skel2, "skel2", 3)


if __name__ == "__main__":
    main()
