"""skeleton"""
import cv2
import numpy as np


# input img: 200x200 with free space = 255, obstacles = 0, unknown = 128
img = cv2.imread('map.png', cv2.IMREAD_GRAYSCALE)
img1 = img.copy()

kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
thin = np.zeros(img.shape, dtype='uint8')

while cv2.countNonZero(img1) != 0:
    eroded = cv2.erode(img1, kernel)

    opening = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, kernel)

    subset = eroded - opening
    thin = cv2.bitwise_or(subset, thin)

    img1 = eroded.copy()

cv2.imshow('img', img)
cv2.imshow('thinned', thin)
cv2.waitKey(0)
