"""Some utils to work with cv2"""
import numpy as np
import cv2


def viz_img(img: np.array, scale: float = 1.0) -> None:
    """Visualize image"""
    # Method 2: int8 to uint8
    if scale != 1.0:
        aux = resize(img, scale)
    cv2.imshow('img', aux)
    cv2.waitKey(0)


def resize(img: np.array, factor: float) -> np.array:
    """Rezise image by given factor"""
    return cv2.resize(img, (img.shape[0]*factor, img.shape[1]*factor))
