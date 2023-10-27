"""Some utils to work with cv2"""
import numpy as np
import cv2


def viz_img(img: np.array, name: str = "img",
            scale: float = 1.0, wait: bool = True, save: bool = True) -> None:
    """Visualize image"""
    aux = img.copy()
    if scale != 1.0:
        aux = resize(img, scale)
    cv2.imshow(name, aux)
    if save:
        cv2.imwrite(f'/tmp/{name}.png', aux)
    if wait:
        cv2.waitKey(0)


def resize(img: np.array, factor: float) -> np.array:
    """Rezise image by given factor"""
    return cv2.resize(img, (img.shape[0]*factor, img.shape[1]*factor))
