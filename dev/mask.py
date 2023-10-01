"""
Some testing for masking images
"""

import cv2
import numpy as np

full_white = np.ones((500, 500), dtype=np.uint8) * 255
print(full_white.dtype)

mask = np.zeros(full_white.shape[:2], dtype=np.uint8)

mask = cv2.rectangle(mask, (100, 100), (150, 150), 255, -1)
cv2.imshow('mask', mask)

masked = cv2.bitwise_and(full_white, full_white, mask=mask)

cv2.imshow('full_white', masked)
cv2.waitKey(0)
