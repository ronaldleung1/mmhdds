import cv2
import numpy as np


# 11 in = 0.2794 m
# focal_l = 11 * 220 / 0.2794 # F = (P x D) / W
# rho = 0.2794 * focal_l / (box_w*220)
# tennis ball diameter = 0.067 m

cap = cv2.VideoCapture(0)
ret, img = cap.read()
if ret:
    img = cv2.resize(img, (int(img.shape[1] * 0.5), int(img.shape[0] * 0.5)), interpolation=cv2.INTER_AREA)
    cv2.imshow('Picture', img)
    cv2.imwrite('test.jpg', img)
else:
    print("Error: Cannot read video stream")
    exit()

cap.release()
cv2.destroyAllWindows()
