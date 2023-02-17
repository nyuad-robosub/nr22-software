import numpy as np
import matplotlib.pyplot as plt
import sys
import cv2

#read image and convert to grayscale
img = cv2.imread('/Users/nasheed-x/Desktop/RoboSub/GatePos/gate.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#detect edges in the frame
gray = cv2.bitwise_not(gray)
gray_edges = cv2.Canny(gray, 50, 100)

#convert to binary image
bw = cv2.adaptiveThreshold(gray_edges, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, -2)

vertical = np.copy(bw)

rows = vertical.shape[0]
verticalsize = rows // 50

#isolate vertical edges
verticalStructure = cv2.getStructuringElement(cv2.MORPH_RECT, (1, verticalsize))

vertical = cv2.erode(vertical, verticalStructure)
vertical = cv2.dilate(vertical, verticalStructure)
vertical = cv2.bitwise_not(vertical)

#filter noise and smoothen edges
edges = cv2.adaptiveThreshold(vertical, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, -2)
kernel = np.ones((3, 3), np.uint8)
edges = cv2.dilate(edges, kernel)

smooth = np.copy(vertical)
smooth = cv2.blur(smooth, (2, 2))

(rows, cols) = np.where(edges != 0)
vertical[rows, cols] = smooth[rows, cols]

cv2.imwrite("output.jpg", vertical)
