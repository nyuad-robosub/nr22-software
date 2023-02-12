import os
import cv2
import matplotlib.pyplot as plt
import numpy as np

img = cv2.imread("Image Path")

canvas = np.zeros(img.shape)
canvas.fill(0)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# lower bound and upper bound for orange color
lower_bound = np.array([0,120,60])
upper_bound = np.array([65,255,255])

#find color within the boundaries
mask = cv2.inRange(hsv, lower_bound, upper_bound)

#define kernel size  
kernel = np.ones((30,30),np.uint8)

# remove unnecessary noise from mask
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

edges= cv2.Canny(mask,30,200)
contours, hierarchy= cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

cv2.drawContours(canvas, contours, -1, (0,255,0),3)

sorted_cont = sorted(contours, key=cv2.contourArea)

rect = cv2.minAreaRect(sorted_cont[-1])
box = cv2.boxPoints(rect)
box = np.int0(box)

center = (int(rect[0][0]),int(rect[0][1])) 
width = int(rect[1][0])
height = int(rect[1][1])
angle = int(rect[2])

cv2.drawContours(canvas,[box],0,(255, 0,0),5)
cv2.circle(canvas, center, 10, (0,0,255), -5)

print(width)
print(height)

if height < width :
    angle = angle
else:
    angle = 180 - angle


print(angle)
