import os
import cv2
import matplotlib.pyplot as plt
import numpy as np

def calculate_contour_distance(contour1, contour2): 
    x1, y1, w1, h1 = cv2.boundingRect(contour1)
    c_x1 = x1 + w1/2
    c_y1 = y1 + h1/2

    x2, y2, w2, h2 = cv2.boundingRect(contour2)
    c_x2 = x2 + w2/2
    c_y2 = y2 + h2/2

    return max(abs(c_x1 - c_x2) - (w1 + w2)/2, abs(c_y1 - c_y2) - (h1 + h2)/2)

def merge_contours(contour1, contour2):
    return np.concatenate((contour1, contour2), axis=0)

def agglomerative_cluster(contours, threshold_distance):
    current_contours = []
    current_contours = contours
    while len(current_contours) > 1:
        min_distance = None
        min_coordinate = None

        for x in range(len(current_contours)-1):
            for y in range(x+1, len(current_contours)):
                distance = calculate_contour_distance(current_contours[x], current_contours[y])
                if min_distance is None:
                    min_distance = distance
                    min_coordinate = (x, y)
                elif distance < min_distance:
                    min_distance = distance
                    min_coordinate = (x, y)

        if min_distance < threshold_distance:
            index1, index2 = min_coordinate
            current_contours[index1] = merge_contours(current_contours[index1], current_contours[index2])
            del current_contours[index2]
        else: 
            break

    return current_contours

img = cv2.imread("/Users/nasheed-x/Desktop/HoleDetection/real.png")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#lower red
lower1 = np.array([0, 100, 20])
upper1 = np.array([10, 255, 255])

# upper red
lower2 = np.array([160,100,20])
upper2 = np.array([179,255,255])

#find color within the boundaries
mask1 = cv2.inRange(hsv, lower1, upper1)
mask2 = cv2.inRange(hsv, lower2, upper2)

mask = mask1 + mask2;

#define kernel size  
kernel = np.ones((3,3),np.uint8)

# remove unnecessary noise from mask
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

#create canvas
canvas = np.zeros(img.shape)
canvas.fill(255)

contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnt_ = agglomerative_cluster(list(contours), 100)
cv2.drawContours(canvas, cnt_, -1, (0, 255, 0), 3)

sorted_cont = sorted(contours, key=cv2.contourArea)

Rect1 = cv2.minAreaRect(sorted_cont[-1])
Box1 = cv2.boxPoints(Rect1)
Box1 = np.int0(Box1)

Rect2 = cv2.minAreaRect(sorted_cont[-2])
Box2 = cv2.boxPoints(Rect2)
Box2 = np.int0(Box2)

X1 = int(Rect1[0][0])
Y1 = int(Rect1[0][1])

X2 = int(Rect2[0][0])
Y2 = int(Rect2[0][1])

cv2.circle(canvas, (X1,Y1), 20, (0,0,255), -5)
cv2.circle(canvas, (X2,Y2), 20, (0,0,255), -5)

cv2.drawContours(canvas,[Box1],0,(255, 0,0),5)
cv2.drawContours(canvas,[Box2],0,(255, 0,0),5)

print('Larger Hole Center: ' + str(X1), str(Y1))
print('Larger Hole Region: ', Box1)


print('Smaller Hole Center: ' + str(X2), str(Y2))
print('Smaller Hole Region: ', Box2)
