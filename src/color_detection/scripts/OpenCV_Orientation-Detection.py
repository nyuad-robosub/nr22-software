"""
Install OpenCV, NumPy, DepthAI, MatplotLib
%pip install opencv-python
%pip install matplotlib
%pip install depthai
"""
# Import dependencies
import cv2
import matplotlib.pyplot as plt
import numpy as np
from math import atan2, cos, sin, sqrt, pi

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

def getOrientation(img_source):
  
  cnt=[]

  path = img_source
  # Captures the live stream frame-by-frame
  frame = cv2.imread(path)
        
  # Converts images from BGR to HSV 
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      
  # lower bound and upper bound for orange color
  lower_bound = np.array([0,120,60])
  upper_bound = np.array([65,255,255])

  #find color within the boundaries
  mask = cv2.inRange(hsv, lower_bound, upper_bound)

  #define kernel size  
  kernel = np.ones((7,7),np.uint8)

  # remove unnecessary noise from mask
  mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
  mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

  #create canvas
  canvas = np.zeros(frame.shape)
  canvas.fill(0)

  # find contours from the mask
  contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  clust_cnt = agglomerative_cluster(list(contours), 40)
  cnt = tuple(clust_cnt)

  # get minAreaRect of contour with max area
  max_area = -1

  #loop through contours
  for i in range(len(contours)):
    area = cv2.contourArea(contours[i])
    if area>max_area:
      cnt = contours[i]
      max_area = area

  if len(cnt) == 0:
    print("No Marker Detected")
    exit()

  areas = [cv2.contourArea(c) for c in contours]
  if (len(areas) !=0):
    max_index = np.argmax(areas)
    cnt=contours[max_index]

  #assign minAreaRect to contour with max area
  rect = cv2.minAreaRect(cnt)
  box = cv2.boxPoints(rect)
  box = np.int0(box)

  #draw minAreaRect
  cv2.drawContours(canvas,[box],0,(255, 0,0),2)

  #center coordinates of min_rectangle also p1
  x = int(rect[0][0])
  y = int(rect[0][1])

  #draw at center point
  canvas = cv2.circle(canvas, (x,y), 10, (0,255,0), -5)

  ## [pca]
  # Construct a buffer used by the pca analysis
  sz = len(cnt)
  data_pts = np.empty((sz, 2), dtype=np.float64)
  for i in range(data_pts.shape[0]):
    data_pts[i,0] = cnt[i,0,0]
    data_pts[i,1] = cnt[i,0,1]
 
  # perform PCA analysis
  mean = np.empty((0))
  mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
 
  # store the center of the object
  cntr = (int(mean[0,0]), int(mean[0,1]))
  ## [pca]
 
  ## [visualization]
  # draw the principal components
  cv2.circle(canvas, cntr, 3, (255, 0, 255), 2)
  p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
  p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])

 
  angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
  ## [visualization]
 
  # label with the rotation angle
  label = "  Rotation Angle: " + str(int(np.rad2deg(angle)) + 90) + " degrees"
  textbox = cv2.rectangle(canvas, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
  cv2.putText(canvas, label, (cntr[0], cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)


  # get angle
  rotation = int(np.rad2deg(angle)) + 90
  if rotation >= 180:
    rotation=rotation-180

  cv2.imwrite('/Users/nasheed-x/Desktop/Orientation/test.jpeg', canvas)

  return rotation

a = getOrientation('/Users/nasheed-x/Desktop/Orientation/orange2.jpeg')
print(a) 

