from operator import truediv
from turtle import position

import rospy
import threading
import smach
import os
from std_msgs.msg import Bool
# import smach_controller_simulation
import movement_controller as mc
import actions_utils as au
import viso_controller as vs
import geometry_msgs.msg
import tf2_geometry_msgs
import math
import vision_msgs

from vision_msgs.msg import Detection2DArray, BoundingBox2D
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros

import numpy as np
import cv2
        
class marker(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self,marker_label):
        self.marker_label=marker_label
        self.zigzag_threshold=0.7
    def execute(self, userdata):  
        #mc.mov_control.set_focus_point()
        rospy.sleep(0.5)
        #call search which returns once any bbox is detected
        detection=self.search(self.zigzag_threshold,0.7,self.marker_label)

        #next begin the adjustment function
        mc.mov_control.update_tf()

        trans = mc.mov_control.trans.transform.translation
        outcome,value = au.bottom_aligning(
            mc.mov_control,
            # geometry_msgs.msg.Point(trans.x + 0.6, trans.y, trans.z),
            0.1,
            60,
            vs.bottom_camera,
            "marker",
            0.2,
            0.005
        )
        if(outcome!="succeeded"):
            return "outcome2"

        #get rotation
        rotation_angle=self.getMarkerOrientation()
        rospy.sleep(10)

        print("Rotating angle:")
        print(rotation_angle)
        mc.mov_control.stop()
        

        #next begin rotation to align pose with object
        if(rotation_angle<=90):
            #if marker is pointing top right
            mc.mov_control.rotate_ccw(rotation_angle-90)
        else:
            #if marker is pointing top left
            mc.mov_control.rotate_ccw(rotation_angle-90)
        mc.mov_control.set_focus_point()

        mc.mov_control.await_completion()

        mc.mov_control.stop()

        return "outcome1"
           #else get bbox angle and use trigonometry to determine translation amount return adjustment(self,detection)
    def getMarkerOrientation(self):
        
        cnt=[]

        # Captures the live stream frame-by-frame
        frame = vs.bottom_camera.get_cv_img()
                
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

        
        angle = math.atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
        ## [visualization]
        
        # label with the rotation angle
        #label = "  Rotation Angle: " + str(int(np.rad2deg(angle)) + 90) + " degrees"
        #textbox = cv2.rectangle(canvas, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
        #cv2.putText(canvas, label, (cntr[0], cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)


        # get angle
        rotation = int(np.rad2deg(angle)) + 90
        if rotation >= 180:
            rotation=rotation-180

        #cv2.imwrite('/Users/nasheed-x/Desktop/Orientation/test.jpeg', canvas)

        return rotation

    def search(self,threshold, straight_amount,detection_label): #recursive function to go in zig zags till gate detected
        #TODO MAKE STRAIGHT AMOUNT DETERMINED FROM OUR DEPTH TO SWEEP BOTTOM CAMERA FRAME
        detection = vs.bottom_camera.get_detection([detection_label])
        if(len(detection)==0):
            Points=mc.mov_control.generate_zig_zag(threshold)
            print(Points)
            mc.mov_control.set_goal_points(Points)
            rospy.sleep(0.5)
            detection=mc.mov_control.await_completion_detection(detection_label, vs.bottom_camera)
            mc.mov_control.stop()

        return detection[0]
        # if(len(detection)>0):
        #     #if marker detected return 
        #     #marker detected
        #     mc.mov_control.stop() #is this redundant?
        #     return detection
        # elif(mode=="center"): #starts at center of zig zag
        #     #GO LEFT
        #     mc.mov_control.go_left(threshold)
        #     mc.mov_control.await_completion()
        #     return self.search(threshold,straight_amount,detection_label,"left")
        # elif(mode=="left"):
        #     #GO LEFT
        #     mc.mov_control.go_left(threshold*2)
        #     det=mc.mov_control.await_completion_detection(detection_label)
        #     if(det==None):
        #         return self.search(threshold,straight_amount,detection_label,"right")
        #     else:
        #         return det
        # elif(mode=="right"):
        #     #GO right
        #     mc.mov_control.go_left(-threshold*2)
        #     det=mc.mov_control.await_completion_detection(detection_label)
        #     if(det==None):
        #         return self.search(threshold,straight_amount,detection_label,"straight")
        #     else:
        #         return det
        # else:#(mode=="straight")
        #     mc.mov_control.go_straight(straight_amount)
        #     det=mc.mov_control.await_completion_detection(detection_label)
        #     if(det==None):
        #         return self.search(threshold,straight_amount,detection_label,"left")
        #     else:
        #         return det 
    
    # def adjustment(self,detection): #adjusts center of camera frame to center of marker through recursive function
    #     #input: detection()
    #     #if the detection bbox is close to center STOP

    #     if(np.allclose(detection['center'],vs.bottom_camera.get_center_coord(),0.05)):
    #         mc.mov_control.stop()
    #         return True
    #     else:
    #         angle_2D = math.atan2(detection['center'][1]-vs.bottom_camera.width/2,detection['center'][0]-vs.bottom_camera.height/2) #angle in radians with x y at center of image frame
    #         mc.mov_control.go_angle(angle_2D,0.25)
    #         rospy.sleep(0.2)
    #         detection=mc.mov_control.await_completion_detection(detection['label'])
    #         mc.mov_control.stop()
    #         rospy.sleep(0.2)
    #         return self.adjustment(detection)


 

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

