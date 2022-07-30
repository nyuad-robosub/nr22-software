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
        mc.mov_control.set_focus_point()

        #call search which returns once any bbox of marker is detected
        if(len(self.search(self.zigzag_threshold, self.marker_label))==0):
            return "outcome2"

        mc.mov_control.update_tf()

        trans = mc.mov_control.trans.transform.translation

        print("Beginning alignment")
        #begin alignment of bottom with marker
        outcome,value = au.bottom_aligning(
            mc.mov_control,
            # geometry_msgs.msg.Point(trans.x + 0.6, trans.y, trans.z),
            0.3,
            60,
            vs.bottom_camera,
            "marker",
            0.2,
            0.005
        )
        if(outcome!="succeeded"):
            return "outcome2"

        rospy.sleep(5)
        #get rotation of marker 
        rotation_angle=self.getMarkerOrientation()
        if(rotation_angle==None):
            return "outcome2"

        #sleep here for a bit to avoid trajectory completing after we get orientation(because they run in separate threads)
        rospy.sleep(5)   

        if(rotation_angle!=90):
            rotation_angle-=90
            print("Rotation angle:")
            print(rotation_angle)
            mc.mov_control.rotate_ccw(rotation_angle)
        
        mc.mov_control.await_completion()

        return "outcome1"
           #else get bbox angle and use trigonometry to determine translation amount return adjustment(self,detection)
           
    def getMarkerOrientation(self):

        cnt = []

        frame = vs.bottom_camera.get_cv_img()
                
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
        # lower bound and upper bound for orange color
        lower_bound = np.array([0,120,60])
        upper_bound = np.array([65,255,255])

        #create canvas
        canvas = np.zeros(frame.shape)
        canvas.fill(0)

        #find color within the boundaries
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        #define kernel size  
        kernel = np.ones((7,7),np.uint8)

        # remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # find contours from the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cluster = agglomerative_cluster(list(contours), 40)

        # get minAreaRect of contour with max area
        max_area = -1

        #loop through contours
        for i in range(len(cluster)):
            area = cv2.contourArea(cluster[i])
            if area>max_area:
                cnt = cluster[i]
                max_area = area

        if len(cnt) == 0:
            print("No Marker Detected")
            return None

        areas = [cv2.contourArea(c) for c in cluster]

        if (len(areas) !=0):
            max_index = np.argmax(areas)
            cnt=cluster[max_index]

        #assign minAreaRect to contour with max area
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        #center coordinates of min_rectangle also p1
        x = int(rect[0][0])
        y = int(rect[0][1])

        pts = cnt
        sz = len(pts)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i,0] = pts[i,0,0]
            data_pts[i,1] = pts[i,0,1]
        
        # Perform PCA analysis
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)

        # Store the center of the object
        cntr = (int(mean[0,0]), int(mean[0,1]))
        ## [pca]
        a = math.atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians

        p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
        p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])

        #Visualisation
        label = "  Rotation Angle: " + str(-int(np.rad2deg(a)) - 90) + " degrees"
        textbox = cv2.rectangle(frame, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
        cv2.putText(frame, label, (cntr[0], cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
        drawAxis(frame, cntr, p1, (255, 255, 0), 1)
        drawAxis(frame, cntr, p2, (0, 0, 255), 5)
        cv2.drawContours(frame,[box],0,(255, 0,0),2)
        cv2.circle(frame, (x,y), 10, (0,255,0), -5)

        rotation = 180 -int(np.rad2deg(a))

        if rotation >= 180:
            rotation=rotation-180

        return rotation


    def search(self,threshold, detection_label): #recursive function to go in zig zags till gate detected
        #TODO MAKE STRAIGHT AMOUNT DETERMINED FROM OUR DEPTH TO SWEEP BOTTOM CAMERA FRAME
        detection = vs.bottom_camera.get_detection_t(detection_label,0.5)
        if(len(detection)==0):
            Points=mc.mov_control.generate_zig_zag(threshold)
            mc.mov_control.set_goal_points(Points)
            print("Trajectory set")
            detection=mc.mov_control.await_completion_detection(detection_label, vs.bottom_camera)
            mc.mov_control.stop()
        return detection

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

def drawAxis(img, p_, q_, colour, scale):
    p = list(p_)
    q = list(q_)
    
    angle = math.atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
    hypotenuse = math.sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * math.cos(angle)
    q[1] = p[1] - scale * hypotenuse * math.sin(angle)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    # create the arrow hooks
    p[0] = q[0] + 9 * math.cos(angle + math.pi / 4)
    p[1] = q[1] + 9 * math.sin(angle + math.pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    p[0] = q[0] + 9 * math.cos(angle - math.pi / 4)
    p[1] = q[1] + 9 * math.sin(angle - math.pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
