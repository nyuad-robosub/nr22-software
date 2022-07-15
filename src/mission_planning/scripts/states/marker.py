from operator import truediv
from turtle import position

import rospy
import threading
import smach
import os
from std_msgs.msg import Bool
# import smach_controller_simulation
import movement_controller as mc
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
        
class marker(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self,marker_label):
        self.marker_label=marker_label
        self.zigzag_threshold=0.7
    def execute(self, userdata):  
        #call search which returns once any bbox is detected
        detection=self.search(self.zigzag_threshold,0.7,self.marker_label,"center")
    
        #next begin the adjustment function
        
         
        pass
        

    def search(self,threshold, straight_amount,detection_label, mode : str): #recursive function to go in zig zags till gate detected
        #TODO MAKE STRAIGHT AMOUNT DETERMINED FROM OUR DEPTH TO SWEEP BOTTOM CAMERA FRAME
        detection = vs.viso_control.get_detection([detection_label])
        if(len(detection)>0):
            #if marker detected return 
            #marker detected
            mc.mov_control.stop() #is this redundant?
            return detection
        elif(mode=="center"): #starts at center of zig zag
            #GO LEFT
            mc.mov_control.go_left(threshold)
            mc.mov_control.await_completion()
            return self.search(threshold,straight_amount,detection_label,"left")
        elif(mode=="left"):
            #GO LEFT
            mc.mov_control.go_left(threshold*2)
            det=mc.mov_control.await_completion_detection(detection_label)
            if(det==None):
                return self.search(threshold,straight_amount,detection_label,"right")
            else:
                return det
        elif(mode=="right"):
            #GO right
            mc.mov_control.go_left(-threshold*2)
            det=mc.mov_control.await_completion_detection(detection_label)
            if(det==None):
                return self.search(threshold,straight_amount,detection_label,"straight")
            else:
                return det
        else:#(mode=="straight")
            mc.mov_control.go_straight(straight_amount)
            det=mc.mov_control.await_completion_detection(detection_label)
            if(det==None):
                return self.search(threshold,straight_amount,detection_label,"left")
            else:
                return det 
    
    def adjustment(self,detection,center): #adjusts center of camera frame to center of marker through recursive function
        #if the detection bbox is close to center STOP

        if(np.allclose(detection['center'],center,0.05)):
            pass

        #else get bbox angle and use trigonometry to determine translation amount return adjustment(self,detection)


