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
        self.adjustment(detection)
        

    def search(self,threshold, straight_amount,detection_label): #recursive function to go in zig zags till gate detected
        #TODO MAKE STRAIGHT AMOUNT DETERMINED FROM OUR DEPTH TO SWEEP BOTTOM CAMERA FRAME
        detection = vs.viso_control.get_detection([detection_label])
        if(len(detection)==0):
            Points=mc.mov_control.generate_zig_zag(threshold)
            for p in Points:
                mc.mov_control.set_goal_point(p)
                rospy.sleep(0.1)
            detection=mc.mov_control.await_completion_detection(detection_label)

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
    
    def adjustment(self,detection): #adjusts center of camera frame to center of marker through recursive function
        #input: detection()
        #if the detection bbox is close to center STOP

        if(np.allclose(detection['center'],vs.viso_controller.OAK_1.get_center_coord(),0.05)):
            mc.mov_control.stop()
            return True
        else:
            angle_2D = math.atan2(detection[0]['center'][1]-vs.viso_control.OAK_1.width/2,detection[0]['center'][0]-vs.viso_control.OAK_1.height/2) #angle in radians with x y at center of image frame
            mc.mov_control.go_angle(angle_2D,0.25)
            rospy.sleep(0.2)
            detection=mc.mov_control.await_completion_detection(detection['label'])
            mc.mov_control.stop()
            rospy.sleep(0.2)
            return self.adjustment(detection)


        #else get bbox angle and use trigonometry to determine translation amount return adjustment(self,detection)


