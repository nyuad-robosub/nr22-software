from operator import truediv

import numpy
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
        
class pass_gate(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self,label_1,label_2):
        self.label_1=label_1
        self.label_2=label_2
    def execute(self, userdata):      
        vs.viso_control.add_detection_of_interest(self.label_1)
        vs.viso_control.add_detection_of_interest(self.label_2)
        x_center=320
        y_center=200
        mc.mov_control.go_straight(5)
        detect=[[BoundingBox2D()],[BoundingBox2D()]]
        while(mc.mov_control.get_running_confirmation()):
            #check if you detect them BOTH, if you do, estimate gate pose, stop, and set map points
            temp_det1 = vs.viso_control.get_detection(self.label_1)
            temp_det2 = vs.viso_control.get_detection(self.label_2)
            
            if(temp_det1!=None and temp_det2!=None):
                pass
            elif(temp_det1!=None):
                detect[1][1]=temp_det1.bbox
            elif(temp_det2!=None):
                detect[1][1]=temp_det2.bbox


        mc.mov_control.stop()

        