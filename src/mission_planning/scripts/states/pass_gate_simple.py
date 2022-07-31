from operator import truediv
from turtle import position

from chardet import detect

import rospy
import threading
import smach
import os
from std_msgs.msg import Bool
# import smach_controller_simulation
import movement_controller as mc
import viso_controller as vs
import progress_tracker as pt
import geometry_msgs.msg
import tf2_geometry_msgs
import math
import vision_msgs
from geometry_msgs.msg import Pose, PoseStamped
import actions_utils as au

from vision_msgs.msg import Detection2DArray, BoundingBox2D
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros

import numpy as np
from transforms3d import euler
from calc_utils import is_approx_equal, pose_to_np, pose_get_yaw
import cv2
class pass_gate_simple(smach.State):
    _outcomes=['outcome1','outcome2','outcome3']
    _input_keys=[],
    _output_keys=[]
    def __init__(self,label_1,label_2, marker_label):
        self.label_1=label_1 # "image_bootlegger"
        self.label_2=label_2 # "gman_image"
        self.marker_label=marker_label
        self.pose_pub = rospy.Publisher('/POSE_DETECTION', PoseStamped, queue_size=5, latch=True)
    def save(self,detection): #saving data
        pt.pr_track.progress['pass_gate']['done']=True #better to save automatically based off state machine?
        pt.pr_track.progress['pass_gate']['chosen_detection']=detection['label']
        print("CHOSE")
        print(detection['label'])
        
    def execute(self, userdata): 
        #get pose of gate, align with gate, go forward till you see amrker

        print("GOING STRAIGHT") 
        rospy.sleep(1)

        mc.mov_control.go_straight(6)

        last_detection = None
        marker_detection = None

        while(mc.mov_control.get_running_confirmation()):
            detections = vs.front_camera.get_detection_t([self.label_1,self.label_2], 0.05)
            marker_det = vs.bottom_camera.get_detection_t([self.marker_label], 0.05)

            if(len(detections) > 0):
                last_detection = detections[0]

            if(len(marker_det) > 0):
                marker_detection = marker_det[0]
                break
            
        if(marker_detection==None):
            return "outcome2"
        
        if(last_detection==None):
            last_detection = self.label_1

        pt.pr_track.progress['pass_gate']['chosen_detection'] = last_detection
        return "outcome1"


        