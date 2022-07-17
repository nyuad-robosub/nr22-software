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
import progress_tracker as pt
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
        
class bouy(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self):
        pass
    def execute(self):  
        mc.mov_control.go_straight(5)

        #fetch side chosen in pass_gate state
        side = pt.pr_track.progress['pass_gate']['chosen_detection']
        
        #move till you detect any one of them
        detections=mc.mov_control.await_completion_detection(["image_tommygun","image_badge"])

        mc.mov_control.stop()
        if(len(detection>0)):
            #detected the object
            if(side=="image_bootlegger"):    
                detection=mc.mov_control.await_completion_detection(["image_tommygun"])
            else: # image_gman
                detection=mc.mov_control.await_completion_detection(["image_badge"])

            #get pose of object of interest
            pose_obji = vs.front_camera.estimate_pose_svd(detection[0]['center'],detection[0]['size'])
            pose_objo = vs.front_camera.estimate_pose_svd(detection[1]['center'],detection[1]['size'])

            #align camera with detection
            position_i=[pose_obji.position.x,pose_obji.position.y,pose_obji.position.z]
            position_o=[pose_objo.position.x,pose_objo.position.y,pose_objo.position.z]

            focus_point=list(position_i)
            #align with bouys center
            center=(position_i+position_o)/2

            # L=d/(2*math.tan(oakd.fov))
            L=vs.front_camera.height/(4*math.tan(vs.front_camera.Vfov))

            mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(center,[L,0,0],))

            
            
            
            
            
            # if(detections[0]['center'][0]<detections[1]['center']): #image on the left of frame y axis is on our left
            #     position_data[1]-=0.762
            # else:
            #     position_data[1]+=0.762
            # position_data[2]-= 0.6 #translate z under
            # mc.mov_control.set_goal_point(position_data) #go under the object
        



            return "outcome1"

        return "outcome2"