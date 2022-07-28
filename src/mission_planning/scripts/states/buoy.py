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
import actions_utils as au
import cv2
from calc_utils import pose_to_np, pose_get_yaw
from transforms3d import euler
from geometry_msgs.msg import PoseStamped
class buoy(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self,bootlegger_label,gman_label):#, label_1, label_2):
        self.bootlegger_label=bootlegger_label #"image_tommygun"
        self.gman_label=gman_label #"image_badge"
        self.pose_pub = rospy.Publisher('/POSE_DETECTION_BUOY', PoseStamped, queue_size=5, latch=True)
    def execute(self,userdata):  
        
        trans=(mc.mov_control.get_tf()).transform.translation
        mc.mov_control.update_tf()
        rospy.sleep(1)
        outcome, detection_dict = au.forward_search(
                    mc.mov_control,
                    geometry_msgs.msg.Point(trans.x, trans.y, trans.z - mc.mov_control.sonar_ping + 1.3),
                    100,
                    mc.mov_control.euler[2],
                    120,
                    vs.front_camera,
                    [self.bootlegger_label, self.gman_label],
                    2,
                    0.05,
                    0.005
                )
        print(outcome)
        if(outcome!="detected"):
            return "outcome2"
        
        #fetch side chosen in pass_gate state
        side = pt.pr_track.progress['pass_gate']['chosen_detection']
        
        #move till you detect any one of them
        detections=vs.front_camera.get_detection([self.bootlegger_label,self.gman_label])
        
        mc.mov_control.stop()
        if(len(detections)>1):
            mc.mov_control.update_tf()

            #detected the object
            if(side=="image_bootlegger"):    
                detection_i=vs.front_camera.get_detection([self.bootlegger_label])
                detection_o=vs.front_camera.get_detection([self.gman_label])
            else: # image_gman
                detection_i=vs.front_camera.get_detection([self.gman_label])
                detection_o=vs.front_camera.get_detection([self.bootlegger_label])
            if(len(detection_i)==0 or len(detection_o)==0):
                return "outcome2"
            
            detection_i = detection_i[0]
            detection_o = detection_o[0]

            #get pose of object of interest
            ps_obji = vs.front_camera.get_ps(detection_i)
            ps_objo = vs.front_camera.get_ps(detection_o) 

            L=vs.front_camera.get_min_approach_dist(0.7)

            if(ps_obji==None and ps_objo==None):
                return "outcome2"
            elif(ps_obji!=None and ps_objo==None):
                # align pose
                temp_position=pose_to_np(ps_obji.pose)
                yaw = pose_get_yaw(ps_obji.pose)
                mc.mov_control.set_focus_point(mc.mov_control.translate_axis_xyz(temp_position,[100,0,0],yaw))
                mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(temp_position,[-L,0,0],yaw))
                return "outcome2"
            elif(ps_obji==None and ps_objo!=None):
                temp_position=pose_to_np(ps_objo.pose)
                yaw = pose_get_yaw(ps_objo.pose)
                mc.mov_control.set_focus_point(mc.mov_control.translate_axis_xyz(temp_position,[100,0,0],yaw))
                mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(temp_position,[-L,0,0],yaw))
                return "outcome2"
            
            pose_obji=ps_obji.pose
            pose_objo=ps_objo.pose
            self.pose_pub.publish(ps_obji)
            rospy.sleep(5)

            #align camera with detection
            position_i=pose_to_np(pose_obji)
            position_o=pose_to_np(pose_objo)

            #align with bouys center
            center=(position_i+position_o)/2
            #focus_point=list(position_i)

            #select which detectio nclosest to center and make it detection of interest
            yaw = pose_get_yaw(pose_obji) #euler.quat2euler([pose_obji.orientation.w, pose_obji.orientation.x, pose_obji.orientation.y, pose_obji.orientation.z], 'sxyz')

            #put center of gate in view
            mc.mov_control.set_focus_point(mc.mov_control.translate_axis_xyz(center,[100,0,0],yaw))
            rospy.sleep(1)
            mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(center,[-L,0,0],yaw))
            mc.mov_control.await_completion()

            #images are 1.2 meters in height, so to hit top go around 0.4-5 meters
            mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_i,[0,0,0.4],yaw))
            mc.mov_control.await_completion()
            mc.mov_control.set_focus_point()

            #go a bit forward after hitting
            mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_i,[0.3,0,0.4],yaw))
            mc.mov_control.await_completion()
            
            return "outcome1"

        return "outcome2"