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
from transforms3d import euler
        
class pass_gate(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self,label_1,label_2):
        self.label_1=label_1 # "image_bootlegger"
        self.label_2=label_2 # "image_gman"
    def save(self,detection): #saving data
        pt.pr_track.progress['pass_gate']['done']=True #better to save automatically based off state machine?
        pt.pr_track.progress['pass_gate']['chosen_detection']=detection[0]['label']

    def execute(self, userdata):      
        x_center=320
        y_center=200
        mc.mov_control.go_straight(5)
        while(mc.mov_control.get_running_confirmation()):#should this be by time? what if it stops
            #check if you detect them BOTH, if you do, estimate gate pose, stop, and set map points
            detections = vs.viso_control.get_detection([self.label_1],[self.label_2])
            gate_detection = vs.viso_control.get_detection(["gate"])
            if len(detections)>1 and len(gate_detection)>1:
                if detections[0]['score'] > 0.5:
                    mc.mov_control.stop() #stop the machine and now begin alignment

                    #get pose of object
                    pose_obj = vs.viso_control.estimate_pose_svd(detections[0]['center'],detections[0]['size'])
                    roll, pitch, yaw = euler.quat2euler([pose_obj.orientation.w, pose_obj.orientation.x, pose_obj.orientation.y, pose_obj.orientation.z], 'sxyz')
                    #get position data
                    position_data=[pose_obj.position.x,pose_obj.position.y,pose_obj.position.z]
                    
                    # #align camera with detection
                    # position_temp=list(position_data)
                    # position_data[0]+=

                    position_data[2]-= 0.6 #translate z under
                    mc.mov_control.set_goal_point(position_data) #go under the object

                    #wait for that to finish
                    mc.mov_control.await_completion()

                    #translate 1 meter opposite objects +x axis 
                    position_data[0]=position_data[0]-math.cos(yaw)*1
                    position_data[1]=position_data[1]-math.sin(yaw)*1

                    if(detections[0]['center'][0]<gate_detection[0]['center']): #image on the left of frame y axis is on our left
                        #translate 0.762 meter opposite objects along y axis 
                        position_data[1]=position_data[0]-math.cos(yaw)*0.762
                        position_data[0]=position_data[1]-math.sin(yaw)*0.762
                    else:
                        #translate 0.762 meter opposite objects along y axis 
                        position_data[1]=position_data[0]-math.cos(yaw)*0.762
                        position_data[0]=position_data[1]-math.sin(yaw)*0.762

                    mc.mov_control.set_goal_point(position_data)
                    
                    position_data[0]-=100
                    mc.mov_control.set_focus_point(position_data)
                    
                    mc.mov_control.await_completion()

                    mc.mov_control.stop()

                    self.save(pass_gate)
                    return "outcome1"
            else:
                continue
        return "outcome2"


        