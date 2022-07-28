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
class pass_gate(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self,label_1,label_2):
        self.label_1=label_1 # "image_bootlegger"
        self.label_2=label_2 # "gman_image"
        self.pose_pub = rospy.Publisher('/POSE_DETECTION', PoseStamped, queue_size=5, latch=True)
    def save(self,detection): #saving data
        pt.pr_track.progress['pass_gate']['done']=True #better to save automatically based off state machine?
        pt.pr_track.progress['pass_gate']['chosen_detection']=detection['label']
        print("CHOSE")
        print(detection['label'])
    # def get_closest_detection(self, detections):
    #     rov_position=mc.mov_control.get_np_position()
    #     for det in detections:
    #         ps=vs.front_camera.get_ps(det, 0.25, 0.05)
    #         if(ps!=None):
    #             dist = np.linalg.norm(np.array(pose_to_np(ps.pose))-rov_position)
    #             if(dist<=5):
    #                 self.ps_arr.append(ps)
    #                 self.dist_arr.append(dist)
    #                 self.det_arr.append(det)
    #     size = len(self.ps_arr)
    #     max_index=0
    #     max_value=0
    #     if(size==0):
    #         return "outcome2"
    #     else:
    #         for index, x in enumerate(self.ps_arr):
    #             if(self.dist_arr[index]>=max_value):
    #                 max_value=self.dist_arr[index]
    #                 max_index=index
    #     self.chosen_ps = self.ps_arr[max_index]
    #     self.chosen_detection=self.det_arr[max_index]

    def getGateOrientation(self):

        frame = vs.front_camera.get_cv_img()

        # lower bound and upper bound for orange color
        lower_bound = np.array([0,0,0])
        upper_bound = np.array([77,115,255])

        #find color within the boundaries
        mask = cv2.inRange(frame, lower_bound, upper_bound)
        
        #get all non zero values
        coord = np.array(cv2.findNonZero(mask).reshape(-1,2)).astype(int)

        ps = vs.front_camera.estimate_gate_pose(coord)
        if(ps!=None):
            self.pose_pub.publish(ps)
            return ps.pose


        
    def execute(self, userdata): 
        #Go straight till see gate get pose and memorize
        #GO straight till see at least one poster, align pose with gate relative to poster position
        #Go to goal point under poster
        #Go behind gate using gate pose from earlier

        print("GOING STRAIGHT") 
        rospy.sleep(1)
        trans=(mc.mov_control.get_tf()).transform.translation
        outcome, detection_dict = au.forward_search(
                    mc.mov_control,
                    geometry_msgs.msg.Point(trans.x, trans.y, trans.z),
                    10,
                    mc.mov_control.euler[2],
                    120,
                    vs.front_camera,
                    ['qual_gate'],
                    3,
                    0.05,
                    0.005
                )
        
        if(outcome!="detected"):
            return "outcome2"
        
        #check if you detect them BOTH, if you do, estimate gate pose, stop, and set map points
        pose_gate = self.getGateOrientation()
        
        rov_position=mc.mov_control.get_np_position()
        det_arr=[]
        ps_arr=[]
        dist_arr=[]
        for det in [detection_dict[self.label_1][0],detection_dict[self.label_2][0]]:
            ps=vs.front_camera.get_ps(det, 0.25, 0.05)
            if(ps!=None):
                dist = np.linalg.norm(np.array(pose_to_np(ps.pose))-rov_position)
                if(dist<=5):
                    ps_arr.append(ps)
                    dist_arr.append(dist)
                    det_arr.append(det)
        size = len(ps_arr)
        max_index=0
        max_value=0
        if(size==0):
            return "outcome2"
        else:
            for index, x in enumerate(ps_arr):
                if(dist_arr[index]>=max_value):
                    max_value=dist_arr[index]
                    max_index=index
        chosen_pose = ps_arr[max_index].pose
        chosen_detection=det_arr[max_index]

        position_gate=pose_to_np(pose_gate)
        position_chosen=pose_to_np(chosen_pose)

        yaw_gate = pose_get_yaw(pose_gate)
        yaw_chosen = pose_get_yaw(position_chosen)

        clearance_d=vs.front_camera.get_min_approach_dist(1.5)

        mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_gate,[-clearance_d*1.5,0,0],yaw_gate))
        mc.mov_control.set_focus_point(mc.mov_control.translate_axis_xyz(position_gate,[1000,0,0],yaw_gate))
        
        mc.mov_control.await_completion()

        mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_chosen,[0,0,-0.8],yaw_chosen))
        mc.mov_control.set_focus_point(mc.mov_control.translate_axis_xyz(position_chosen,[0,0,0],yaw_chosen))

        mc.mov_control.await_completion()
        mc.mov_control.set_focus_point()

        self.save(chosen_detection)
        return "outcome1"


        