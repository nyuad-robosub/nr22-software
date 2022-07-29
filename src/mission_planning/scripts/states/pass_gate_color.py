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
class pass_gate_color(smach.State):
    _outcomes=['outcome1','outcome2','outcome3']
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

    def getGateOrientation(self, det):

        frame = vs.front_camera.get_cv_img()

        # lower bound and upper bound for orange color
        lower_bound = np.array([0,0,80])
        upper_bound = np.array([60,90,255])

        #find color within the boundaries
        mask = cv2.inRange(frame, lower_bound, upper_bound)
        # ll_vertex=(int(det.bbox.center.x-det.bbox.size_x/2),int(det.bbox.center.y-det.bbox.size_y/2))
        # rr_vertex=(int(det.bbox.center.x+det.bbox.size_x/2),int(det.bbox.center.y+det.bbox.size_y/2))
        
        # mask[0::ll_vertex]=0
        # mask[rr_vertex::]=0

        #get all non zero values
        coord = np.array(cv2.findNonZero(mask).reshape(-1,2)).astype(int)
        cv2.imwrite("/home/rami/nr22-software/src/gate_cropped.png",mask)
        ps = vs.front_camera.estimate_gate_pose(coord)
        if(ps!=None):
            self.pose_pub.publish(ps)
            return ps.pose
        return None


        
    def execute(self, userdata): 
        #get pose of gate, align with gate, go forward till you see amrker

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
                    2,
                    0.1,
                    0.3
                )
        
        if(outcome!="detected"):
            return "outcome2"
        print("GATE DETECTED")
        gate_detection = detection_dict['qual_gate'][0]
        pose_gate = self.getGateOrientation(gate_detection)
        if(pose_gate==None):
            return "outcome3" #map outcome 3 to old tactic using marker poses solely

        position_gate=pose_to_np(pose_gate)
        yaw_gate = pose_get_yaw(pose_gate)

        #align pose with gate, used to get better view on image labels
        clearance_d=vs.front_camera.get_min_approach_dist(0.6)

        print(position_gate)
        mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_gate,[-clearance_d,0,0],yaw_gate))
        mc.mov_control.set_focus_point(mc.mov_control.translate_axis_xyz(position_gate,[1000,0,0],yaw_gate))
        mc.mov_control.await_completion()
        rospy.sleep(5)

        trans=(mc.mov_control.get_tf()).transform.translation
        outcome, detection_dict = au.forward_search(
                    mc.mov_control,
                    geometry_msgs.msg.Point(trans.x, trans.y, -0.8),
                    clearance_d*0.8,
                    mc.mov_control.euler[2],
                    60,
                    vs.front_camera,
                    [self.label_1, self.label_2],
                    5,
                    0.1,
                    0.005
                ) 
        if(outcome!="detected"):
            #not working for some odd reason
            return "outcome1"
            # #go through gate side no matter what
            # mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_gate,[-clearance_d,0,0],yaw_gate))
            # mc.mov_control.await_completion()

            # #now just pick the right of the gate
            # mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_gate,[0,0.6,0],yaw_gate))
            # mc.mov_control.await_completion()
            # mc.mov_control.set_focus_point()

            # mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_gate,[0,0.6,0],yaw_gate))
            # mc.mov_control.await_completion()
            # mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_gate,[0.3,0.6,0],yaw_gate))
            # return "outcome2"

        #get pose of labelled images
        rov_position=mc.mov_control.get_np_position()
        det_arr=[]
        ps_arr=[]
        dist_arr=[]
        temp_detections = vs.front_camera.get_detection([self.label_1,self.label_2],0)
        for det in temp_detections:
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
        position_chosen=pose_to_np(chosen_pose)
        
        # #set focus point to our detection 
        # mc.mov_control.set_focus_point()

        #get to the object using the accurate rigid pose of the best fit plane of the gate
        mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_chosen,[0,0,-0.4],yaw_gate)) #use yaw gate!
        mc.mov_control.await_completion()
        mc.mov_control.set_focus_point()

        mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_chosen,[0.3,0,-0.4],yaw_gate))
        mc.mov_control.await_completion()

        self.save(chosen_detection)
        return "outcome1"


        