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
class pass_gate_qual(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self,label_1,label_2):
        self.label_1=label_1 # "image_bootlegger"
        self.label_2=label_2 # "gman_image"
        self.pose_pub = rospy.Publisher('/POSE_DETECTION', PoseStamped, queue_size=5, latch=True)

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
        mc.mov_control.update_tf()
        rospy.sleep(1)

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
                    0.25
                )
        if(outcome!="detected"):
            return "outcome2"

        print("GATE DETECTED")
        gate_detection = detection_dict['qual_gate'][0]
        pose_gate = self.getGateOrientation(gate_detection)
        if(pose_gate==None):
            return "outcome2" #map outcome 3 to old tactic using marker poses solely

        position_gate=pose_to_np(pose_gate)
        yaw_gate = pose_get_yaw(pose_gate)

        #align pose with gate, used to get better view on image labels
        clearance_d=vs.front_camera.get_min_approach_dist(0.6)

        mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_gate,[-clearance_d,0,0],yaw_gate))
        print(position_gate)
        print(trans)
        rospy.sleep(10)
        print("SET GOAL POINTS")
        mc.mov_control.set_focus_point(mc.mov_control.translate_axis_xyz(position_gate,[1000,0,0],yaw_gate))

        mc.mov_control.await_completion()
        mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_gate,[0,0,0],yaw_gate))

        mc.mov_control.await_completion()
        mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_gate,[1,0,0],yaw_gate))
        mc.mov_control.await_completion()

        return "outcome1"


        