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
from geometry_msgs.msg import Pose, PoseStamped
import actions_utils as au

from vision_msgs.msg import Detection2DArray, BoundingBox2D
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros

import numpy as np
from transforms3d import euler
from calc_utils import is_approx_equal, pose_to_np
class pass_gate_label(smach.State):
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
    def __is_top_approx_equal(self,detection1, detection2):
        return is_approx_equal(detection1['center'][1]+detection1['size'][1]/2,detection2['center'][1]+detection2['size'][1]/2,0.3)
            
    def execute(self, userdata): 
        print("GOING STRAIGHT") 
        rospy.sleep(1)
        trans=(mc.mov_control.get_tf()).transform.translation
        
        outcome, detection_dict = au.forward_search(
                    mc.mov_control,
                    geometry_msgs.msg.Point(trans.x, trans.y, trans.z),
                    10,
                    mc.mov_control.euler[2], # this euler angle is incorrect!
                    120,
                    vs.front_camera,
                    [self.label_1, self.label_2],
                    2,
                    0.05,
                    0.005
                )
        if(outcome!="detected"):
            return "outcome2"
        
        #check if you detect them BOTH, if you do, estimate gate pose, stop, and set map points
        rospy.sleep(0.05)
        detections=[]
        temp_detections = vs.front_camera.get_detection([self.label_1,self.label_2],0.15)
        # gate_detection = vs.front_camera.get_detection(["qual_gate"],0.5)
        lend=len(temp_detections)
        if lend>0: 
            print("Detected something of interest!")
            detections=temp_detections
            detection=None
            mc.mov_control.stop() #stop the machine and now begin alignment
            rospy.sleep(0.1)
            mc.mov_control.update_tf()

            rov_position=mc.mov_control.get_np_position()
            rov_orientation=mc.mov_control.get_np_orientation()
            det_arr=[]
            ps_arr=[]
            dist_arr=[]
            for det in detections:
                ps=vs.front_camera.get_ps(det, 0.25, 0.05)
                if(ps!=None):
                    print(np.array(pose_to_np(ps.pose)))
                    print(rov_position)
                    print(np.linalg.norm(np.array(pose_to_np(ps.pose))-rov_position))
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
            ps = ps_arr[max_index]
            detection=det_arr[max_index]
            
            pose_obj = ps.pose
            self.pose_pub.publish(ps)
            rospy.sleep(5)

            print("DETECTED AN OBJECT!")
            print("PS IS DETECTED")
            
            #select which detectio nclosest to center and make it detection of interest
            roll, pitch, yaw = euler.quat2euler([pose_obj.orientation.w, pose_obj.orientation.x, pose_obj.orientation.y, pose_obj.orientation.z], 'sxyz')

            #get position data
            position_data=pose_to_np(pose_obj)#np.array([pose_obj.position.x,pose_obj.position.y,pose_obj.position.z])#[pose_obj.position.x,pose_obj.position.y,pose_obj.position.z]
            print(position_data)
            
           
            #align ORIENTATION with DETECTION
            mc.mov_control.stop()
            mc.mov_control.set_focus_point(mc.mov_control.translate_axis_xyz(position_data,[0,0,0],yaw)) #something strange with focus points
            mc.mov_control.await_completion()

            print("FOCUS")

            #rospy.sleep(5)
            
            #LOOK AT BOTH 
            print("Look at both")
            clearance_d=vs.front_camera.get_min_approach_dist(1.5)
            print("Clearance distance is")
            print(clearance_d)
            print(position_data)
            mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_data,[-clearance_d*1.5, 0 ,-0.4],yaw))
            print(position_data)
            mc.mov_control.await_completion()

            # ps=vs.front_camera.get_ps(detection, 5, 0.2)
            # if(ps==None):
            #     return "outcome2"
            # pose_obj = ps.pose #to get more accurate pose
            # position_data=pose_to_np(pose_obj)

            #GO UNDER
            mc.mov_control.set_focus_point()
            mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_data,[0,0,-0.4],yaw))
            mc.mov_control.await_completion()

            #GO BEHIND
            mc.mov_control.set_goal_point(mc.mov_control.translate_axis_xyz(position_data,[0.3,0,-0.4],yaw))
            mc.mov_control.await_completion()
        
            mc.mov_control.set_rotation(rov_orientation[0],rov_orientation[1],yaw)
            mc.mov_control.set_focus_point()
            mc.mov_control.await_completion()
            # #translate z under
            # position_data=mc.mov_control.translate_axis_xyz(position_data,[0,0,-0.8],yaw) 

            # mc.mov_control.set_goal_point(position_data) #go under the object
            # print("GOING UNDER OBJECT")
            # #wait for that to finish
            # mc.mov_control.await_completion()

            # y_delta=0
            # if(detection['center'][0]<gate_detection[0]['center']): #image on the left of frame y axis is on our left
            #     #translate 0.762 meter opposite objects along y axis 
            #     y_delta=-0.762
            # else:
            #     #translate 0.762 meter along y axis 
            #     y_delta=0.762
            
            # #translate 1 meter opposite objects +x axis  and 0.76 opposite or along +y axis
            # position_data=mc.mov_control.translate_axis_xyz(position_data,[-1,y_delta,0],yaw)
            # mc.mov_control.set_goal_point(position_data)
            
            # position_data=mc.mov_control.translate_axis_xyz(position_data,[100,0,0],yaw)
            # mc.mov_control.set_focus_point(position_data)
            
            # mc.mov_control.await_completion()

            # mc.mov_control.stop()

            self.save(detection)
            return "outcome1"
        else:    
            return "outcome2"


        