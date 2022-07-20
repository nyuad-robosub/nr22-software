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
from calc_utils import is_approx_equal
class pass_gate(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self,label_1,label_2):
        self.label_1=label_1 # "image_bootlegger"
        self.label_2=label_2 # "image_gman"
    def save(self,detection): #saving data
        pt.pr_track.progress['pass_gate']['done']=True #better to save automatically based off state machine?
        pt.pr_track.progress['pass_gate']['chosen_detection']=detection['label']

    def __is_top_approx_equal(self,detection1, detection2):
        return is_approx_equal(detection1['center'][1]+detection1['size'][1]/2,detection2['center'][1]+detection2['size'][1]/2)

    def execute(self, userdata):  
        mc.mov_control.go_straight(3)
        while(mc.mov_control.get_running_confirmation()):#should this be by time? what if it stops
            #check if you detect them BOTH, if you do, estimate gate pose, stop, and set map points
            detections = vs.front_camera.get_detection([self.label_1,self.label_2],0.5)
            gate_detection = vs.front_camera.get_detection(["qual_gate"])
            lend=len(detections)

            # if(lend>0):
            #     count=0
            #     while(count<3):
            #         rospy.sleep(0.1)
            #         detections = vs.front_camera.get_detection([self.label_1,self.label_2])
            #         lend=len(detections)
            #         if(lend==0):
            #             break
            #         count+=1     

            if lend>0 and len(gate_detection)>0:
                for det in detections:
                    if self.__is_top_approx_equal(det, gate_detection[0]):
                        print("DETECTED AN OBJECT THATS PLACED CORRECTLY")
                        detections=[det]
                        lend=1
                        break
                if lend == 1:
                    print("DETECTED GATE+OBJECTS!")
                    ps=vs.front_camera.estimate_pose_svd(detections[0]['center'],detections[0]['size'])
                    # if(lend>1):
                    #     ps2=vs.front_camera.estimate_pose_svd(detections[1]['center'],detections[1]['size'])
                    #     while(ps==None and ps2==None):
                    #         ps=vs.front_camera.estimate_pose_svd(detections[0]['center'],detections[0]['size'])
                    #         if(lend>1):
                    #             ps2=vs.front_camera.estimate_pose_svd(detections[1]['center'],detections[1]['size'])
                    #         rospy.sleep(0.05)
                    # else:
                    #     while(ps==None):
                    #         ps=vs.front_camera.estimate_pose_svd(detections[0]['center'],detections[0]['size'])
                    #         if(lend>1):
                    #             ps2=vs.front_camera.estimate_pose_svd(detections[1]['center'],detections[1]['size'])
                    #         rospy.sleep(0.05)
                    if detections[0]['score'] > 0.5:
                        mc.mov_control.stop() #stop the machine and now begin alignment

                        detection=detections[0]

                        #get pose of object
                        # if(ps!=None):
                        pose_obj = ps.pose #(vs.front_camera.estimate_pose_svd(detections[0]['center'],detections[0]['size'])).pose
                        # else:
                        #     pose_obj = ps2.pose
                        #select which detectio nclosest to center and make it detection of interest
                        roll, pitch, yaw = euler.quat2euler([pose_obj.orientation.w, pose_obj.orientation.x, pose_obj.orientation.y, pose_obj.orientation.z], 'sxyz')

                        #get position data
                        position_data=[pose_obj.position.x,pose_obj.position.y,pose_obj.position.z]
                        print(position_data)

                        #translate z under
                        position_data=mc.mov_control.translate_axis_xyz(position_data,[0,0,-0.8],yaw) 

                        mc.mov_control.set_goal_point(position_data) #go under the object
                        print("GOING UNDER OBJECT")
                        #wait for that to finish
                        mc.mov_control.await_completion()

                        y_delta=0
                        if(detections[0]['center'][0]<gate_detection[0]['center']): #image on the left of frame y axis is on our left
                            #translate 0.762 meter opposite objects along y axis 
                            y_delta=-0.762
                        else:
                            #translate 0.762 meter along y axis 
                            y_delta=0.762
                        
                        #translate 1 meter opposite objects +x axis  and 0.76 opposite or along +y axis
                        position_data=mc.mov_control.translate_axis_xyz(position_data,[1,y_delta,0],yaw)
                        mc.mov_control.set_goal_point(position_data)
                        
                        position_data[0]-=100
                        mc.mov_control.set_focus_point(position_data)
                        
                        mc.mov_control.await_completion()

                        mc.mov_control.stop()

                        self.save(detection)
                        return "outcome1"
            else:
                continue
        return "outcome2"


        