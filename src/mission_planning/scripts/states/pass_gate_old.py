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
    def __pose_to_np(self,pose_obj):
        position_data=np.array([pose_obj.position.x,pose_obj.position.y,pose_obj.position.z])
        return position_data
    def __is_top_approx_equal(self,detection1, detection2):
        return is_approx_equal(detection1['center'][1]+detection1['size'][1]/2,detection2['center'][1]+detection2['size'][1]/2,0.3)
    def __get_ps(self, detection, timeout = 0.5, delay = 0.1, counter = 0):
        if(counter >= timeout / delay):
            return None
        else:
            rospy.sleep(delay)
            counter += 1
            ps=vs.front_camera.estimate_pose_svd(detection['center'],detection['size'])
            detection_arr = vs.front_camera.get_detection([detection['label']])
            if(ps!=None):
                print("RETURNING PS")
                return ps
            elif(len(detection_arr)==0):
                return self.__get_ps(detection, timeout, delay, counter)
            else:
                return self.__get_ps(detection_arr[0], timeout, delay, counter)
            
    #         if(len(detections)>1):
    #             ps2=vs.front_camera.estimate_pose_svd(detections[1]['center'],detections[1]['size'])

    #         return self.__get_ps()
        # counter = 0
        # # Find current location
        # while counter < timeout / delay:     
        #     ps=vs.front_camera.estimate_pose_svd(detections[0]['center'],detections[0]['size'])
        #     if(ps!=None):
        #         return ps
        #     else:
        #         detections = vs.front_camera.get_detection([self.label_1,self.label_2])
        #         print("Cannot find points in pointcloud")
        #         rospy.sleep(delay)
        #         counter += 1
        # return None
    def execute(self, userdata): 
        print("GOING STRAIGHT") 
        rospy.sleep(1)
        mc.mov_control.go_straight(3)
        while(mc.mov_control.get_running_confirmation()):
            #check if you detect them BOTH, if you do, estimate gate pose, stop, and set map points
            rospy.sleep(0.05)
            detections=[]
            temp_detections = vs.front_camera.get_detection([self.label_1,self.label_2],0.15)
            gate_detection = vs.front_camera.get_detection(["qual_gate"],0.5)
            lend=len(temp_detections)

            # #To stop detection flickers
            # if(lend>0):
            #     count=0
            #     while(count<3):
            #         rospy.sleep(0.1)
            #         temp_detections = vs.front_camera.get_detection([self.label_1,self.label_2])
            #         lend=len(temp_detections)
            #         if(lend==0):
            #             break
            #         count+=1     
        
            if lend>0: #and len(gate_detection)>0: #detecting gate and object in one frame is hard!
                print("Detected something of interest!")
                # lend=0
                # for det in temp_detections:
                #     #To stop mistaking torpedo detection for gate g man
                #     if det['center'][1]>vs.front_camera.height/1.5 or self.__is_top_approx_equal(det, gate_detection[0]):
                #         print("DETECTED AN OBJECT THATS PLACED CORRECTLY")
                #         detections.append(det)
                #         lend+=1
                
                detection=None
                mc.mov_control.stop() #stop the machine and now begin alignment
                rospy.sleep(0.1)
                mc.mov_control.update_tf()
                ps=self.__get_ps(detections[0], 0.3, 0.05)
                if(ps==None):
                    continue
                print("DETECTED GATE+OBJECTS!")
                print("PS IS DETECTED")
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
                rov_position=mc.mov_control.get_np_position()
                rov_orientation=mc.mov_control.get_np_orientation()
                pose_obj = ps.pose

                #if seeing both objects
                if(lend>1):
                    ps2=vs.front_camera.estimate_pose_svd(detections[1]['center'],detections[1]['size'])
                    if(ps2!=None):
                        #now choose closest by distance
                        dist_1 = np.linalg.norm(np.array(self.__pose_to_np(ps.pose))-rov_position)
                        dist_2 = np.linalg.norm(np.array(self.__pose_to_np(ps2.pose))-rov_position)
                        if(dist_1>=dist_2):
                            detection=detections[1]
                            pose_obj=ps2.pose
                        else:
                            detection=detections[0]
                            pose_obj=ps.pose
                    else:
                        detection=detections[0]
                        pose_obj=ps.pose
                else:
                    detection=detections[0]
                #figure out which orientation is x,y with z pointing upwards
                #if angle_difference in yaw <90 front of camera x is front of object
                #if greater or equal then fron camera_x is opposite the x of the plane so do translations accordingly

                #select which detectio nclosest to center and make it detection of interest
                roll, pitch, yaw = euler.quat2euler([pose_obj.orientation.w, pose_obj.orientation.x, pose_obj.orientation.y, pose_obj.orientation.z], 'sxyz')

                #get position data
                position_data=self.__pose_to_np(pose_obj)#np.array([pose_obj.position.x,pose_obj.position.y,pose_obj.position.z])#[pose_obj.position.x,pose_obj.position.y,pose_obj.position.z]
                print(position_data)

                #translate z under
                position_data=mc.mov_control.translate_axis_xyz(position_data,[0,0,-0.8],yaw) 

                mc.mov_control.set_goal_point(position_data) #go under the object
                print("GOING UNDER OBJECT")
                #wait for that to finish
                mc.mov_control.await_completion()

                y_delta=0
                if(detection['center'][0]<gate_detection[0]['center']): #image on the left of frame y axis is on our left
                    #translate 0.762 meter opposite objects along y axis 
                    y_delta=-0.762
                else:
                    #translate 0.762 meter along y axis 
                    y_delta=0.762
                
                #translate 1 meter opposite objects +x axis  and 0.76 opposite or along +y axis
                position_data=mc.mov_control.translate_axis_xyz(position_data,[-1,y_delta,0],yaw)
                mc.mov_control.set_goal_point(position_data)
                
                position_data=mc.mov_control.translate_axis_xyz(position_data,[100,0,0],yaw)
                mc.mov_control.set_focus_point(position_data)
                
                mc.mov_control.await_completion()

                mc.mov_control.stop()

                self.save(detection)
                return "outcome1"
            else:
                continue
        return "outcome2"


        