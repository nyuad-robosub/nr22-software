from cmath import pi
from operator import truediv

from numpy import angle
import rospy
import threading
import smach
import os
from std_msgs.msg import Bool
# import smach_controller_simulation
import movement_controller as mc
import viso_controller as vs
import geometry_msgs.msg
import math
from transforms3d import euler
import vision_msgs
from calc_utils import is_approx_equal
import actions_utils as au
import threading

class coin_flip(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self,gate_label):   
        self.gate_label=gate_label

    def rotate_cc(self): #rotate by 179 degrees
        mc.mov_control.update_tf()

        # Rotate from initial orientation
        rotation = mc.mov_control.trans.transform.rotation
        roll, pitch, yaw = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')
        yaw=math.degrees(yaw)
        rot_total = yaw + 179

        print("ROTATING BY::")
        print(math.degrees(mc.mov_control.to_euler(rot_total)))
        mc.mov_control.set_rotation(roll,pitch,mc.mov_control.to_euler(rot_total))
        
    def execute(self, userdata):     
        rospy.sleep(3)

        # Rotate from initial orientation
        mc.mov_control.update_tf()
        mc.mov_control.arm()

        self.rotate_cc()

        # Get dictionary of detections
        detections_dict = {}
        detections_dict[self.gate_label] = []  

        # Get end time
        end_time = rospy.get_time() + 60
        
        # Get a detection of the object
        outcome = au.search_waiter(mc.mov_control, vs.front_camera, [self.gate_label], 5, 0.3, 
                                0.1, detections_dict, end_time, lambda: True)
        print("Search ended")
        if outcome != "detected": 
            print("NOT DETECTED")
            return "outcome2"

        if(self.recursive()):
            mc.mov_control.set_height(-0.8)
            mc.mov_control.await_completion()
            return 'outcome1'
        else:
            print("Couldnt align gate")
            return 'outcome2'

    def recursive(self):
        mc.mov_control.update_tf()
        rotation = mc.mov_control.trans.transform.rotation
        roll, pitch, yaw = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')
        gate_detection = vs.front_camera.get_detection_t(self.gate_label,0.5)

        if(len(gate_detection)==0):
            print("Gate couldnt be detected for the first time")
            return False

        angle_x = self.get_closest_side(gate_detection)
        mc.mov_control.rotate_ccw(-angle_x)

        gate_detection = vs.front_camera.get_detection_t(self.gate_label,2)
        if(len(gate_detection)==0):
            print("Gate couldnt be detected a second time")
            return False

        # Rotate to center of the gate
        angle_x_2 = self.get_closest_side(gate_detection)
        mc.mov_control.rotate_ccw(-angle_x)
        #get angle between center of bbox and image frame

        if(is_approx_equal(angle_x,angle_x_2,0.15)):
            return True
        else:
            return self.recursive()

    def get_closest_side (self, gate_detection):
        center = gate_detection[0]['center']
        size = gate_detection[0]['size']
        # Rotate to center of the gate
        if(center[0]<=vs.front_camera.get_center_coord()[0]):
            angle_x, angle_y = vs.front_camera.get_angles(center[0]+size[0]/4, 0) #get angle between center of bbox and image frame
        else:
            angle_x, angle_y = vs.front_camera.get_angles(center[0]-size[0]/4, 0) #get angle between center of bbox and image frame
        return angle_x
            

        