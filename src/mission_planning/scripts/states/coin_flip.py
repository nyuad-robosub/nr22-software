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

class coin_flip(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self,gate_label):   
        self.gate_label=gate_label
    def execute(self, userdata):     
        rospy.sleep(3)

        # Rotate from initial orientation
        mc.mov_control.update_tf()
        mc.mov_control.arm()
        rotation = mc.mov_control.trans.transform.rotation
        detected = False
        detections = []
        angle_sign = 0
        center = (0.0, 0.0)
        size = (0.0, 0.0)
        angle_x, angle_y = 0.0, 0.0
        roll, pitch, yaw = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')
        for i in range(4):
            print("ROTATING yaw:")
            print(yaw)
            print(yaw+math.pi/2*(i+1))
            mc.mov_control.set_rotation(roll,pitch,yaw+math.pi/2*(i+1))
            # Wait a bit for control to start 
            rospy.sleep(0.1)
            while(mc.mov_control.get_running_confirmation()):
                if vs.front_camera.is_fetched:
                    detections = vs.front_camera.get_detection([self.gate_label],0.05)
                    # rospy.sleep(0.1) # delay already present in get_running_confirmation
                    if len(detections) > 0: 
                        # if(temp_detect!=None):
                        #     detect=temp_detect

                        # If area of bounding box too small (< 1/10 area of image): maybe not our gate
                        if detections[0]['size'][0] * detections[0]['size'][1] < vs.front_camera.height * vs.front_camera.width / 12:
                            print("Gate too small, ignoring...")
                            continue
                            
                        # If center of bounding box is already at image center: no need for more analysis
                        if detections[0]['center'][0] == vs.front_camera.get_center_coord()[0]:
                            print("DETECTED AND EXISTS")
                            mc.mov_control.stop()
                            rospy.sleep(1)

                            center = detections[0]['center']
                            size = detections[0]['size']
                            detected = True
                            break

                        # First time seeing the gate: maybe at edge
                        angle_x, angle_y = vs.front_camera.get_angles(detections[0]['center'][0], detections[0]['center'][1])
                        # print(detections[0]['size'][0] * detections[0]['size'][1])
                        # print("ANGLE_X")
                        # print(angle_x)
                        if angle_sign == 0: #only set when first starting
                            print("Center of gate found...")
                            angle_sign = int(angle_x / abs(angle_x))

                        # if (is_approx_equal(detections[0]['center'][0], x_center, 0.4)): # or is_approx_equal(detect.bbox.center.y,y_center)):
                        # Otherwise see if the gate angle sign changed (crossed the center) or near the center
                        elif is_approx_equal(angle_x, 0) or int(angle_x / abs(angle_x)) == -angle_sign: #
                            print("DETECTED AND EXISTS")
                            mc.mov_control.stop()
                            rospy.sleep(1)
                            angle_x, angle_y = vs.front_camera.get_angles(detections[0]['center'][0], detections[0]['center'][1])
                            
                            center = detections[0]['center']
                            size = detections[0]['size']

                            detected = True
                            break
                
            if (detected):
                break     
            
            mc.mov_control.update_tf()

        # vs.viso_control.reset_detection_of_interest()
        if(detected):
            mc.mov_control.update_tf()
            altitude= mc.mov_control.trans.transform.translation.z
            rotation = mc.mov_control.trans.transform.rotation
            roll, pitch, yaw = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')



            # Rotate and head thru center of the gate
            angle_x, angle_y = vs.front_camera.get_angles(center[0], 0) #get angle between center of bbox and image frame
            rospy.sleep(0.05)
            mc.mov_control.set_rotation(0, 0, yaw - angle_x)
            # Wait a bit for control to start 
            rospy.sleep(0.1)
            while (mc.mov_control.get_running_confirmation()):
                rospy.sleep(0.001)
            rospy.sleep(1)
            mc.mov_control.set_height(-0.8)
            mc.mov_control.await_completion()
            # # Go straight
            # mc.mov_control.go_straight(15)

            return 'outcome1'
        else: 
            return 'outcome2'
        