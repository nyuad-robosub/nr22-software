from cmath import pi
from operator import truediv
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

def is_approx_equal(x,y):
    if(x>(y*0.90) and x<(y*1.10)):
        return True
    else:
        return False

class coin_flip(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self):   
        pass
        #vs.viso_control.add_detection_of_interest("gate")
    def execute(self, userdata):     
        rospy.sleep(3)

        x_center=320
        y_center=200
        detect=None

        # Rotate from initial orientation
        mc.mov_control.update_tf()
        mc.mov_control.arm()
        rotation = mc.mov_control.trans.transform.rotation
        detected=False
        detect=None
        roll, pitch, yaw = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')
        for i in range(4):
            print("ROTATING yaw:")
            print(yaw)
            print(yaw+math.pi/2*(i+1))
            mc.mov_control.set_rotation(roll,pitch,yaw+math.pi/2*(i+1)) 
            while(mc.mov_control.get_running_confirmation()):
                print("RUNNING")
                temp_detect = vs.viso_control.get_detection("gate")
                rospy.sleep(0.1)
                if(temp_detect!=None or detect!=None):
                    print("DETECTED AND EXISTS")
                    if(temp_detect!=None):
                        detect=temp_detect
                    if(is_approx_equal(detect.bbox.center.x,x_center)): # or is_approx_equal(detect.bbox.center.y,y_center)):
                        mc.mov_control.stop()
                        detected=True
                        break
            if(detected):
                break     
            
            mc.mov_control.update_tf()

        vs.viso_control.reset_detection_of_interest()
        if(detected):
            return 'outcome1'
        else: 
            return 'outcome2'
        