import rospy
import threading
import smach
import os
from std_msgs.msg import Bool
# import smach_controller_simulation
import movement_controller as mc
import geometry_msgs.msg
import math
from transforms3d import euler
class coin_flip(smach.State):
    _outcomes=['outcome1']
    _input_keys=[],
    _output_keys=[]
    def __init__(self):
        pass
    def execute(self, userdata):
        pass


        return 'outcome1'