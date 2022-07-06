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
        
        self.pub8 = rospy.Publisher('controller/isRunning', Bool, queue_size=1, latch=True)
        #self.alt_subscriber = rospy.Subscriber('/altitude', Float32 , self.callback)
        #self.mutex = threading.Lock()
        #self.is_submerged = False
        #pass #controlerr/goalRotation
        #controller/isRunning
    
    def execute(self, userdata):
        mc.mov_control.rotate_ccw(360)
        #insert mobilenet listener 


        return 'outcome1'