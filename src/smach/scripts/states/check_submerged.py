import rospy
import threading
import smach
import os
from std_msgs.msg import Float32
import smach_controller

class check_sub(smach.State):
    def __init__(self, outcomes=['detected']):
        self.alt_subscriber = rospy.Subscriber('/altitude', Float32 , self.callback)
        self.mutex = threading.Lock()
        self.is_submerged = False;

    def callback(self, alt):
        if(alt < -0.2):
            self.mutex.acquire()
            is_submerged=True;
            self.mutex.release()
        
    def execute(self, userdata):
        self.mutex.acquire()
        if(self.is_submerged):
            #initialize viso.launch
            smach_controller.ls.launch("viso.launch")

            #submerge the vehicle
            trans = tfBuffer.lookup_transform(WORLD_FRAME, ROV_FRAME, rospy.Time(), rospy.Duration(4))
            

            #go to coin flip
            return 'detected'
        self.mutex.release()