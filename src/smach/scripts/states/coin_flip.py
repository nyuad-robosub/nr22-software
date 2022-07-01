import rospy
import threading
import smach
import os
from std_msgs.msg import Float32
import smach_controller

class rotate_ccw(smach.State):
    def __init__(self, outcomes=['outcome1']):
        #self.alt_subscriber = rospy.Subscriber('/altitude', Float32 , self.callback)
        #self.mutex = threading.Lock()
        #self.is_submerged = False
        print()
    def execute(self, userdata):
        print()