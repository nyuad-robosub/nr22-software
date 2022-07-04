import rospy
import threading
import smach
import os
from std_msgs.msg import Bool
import smach_controller_simulation
import geometry_msgs.msg
import math
from transforms3d import euler
class coin_flip(smach.State):
    _outcomes=['outcome1']
    _input_keys=[],
    _output_keys=[]
    def __init__(self):
        self.pub8 = rospy.Publisher('controller/isRunning', Bool, queue_size=1, latch=True)
        self.pub9 = rospy.Publisher('controller/goalRotation', geometry_msgs.msg.Quaternion, queue_size=1, latch=True)
        #self.alt_subscriber = rospy.Subscriber('/altitude', Float32 , self.callback)
        #self.mutex = threading.Lock()
        #self.is_submerged = False
        #pass #controlerr/goalRotation
        #controller/isRunning
    
    def execute(self, userdata):
        #extract z roll from tf2 position angle about z is
        trans = smach_controller_simulation.mov_control.get_tf()
        msg9 = geometry_msgs.msg.Quaternion()
        q = euler.euler2quat(math.radians(360), 0, 0, 'sxyz')
        msg9.w = q[0]
        msg9.x = q[1]
        msg9.y = q[2]
        msg9.z = q[3]
        self.pub9.publish(msg9)

        #insert mobilenet listener 


        return 'outcome1'