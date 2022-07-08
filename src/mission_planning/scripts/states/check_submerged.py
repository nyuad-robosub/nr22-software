import rospy
import threading
import smach
import os
from std_msgs.msg import Float32
# from smach_controller_simulation import mov_control
import movement_controller as mc
import tf2_ros
import geometry_msgs.msg
from transforms3d import euler

class check_sub(smach.State):

    _outcomes=['outcome1']
    _input_keys=[],
    _output_keys=[]
    def __init__(self):
        self.alt_subscriber = rospy.Subscriber('/altitude', Float32 , self.callback)
        self.mutex = threading.Lock()
        self.is_submerged = False


    def callback(self, alt):
        
        if(alt.data < -0.2):
            self.mutex.acquire()
            self.is_submerged=True
            self.mutex.release()
        
    def execute(self, userdata):
        
        print("EXEXUTING")
        while(not rospy.is_shutdown()):
            self.mutex.acquire()
            if(self.is_submerged):
                #initialize viso.launch
                #smach_controller.ls.launch("viso.launch")
                print("submerged")
                #submerge the vehicle
                mc.mov_control.update_tf()
                mc.mov_control.change_height(-0.8)
                mc.mov_control.await_completion()

                #go to coin flip
                return 'outcome1'
            self.mutex.release()
            rospy.sleep(0.05)