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
        self.pub1 = rospy.Publisher('controller/isArmed', Bool, queue_size=1, latch=True)
        self.pub8 = rospy.Publisher('controller/isRunning', Bool, queue_size=1, latch=True)
        self.pub9 = rospy.Publisher('controller/goalRotation', geometry_msgs.msg.Quaternion, queue_size=1, latch=True)
        #self.alt_subscriber = rospy.Subscriber('/altitude', Float32 , self.callback)
        #self.mutex = threading.Lock()
        #self.is_submerged = False
        #pass #controlerr/goalRotation
        #controller/isRunning
    
    def execute(self, userdata):
        #extract z roll from tf2 position angle about z is
        trans = mc.mov_control.get_tf()

        # Arm
        msg1 = Bool()
        msg1.data = True
        self.pub1.publish(msg1)
        rospy.sleep(2) # Wait for arm to finish

        # Rotate from initial orientation
        rotation = trans.transform.rotation
        roll, pitch, yaw = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')
        print(roll, pitch, yaw)

        # Do a spin
        detected = False
        i = 0
        msg9 = geometry_msgs.msg.Quaternion()
        while not detected and i < 12: 
            angle = math.degrees(yaw) + i * 30
            print(angle)
            if angle >= 360:
                angle -= 360
            q = euler.euler2quat(0, 0, math.radians(angle), 'sxyz')
            msg9.w = q[0]
            msg9.x = q[1]
            msg9.y = q[2]
            msg9.z = q[3]
            self.pub9.publish(msg9)
            # Wait until movement finished
            while mc.mov_control.get_running_confirmation():    
                rospy.sleep(0.5)
            i += 1

        #insert mobilenet listener 


        return 'outcome1'