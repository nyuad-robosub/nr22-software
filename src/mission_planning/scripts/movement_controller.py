
#from smach_controller_simulation import goal_topic,isRunning_topic,world_frame,rov_frame
from concurrent.futures import thread
from shutil import move
import rospy 
import math
import numpy as np
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Bool
from transforms3d import euler, quaternions
from datetime import datetime

class movement_controller():
    # controlerr/goalRotation
    # controller/isRunning
    arm_wait_time=0.5
    
    def __init__(self,goal_topic,world_frame,rov_frame,isRunning_topic):
        self.world_frame=world_frame
        self.rov_frame=rov_frame
        self.isRunning_topic=isRunning_topic
        self.goal_topic=goal_topic

        self.trans = geometry_msgs.msg.TransformStamped()
        self.target_point = geometry_msgs.msg.PointStamped()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.goal_publisher = rospy.Publisher(goal_topic, geometry_msgs.msg.PointStamped, queue_size=5)
        self.focus_publisher = rospy.Publisher('controller/focusPoint', geometry_msgs.msg.PointStamped, queue_size=1, latch=True)
        self.goal_rotation_publisher = rospy.Publisher('controller/goalRotation', geometry_msgs.msg.Quaternion, queue_size=1, latch=True)
        self.running_publisher = rospy.Publisher('controller/isRunning', Bool, queue_size=1, latch=True)
        self.running_subscriber = rospy.Subscriber(isRunning_topic, Bool, self.running_callback)
        self.arm_publisher = rospy.Publisher('controller/isArmed', Bool, queue_size=1, latch=True)
        

        # Have two flags to keep track of the past
        self.wasRunning = False
        self.isRunning = False

        #dictionary of detections and info 
        # self.image_detections = {
        #     "gate" : {
        #         "label_id" : 0,
        #         "center" : {
        #             "x" : 0,
        #             "y" : 0
        #         },
        #         "left_center" : 0,
        #         "bbox_area" : 0
        #     } 
        # }

        #for(obj in self.image_detections):
            #obj["label_id"]=
    def stop(self): 
        msg1 = Bool()
        msg1.data = False
        self.running_publisher.publish(msg1)
        #rospy.sleep(self.arm_wait_time) # Wait for stopping to finish

    def arm(self):
         # Arm
        msg1 = Bool()
        msg1.data = True
        self.arm_publisher.publish(msg1)
        rospy.sleep(self.arm_wait_time) # Wait for arm to finish
    
    def update_tf(self, timeout=5):
        """Function to update stored tf of ROV
        :param timeout: How much time in seconds lookup should wait for"""
        delay = 0.1
        counter = 0
        # Find current location
        while counter < timeout / delay:
            try:
                self.trans = self.tfBuffer.lookup_transform(self.world_frame, self.rov_frame, rospy.Time(), rospy.Duration(4))
                self.target_point = geometry_msgs.msg.PointStamped()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(delay)
                counter += 1
            break
        # Check if there were any issues
        return (counter < timeout / delay)

        # self.trans = self.tfBuffer.lookup_transform(
        #     self.world_frame, self.rov_frame, rospy.Time(), rospy.Duration(4))
        # self.target_point = geometry_msgs.msg.PointStamped()

    def get_tf(self):
        self.update_tf()
        return self.trans

    def rotate_ccw(self):
        self.update_tf()
        self.arm()

        # Rotate from initial orientation
        rotation = self.trans.transform.rotation
        roll, pitch, yaw = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')
        #print(roll, pitch, yaw)

        # Do a spin
        detected = False

        #SEPERATE THREAD NEEDED
        i = 0
        msg9 = geometry_msgs.msg.Quaternion()
        while not detected and i < 4: 
            angle = math.degrees(yaw) + i * 90 #is euler i ndegrees?
            print(angle)
            if angle < 0:
                angle += 360
            if angle >= 360:
                angle -= 360
            q = euler.euler2quat(0, 0, math.radians(angle), 'sxyz')
            msg9.w = q[0]
            msg9.x = q[1]
            msg9.y = q[2]
            msg9.z = q[3]
            self.goal_rotation_publisher.publish(msg9)
            self.await_completion()
            i+=1
    def set_rotation(self,roll,pitch,yaw):
        msg9 = geometry_msgs.msg.Quaternion()
        q = euler.euler2quat(math.radians(roll), math.radians(pitch), math.radians(yaw), 'sxyz')
        msg9.w = q[0]
        msg9.x = q[1]
        msg9.y = q[2]
        msg9.z = q[3]
        self.goal_rotation_publisher.publish(msg9)

    def running_callback(self, flag):
        self.isRunning = flag.data
    def get_running(self):
        return self.isRunning
    def await_completion(self):
        # Wait until movement finished
            while self.get_running_confirmation():    
                rospy.sleep(0.1)
    def get_running_confirmation(self):
        # If current flag is the same as new flag:
        # - Either the controller has not registered the movement, or
        # - Function called when there is no change in running status
        # -> wait a bit to discern
        if self.wasRunning == self.isRunning:
            rospy.sleep(0.01)
            if self.wasRunning == self.isRunning:
                # No change
                return self.wasRunning
        
        # Else there is change
        self.wasRunning = self.isRunning
        return self.wasRunning

    def set_focus_point(self):
        self.update_tf()
        rotation = self.trans.transform.rotation
        roll, pitch, yaw = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')

        self.target_point.header.frame_id = self.world_frame
        self.target_point.header.stamp = rospy.Time.now()
        self.target_point.point.x = self.trans.transform.translation.x + math.cos(yaw) * 1000
        self.target_point.point.y = self.trans.transform.translation.y + math.sin(yaw) * 1000
        self.target_point.point.z = 0
        
        self.focus_publisher.publish(self.target_point)

    def change_height(self, amount):
        self.update_tf()
        self.set_focus_point()
        self.target_point.header.frame_id = self.world_frame
        self.target_point.header.stamp = rospy.Time.now()
        self.target_point.point.x = self.trans.transform.translation.x
        self.target_point.point.y = self.trans.transform.translation.y
        self.target_point.point.z = self.trans.transform.translation.z + amount
        
        self.goal_publisher.publish(self.target_point)
    def set_height(self, amount):
        self.update_tf()
        self.set_focus_point()
        self.target_point.header.frame_id = self.world_frame
        self.target_point.header.stamp = rospy.Time.now()
        self.target_point.point.x = self.trans.transform.translation.x
        self.target_point.point.y = self.trans.transform.translation.y
        self.target_point.point.z = amount
        
        self.goal_publisher.publish(self.target_point)

    def go_straight(self,amount):
        self.update_tf()
        # rotation = self.trans.transform.rotation
        # roll, pitch, yaw = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')

        # self.target_point.header.frame_id = self.world_frame
        # self.target_point.header.stamp = rospy.Time.now()
        # mag=math.sqrt(pow(self.trans.transform.translation.x,2)+pow(self.trans.transform.translation.y,2)+pow(self.trans.transform.translation.z,2))
        # mag+=amount
        # self.target_point.point.x = mag*math.cos(roll)
        # self.target_point.point.y = mag*math.cos(pitch)
        # self.target_point.point.z = mag*math.cos(yaw)


        
        self.goal_publisher.publish(self.target_point)
        
    def get_compass_angle():
        pass

    def set_precision_mode():
        pass


# Global variable courtesy of:
# https://stackoverflow.com/a/13034908
def init(goal_topic, world_frame, rov_frame, isRunning_topic):
    global mov_control
    mov_control = movement_controller(goal_topic, world_frame, rov_frame, isRunning_topic)