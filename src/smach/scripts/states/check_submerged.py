import rospy
import threading
import smach
import os
from std_msgs.msg import Float32
import smach_controller
import tf2_ros
import geometry_msgs.msg

class check_sub(smach.State):

    def __init__(self, outcomes=['detected']):
        self.alt_subscriber = rospy.Subscriber('/altitude', Float32 , self.callback)
        self.mutex = threading.Lock()
        self.is_submerged = False;

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.goal_publisher = rospy.Publisher(smach_controller.goal_topic, geometry_msgs.msg.PointStamped, queue_size=5)

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
            trans = self.tfBuffer.lookup_transform(smach_controller.world_frame, smach_controller.rov_frame, rospy.Time(), rospy.Duration(4))
            point_down =geometry_msgs.msg.PointStamped()
            point_down.header.frame_id=smach_controller.world_frame
            point_down.header.stamp=rospy.Time.now()
            point_down.point.x=trans.transform.translation.x
            point_down.point.x=trans.transform.translation.y
            point_down.point.x=trans.transform.translation.z - 0.8
            self.goal_publisher.publish(point_down)

            #go to coin flip
            return 'detected'
        self.mutex.release()