
#from smach_controller_simulation import goal_topic,isRunning_topic,world_frame,rov_frame
import rospy 

import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Bool

class movement_controller():
    # controlerr/goalRotation
    # controller/isRunning
    def __init__(self,goal_topic,world_frame,rov_frame,isRunning_topic):
        self.world_frame=world_frame
        self.rov_frame=rov_frame
        self.isRunning_topic=isRunning_topic
        self.goal_topic=goal_topic

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.goal_publisher = rospy.Publisher(goal_topic, geometry_msgs.msg.PointStamped, queue_size=5)
        self.goal_rotation_publisher = rospy.Publisher('controller/goalRotation', geometry_msgs.msg.Quaternion, queue_size=1, latch=True)
        self.running_subscriber = rospy.Subscriber(
            isRunning_topic, Bool, self.running_callback)

    def update_tf(self):
        self.trans = self.tfBuffer.lookup_transform(
            self.world_frame, self.rov_frame, rospy.Time(), rospy.Duration(4))
        self.target_point = geometry_msgs.msg.PointStamped()

    def get_tf(self):
        self.update_tf()
        return self.trans

    def rotate_cw(angle):
        pass

    def running_callback():
        pass

    def go_down(self, amount):
        self.target_point.header.frame_id = self.world_frame
        self.target_point.header.stamp = rospy.Time.now()
        self.target_point.point.x = self.trans.transform.translation.x
        self.target_point.point.y = self.trans.transform.translation.y
        self.target_point.point.z = self.trans.transform.translation.z - amount
        self.goal_publisher.publish(self.target_point)

    def get_compass_angle():
        pass

    def set_precision_mode():
        pass
