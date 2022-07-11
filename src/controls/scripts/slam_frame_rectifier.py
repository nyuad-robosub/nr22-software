#!/usr/bin/env python
"""
Try to rectify & publish SLAM frame
"""

# ---------------------------------------------
#   ROS area
# ---------------------------------------------

from audioop import mul
import rospy
from std_msgs.msg import String
import math
import tf2_ros
import geometry_msgs.msg
from transforms3d import euler, quaternions

# Does not create master yet
master = None

if __name__== '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('slam_frame_rectifier', anonymous=True)

    WORLD_FRAME = rospy.get_param("~world_frame")
    SLAM_FRAME = rospy.get_param("~slam_frame")

    # Publishing frame courtesy of:
    # http://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28Python%29
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = rospy.get_param('~world_frame')
    t.child_frame_id = rospy.get_param('~publish_frame')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(30.0)
    const_quat = [0.5, 0.5, -0.5, 0.5]
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(WORLD_FRAME, SLAM_FRAME, rospy.Time(), rospy.Duration(4))
            # Translate
            t.transform.translation.x = trans.transform.translation.z
            t.transform.translation.y = -trans.transform.translation.x
            t.transform.translation.z = -trans.transform.translation.y

            # Rotate courtesy of:
            # https://stackoverflow.com/a/65250201
            mult_quat = quaternions.qmult(
                quaternions.qinverse(const_quat),
                quaternions.qmult([
                        trans.transform.rotation.w, trans.transform.rotation.x,
                        trans.transform.rotation.y, trans.transform.rotation.z
                    ],
                    const_quat
                )
            )
            t.transform.rotation.w = mult_quat[0]
            t.transform.rotation.x = mult_quat[1]
            t.transform.rotation.y = mult_quat[2]
            t.transform.rotation.z = mult_quat[3]
            # roll, pitch, yaw = euler.quat2euler([
            #     trans.transform.rotation.w, trans.transform.rotation.x,
            #     trans.transform.rotation.y, trans.transform.rotation.z
            # ])
            # trans.transform.rotation.w, trans.transform.rotation.x, \
            #     trans.transform.rotation.y, trans.transform.rotation.z = \
            #     euler.euler2quat(roll, pitch, yaw, 'sxyz')

            br.sendTransform(t)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(0.001)
            continue
        rate.sleep()