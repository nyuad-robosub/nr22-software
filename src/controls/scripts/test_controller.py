#!/usr/bin/env python
"""
Try to test controller
Relevant links:
"""


# ---------------------------------------------
#   ROS area
# ---------------------------------------------
import time

import rospy
from std_msgs.msg import String, Bool
import math
import numpy as np
import tf2_ros
import geometry_msgs.msg
import visualization_msgs.msg

focusPoint = geometry_msgs.msg.Point(0, 1000, 0)
goalPoint = geometry_msgs.msg.Point(4.2, -2.5, 1)
goalPoints = [geometry_msgs.msg.Point(20.2, -2.5, 3),
              geometry_msgs.msg.Point(20.2, -10.5, 3),
              # geometry_msgs.msg.Point(0, 0, 2.75),
              # geometry_msgs.msg.Point(20.2, 2.5, 3),
              geometry_msgs.msg.Point(20.2, 8.5, 3),
              geometry_msgs.msg.Point(20.2, 8.5, 1),
              geometry_msgs.msg.Point(20.2, -10.5, 1)]
              # geometry_msgs.msg.Point(0, 9, 2.75)]

if __name__== '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('test_controller', anonymous=True)

    rospy.sleep(3)

    pub1 = rospy.Publisher('controller/isArmed', Bool, queue_size=1, latch=True)
    pub2 = rospy.Publisher('controller/usePCL', Bool, queue_size=1, latch=True)
    pub3 = rospy.Publisher('controller/focusPoint', geometry_msgs.msg.PointStamped, queue_size=1, latch=True)
    pub4 = rospy.Publisher('controller/goalPoint', geometry_msgs.msg.PointStamped, queue_size=1, latch=True)
    pub5 = rospy.Publisher('controller/goalPath', visualization_msgs.msg.Marker, queue_size=1, latch=True)

    rate = rospy.Rate(10)  # 10hz
    count = 1
    while count > 0:
        msg1 = Bool()
        msg1.data = True
        pub1.publish(msg1)

        msg2 = Bool()
        msg2.data = False
        pub2.publish(msg2)

        msg3 = geometry_msgs.msg.PointStamped()
        msg3.header.stamp = rospy.Time.now()
        msg3.header.frame_id = 'world'
        msg3.point = focusPoint
        pub3.publish(msg3)
        rate.sleep()
        count -= 1


    time.sleep(3)
    # msg4 = geometry_msgs.msg.PointStamped()
    # msg4.header.stamp = rospy.Time.now()
    # msg4.header.frame_id = 'world'
    # msg4.point = goalPoint
    # pub4.publish(msg4)

    msg5 = visualization_msgs.msg.Marker()
    msg5.header.stamp = rospy.Time.now()
    msg5.header.frame_id = 'world'
    for p in goalPoints:
        msg5.points.append(p)
    pub5.publish(msg5)


