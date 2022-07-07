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
from std_msgs.msg import String, Bool, Float64
import math
import numpy as np
import tf2_ros
import geometry_msgs.msg
import visualization_msgs.msg
from transforms3d import euler

focusPoint = geometry_msgs.msg.Point(0, -10, 0)
# goalPoint = geometry_msgs.msg.Point(4.2, -2.5, 1)
# goalPoints = [geometry_msgs.msg.Point(8.2, -2.5, 3),
#               geometry_msgs.msg.Point(2.5, 8.2, 2),
#               geometry_msgs.msg.Point(-8.2, 2.5, 3),
#               geometry_msgs.msg.Point(-2.5, -8.2, 2),
#               geometry_msgs.msg.Point(8.2, -2.5, 3)]
goalPoints = [geometry_msgs.msg.Point(10, -10, -2),
              geometry_msgs.msg.Point(5.2, -10.5, -1),
              # geometry_msgs.msg.Point(0, 0, 2.75),
              # geometry_msgs.msg.Point(20.2, 2.5, 3),
              geometry_msgs.msg.Point(-5.2, -10.5, -1)]
            #   geometry_msgs.msg.Point(20.2, 8.5, 1),
            #   geometry_msgs.msg.Point(20.2, -10.5, 1)]
              # geometry_msgs.msg.Point(0, 9, 2.75)]

if __name__== '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('test_controller', anonymous=True)

    rospy.sleep(1)

    pub1 = rospy.Publisher('controller/isArmed', Bool, queue_size=1, latch=True)
    pub2 = rospy.Publisher('controller/usePCL', Bool, queue_size=1, latch=True)
    pub3 = rospy.Publisher('controller/focusPoint', geometry_msgs.msg.PointStamped, queue_size=1, latch=True)
    pub4 = rospy.Publisher('controller/goalPoint', geometry_msgs.msg.PointStamped, queue_size=1, latch=True)
    pub5 = rospy.Publisher('controller/goalPath', visualization_msgs.msg.Marker, queue_size=1, latch=True)
    pub6 = rospy.Publisher('controller/segmentSize', Float64, queue_size=1, latch=True)
    pub7 = rospy.Publisher('controller/goalThreshold', Float64, queue_size=1, latch=True)

    pub8 = rospy.Publisher('controller/isRunning', Bool, queue_size=1, latch=True)
    pub9 = rospy.Publisher('controller/goalRotation', geometry_msgs.msg.Quaternion, queue_size=1, latch=True)

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

    rospy.sleep(2)
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

for i in range(4):
    rospy.sleep(6)
    msg8 = Bool()
    msg8.data = False
    pub8.publish(msg8)
    rospy.sleep(1)
    msg9 = geometry_msgs.msg.Quaternion()
    q = euler.euler2quat(math.radians(90), 0, math.radians(180 + i * 90), 'sxyz')
    msg9.w = q[0]
    msg9.x = q[1]
    msg9.y = q[2]
    msg9.z = q[3]
    pub9.publish(msg9)
    # msg6 = Float64()
    # msg6.data = 0.3
    # pub6.publish(msg6)
    # msg7 = Float64()
    # msg7.data = 0.15
    # pub7.publish(0.15)

    # rospy.sleep(20)
    # msg1.data = False
    # pub1.publish(msg1)