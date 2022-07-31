#!/usr/bin/env python
"""
Try to filter & publish fake sonar_ping data
"""

# ---------------------------------------------
#   ROS area
# ---------------------------------------------

import math
import random
import rospy
from std_msgs.msg import Float32
import tf2_ros
from transforms3d import euler

if __name__== '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('fake_hydro', anonymous=True)
    
    # Create publisher, subscriber & get topic names
    global x, y, z, noise, interval, hydro_publisher
    x = rospy.get_param('~x')
    y = rospy.get_param('~y')
    z = rospy.get_param('~z')
    noise = rospy.get_param('~noise')
    interval = rospy.get_param('~interval')
    world_frame = rospy.get_param('~world_frame')
    rov_frame = rospy.get_param('~rov_frame')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    hydro_publisher = rospy.Publisher(rospy.get_param('~hydro_topic'), Float32, queue_size=5)
    rate = rospy.Rate(1 / interval)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(world_frame, rov_frame, rospy.Time(), rospy.Duration(4))
            rotation = trans.transform.rotation
            roll, pitch, yaw = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')
            abs_angle = math.atan2(y - trans.transform.translation.y, x - trans.transform.translation.x)
            rel_angle = abs_angle - yaw
            if rel_angle > math.pi:
                rel_angle -= math.pi * 2
            elif rel_angle <= -math.pi:
                rel_angle += math.pi * 2
            hydro_publisher.publish(Float32(random.uniform(rel_angle - noise, rel_angle + noise)))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        rate.sleep()