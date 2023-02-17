#!/usr/bin/env python
"""
Try to filter & publish fake sonar_ping data
"""

# ---------------------------------------------
#   ROS area
# ---------------------------------------------

import rospy
from std_msgs.msg import Float32

def altitude_callback(alt):
    global sonar_ping_publisher, pool_depth, ema_alpha
    # Dodging globals courtesy of:
    # https://stackoverflow.com/a/279586
    if "avg_depth" not in altitude_callback.__dict__:
        altitude_callback.avg_depth = 0

    # If initial: start with current ping data
    if altitude_callback.avg_depth == 0:
        altitude_callback.avg_depth = pool_depth + alt.data
    else:
        # EMA courtesy of:
        # https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
        altitude_callback.avg_depth = ema_alpha * (pool_depth + alt.data) + (1 - ema_alpha) * altitude_callback.avg_depth

    # Publish
    sonar_ping_publisher.publish(Float32(altitude_callback.avg_depth))

if __name__== '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('fake_ping', anonymous=True)
    
    # Create publisher, subscriber & get topic names
    global sonar_ping_publisher, pool_depth, ema_alpha
    pool_depth = rospy.get_param('~pool_depth')
    ema_alpha = rospy.get_param('~ema_alpha')
    sonar_ping_publisher = rospy.Publisher(rospy.get_param('~sonar_ping_topic'), Float32, queue_size=5)
    rospy.Subscriber(rospy.get_param('~altitude_topic'), Float32, altitude_callback)
    rospy.spin()