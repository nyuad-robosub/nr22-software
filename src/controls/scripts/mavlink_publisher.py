#!/usr/bin/env python
"""
Try to publish local position from MAVlink
Relevant links:
https://ardupilot.org/dev/docs/mavlink-requesting-data.html
https://discuss.ardupilot.org/t/can-not-receive-message-attitude-quaternion-using-pymavlink/68219/3
https://www.ardusub.com/developers/pymavlink.html#receive-data-and-filter-by-message-type
"""

# ---------------------------------------------
#   Mavlink area
# ---------------------------------------------

import time
# Import mavutil
#import threading
from pymavlink import mavutil, quaternion
from std_msgs.msg import Float32

# Request data stream
def set_msg_interval(master, interval_us, message_id): # , x, y, z
    "Sends SET_MESSAGE_INTERVAL to flight controller"
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,              # confirmation
        message_id,     # param1: message id
        interval_us,    # param2: interval in us
        0,  # param3
        0,  # param4
        0,  # param5
        0,  # param6
        0)  # param7

def tf_publish(t,last_pos,last_rot,br): #maybe make this a class?
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = -last_pos[0]
    t.transform.translation.y = -last_pos[1]
    t.transform.translation.z = -last_pos[2]
    t.transform.rotation.w = last_rot[0]
    t.transform.rotation.x = -last_rot[1]
    t.transform.rotation.y = -last_rot[2]
    t.transform.rotation.z = -last_rot[3]
    br.sendTransform(t)
    

# ---------------------------------------------
#   ROS area
# ---------------------------------------------

import rospy
from std_msgs.msg import String
import math
import tf2_ros
import geometry_msgs.msg
from transforms3d import euler, quaternions
from datetime import datetime

# Does not create master yet
master = None

if __name__== '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('send_viso_pose', anonymous=True)

    # Create the connection
    master = mavutil.mavlink_connection(rospy.get_param('~mav_addr'), dialect='ardupilotmega')
    # Make sure the connection is valid
    master.wait_heartbeat()

    # Set message interval (listening to LOCAL_POSITION_NED (#32) and ATTITUDE_QUATERNION (#31))
    set_msg_interval(master, 10000, 32)
    set_msg_interval(master, 10000, 31)
    set_msg_interval(master, 10000, 178)

    # Retaining old information (position & quaternion, mavlink style wxyz)
    last_pos = [0.0, 0.0, 0.0]
    last_rot = [0.0, 0.0, 0.0, 0.0]

    # Publishing frame courtesy of:
    # http://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28Python%29
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    alt_publisher = rospy.Publisher('altitude', Float32, queue_size=5)

    t.header.frame_id = rospy.get_param('~base_frame')
    t.child_frame_id = rospy.get_param('~publish_frame')

    # Receiving MAVLink data courtesy of:
    # https://stackoverflow.com/questions/53394660/receiving-and-sending-mavlink-messages-using-pymavlink-library
    # Only publish after 10 data points
    publishable = [False, False]
    dataCount = [0, 0]
    while not rospy.is_shutdown():
        fcu_msg = master.recv_match(blocking=True)
        # Check if has not received data in a while
        if master.time_since('ATTITUDE_QUATERNION') >= 1:
            publishable[0] = False
            #print("WARN: Not received ATTITUDE_QUATERNION for a while")
        if master.time_since('LOCAL_POSITION_NED') >= 1:
            publishable[1] = False
            #print("WARN: Not received LOCAL_POSITION_NED for a while")
        if fcu_msg is None:
            pass
        elif fcu_msg.get_type() != 'BAD_DATA':
            if fcu_msg.get_type() == 'ATTITUDE_QUATERNION' or fcu_msg.get_type() == 'LOCAL_POSITION_NED':
                msg_dict = fcu_msg.to_dict()
                if fcu_msg.get_type() == 'ATTITUDE_QUATERNION':
                    last_rot = [msg_dict['q1'], msg_dict['q2'], msg_dict['q3'], msg_dict['q4']]
                    if not publishable[0]:
                        dataCount[0] += 1
                        if dataCount[0] == 2:
                            dataCount[0] = 0
                            publishable[0] = True

                if fcu_msg.get_type() == 'LOCAL_POSITION_NED':
                    last_pos = [msg_dict['x'], msg_dict['y'], msg_dict['z']]
                    if not publishable[1]:
                        dataCount[1] += 1
                        if dataCount[1] == 2:
                            dataCount[1] = 0
                            publishable[1] = True
            
                if publishable[0] and publishable[1]:
                    tf_publish(t,last_pos,last_rot,br)
                # elif publishable[0]:
                #     pass
                #     #tf_publish(t,[0,0,0],last_rot,br)
                tf_publish(t,[0,0,0],last_rot,br)
            elif fcu_msg.get_type() == 'AHRS2':
                msg_dict = fcu_msg.to_dict()
                alt = msg_dict['altitude']
                alt_publisher.publish(Float32(alt))

                # print(fcu_msg)
        rospy.sleep(0.001)