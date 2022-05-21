#!/usr/bin/env python
"""
Try to send position command to MAVlink
Relevant links:
https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
https://mavlink.io/en/messages/common.html#POSITION_TARGET_LOCAL_NED
"""


# ---------------------------------------------
#   Mavlink area
# ---------------------------------------------

import time
# Import mavutil
from pymavlink import mavutil

# Disable "Bare exception" warning
# pylint: disable=W0702

# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
# Note: The connection is done with 'udpin' and not 'udpout'.
#  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
#  uses a 'udpbcast' (client) and not 'udpin' (server).
#  If you want to use QGroundControl in parallel with your python script,
#  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
#  E.g: --out udpbcast:192.168.2.255:yourport
master = mavutil.mavlink_connection('udpin:127.0.0.1:14551', dialect='ardupilotmega')

# Make sure the connection is valid
master.wait_heartbeat()

# Send vision delta courtesy of:
# https://www.ardusub.com/developers/pymavlink.html
# Imports for attitude
from pymavlink.quaternion import QuaternionBase
def set_target_depth(sys_time, depth):
    """ Sets the target depth while in depth-hold mode.
    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT
    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.
    """
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=( # ignore everything except z position
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )
def set_target_attitude(sys_time, roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode.
    'roll', 'pitch', and 'yaw' are angles in degrees.
    """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )

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

WORLD_FRAME = 'world'
ROV_FRAME = 'bluerov2/base_link'

last_pos = [0.0, 0.0, 0.0]
last_rot = [0.0, 0.0, 0.0]
# curr_pos = [0.0, 0.0, 0.0]
# curr_rot = [0.0, 0.0, 0.0]
boot_time = datetime.now()
last_time = datetime.now()

def processTransform():
    global last_time, last_rot, last_pos
    curr_time = datetime.now()
    sys_time = (curr_time - boot_time).total_seconds() * 1e3

def controller():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('test_frame_listener', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            processTransform()
        finally:
            pass
        rate.sleep()

    # rospy.Subscriber("chatter", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__== '__main__':
    controller()

# ---------------------------------------------
#   Code bank area
# ---------------------------------------------
