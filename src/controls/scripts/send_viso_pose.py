#!/usr/bin/env python
"""
Try to send external position to MAVlink
Relevant links:
# EKF related
https://discuss.bluerobotics.com/t/ardusub-research-development-dvl/6990
https://github.com/Williangalvani/ardupilot/commit/e1d009555e7cadaf69c1d901e5b5ef5fc4b5c3ca#diff-44fb8d1e593cf689717f7e036207a553ff61b27792d44f8fbc5f97b9ccbc8ae2R1
https://github.com/thien94/vision_to_mavros/blob/master/scripts/t265_to_mavlink.py
https://github.com/Teledyne-Marine/Wayfinder/blob/master/integration/blueROV/blueROVIntegration.py
https://discuss.ardupilot.org/t/vision-position-estimate-not-appearing-in-qgroundcontrol/23978
https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA
https://www.ardusub.com/developers/dvl-integration.html
https://ardupilot.org/copter/docs/common-ekf-sources.html
https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_VisualOdom

https://discuss.bluerobotics.com/t/x-y-z-axis-reference-bluerov2/11565
https://ardupilot.org/copter/docs/common-vio-tracking-camera.html#configure-ardupilot
https://discuss.ardupilot.org/t/integration-of-ardupilot-and-vio-tracking-camera-part-4-non-ros-bridge-to-mavlink-in-python/44001

# Mix with other yaw sources
https://discuss.ardupilot.org/t/vio-alignment-in-primary-lane/72308/9
https://discuss.ardupilot.org/t/external-visual-odometry-and-gps-at-the-same-time/52543

# Home related
https://github.com/ArduPilot/MAVProxy/blob/master/MAVProxy/modules/mavproxy_map/__init__.py
https://discuss.bluerobotics.com/t/sending-mavproxy-messages-from-a-python-program/1515/2
https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html
https://github.com/mavlink/mavros/issues/1641
"""


# ---------------------------------------------
#   Mavlink area
# ---------------------------------------------

import time
# Import mavutil
from pymavlink import mavutil

# Try to set home
def cmd_set_home(master, lat, lon, alt):
    '''called when user selects "Set Home (with height)" on map'''
    print("Setting home to: ", lat, lon, alt)
    print(master.mav.set_gps_global_origin_send(
        0,
        lat, # lat
        lon, # lon
        alt)) # param7

    master.mav.command_int_send(
        0, 0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        0,
        1,  # set position
        0,  # param1
        0,  # param2
        0,  # param3
        0,  # param4
        lat,  # lat
        lon,  # lon
        alt)  # param7

# Send vision delta courtesy of:
# https://github.com/Williangalvani/ardupilot/commit/e1d009555e7cadaf69c1d901e5b5ef5fc4b5c3ca#diff-44fb8d1e593cf689717f7e036207a553ff61b27792d44f8fbc5f97b9ccbc8ae2R1
def send_vision(master, sys_time, delt_time, position=[0.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.0], confidence=100): # , x, y, z
    "Sends message VISION_POSITION_DELTA to flight controller"
    print("Sending VPD: ", position)
    master.mav.vision_position_delta_send(
        sys_time, # 0, # time (us)
        delt_time,  # delta time (us)
        rotation,  # angle delta
        position, # position delta [x, y, z],
        confidence)  # confidence (0-100%)

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

last_pos = [0.0, 0.0, 0.0]
last_rot = [0.0, 0.0, 0.0]
boot_time = datetime.now()
last_time = datetime.now()

# Does not create master yet
master = None

def processTransform(p_pose):
    """ Receive data and send to MAVLink
    """
    global last_time, last_rot, last_pos
    curr_time = datetime.now()
    delt_time = (curr_time - last_time).total_seconds() * 1e6
    sys_time = (curr_time - boot_time).total_seconds() * 1e6
    last_time = curr_time

    curr_pos = [p_pose.position.x,
                p_pose.position.y,
                p_pose.position.z]

    ai, aj, ak = euler.quat2euler([
                p_pose.orientation.x,
                p_pose.orientation.y,
                p_pose.orientation.z,
                p_pose.orientation.w
               ])

    curr_rot = [ai, aj, ak]

    delt_pos = list(map(float.__sub__, curr_pos, last_pos))
    delt_rot = list(map(float.__sub__, curr_rot, last_rot))

    # Change to NED framey
    delt_pos[1] = -delt_pos[1]
    delt_pos[2] = -delt_pos[2]
    delt_rot[1] = -delt_rot[1]
    delt_rot[2] = -delt_rot[2]
    delt_rot[2] = delt_rot[2] % (math.pi * 2)

    send_vision(master, sys_time, delt_time, delt_pos, delt_rot, 100)
    last_pos = curr_pos
    last_rot = curr_rot

if __name__== '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('mavlink_publisher', anonymous=True)

    # Create the connection
    master = mavutil.mavlink_connection(rospy.get_param('~mav_addr'), dialect='ardupilotmega')
    # Make sure the connection is valid
    master.wait_heartbeat()

    # Check if need to set home
    if (rospy.get_param('~set_home')):
        cmd_set_home(master, rospy.get_param('~home_lat') * 1e7, rospy.get_param('~home_long') * 1e7, rospy.get_param('~home_alt'))

    rospy.Subscriber(rospy.get_param('~pose_topic'), geometry_msgs.msgs.PoseStamped, processTransform)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
