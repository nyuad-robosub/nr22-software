#!/usr/bin/env python
"""
Try to send position command to MAVlink
Relevant links:
https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
https://mavlink.io/en/messages/common.html#POSITION_TARGET_LOCAL_NED
"""

# Segment size
SEGMENT_SIZE = 10
# Goal threshold
GOAL_THRESHOLD = 0.1

# ---------------------------------------------
#   Mavlink area
# ---------------------------------------------
import rosparam
import visualization_msgs.msg

import time
# Import mavutil
from pymavlink import mavutil
# Imports for attitude
from pymavlink.quaternion import QuaternionBase

# Send vision delta courtesy of:
# https://www.ardusub.com/developers/pymavlink.html
def set_target_local_position(master, sys_time, x, y, z, yaw=0, vx=0, vy=0, vz=0):
    """ Sets local position courtesy of:
    https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
    """
    master.mav.set_position_target_local_ned_send(
        sys_time, # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask=( # ignore velocity and other stuff
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), x=x, y=y, z=z, # (x, y WGS84 frame pos - not used), z [m]
        vx=vx, vy=vy, vz=vz, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=yaw, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )
def set_target_attitude(master, sys_time, roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode.
    'roll', 'pitch', and 'yaw' are angles in degrees.
    """
    master.mav.set_attitude_target_send(
        sys_time, # int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )
def set_target_yaw(master, yaw):
    """ Sets the target yaw as angle in degrees.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        yaw, 5, 1, 0, 0, 0, 0)

# ---------------------------------------------
#   ROS area
# ---------------------------------------------

import rospy
from std_msgs.msg import String, Bool
import math
import numpy as np
import tf2_ros
import geometry_msgs.msg
from transforms3d import euler, quaternions
from datetime import datetime
RAD_TO_DEG = 180 / math.pi

WORLD_FRAME = 'world'
ROV_FRAME = 'bluerov2/base_link'

last_pos = [0.0, 0.0, 0.0]
last_rot = [0.0, 0.0, 0.0]
# curr_pos = [0.0, 0.0, 0.0]
# curr_rot = [0.0, 0.0, 0.0]
boot_time = datetime.now()
last_time = datetime.now()

# Global variables changed through topics
# Initialization here
usePCL = False
isArmed = False
focusPoint = geometry_msgs.msg.Point(0, 0, 0)

# Undeclared master
master = None

def pcl_callback(flag):
    """ Callback to set PCL usage
        If used, the controller will rely on the path produced from the path planning node
    :param flag: std_msgs.msg.Bool flag monitored
    """
    global usePCL
    if flag.data:
        usePCL = True
        print("INFO: Using PCL!")
    else:
        usePCL = False
        print("INFO: Not using PCL!")

def arm_callback(flag):
    """ Callback to set arm / disarm
        Controller will monitor boolean to stop sending commands whenever disarmed
    :param flag: std_msgs.msg.Bool flag monitored
    """
    global master, isArmed
    if flag.data:
        # arm ArduSub autopilot and wait until confirmed
        master.arducopter_arm()
        master.motors_armed_wait()
        isArmed = True
        print("INFO: Armed!")
    else:
        # clean up (disarm) at the end
        isArmed = False
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        print("INFO: Disarmed!")

def focus_callback(focus):
    """ Get point for the controller to orient the rov towards
    :param focus: geometry_msgs.msg.PointStamped, point to orient towards (only using xy as of now)"""
    global focusPoint
    focusPoint = focus.point
    print("INFO: Focus point set!")

def path_callback(path):
    """ Callback function when receiving path to follow.
        This path will either be generated from pointcloud algorithms (usePCL = True)
        or passed directly from the goal_callback() below.
    :param path: visualization_msgs.Marker (LineStrip type, starting from the next position to go to)
    """
    global focusPoint
    if not isArmed:
        print("Trajectory planned but ROV not armed!")
        return

    # Monitoring position of ROV
    WORLD_FRAME = rospy.get_param("~world_frame")
    ROV_FRAME = rospy.get_param("~rov_frame")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    for p in path.points:
        # Find current location
        while True:
            try:
                trans = tfBuffer.lookup_transform(WORLD_FRAME, ROV_FRAME, rospy.Time(), rospy.Duration(4))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(0.2)
                if isArmed:
                    continue
            break
        if not isArmed:
            break
        p_arr = np.array([p.x, p.y, p.z])
        trans_arr = np.array(
            [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        delt = p_arr - trans_arr
        mag = np.linalg.norm(delt)

        while mag > GOAL_THRESHOLD:
            if not isArmed:
                break
            if mag < SEGMENT_SIZE:
                nextPoint = p
            else:
                # Move one meter at a time
                tmp_arr = trans_arr + (delt / mag) * SEGMENT_SIZE
                nextPoint = geometry_msgs.msg.Point(tmp_arr[0], tmp_arr[1], tmp_arr[2])

            # TRANSLATION
            # set the desired operating mode
            guided = 'GUIDED'
            guided_mode = master.mode_mapping()[guided]
            while not master.wait_heartbeat().custom_mode == guided_mode:
                master.set_mode(guided_mode)
            if not isArmed:
                break

            # set a position target
            if focusPoint.x == nextPoint.x and focusPoint.y == nextPoint.y:
                yaw = 0
            else:
                yaw = math.atan2(focusPoint.y - nextPoint.y, focusPoint.x - nextPoint.x)
                if yaw < 0:
                    yaw += 2 * math.pi
                yaw = yaw * RAD_TO_DEG
            print("setting position")
            sys_time = (datetime.now() - boot_time).total_seconds() * 1e3
            set_target_local_position(master, sys_time, nextPoint.x, nextPoint.y, nextPoint.z)
            np_arr = np.array([nextPoint.x, nextPoint.y, nextPoint.z])
            delt = np_arr - trans_arr
            mag = np.linalg.norm(delt)
            while mag > 0.1:
                # Find current location
                while True:
                    try:
                        trans = tfBuffer.lookup_transform(WORLD_FRAME, ROV_FRAME, rospy.Time(), rospy.Duration(4))
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rospy.sleep(0.2)
                        if isArmed:
                            continue
                    break
                if not isArmed:
                    break
                trans_arr = np.array(
                    [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                delt = np_arr - trans_arr
                mag = np.linalg.norm(delt)
            # rospy.sleep(5)

            # ROTATION
            # # set the desired operating mode
            # depth_hold = 'ALT_HOLD'
            # depth_hold_mode = master.mode_mapping()[depth_hold]
            # while not master.wait_heartbeat().custom_mode == depth_hold_mode:
            #     master.set_mode(depth_hold)
            # if not isArmed:
            #     break
            #
            # set a rotation target
            print("setting rotation")
            set_target_yaw(master, yaw)
            # sys_time = (datetime.now() - boot_time).total_seconds() * 1e3
            # set_target_attitude(master, sys_time, 0, 0, yaw)
            # rospy.sleep(3.5)
            if not isArmed:
                break

            # Find current location
            while True:
                try:
                    trans = tfBuffer.lookup_transform(WORLD_FRAME, ROV_FRAME, rospy.Time(), rospy.Duration(4))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.sleep(0.2)
                    if isArmed:
                        continue
                break
            if not isArmed:
                break
            trans_arr = np.array(
                [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            delt = p_arr - trans_arr
            mag = np.linalg.norm(delt)
            print("Distance to goal:", mag)

    if not isArmed:
        print("Robot disarmed, controls will stop")
        return

    # set the desired operating mode
    pos_hold = 'POSHOLD'
    pos_hold_mode = master.mode_mapping()[pos_hold]
    while not master.wait_heartbeat().custom_mode == pos_hold_mode:
        master.set_mode(pos_hold)

def goal_callback(goal):
    """ Callback function when receiving destination
    :param goal: geometry_msgs.msg.PointStamped, position to go to
    """
    if not isArmed:
        print("Goal received but ROV not armed!")
        return
    # Monitoring position of ROV
    WORLD_FRAME = rospy.get_param("~world_frame")
    ROV_FRAME = rospy.get_param("~rov_frame")

    # if goal.header.frame_id is not WORLD_FRAME:
    #     print("Inconsistent frames between goal and LOCAL_POSITION_NED!")
    #     return

    # If PCL is not used: direct the controller to go directly to the point
    if not usePCL:
        faux_msg = visualization_msgs.msg.Marker()
        faux_msg.header.stamp = rospy.Time.now()
        faux_msg.header.frame_id = WORLD_FRAME
        faux_msg.points.append(goal.point)
        path_callback(faux_msg)
    else:
        pass

def controller():
    global master
    # Create the connection
    master = mavutil.mavlink_connection(rospy.get_param('~mav_addr'), dialect='ardupilotmega')
    # Make sure the connection is valid
    master.wait_heartbeat()

    rospy.Subscriber(rospy.get_param('~arm_topic'), Bool, arm_callback)
    rospy.Subscriber(rospy.get_param('~pcl_topic'), Bool, pcl_callback)
    rospy.Subscriber(rospy.get_param('~focus_topic'), geometry_msgs.msg.PointStamped, focus_callback)
    rospy.Subscriber(rospy.get_param('~goal_topic'), geometry_msgs.msg.PointStamped, goal_callback)
    rospy.Subscriber(rospy.get_param('~goal_path_topic'), visualization_msgs.msg.Marker, path_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__== '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('controller', anonymous=True)

    # Run controller
    controller()

# ---------------------------------------------
#   Code bank area
# ---------------------------------------------
