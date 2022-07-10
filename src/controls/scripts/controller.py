#!/usr/bin/env python
"""
Try to send position command to MAVlink
Relevant links:
https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
https://mavlink.io/en/messages/common.html#POSITION_TARGET_LOCAL_NED
"""

# Segment size
# SEGMENT_SIZE = 2
# Goal threshold
# GOAL_THRESHOLD = 0.1

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

# https://www.ardusub.com/developers/pymavlink.html
def set_target_depth(sys_time, depth):
    """ Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

    """
    master.mav.set_position_target_global_int_send(
        sys_time, # ms since boot
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
        # Accounting for reversed z axis
        ), x=x, y=-y, z=-z, # (x, y WGS84 frame pos - not used), z [m]
        vx=vx, vy=vy, vz=vz, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=yaw, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )
def set_target_attitude(master, sys_time, roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode.
    'roll', 'pitch', and 'yaw' are angles in radians.
    """
    # Accounting for reversed z axis
    q = QuaternionBase([roll, pitch, yaw])
    q.q[2] = -q.q[2]
    q.q[3] = -q.q[3]
    master.mav.set_attitude_target_send(
        sys_time, # int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        q, # QuaternionBase([roll, pitch, yaw]), # math.radians(angle) for angle in (roll, pitch, yaw)
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )
def set_target_yaw(master, yaw):
    """ Sets the target yaw as angle in degrees.
    """
    # Accounting for reversed z axis
    r_yaw = 360 - yaw
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        r_yaw, 5, 1, 0, 0, 0, 0)

# ---------------------------------------------
#   ROS area
# ---------------------------------------------
from datetime import datetime
import geometry_msgs.msg
import math
import numpy as np
import rospy
from std_msgs.msg import String, Bool, Float64
import tf2_ros
import threading
from transforms3d import euler, quaternions
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
isRunning = False
focusPoint = geometry_msgs.msg.Point(0, 0, 0)
segmentSize = 3
goalThreshold = 0.2
angleSize = 10
angleThreshold = 4

# Global monitor of attitude
attitude = geometry_msgs.msg.Quaternion(0, 0, 0, 0)
attitude_lock = threading.Lock()

# Undeclared master & publisher
master = None
run_publisher = None

def arm_callback(flag): 
    """ Callback to set arm / disarm
        Controller will monitor boolean to stop sending commands whenever disarmed
    :param flag: std_msgs.msg.Bool flag monitored
    """
    global master, isArmed, isRunning, run_publisher
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

        if isRunning:
            msg = Bool()
            msg.data = False
            run_publisher.publish(msg)

def run_callback(flag): 
    """ Callback to operate/stop the ROV
        Controller will monitor boolean to stop sending commands whenever stopped
    :param flag: std_msgs.msg.Bool flag monitored
    """
    global isRunning, isArmed, run_publisher
    if flag.data:
        if isArmed:
            # run
            if not isRunning:
                print("INFO: Running!")
                isRunning = True
            else:
                pass
                # print("INFO: Has been running!")
        else:
            print("Disarmed, cannot run!")
            msg = Bool()
            msg.data = False
            run_publisher.publish(msg)
    else:
        # stop
        if isRunning:
            isRunning = False
            print("INFO: Holding position!")
        else:
            print("INFO: Has stopped!")

# ---------------------------------------------
#   PATH FOLLOW
# ---------------------------------------------

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
    global focusPoint, isArmed, isRunning
    if not isArmed:
        print("Trajectory planned but ROV not armed!")
        return
    
    if isRunning:
        print("Another control process is running!")
        return

    # Starting the process
    isRunning = True
    msg = Bool()
    msg.data = True
    run_publisher.publish(msg)

    # Monitoring position of ROV
    WORLD_FRAME = rospy.get_param("~world_frame")
    ROV_FRAME = rospy.get_param("~rov_frame")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # set the desired operating mode
    guided = 'GUIDED'
    guided_mode = master.mode_mapping()[guided]
    while not master.wait_heartbeat().custom_mode == guided_mode:
        master.set_mode(guided_mode)

    for p in path.points:
        # Find current location
        while True:
            try:
                trans = tfBuffer.lookup_transform(WORLD_FRAME, ROV_FRAME, rospy.Time(), rospy.Duration(4))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(0.2)
                if isArmed and isRunning:
                    continue
            break
        if not isArmed or not isRunning:
            break
        p_arr = np.array([p.x, p.y, p.z])
        trans_arr = np.array(
            [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        delt = p_arr - trans_arr
        mag = np.linalg.norm(delt)

        while mag > goalThreshold:
            if not isArmed or not isRunning:
                break
            if mag < segmentSize:
                nextPoint = p
            else:
                # Move one segment at a time
                tmp_arr = trans_arr + (delt / mag) * segmentSize
                nextPoint = geometry_msgs.msg.Point(tmp_arr[0], tmp_arr[1], tmp_arr[2])

            # TRANSLATION
            # set a position target
            print("setting position")
            sys_time = (datetime.now() - boot_time).total_seconds() * 1e3
            set_target_local_position(master, sys_time, nextPoint.x, nextPoint.y, nextPoint.z)
            np_arr = np.array([nextPoint.x, nextPoint.y, nextPoint.z])
            delt = np_arr - trans_arr
            mag = np.linalg.norm(delt)
            while mag > goalThreshold:
                # Find current location
                while True:
                    try:
                        trans = tfBuffer.lookup_transform(WORLD_FRAME, ROV_FRAME, rospy.Time(), rospy.Duration(4))
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rospy.sleep(0.2)
                        if isArmed and isRunning:
                            continue
                    break
                if not isArmed or not isRunning:
                    break
                trans_arr = np.array(
                    [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                delt = np_arr - trans_arr
                mag = np.linalg.norm(delt)

                # ROTATION
                # set a rotation target while moving
                if focusPoint.x == trans_arr[0] and focusPoint.y == trans_arr[1]:
                    yaw = 0
                else:
                    yaw = math.atan2(focusPoint.y - trans_arr[1], focusPoint.x - trans_arr[0])
                    if yaw < 0:
                        yaw += 2 * math.pi
                    yaw = yaw * RAD_TO_DEG
                    
                # print("setting rotation")
                set_target_yaw(master, yaw)
                rospy.sleep(0.2)

            # Find current location
            while True:
                try:
                    trans = tfBuffer.lookup_transform(WORLD_FRAME, ROV_FRAME, rospy.Time(), rospy.Duration(4))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.sleep(0.2)
                    if isArmed and isRunning:
                        continue
                break
            if not isArmed or not isRunning:
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
    # hold position after done with path callback
    pos_hold = 'POSHOLD'
    pos_hold_mode = master.mode_mapping()[pos_hold]
    while not master.wait_heartbeat().custom_mode == pos_hold_mode:
        master.set_mode(pos_hold)
    
    # Detect if stopped from successful trip
    if isRunning:
        print("Trajectory completed")
        msg.data = False
        run_publisher.publish(msg)

def goal_callback(goal):
    """ Callback function when receiving destination
    :param goal: geometry_msgs.msg.PointStamped, position to go to
    """
    global isArmed
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
        print("PCL usage not implemented!")

def segment_size_callback(segment_size):
    """ Maximum distance the ROV will move at a time
    :param segment_size: Float64, segment size in meters"""
    global segmentSize
    segmentSize = segment_size.data
    print("INFO: Segment size set!")

def goal_threshold_callback(goal_threshold):
    """ How close to the goal is acceptable
    :param goal_threshold: Float64, goal threshold in meters"""
    global goalThreshold
    goalThreshold = goal_threshold.data
    print("INFO: Goal threshold set!")

# ---------------------------------------------
#   ROTATION
# ---------------------------------------------

# Helper functions
def getDegsFromQuats(q): # input list in wxyz format
    tmp = [math.degrees(x) for x in euler.quat2euler(q, 'sxyz')]
    if tmp[2] < 0:
        tmp[2] += 360
    elif tmp[2] >= 360:
        tmp[2] -= 360
    return tmp

def getDegsDiff(a, b): # get smallest angle from a to b
    diff = a - b
    if diff <= -180:
        diff += 360
    elif diff > 180:
        diff -= 360
    return diff

def rotation_callback(rotation):
    """ Callback function to do rotation.
    :param rotation: geometry_msgs.msg.QuaternionStamped
    """
    global run_publisher, isArmed, isRunning, attitude, attitude_lock
    if not isArmed:
        print("Rotation received but ROV not armed!")
        return

    if isRunning:
        print("Another control process is running!")
        return

    # Starting the process
    isRunning = True
    msg = Bool()
    msg.data = True
    run_publisher.publish(msg)

    # ROTATION
    # set the desired operating mode
    depth_hold = 'ALT_HOLD'
    depth_hold_mode = master.mode_mapping()[depth_hold]
    while not master.wait_heartbeat().custom_mode == depth_hold_mode:
        master.set_mode(depth_hold)

    sys_time = (datetime.now() - boot_time).total_seconds() * 1e3
    
    # Get goal orientation
    goal_deg = getDegsFromQuats([rotation.w, rotation.x, rotation.y, rotation.z])

    # Get current orientation
    att = geometry_msgs.msg.Quaternion()
    attitude_lock.acquire()
    att = attitude
    attitude_lock.release()

    # Yaw
    curr_deg = getDegsFromQuats([att.w, att.x, att.y, att.z])
    delt = getDegsDiff(goal_deg[2], curr_deg[2])
    while abs(delt) > angleThreshold:
        if not isArmed or not isRunning:
            break
        if abs(getDegsDiff(goal_deg[2], curr_deg[2])) < angleSize:
            next_deg = goal_deg
        else:
            # Move one segment at a time
            next_deg = curr_deg
            next_deg[2] += delt / abs(delt) * angleSize
            if next_deg[2] < 0:
                next_deg[2] += 360
            elif next_deg[2] >= 360:
                next_deg[2] -= 360

        # set a rotation target
        print("setting rotation")
        sys_time = (datetime.now() - boot_time).total_seconds() * 1e3
        set_target_attitude(master, sys_time, math.radians(next_deg[0]), math.radians(next_deg[1]), math.radians(next_deg[2]))
        attitude_lock.acquire()
        att = attitude
        attitude_lock.release()
        curr_deg = getDegsFromQuats([att.w, att.x, att.y, att.z])
        delt = getDegsDiff(next_deg[2], curr_deg[2])
        while abs(delt) > angleThreshold:
            sys_time = (datetime.now() - boot_time).total_seconds() * 1e3
            set_target_attitude(master, sys_time, math.radians(next_deg[0]), math.radians(next_deg[1]), math.radians(next_deg[2]))
            rospy.sleep(0.3)
            if not isArmed or not isRunning:
                break
            
            attitude_lock.acquire()
            att = attitude
            attitude_lock.release()
            curr_deg = getDegsFromQuats([att.w, att.x, att.y, att.z])
            delt = getDegsDiff(next_deg[2], curr_deg[2])
            # print("Next degs:", next_deg)
            # print("Current degs:", curr_deg)

        if not isArmed or not isRunning:
            break
    
        attitude_lock.acquire()
        att = attitude
        attitude_lock.release()
        curr_deg = getDegsFromQuats([att.w, att.x, att.y, att.z])
        delt = getDegsDiff(goal_deg[2], curr_deg[2])
        print("Angle to goal:", delt)
    
    # for i in range(8):
    #     sys_time = (datetime.now() - boot_time).total_seconds() * 1e3
    #     roll, pitch, yaw = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')
    #     set_target_attitude(master, sys_time, 0, 0, yaw)
    #     # print(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
    #     if not isArmed or not isRunning:
    #         break
    #     rospy.sleep(0.25)

    # Roll & pitch
    # for i in range(8):
    #     sys_time = (datetime.now() - boot_time).total_seconds() * 1e3
    #     roll, pitch, yaw = euler.quat2euler([rotation.w, rotation.x, rotation.y, rotation.z], 'sxyz')
    #     set_target_attitude(master, sys_time, roll, pitch, yaw)
    #     # print(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
    #     if not isArmed or not isRunning:
    #         break
    #     rospy.sleep(0.25)

    if not isArmed:
        print("Robot disarmed, controls will stop")
        return

    # set the desired operating mode
    # hold position after done with path callback
    pos_hold = 'POSHOLD'
    pos_hold_mode = master.mode_mapping()[pos_hold]
    while not master.wait_heartbeat().custom_mode == pos_hold_mode:
        master.set_mode(pos_hold)
    
    # Detect if stopped from successful rotation
    if isRunning:
        print("Rotation completed")
        msg.data = False
        run_publisher.publish(msg)

def angle_size_callback(angle_size):
    """ Maximum angle the ROV will rotate at a time
    :param angle_size: Float64, angle size in degrees"""
    global angleSize
    angleSize = angle_size.data
    print("INFO: Angle size set!")

def angle_threshold_callback(angle_threshold):
    """ How close to the angle goal is acceptable
    :param angle_threshold: Float64, angle threshold in degrees"""
    global angleThreshold
    angleThreshold = angle_threshold.data
    print("INFO: Angle threshold set!")

def attitude_callback(att):
    global attitude, attitude_lock
    attitude_lock.acquire()
    attitude = att
    attitude_lock.release()

def controller():
    global master, run_publisher
    # Create the connection
    master = mavutil.mavlink_connection(rospy.get_param('~mav_addr'), dialect='ardupilotmega')
    # Make sure the connection is valid
    master.wait_heartbeat()

    # Base params
    rospy.Subscriber(rospy.get_param('~arm_topic'), Bool, arm_callback) #true or false 
    rospy.Subscriber(rospy.get_param('~run_topic'), Bool, run_callback)
    run_publisher = rospy.Publisher(rospy.get_param('~run_topic'), Bool, queue_size=1, latch=True)

    # Translation params
    rospy.Subscriber(rospy.get_param('~pcl_topic'), Bool, pcl_callback)
    rospy.Subscriber(rospy.get_param('~focus_topic'), geometry_msgs.msg.PointStamped, focus_callback) # what to focus on visually while you move along path
    rospy.Subscriber(rospy.get_param('~goal_topic'), geometry_msgs.msg.PointStamped, goal_callback) #just oen points
    rospy.Subscriber(rospy.get_param('~goal_path_topic'), visualization_msgs.msg.Marker, path_callback) #the path to follow
    rospy.Subscriber(rospy.get_param('~segment_size_topic'), Float64, segment_size_callback) #how much rov moves every step
    rospy.Subscriber(rospy.get_param('~goal_threshold_topic'), Float64, goal_threshold_callback) #how close you want to be to the goal point

    # Rotation params
    rospy.Subscriber(rospy.get_param('~goal_rotation_topic'), geometry_msgs.msg.Quaternion, rotation_callback)
    rospy.Subscriber(rospy.get_param('~angle_size_topic'), Float64, angle_size_callback) #how much rov rotates every step in degrees
    rospy.Subscriber(rospy.get_param('~angle_threshold_topic'), Float64, angle_threshold_callback) #how close you want to be to the angle goal
    rospy.Subscriber(rospy.get_param('~attitude_topic'), geometry_msgs.msg.Quaternion, attitude_callback)

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

# def path_callback(path):
#     """ Callback function when receiving path to follow.
#         This path will either be generated from pointcloud algorithms (usePCL = True)
#         or passed directly from the goal_callback() below.
#     :param path: visualization_msgs.Marker (LineStrip type, starting from the next position to go to)
#     """
#     global focusPoint
#     if not isArmed:
#         print("Trajectory planned but ROV not armed!")
#         return

#     if not isRunning:
#         print("Trajectory planned but ROV not running!")
#         return

#     # Monitoring position of ROV
#     WORLD_FRAME = rospy.get_param("~world_frame")
#     ROV_FRAME = rospy.get_param("~rov_frame")

#     tfBuffer = tf2_ros.Buffer()
#     listener = tf2_ros.TransformListener(tfBuffer)

#     # set the desired operating mode
#     guided = 'GUIDED'
#     guided_mode = master.mode_mapping()[guided]
#     while not master.wait_heartbeat().custom_mode == guided_mode:
#         master.set_mode(guided_mode)

#     for p in path.points:
#         # Find current location
#         while True:
#             try:
#                 trans = tfBuffer.lookup_transform(WORLD_FRAME, ROV_FRAME, rospy.Time(), rospy.Duration(4))
#             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                 rospy.sleep(0.2)
#                 if isArmed:
#                     continue
#             break
#         if not isArmed or not isRunning:
#             break
#         p_arr = np.array([p.x, p.y, p.z])
#         trans_arr = np.array(
#             [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
#         delt = p_arr - trans_arr
#         mag = np.linalg.norm(delt)

#         while mag > goalThreshold:
#             if not isArmed or not isRunning:
#                 break
#             if mag < segmentSize:
#                 nextPoint = p
#             else:
#                 # Move one meter at a time
#                 tmp_arr = trans_arr + (delt / mag) * segmentSize
#                 nextPoint = geometry_msgs.msg.Point(tmp_arr[0], tmp_arr[1], tmp_arr[2])

#             # TRANSLATION
#             # # set the desired operating mode
#             # guided = 'GUIDED'
#             # guided_mode = master.mode_mapping()[guided]
#             # while not master.wait_heartbeat().custom_mode == guided_mode:
#             #     master.set_mode(guided_mode)
#             # if not isArmed or not isRunning:
#             #     break

#             # set a position target
#             if focusPoint.x == nextPoint.x and focusPoint.y == nextPoint.y:
#                 yaw = 0
#             else:
#                 yaw = math.atan2(focusPoint.y - nextPoint.y, focusPoint.x - nextPoint.x)
#                 if yaw < 0:
#                     yaw += 2 * math.pi
#                 yaw = yaw * RAD_TO_DEG
#             print("setting position")
#             sys_time = (datetime.now() - boot_time).total_seconds() * 1e3
#             set_target_local_position(master, sys_time, nextPoint.x, nextPoint.y, nextPoint.z)
#             np_arr = np.array([nextPoint.x, nextPoint.y, nextPoint.z])
#             delt = np_arr - trans_arr
#             mag = np.linalg.norm(delt)
#             while mag > goalThreshold:
#                 # Find current location
#                 while True:
#                     try:
#                         trans = tfBuffer.lookup_transform(WORLD_FRAME, ROV_FRAME, rospy.Time(), rospy.Duration(4))
#                     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                         rospy.sleep(0.2)
#                         if isArmed:
#                             continue
#                     break
#                 if not isArmed or not isRunning:
#                     break
#                 trans_arr = np.array(
#                     [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
#                 delt = np_arr - trans_arr
#                 mag = np.linalg.norm(delt)
#             # rospy.sleep(5)

#             # ROTATION
#             # # set the desired operating mode
#             # depth_hold = 'ALT_HOLD'
#             # depth_hold_mode = master.mode_mapping()[depth_hold]
#             # while not master.wait_heartbeat().custom_mode == depth_hold_mode:
#             #     master.set_mode(depth_hold)
#             # if not isArmed or not isRunning:
#             #     break
            
#             # set a rotation target
#             print("setting rotation")
#             set_target_yaw(master, yaw)
#             # sys_time = (datetime.now() - boot_time).total_seconds() * 1e3
#             # set_target_attitude(master, sys_time, 0, 90, yaw)
#             rospy.sleep(2.5)
#             if not isArmed or not isRunning:
#                 break

#             # Find current location
#             while True:
#                 try:
#                     trans = tfBuffer.lookup_transform(WORLD_FRAME, ROV_FRAME, rospy.Time(), rospy.Duration(4))
#                 except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                     rospy.sleep(0.2)
#                     if isArmed:
#                         continue
#                 break
#             if not isArmed or not isRunning:
#                 break
#             trans_arr = np.array(
#                 [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
#             delt = p_arr - trans_arr
#             mag = np.linalg.norm(delt)
#             print("Distance to goal:", mag)

#     if not isArmed:
#         print("Robot disarmed, controls will stop")
#         return

#     # set the desired operating mode
#     # hold position after done with path callback
#     pos_hold = 'POSHOLD'
#     pos_hold_mode = master.mode_mapping()[pos_hold]
#     while not master.wait_heartbeat().custom_mode == pos_hold_mode:
#         master.set_mode(pos_hold)