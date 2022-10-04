#!/usr/bin/env python2
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
from operator import pos
import time
# Import mavutil
#import pymavlink.quaternion
#import transforms3d.quaternions
from pymavlink import mavutil
from std_msgs.msg import Bool

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
    # print("Sending VPD: ", position, rotation)
    # print(rotation[2])
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
import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
from transforms3d import euler, quaternions, affines
from datetime import datetime
import threading 

boot_time = datetime.now()
last_time = datetime.now()
mutex = threading.Lock()
isLost = False

# Flag for publisher for velocities
PUBLISH_VELOCITES = True
# Flag for non-MAVLink testing
USE_MAVLINK = False
# Flag for simulation
IS_SIM = False

# Message conversion courtesy of:
# https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
# import tf.transformations as tr
def transform_to_pq(msg):
    """Convert a C{geometry_msgs/Transform} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = np.array([msg.translation.x.real, msg.translation.y.real, msg.translation.z.real])
    q = np.array([msg.rotation.w.real, msg.rotation.x.real,
                  msg.rotation.y.real, msg.rotation.z.real])
    return p, q
def transform_stamped_to_pq(msg):
    """Convert a C{geometry_msgs/TransformStamped} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    return transform_to_pq(msg.transform)
def msg_to_se3(msg):
    """Conversion from geometric ROS messages into SE(3)

    @param msg: Message to transform. Acceptable types - C{geometry_msgs/Pose}, C{geometry_msgs/PoseStamped},
    C{geometry_msgs/Transform}, or C{geometry_msgs/TransformStamped}
    @return: a 4x4 SE(3) matrix as a numpy array
    @note: Throws TypeError if we receive an incorrect type.
    """
    if isinstance(msg, geometry_msgs.msg.TransformStamped):
        p, q = transform_stamped_to_pq(msg)
    else:
        raise TypeError("Invalid type for conversion to SE(3)")
    norm = np.linalg.norm(q)
    if np.abs(norm - 1.0) > 1e-3:
        raise ValueError(
            "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                str(q), np.linalg.norm(q)))
    elif np.abs(norm - 1.0) > 1e-6:
        q = q / norm
    # g = tr.quaternion_matrix(q)
    g = quaternions.quat2mat(q)
    tmp = np.eye(4)
    tmp[:3, :3] = g
    tmp[:-1, -1] = p
    # g[0:3, -1] = p
    # print(tmp)
    # print("yes")
    return tmp

initCount = 3
last_pos = [0.0, 0.0, 0.0]
last_rot = euler.euler2mat(0, 0, 0)
last_trans = np.eye(4)
last_delt_pos = [0.0, 0.0, 0.0]
last_delt_rot = [0.0, 0.0, 0.0]
def processTransform(master, p_trans):
    """ Receive data and send to MAVLink
    :param master: FCU to send to
    """
    global last_time, last_rot, last_pos, last_trans, initCount, isSim
    global last_delt_pos, last_delt_rot
    global WORLD_FRAME, vel_publisher

    if initCount > 0:
        last_trans = msg_to_se3(p_trans)
        initCount -= 1
        return

    curr_time = datetime.now()
    delt_time = (curr_time - last_time).total_seconds() * 1e6
    sys_time = (curr_time - boot_time).total_seconds() * 1e6

    curr_trans = msg_to_se3(p_trans)

    delt_trans = np.linalg.inv(last_trans).dot(curr_trans)

    delt_pos_vec, delt_rot_mat, z_vec, s_vec = affines.decompose44(delt_trans)
    delt_pos = list(delt_pos_vec)
    ai, aj, ak = euler.mat2euler(delt_rot_mat)
    delt_rot = [ai, aj, ak]
    # print(delt_rot)

    # Change to NED framey
    delt_pos[1] = -delt_pos[1]
    delt_pos[2] = -delt_pos[2]
    delt_rot[1] = -delt_rot[1]
    delt_rot[2] = -delt_rot[2]
    if delt_rot[2] > math.pi:
        delt_rot[2] -= 2 * math.pi
    elif delt_rot[2] < -math.pi:
        delt_rot[2] += 2 * math.pi

    # Check for issues in the SLAM data
    if not np.allclose(delt_pos, [0.0, 0.0, 0.0], atol=1e-8) and not np.allclose(delt_pos, last_delt_pos, atol=1e-8):
        last_time = curr_time
        last_trans = curr_trans
        last_delt_pos = delt_pos
        last_delt_rot = delt_rot
        # Check for large jumps
        if np.linalg.norm(delt_pos) < 4:
            # Check if using MAVLink
            if USE_MAVLINK:
                send_vision(master, sys_time, delt_time, delt_pos, delt_rot, 90)

            # Check if publish calculated velocities
            if PUBLISH_VELOCITES:
                msg = geometry_msgs.msg.TwistStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = WORLD_FRAME
                msg.twist.linear = geometry_msgs.msg.Vector3(delt_pos[0], delt_pos[1], delt_pos[2])
                msg.twist.angular = geometry_msgs.msg.Vector3(delt_rot[0], delt_rot[1], delt_rot[2])
                vel_publisher.publish(msg)

def lost_callback(isl):
    global isLost
    mutex.acquire()
    isLost=isl
    mutex.release()

def receiver():
    global isLost, IS_SIM, USE_MAVLINK, PUBLISH_VELOCITES
    global WORLD_FRAME, vel_publisher
    # Check if is in simulation
    IS_SIM = rospy.get_param('~is_sim')

    # Check if using MAVLink
    master = None
    if USE_MAVLINK:
        # Create the connection
        master = mavutil.mavlink_connection(rospy.get_param('~mav_addr'), dialect='ardupilotmega')

        # Make sure the connection is valid
        master.wait_heartbeat()

        # Check if need to set home
        if (rospy.get_param('~set_home')):
            cmd_set_home(master, rospy.get_param('~home_lat') * 1e7, rospy.get_param('~home_long') * 1e7, rospy.get_param('~home_alt'))

    WORLD_FRAME = rospy.get_param("~world_frame")
    ROV_FRAME = rospy.get_param("~rov_frame")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # lost_listener = rospy.Subscriber('/orb_slam_3_lost', Bool, lost_callback)

    # Check if publish calculated velocities
    if PUBLISH_VELOCITES:
        vel_publisher = rospy.Publisher('viso_velocities', geometry_msgs.msg.TwistStamped, queue_size=5)

    rate = rospy.Rate(15.0)
    max_timeout = 10
    timeout_count = 0
    while not rospy.is_shutdown():
        # If in simulation: simple send
        if IS_SIM:
            try:
                trans = tfBuffer.lookup_transform(WORLD_FRAME, ROV_FRAME, rospy.Time(), rospy.Duration(4))
                processTransform(master, trans)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            except AttributeError:
                if USE_MAVLINK:
                    # Make sure the connection is valid
                    print("Fail to connect to MAVLink") # master.wait_heartbeat()
                    print("Retrying...")
                    # Create the connection
                    master = mavutil.mavlink_connection(rospy.get_param('~mav_addr'), dialect='ardupilotmega')
                    # Make sure the connection is valid
                    master.wait_heartbeat()

        # If not is in simulation: add checks
        else:
            try:
                trans = tfBuffer.lookup_transform(WORLD_FRAME, ROV_FRAME, rospy.Time(), rospy.Duration(4))
                processTransform(master, trans)
                # mutex.acquire()
                # if (not isLost):
                # mutex.release()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            except AttributeError:
                if USE_MAVLINK:
                    # Make sure the connection is valid
                    print("Fail to connect to MAVLink") # master.wait_heartbeat()
                    print("Retrying...")
                    # rospy.sleep(5)
                    # Create the connection
                    master = mavutil.mavlink_connection(rospy.get_param('~mav_addr'), dialect='ardupilotmega')
                    # Make sure the connection is valid
                    master.wait_heartbeat()

        if rospy.is_shutdown():
            return
        rate.sleep()

    # rospy.Subscriber("chatter", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__== '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('send_viso_tf2', anonymous=True)

    # Run receiver
    receiver()

# ---------------------------------------------
#   Code bank area
# ---------------------------------------------

# last_pos = [0.0, 0.0, 0.0]
# last_rot = [0.0, 0.0, 0.0]
# def processTransform(master, p_trans):
#     """ Receive data and send to MAVLink
#     :param master: FCU to send to
#     """
#     global last_time, last_rot, last_pos
#     curr_time = datetime.now()
#     delt_time = (curr_time - last_time).total_seconds() * 1e6
#     sys_time = (curr_time - boot_time).total_seconds() * 1e6
#     last_time = curr_time
#
#     curr_pos = [p_trans.transform.translation.x.real,
#                 p_trans.transform.translation.y.real,
#                 p_trans.transform.translation.z.real]
#
#     ai, aj, ak = euler.quat2euler([
#                 p_trans.transform.rotation.w.real,
#                 p_trans.transform.rotation.x.real,
#                 p_trans.transform.rotation.y.real,
#                 p_trans.transform.rotation.z.real
#                ])
#
#     curr_rot = [ai, aj, ak]
#     # curr_rot = p_trans.transform.rotation
#
#     delt_pos = list(map(float.__sub__, curr_pos, last_pos))
#     delt_rot = list(map(float.__sub__, curr_rot, last_rot))
#     # delt_rot = [0.0, 0.0, 0.0]
#     # delt_rot[0], delt_rot[1], delt_rot[2] = euler.quat2euler(
#     #     [curr_rot.x.real - last_rot.x.real,
#     #      curr_rot.y.real - last_rot.y.real,
#     #      curr_rot.z.real - last_rot.z.real,
#     #      curr_rot.w.real - last_rot.w.real])
#
#     # Change to NED framey
#     delt_pos[1] = -delt_pos[1]
#     delt_pos[2] = -delt_pos[2]
#     delt_rot[1] = -delt_rot[1]
#     delt_rot[2] = -delt_rot[2]
#     if delt_rot[2] > math.pi:
#         delt_rot[2] -= 2 * math.pi
#     elif delt_rot[2] < -math.pi:
#         delt_rot[2] += 2 * math.pi
#     # delt_rot[2] = delt_rot[2] % (math.pi * 2)
#
#     send_vision(master, sys_time, delt_time, delt_pos, delt_rot, 100)
#     last_pos = curr_pos
#     last_rot = curr_rot

# last_pos = [0.0, 0.0, 0.0]
# last_rot = euler.euler2mat(0, 0, 0)
# def processTransform(master, p_trans):
#     """ Receive data and send to MAVLink
#     :param master: FCU to send to
#     """
#     global last_time, last_rot, last_pos
#     curr_time = datetime.now()
#     delt_time = (curr_time - last_time).total_seconds() * 1e6
#     sys_time = (curr_time - boot_time).total_seconds() * 1e6
#     last_time = curr_time
#
#     curr_pos = [p_trans.transform.translation.x.real,
#                 p_trans.transform.translation.y.real,
#                 p_trans.transform.translation.z.real]
#
#     curr_rot = quaternions.quat2mat(np.array([
#         p_trans.transform.rotation.w.real,
#         p_trans.transform.rotation.x.real,
#         p_trans.transform.rotation.y.real,
#         p_trans.transform.rotation.z.real]))
#
#     delt_pos = list(map(float.__sub__, curr_pos, last_pos))
#     # Rotate to the ROV frame
#     delt_pos = list(last_rot.dot(np.array(delt_pos)))
#     delt_rot_mat = curr_rot.dot(np.linalg.inv(last_rot))
#     ai, aj, ak = euler.mat2euler(delt_rot_mat)
#     delt_rot = [ai, aj, ak]
#     # print(delt_rot)
#
#     # Change to NED framey
#     delt_pos[1] = -delt_pos[1]
#     delt_pos[2] = -delt_pos[2]
#     delt_rot[1] = -delt_rot[1]
#     delt_rot[2] = -delt_rot[2]
#     if delt_rot[2] > math.pi:
#         delt_rot[2] -= 2 * math.pi
#     elif delt_rot[2] < -math.pi:
#         delt_rot[2] += 2 * math.pi
#
#     send_vision(master, sys_time, delt_time, delt_pos, delt_rot, 70)
#     last_pos = curr_pos
#     last_rot = curr_rot