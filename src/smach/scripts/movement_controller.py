
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
        QuaternionBase([roll, pitch, yaw]), # math.radians(angle) for angle in (roll, pitch, yaw)
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

#from smach_controller_simulation import goal_topic,isRunning_topic,world_frame,rov_frame
from concurrent.futures import thread
from shutil import move
import rospy 
import math
import numpy as np
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Bool
from transforms3d import euler, quaternions
from datetime import datetime

class movement_controller():
    # controlerr/goalRotation
    # controller/isRunning
    def __init__(self,goal_topic,world_frame,rov_frame,isRunning_topic):
        self.world_frame=world_frame
        self.rov_frame=rov_frame
        self.isRunning_topic=isRunning_topic
        self.goal_topic=goal_topic

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.goal_publisher = rospy.Publisher(goal_topic, geometry_msgs.msg.PointStamped, queue_size=5)
        self.goal_rotation_publisher = rospy.Publisher('controller/goalRotation', geometry_msgs.msg.Quaternion, queue_size=1, latch=True)
        self.running_subscriber = rospy.Subscriber(
            isRunning_topic, Bool, self.running_callback)

        # Have two flags to keep track of the past
        self.wasRunning = False
        self.isRunning = False

    def update_tf(self, timeout=5):
        """Function to update stored tf of ROV
        :param timeout: How much time in seconds lookup should wait for"""
        delay = 0.1
        counter = 0
        # Find current location
        while counter < timeout / delay:
            try:
                self.trans = self.tfBuffer.lookup_transform(self.world_frame, self.rov_frame, rospy.Time(), rospy.Duration(4))
                self.target_point = geometry_msgs.msg.PointStamped()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(delay)
                counter += 1
            break
        # Check if there were any issues
        return (counter < timeout / delay)

        # self.trans = self.tfBuffer.lookup_transform(
        #     self.world_frame, self.rov_frame, rospy.Time(), rospy.Duration(4))
        # self.target_point = geometry_msgs.msg.PointStamped()

    def get_tf(self):
        self.update_tf()
        return self.trans

    def rotate_cw(angle):
        pass

    def running_callback(self, flag):
        self.isRunning = flag.data

    def get_running_confirmation(self):
        # If current flag is the same as new flag:
        # - Either the controller has not registered the movement, or
        # - Function called when there is no change in running status
        # -> wait a bit to discern
        if self.wasRunning == self.isRunning:
            rospy.sleep(1)
            if self.wasRunning == self.isRunning:
                # No change
                return self.wasRunning
        
        # Else there is change
        self.wasRunning = self.isRunning
        return self.wasRunning

    def go_down(self, amount):
        self.target_point.header.frame_id = self.world_frame
        self.target_point.header.stamp = rospy.Time.now()
        self.target_point.point.x = self.trans.transform.translation.x
        self.target_point.point.y = self.trans.transform.translation.y
        self.target_point.point.z = self.trans.transform.translation.z - amount
        self.goal_publisher.publish(self.target_point)

    def get_compass_angle():
        pass

    def set_precision_mode():
        pass

# Global variable courtesy of:
# https://stackoverflow.com/a/13034908
def init(goal_topic, world_frame, rov_frame, isRunning_topic):
    global mov_control
    mov_control = movement_controller(goal_topic, world_frame, rov_frame, isRunning_topic)
