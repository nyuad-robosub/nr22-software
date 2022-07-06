#!/usr/bin/env python
from coin_flip import coin_flip
import roslaunch
import rospy
import smach
import os
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Bool
import movement_controller 

# state imports
import check_submerged
import coin_flip

# just a class to run ros launch files
#mov_control,orb_slam_3_frame,rov_frame,world_frame,goal_topic,goal_rotation_topic,isRunning_topic,ls= None

class launch_stuff:
    def __init__(self):
        self.launch_dir = os.getcwd() + "/../launch/"

    def launch(self, filename):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        launch = roslaunch.parent.ROSLaunchParent(
            uuid, [self.launch_dir+filename])
        launch.start()

# make a dictionary holding start_run parameters
# params needed: world_frame rov_frame imu_frame orb_slam_3 frame



def main_controller():
    rospy.init_node('smach_controller', anonymous=True)

    # list launch parameters
    global orb_slam_3_frame
    orb_slam_3_frame = rospy.get_param('~orb_slam_3_frame')

    global rov_frame
    rov_frame = rospy.get_param('~rov_frame')

    global world_frame
    world_frame = rospy.get_param('~world_frame')

    global goal_topic
    goal_topic = rospy.get_param('~goal_topic')

    global goal_rotation_topic
    goal_rotation_topic = rospy.get_param('~goal_rotation_topic')

    global isRunning_topic
    isRunning_topic = rospy.get_param('~isrunning_topic')

    # initialize the mavlink_publisher launch file
    # global ls
    # ls = launch_stuff()
    # ls.launch("mavlink.launch")

    # initialize mavproxy
    os.system('mavproxy.py --out 0.0.0.0:14550 --out 0.0.0.0:14551 --out 0.0.0.0:14552 --out 0.0.0.0:14553 --out 0.0.0.0:14554 &> /dev/null &')

    # intialize movement controller
    global mov_control
    mov_control = movement_controller()

    # start state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    with sm:
        smach.StateMachine.add('check_submerged', check_submerged.check_sub(),
                                transitions={'outcome1':'coin_flip'})
        smach.StateMachine.add('coin_flip', coin_flip.coin_flip(),
                                transitions={'outcome1':'outcome4'})

    # create concurrent sm


if __name__ == '__main__':
    main_controller()

