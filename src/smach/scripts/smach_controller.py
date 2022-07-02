#!/usr/bin/env python
import roslaunch
import rospy
import smach
import os

# state imports
import check_submerged

# just a class to run ros launch files


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
class movement_controller():
    
    def __init__(self) -> None:
        pass
    
    def rotate_cw(angle) -> None:
        pass

    def go_down() -> None:
        pass

    def get_compass_angle() -> float:
        pass

    def set_precision_mode() -> None:
        pass



def main_controller():
    rospy.init_node('smach_controller', anonymous=True)

    # list launch parameters
    global orb_slam_3_frame 
    orb_slam_3_frame = rospy.get_param('~orb_slam_3_frame')

    global rov_frame 
    rov_fram = rospy.get_param('~rov_frame')

    global world_frame 
    world_frame = rospy.get_param('~world_frame')

    global goal_topic
    goal_topic = rospy.get_param('~goal_topic')
    
    # initialize the mavlink_publisher launch file
    global ls 
    ls = launch_stuff()
    ls.launch("mavlink.launch")

    # initialize mavproxy
    os.system('mavproxy.py --out 0.0.0.0:14552 --out 0.0.0.0:14553')

    # start state machine

    # create concurrent sm


if __name__ == '__main__':
    mc = main_controller()

