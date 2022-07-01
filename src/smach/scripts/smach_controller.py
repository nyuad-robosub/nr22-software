#!/usr/bin/env python
import roslaunch
import smach
import os

#state imports
from states.check_submerged import *


#just a class to run ros launch files
class launch_stuff:
    def __init__(self):
        self.launch_dir = os.getcwd()+ "/../launch/";

    def launch(self, filename):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        launch = roslaunch.parent.ROSLaunchParent(
        uuid, [self.launch_dir+filename])
        launch.start()

def main():
    #list launch parameters
    

    rospy.init_node('smach_controller', anonymous=True)

    #initialize mavproxy
    os.system('mavproxy.py --out 0.0.0.0:14552 --out 0.0.0.0:14553')

    #initialize the mavlink_publisher launch file
    ls = launch_stuff()
    ls.launch("mavlink.launch")

    #start state machine
    
    #create concurrent sm

if __name__ == '__main__':
    main()
