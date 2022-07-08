#!/usr/bin/env python
import roslaunch
import rospy
import smach
import os
from std_msgs.msg import Bool
# from movement_controller import movement_controller
import movement_controller as mc

import sys
sys.path.insert(0, './states')
from states.check_submerged import check_sub
from states.coin_flip import coin_flip

# just a class to run ros launch files FIX LANCH FILE NO LOGGING NEEDED
class launch_stuff:
    def __init__(self):
        self.launch_dir = os.getcwd() + "/../launch/"

    def launch(self, filename):
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # launch = roslaunch.parent.ROSLaunchParent(
        #     uuid, [self.launch_dir+filename])
        # launch.start()
        os.system("roslaunch mission_planning"+filename+"&> /dev/null &")


#mov_control, rov_frame,world_frame,goal_topic,goal_rotation_topic,isRunning_topic,ls = None
rov_frame=""
world_frame=""
goal_topic=""
goal_rotation_topic=""
isRunning_topic=""
detection_topic=""
detection_label_path=""
oakd_FOV=0
if __name__ == '__main__':
    rospy.init_node('smach_controller', anonymous=True)
    print("EXECUTING SM")
    #os.system('mavproxy.py --out 0.0.0.0:14550 --out 0.0.0.0:14551 --out 0.0.0.0:14552 --out 0.0.0.0:14553 --out 0.0.0.0:14554 &> /dev/null &')
    # list launch parameters
    #global orb_slam_3_frame
    #orb_slam_3_frame = rospy.get_param('~orb_slam_3_frame')
    rov_frame = rospy.get_param('~rov_frame')

    world_frame = rospy.get_param('~world_frame')

    goal_topic = rospy.get_param('~goal_topic')

    goal_rotation_topic = rospy.get_param('~goal_rotation_topic')

    isRunning_topic = rospy.get_param('~isrunning_topic')

    detection_topic = rospy.get_param('~detection_topic')

    detection_label_path = rospy.get_param('~label_file')

    # intialize movement controller
    mc.init(goal_topic, world_frame, rov_frame, isRunning_topic,detection_topic,detection_label_path)
        
    rospy.sleep(1)
    # state imports

    # start state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    with sm:
        smach.StateMachine.add('check_submerged',check_sub(),
                                transitions={'outcome1':'coin_flip'})
        smach.StateMachine.add('coin_flip', coin_flip(),
                                transitions={'outcome1':'outcome4'})
        
    
    print("EXECUTING SM")
   
    outcome = sm.execute()

     
    #sm._set_current_state('coin_flip')
    # create concurrent sm

