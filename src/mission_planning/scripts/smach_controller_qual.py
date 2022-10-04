#!/usr/bin/env python
import roslaunch
import rospy
import smach
import os
from std_msgs.msg import Bool
# from movement_controller import movement_controller
import movement_controller as mc
import viso_controller as vs
import progress_tracker as pt

import sys
sys.path.insert(0, './states')

from states.check_submerged import check_sub
from states.pass_gate_qual_simple import pass_gate_qual_simple

# just a class to run ros launch files FIX LANCH FILE NO LOGGING NEEDED
class launch_stuff:
    def __init__(self):
        self.launch_dir = os.getcwd() + "/../launch/"

    def launch(self, filename):
        os.system("roslaunch mission_planning"+filename+"&> /dev/null &")

#mov_control, rov_frame,world_frame,goal_topic,goal_rotation_topic,isRunning_topic,ls = None
rov_frame=""
world_frame=""
goal_topic=""
goal_rotation_topic=""
isRunning_topic=""
detection_topic=""
detection_label_path=""
camera_frame=""
oakd_HFOV=50.41
if __name__ == '__main__':
    rospy.init_node('smach_controller', anonymous=True)
    print("EXECUTING SM")
    #os.system('mavproxy.py --out 0.0.0.0:14550 --out 0.0.0.0:14551 --out 0.0.0.0:14552 --out 0.0.0.0:14553 --out 0.0.0.0:14554 &> /dev/null &')
    #os.system('sim_vehicle.py -f gazebo-bluerov2 -v ArduSub -L Home --out=udp:0.0.0.0:14550 --out=udp:0.0.0.0:14551 --out=udp:0.0.0.0:14552 --out=udp:0.0.0.0:14553 --out=udp:0.0.0.0:14554 --out=udp:0.0.0.0:14555 --console &> /dev/null &')
    # list launch parameters
    #global orb_slam_3_frame
    rov_frame = rospy.get_param('~rov_frame')

    world_frame = rospy.get_param('~world_frame')

    goal_topic = rospy.get_param('~goal_topic')

    goal_rotation_topic = rospy.get_param('~goal_rotation_topic')

    isRunning_topic = rospy.get_param('~isrunning_topic')

    front_detection_topic = rospy.get_param('~front_detection_topic')

    detection_label_path = rospy.get_param('~front_label_file')

    front_camera_frame = rospy.get_param('~front_camera_frame')

    pcl_topic = rospy.get_param('~pcl_topic')

    bottom_camera_topic = rospy.get_param('~bottom_camera_topic')

    #oakd_HFOV = rospy.get_param('~oakd_HFOV')
    

    # intialize movement controller
    mc.init(goal_topic, world_frame, rov_frame, isRunning_topic)

    rospy.sleep(5)
    # start state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    with sm:
        smach.StateMachine.add('check_submerged',check_sub(),
                                transitions={'outcome1':'pass_gate_qual'})
        smach.StateMachine.add('pass_gate_qual', pass_gate_qual_simple(),
                                transitions={'outcome1':'outcome4',
                                            'outcome2':'pass_gate_qual'})
    print("EXECUTING SM")
    
    outcome = sm.execute()

     
    #sm._set_current_state('coin_flip')
    # create concurrent sm
