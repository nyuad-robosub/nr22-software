#!/usr/bin/env python
import roslaunch
import rospy
import smach
import os
from std_msgs.msg import Bool
# from movement_controller import movement_controller
import movement_controller as mc

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


#mov_control, rov_frame,world_frame,goal_topic,goal_rotation_topic,isRunning_topic,ls = None
rov_frame=""
world_frame=""
goal_topic=""
goal_rotation_topic=""
isRunning_topic=""
if __name__ == '__main__':
    rospy.init_node('smach_controller', anonymous=True)
    print("EXECUTING SM")

    # list launch parameters
    #global orb_slam_3_frame
    #orb_slam_3_frame = rospy.get_param('~orb_slam_3_frame')
    rov_frame = rospy.get_param('~rov_frame')

    world_frame = rospy.get_param('~world_frame')

    goal_topic = rospy.get_param('~goal_topic')

    goal_rotation_topic = rospy.get_param('~goal_rotation_topic')

    isRunning_topic = rospy.get_param('~isrunning_topic')

    # intialize movement controller
    # mov_control=movement_controller(goal_topic,world_frame,rov_frame,isRunning_topic) #not global variable error
    mc.init(goal_topic, world_frame, rov_frame, isRunning_topic)
        
    rospy.sleep(1)
    # state imports
    import check_submerged
    import coin_flip
    # start state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    with sm:
        smach.StateMachine.add('check_submerged', check_submerged.check_sub(),
                                transitions={'outcome1':'coin_flip'})
        smach.StateMachine.add('coin_flip', coin_flip.coin_flip(),
                                transitions={'outcome1':'outcome4'})

    print("EXECUTING SM")
    outcome = sm.execute()
    # create concurrent sm

