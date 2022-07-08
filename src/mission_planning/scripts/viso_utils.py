import math
import depthai
import rospy
class viso_utils():
    #oakd_fov=
    #oak1_mxid=""
    def __init__(self,oakd_Hfov,width,height) -> None: #should be passed in radians
        aspect_ratio = width/height
        oakd_Vfov= 2 * math.atan(math.tan(oakd_Hfov/2)*aspect_ratio)
        #50.41 VFOV 
        pass
        #calibData = dai_device.readCalibration()
         #print(f"RGB FOV {calibData.getFov(dai.CameraBoardSocket.RGB)}, Mono FOV {calibData.getFov(dai.CameraBoardSocket.LEFT)}")

# Global variable courtesy of:
# https://stackoverflow.com/a/13034908
def init(goal_topic, world_frame, rov_frame, isRunning_topic):
    #global mov_control
    pass
    #mov_control = movement_controller(goal_topic, world_frame, rov_frame, isRunning_topic)
