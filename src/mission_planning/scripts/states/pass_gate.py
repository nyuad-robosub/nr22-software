from operator import truediv
import rospy
import threading
import smach
import os
from std_msgs.msg import Bool
# import smach_controller_simulation
import movement_controller as mc
import viso_controller as vs
import geometry_msgs.msg
import math
from transforms3d import euler
import vision_msgs

from vision_msgs.msg import Detection2DArray, BoundingBox2D
from sensor_msgs.msg import PointCloud2
import numpy as np

def is_approx_equal(x,y):
    if(x>(y*0.95) and x<(y*1.05)):
        return True
    else:
        return False
        
class coin_flip(smach.State):
    _outcomes=['outcome1','outcome2']
    _input_keys=[],
    _output_keys=[]
    def __init__(self):
        pass
    def execute(self, userdata):      
        vs.viso_control.add_detection_of_interest("gate")
        x_center=320
        y_center=200
        
        return 'outcome2'


    
def estimate_gate_pose(bbox1:BoundingBox2D,bbox2:BoundingBox2D,pc_sub: PointCloud2):
    x_center=bbox1.center.x
    y_center=bbox1.center.y
    
    #decrease the bbox size by half because we might have some areas not in the detection if at an angle or slop
    size_x=bbox1.size_x//2
    size_y=bbox1.size_y//2

    

    #get midpoint between both points
    midpoint=[(bbox1.center.x+bbox1.center.x)/2,bbox1.center.y]

    #get slope 
    slope= (bbox2.center.y-bbox2.center.y)/(bbox2.center.x-bbox2.center.x)

    vectors_3D = np.zeros((3, 2))
    for pt_count, dt in enumerate(pc2.read_points(
                        pc_sub,
                        field_names={'x', 'y', 'z'},
                        skip_nans=False, uvs=[[bbox1.center.x,bbox1.center.x],[bbox2.center.y,bbox2.center.y]] # [u,v u,v u,v] a set of xy coords
                    )):
                vectors_3D[pt_count]=dt
    
    pass
        