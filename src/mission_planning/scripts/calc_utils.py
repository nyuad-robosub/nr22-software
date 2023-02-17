import math
import numpy as np
from transforms3d import euler
def is_approx_equal(x, y, margin=0.1):
    if abs(x - y) / math.sqrt(x ** 2 + y ** 2) < margin:# (x>(y*(1-margin)) and x<(y*(1+margin))):
        return True
    else:
        return False

def getDegsDiff(a, b): # get smallest angle from a to b
    diff = a - b
    if diff <= -math.pi:
        diff += 2*math.pi
    elif diff > math.pi:
        diff -= 2*math.pi
    return diff


def pose_to_np(pose_obj):
    position_data=np.array([pose_obj.position.x,pose_obj.position.y,pose_obj.position.z])
    return position_data

def pose_get_yaw(pose_obj):
    roll, pitch, yaw = euler.quat2euler([pose_obj.orientation.w, pose_obj.orientation.x, pose_obj.orientation.y, pose_obj.orientation.z], 'sxyz')
    return yaw