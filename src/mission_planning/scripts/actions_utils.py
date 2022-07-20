""" General actions utils file """
# import viso_controller as vs
# import movement_controller as mc
from geometry_msgs.msg import Point, Vector3
import tf2_geometry_msgs.tf2_geometry_msgs as tfgmtfgm
import math
import numpy as np
import rospy

# --------------------------------------
# SEARCHES
# --------------------------------------

# General parameters:
#   - origin_point: where the search is/was started                     | geometry_msgs.msg.Point
#   - direction: yaw angle to direct the search                         | float
#   - timeout: duration to search, in seconds                           | float
#   - detection_camera: camera object used to do search                 | viso_controller.camera
#   - detection_labels: which objects to detect & return on             | list of strings
#   - detection_count: how many detections until confirmation           | int
#   - min_score_thresh: confidence threshold of search                  | float
#   - min_area_norm: min area of bounding box to accept (normalized)    | float
#   - wait_function: boolean function that is run when search is free   | optional boolean function with no args
#       + should return True when search should continue

# Return tuple:
#   - outcome:
#       + "detected": found object to search for                        | process-ending outcome
#       + "preempted": stopped by external function                     | process-ending outcome
#       + "timeout": running over the time limit                        | process-ending outcome
#       + "notfound": finished without detecting anything               | continuable outcome
#   - detections_dict: dict of label-keyed lists of detections with timestamps

# Search waiter: do the necessary wait for the searches
def search_waiter(mov_control,                              # Movement controller                   | mc.movement_controller
                  detection_camera,                         # Camera object for detection           | vs.viso_controller
                  detection_labels,                         # Labels of objects                     | list of strings
                  detection_count,                          # Number of detections acceptable       | int
                  min_score_thresh,                         # Minimum confidence                    | float
                  min_area_norm,                            # Minimum area of bounding box          | float
                  detections_dict,                          # Detection dictionary to write to      | dictionary of lists of detections
                  end_time,                                 # When to timeout                       | float
                  confirm_function,                         # Confirm function to maintain loop     | boolean function
                  wait_function = None):                    # Wait function to preempt              | boolean function
                  
    # Loop while confirm_function is True
    while confirm_function():
        # Check if new detections contain the wanted objects
        if detection_camera.is_fetched:
            curr_time = rospy.Time.now()
            detections = detection_camera.get_detection(detection_labels)
            for d in detections:
                # Filter area of bounding box or confidence
                if d['size'][0] * d['size'][1] < detection_camera.width * detection_camera.height * min_area_norm:
                    print("Detected bounding box too small, ignoring...")
                    continue
                elif d['score'] < min_score_thresh:
                    print('Detected object but confidence too low, ignoring...')
                    continue
                detections_dict[d['label']].append(d)
        
        # Return if number of objects found exceeds requirements
        for label in detection_labels:
            if len(detections_dict[label]) >= detection_count:
                mov_control.stop()
                return "detected"

        # Run wait_function if exists
        if wait_function is not None:
            if not wait_function():
                mov_control.stop()
                return "preempted"

        # Return if time is finished
        if rospy.get_time() > end_time:
            mov_control.stop()
            return "timeout"
    mov_control.stop()
    return "notfound"

# Primer search: initializing the start location for other searches while also looking for objects
def primer_search(mov_control,                              # Movement controller                   | mc.movement_controller
                  origin_point,                             # Where to move to                      | geometry_msgs.msg.Point             
                  direction,                                # Orientation                           | float
                  detection_camera,                         # Camera object for detection           | vs.viso_controller
                  detection_labels,                         # Labels of objects                     | list of strings
                  detection_count,                          # Number of detections acceptable       | int
                  min_score_thresh,                         # Minimum confidence                    | float
                  min_area_norm,                            # Minimum area of bounding box          | float
                  detections_dict,                          # Detection dictionary to write to      | dictionary of lists of detections
                  end_time,                                 # When to timeout the search            | float
                  wait_function = None):                    # Wait function to preempt              | boolean function
    
    # Arm & update transformation
    mov_control.arm()
    mov_control.update_tf()
    translation = mov_control.trans.transform.translation
    print("Moving to initial search pose")

    # Rotate to appointed yaw
    if not np.isclose(direction, mov_control.euler[2], atol=0.01):
        mov_control.set_rotation(0, 0, direction)
        rospy.sleep(0.1)
        outcome = search_waiter(mov_control, detection_camera, detection_labels, detection_count, min_score_thresh, 
                                min_area_norm, detections_dict, end_time, mov_control.get_running_confirmation, wait_function)
        if outcome != "notfound": return outcome
        rospy.sleep(0.1)

    # Move to appointed xyz
    if not np.allclose([translation.x, translation.y, translation.z], [origin_point.x, origin_point.y, origin_point.z], atol=0.1):
        mov_control.set_focus_point()
        mov_control.set_goal_point((origin_point.x, origin_point.y, origin_point.z))
        rospy.sleep(0.1)
        outcome = search_waiter(mov_control, detection_camera, detection_labels, detection_count, min_score_thresh, 
                                min_area_norm, detections_dict, end_time, mov_control.get_running_confirmation, wait_function)
        if outcome != "notfound": return outcome
    
    return "notfound"

# Forward search
def forward_search(mov_control,                             # Movement controller                   | mc.movement_controller
                   origin_point,                            # Where the line of search starts       | geometry_msgs.msg.Point
                   distance,                                # How far the search should go          | float
                   direction,                               # Yaw angle of line                     | float
                   timeout,                                 # Timeout duration                      | float
                   detection_camera,                        # Camera object for detection           | vs.viso_controller
                   detection_labels,                        # Labels of objects                     | list of strings
                   detection_count,                         # Number of detections acceptable       | int
                   min_score_thresh,                        # Minimum confidence                    | float
                   min_area_norm,                           # Minimum area of bounding box          | float
                   wait_function = None):                   # Wait function to preempt              | boolean function

    # Get dictionary of detections courtesy of: Rami
    detections_dict = {}
    for label in detection_labels:
        detections_dict[label] = []    
    if len(detection_labels) == 0:
        return "notfound", detections_dict

    # Get end time
    end_time = rospy.get_time() + timeout
    # Check if can start the mission right now
    mov_control.update_tf()
    translation = mov_control.trans.transform.translation

    # Check angle proximity
    primer_needed = not np.isclose(mov_control.euler[2], direction, atol=0.01)
    target_point = Point(origin_point.x, origin_point.y, origin_point.z)

    # Check if point is near starting point on horizontal plane
    if not np.allclose([translation.x, translation.y], [origin_point.x, origin_point.y], atol=0.15):
        # Check if point is close enough to line
        distance_from_start = math.sqrt((translation.x - origin_point.x) ** 2 + (translation.y - origin_point.y) ** 2)
        shortest_distance = distance_from_start * math.sin(direction - math.atan2(translation.y - origin_point.y, translation.x - origin_point.x))
        if abs(shortest_distance) < 0.21:
            # Check if z is near target z
            if not np.isclose(translation.z, origin_point.z, atol=0.1):
                # Moving to z
                primer_needed = True
                target_point = Point(translation.x, translation.y, origin_point.z)
        
        # Else move to closest point on line (projection from current position onto line)
        else:
            primer_needed = True
            target_point.x = translation.x + shortest_distance * math.cos(math.pi / 2 + direction)
            target_point.y = translation.y + shortest_distance * math.sin(math.pi / 2 + direction)
            target_point.z = translation.z
            # Check if z is near target z
            if not np.isclose(translation.z, origin_point.z, atol=0.1): 
                target_point.z = origin_point.z
    
    # Check if z is near target z
    else:
        if not np.isclose(translation.z, origin_point.z, atol=0.1):
            # Moving to z
            primer_needed = True
            target_point = Point(translation.x, translation.y, origin_point.z)
            
    # Start primer search if needed
    if primer_needed:
        # Moving to xyz
        outcome = primer_search(mov_control, target_point, direction, detection_camera,
                                detection_labels, detection_count, min_score_thresh, min_area_norm, detections_dict, end_time, wait_function)
        if outcome != "notfound": return outcome, detections_dict  
    
    # Start real search
    mov_control.go_straight(distance)
    rospy.sleep(0.1)
    outcome = search_waiter(mov_control, detection_camera, detection_labels, detection_count, min_score_thresh, 
                            min_area_norm, detections_dict, end_time, mov_control.get_running_confirmation, wait_function)
    return outcome, detections_dict

# Circular search
def circular_search(mov_control,                            # Movement controller                   | mc.movement_controller
                    origin_point,                           # Center of search circle               | geometry_msgs.msg.Point
                    radius,                                 # Radius of search circle               | float
                    segment_count,                          # Circle subdivisions count             | int
                    direction,                              # Yaw angle of line                     | float
                    timeout,                                # Timeout duration                      | float
                    detection_camera,                       # Camera object for detection           | vs.viso_controller
                    detection_labels,                       # Labels of objects                     | list of strings
                    detection_count,                        # Number of detections acceptable       | int
                    min_score_thresh,                       # Minimum confidence                    | float
                    min_area_norm,                          # Minimum area of bounding box          | float
                    wait_function = None):                  # Wait function to preempt              | boolean function

    # Get dictionary of detections
    detections_dict = {}
    for label in detection_labels:
        detections_dict[label] = []    
    if len(detection_labels) == 0:
        return "notfound", detections_dict

    # Get end time
    end_time = rospy.get_time() + timeout
    # Check if can start the mission right now
    mov_control.update_tf()
    translation = mov_control.trans.transform.translation

    # Check angle proximity
    if not np.isclose(mov_control.euler[2], direction, atol=0.01) or not np.isclose(translation.z, origin_point.z, atol=0.1):
        # Rotate to direction
        outcome = primer_search(mov_control, Point(translation.x, translation.y, translation.z), direction, detection_camera,
                                detection_labels, detection_count, min_score_thresh, min_area_norm, detections_dict, end_time, wait_function)
        if outcome != "notfound": return outcome, detections_dict  
    
    # Set focus point
    mov_control.set_focus_point()

    # Create circular path
    points = []
    yaw_base = direction if np.allclose([translation.x, translation.y], [origin_point.x, origin_point.y]) else \
               math.atan2(translation.y - origin_point.y, translation.x - origin_point.x)
    for i in range(segment_count):
        points.append(Point(
            origin_point.x + radius * math.cos(yaw_base + i * 2 * math.pi / segment_count),
            origin_point.y + radius * math.sin(yaw_base + i * 2 * math.pi / segment_count),
            origin_point.z
        ))
    
    # Start real search
    mov_control.set_goal_points(points)
    rospy.sleep(0.1)
    outcome = search_waiter(mov_control, detection_camera, detection_labels, detection_count, min_score_thresh, 
                            min_area_norm, detections_dict, end_time, mov_control.get_running_confirmation, wait_function)
    return outcome, detections_dict

# --------------------------------------
# ACTIONS
# --------------------------------------

# Helper functions
# EMA classes courtesy of:
# https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
class ema_float:
    def __init__(self, value, alpha):
        self.value = value
        self.alpha = alpha
    def update(self, value):
        self.value = value * self.alpha + (1 - self.alpha) * self.value
class ema_point:
    def __init__(self, point, alpha):
        self.x = ema_float(point.x, alpha)
        self.y = ema_float(point.y, alpha)
        self.z = ema_float(point.z, alpha)
    def update(self, point):
        self.x.update(point.x)
        self.y.update(point.y)
        self.z.update(point.z)
    def value(self):
        return Point(self.x.value, self.y.value, self.z.value)

# Get location of bottom objects from sonar_ping and downwards camera
def get_downwards_object_position():
    pass

# Bottom camera centering search & positioning
# Presuming the object is already (confidently) visible
# Will utilize: sonar_ping, bottom camera transform, etc.
# Return tuple:
#   - outcome
#       + "preempted": stopped by external function                     | process-ending outcome
#       + "timeout": running over the time limit                        | process-ending outcome
#       + "notfound": finished without detecting anything               | process-ending outcome
#       + "failed": failed to move the center                           | process-ending outcome
#       + "succeeded": succeeded to move the center                     | process-ending outcome
def bottom_aligning(mov_control,                            # Movement controller                   | mc.movement_controller
                    goal,                                   # Where to move the 3Dcenter of bbox to | geometry_msgs.msg.Point !! z will be ignored, will use current ROV z !!
                    tolerance,                              # Center proximity tolerance (norm)     | float
                    timeout,                                # Timeout duration                      | float
                    detection_camera,                       # Camera object for detection           | vs.viso_controller
                    detection_label,                        # Labels of objects                     | list of strings
                    min_score_thresh,                       # Minimum confidence                    | float
                    min_area_norm,                          # Minimum area of bounding box          | float
                    wait_function = None):                  # Wait function to preempt              | boolean function
    
    # Get dictionary of detections
    detections_dict = {}
    detections_dict[detection_label] = []  

    # Get end time
    end_time = rospy.get_time() + timeout
    
    # Stop all controls
    mov_control.stop()
    rospy.sleep(2) # Wait a bit for stabilization
    # Get a detection of the object
    outcome = search_waiter(mov_control, detection_camera, [detection_label], 1, min_score_thresh, 
                            min_area_norm, detections_dict, end_time, lambda: True, wait_function)
    if outcome != "detected": return outcome, None
    
    # Get relevant tfs
    mov_control.update_tf()
    detection_camera.update_tf()
    translation = mov_control.trans.transform.translation
    cam_translation = detection_camera.trans.transform.translation

    # Get vector from camera to bbox center
    center = detections_dict[detection_label][-1]['center']
    angle_x, angle_y = detection_camera.get_angles(center[0], center[1])
    center_vec_cam = Vector3(math.tan(angle_x), math.tan(angle_y), -1)
    center_vec_world = tfgmtfgm.do_transform_vector3(center_vec_cam, detection_camera.trans)
    length = np.linalg.norm([center_vec_world.vector.x, center_vec_world.vector.y, center_vec_world.vector.z])

    # Presume pool to be flat at small region
    delt_z = translation.z - mov_control.sonar_ping - cam_translation.z
    delt_x = delt_z * center_vec_world.vector.x / length
    delt_y = delt_z * center_vec_world.vector.y / length
    position = ema_point(Point(cam_translation.x + delt_x,
                               cam_translation.y + delt_y,
                               cam_translation.z + delt_z), 0.5)

    # Move the robot
    mov_control.set_focus_point()
    mov_control.set_goal_point((position.value().x, position.value().y, translation.z))

    # Loop until arrived at the postion
    while not np.allclose([position.value().x, position.value().y], [goal.x, goal.y], atol=tolerance):
        while mov_control.get_running_confirmation():
            # Run wait_function if exists
            if wait_function is not None:
                if not wait_function():
                    mov_control.stop()
                    return "preempted"

            # Return if time is finished
            if rospy.get_time() > end_time:
                mov_control.stop()
                return "timeout"

        # Wait a bit for stabilization
        rospy.sleep(2) 
        # Get a detection of the object
        outcome = search_waiter(mov_control, detection_camera, [detection_label], 1, min_score_thresh, 
                                min_area_norm, detections_dict, end_time, lambda: True, wait_function)
        if outcome != "detected": return outcome, None
        
        # Get relevant tfs
        mov_control.update_tf()
        detection_camera.update_tf()
        translation = mov_control.trans.transform.translation
        cam_translation = detection_camera.trans.transform.translation

        # Get vector from camera to bbox center
        center = detections_dict[detection_label][-1]['center']
        angle_x, angle_y = detection_camera.get_angles(center[0], center[1])
        center_vec_cam = Vector3(math.tan(angle_x), math.tan(angle_y), -1)
        center_vec_world = tfgmtfgm.do_transform_vector3(center_vec_cam, detection_camera.trans)
        length = np.linalg.norm([center_vec_world.vector.x, center_vec_world.vector.y, center_vec_world.vector.z])

        # Presume pool to be flat at small region
        delt_z = translation.z - mov_control.sonar_ping - cam_translation.z
        delt_x = delt_z * center_vec_world.vector.x / length
        delt_y = delt_z * center_vec_world.vector.y / length
        position.update(Point(cam_translation.x, cam_translation.y, cam_translation.z), 0.5)

        # Move the robot
        mov_control.set_focus_point()
        mov_control.set_goal_point((position.value().x, position.value().y, translation.z))

        # Run wait_function if exists
        if wait_function is not None:
            if not wait_function():
                mov_control.stop()
                return "preempted"

        # Return if time is finished
        if rospy.get_time() > end_time:
            mov_control.stop()
            return "timeout"