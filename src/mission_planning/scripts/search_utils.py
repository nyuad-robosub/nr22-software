from os import wait3
import viso_controller as vs
import movement_controller as mc
from geometry_msgs.msg import Point
import math
import numpy as np
import rospy

# --------------------------------------
# SEARCHES
# --------------------------------------

# General parameters:
#   - start_point: where the search is/was started                      | geometry_msgs.msg.Point
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
def search_waiter(mov_control,
                  detection_camera,
                  detection_labels,
                  detection_count,
                  min_score_thresh,
                  min_area_norm,
                  detections_dict,
                  end_time,
                  wait_function = None):
    while mov_control.get_running_confirmation():
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
                mc.mov_control.stop()
                return "detected"

        # Run wait_function if exists
        if wait_function is not None:
            if not wait_function():
                mc.mov_control.stop()
                return "preempted"

        # Return if time is finished
        if rospy.get_time() > end_time:
            mc.mov_control.stop()
            return "timeout"
    return "notfound"

# Primer search: initializing the start location for other searches while also looking for objects
def primer_search(mov_control,
                 start_point,
                 direction,
                 detection_camera,
                 detection_labels,
                 detection_count,
                 min_score_thresh,
                 min_area_norm,
                 detections_dict,
                 end_time,
                 wait_function = None):
    
    # Arm & update transformation
    mov_control.arm()
    mov_control.update_tf()
    translation = mov_control.trans.transform.translation

    # Rotate to appointed yaw
    if not np.isclose(direction, mov_control.euler[2], atol=0.01):
        print("Rotating to direction")
        mov_control.set_rotation(0, 0, direction)
        rospy.sleep(0.1)
        outcome = search_waiter(mov_control, detection_camera, detection_labels, detection_count,
                                min_score_thresh, min_area_norm, detections_dict, end_time, wait_function)
        if outcome != "notfound": return outcome
        rospy.sleep(0.1)

    # Move to appointed xyz
    if not np.allclose([translation.x, translation.y, translation.z], [start_point.x, start_point.y, start_point.z], atol=0.1):
        print("Moving to start point")
        mov_control.set_focus_point()
        mov_control.set_goal_point((start_point.x, start_point.y, start_point.z))
        rospy.sleep(0.1)
        outcome = search_waiter(mov_control, detection_camera, detection_labels, detection_count,
                                min_score_thresh, min_area_norm, detections_dict, end_time, wait_function)
        if outcome != "notfound": return outcome
    
    return "notfound"

# Forward search
def forward_search(mov_control,                             # Movement controller                   | mc.movement_controller
                   origin_point,                            # Where the line of search starts       | geometry_msgs.msg.Point
                   direction,                               # Yaw angle of line                     | float
                   timeout,                                 # Timeout duration                      | float
                   detection_camera,                        # Camera object for detection           | vs.viso_controller
                   detection_labels,                        # Labels of objects                     | list of strings
                   detection_count,                         # Number of detections acceptable       | int
                   min_score_thresh,                        # Minimum confidence                    | float
                   min_area_norm,                           # Minimum area of bounding box          | float
                   wait_function = None):                   # Wait function to run                  | boolean function

    # Get dictionary of detections courtesy of: Rami
    detections_dict = {}
    for label in detection_labels:
        detections_dict[label] = []    
    if len(detection_labels) == 0:
        return detections_dict

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
    mov_control.go_straight(100)
    rospy.sleep(0.1)
    outcome = search_waiter(mov_control, detection_camera, detection_labels, detection_count,
                            min_score_thresh, min_area_norm, detections_dict, end_time, wait_function)
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
                    wait_function = None):                  # Wait function to run                  | boolean function

    # Get dictionary of detections courtesy of: Rami
    detections_dict = {}
    for label in detection_labels:
        detections_dict[label] = []    
    if len(detection_labels) == 0:
        return detections_dict

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
    outcome = search_waiter(mov_control, detection_camera, detection_labels, detection_count,
                            min_score_thresh, min_area_norm, detections_dict, end_time, wait_function)
    return outcome, detections_dict