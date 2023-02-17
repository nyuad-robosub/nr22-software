import rospy
import smach
# import smach_controller_simulation
import movement_controller as mc
import viso_controller as vs
import actions_utils as au
import progress_tracker as pr
from geometry_msgs.msg import Point
import math
from transforms3d import euler

class bin_analyze(smach.State):
    # _outcomes = ['outcome1','outcome2']
    # _input_keys = []
    # _output_keys = []

    def __init__(self, name):
        smach.State.__init__(self, outcomes=['found_bins','notfound_bins'])
        self.pr_dict = pr.pr_track.progress[name]
        self.lid_types = ['gman', 'bootlegger']
        self.lid_emas = [au.ema_point(Point(), 0.6), au.ema_point(Point(), 0.6)]
        self.img_labels = ['Collecting_Bins_Gman_Notepad', 'Collecting_Bins_Gman_Phone', 'Collecting_Bins_Gman_Alc', 'Collecting_Bins_Gman_Bin']
        self.img_emas = [au.ema_point(Point(), 0.6), au.ema_point(Point(), 0.6), au.ema_point(Point(), 0.6), au.ema_point(Point(), 0.6)]
        self.search_labels = self.img_labels + ['lid_empty']
        # First 2 in lid type 1, second 2 in lid type 2

    def register_objects(self):
        return True
        # Get initial tf
        mc.mov_control.update_tf()
        translation = mc.mov_control.trans.transform.translation
        current_ping = mc.mov_control.sonar_ping

        # Stealth get detections
        is_fetched = vs.bottom_camera.is_fetched
        detections = vs.bottom_camera.get_detection(self.search_labels)
        if is_fetched:
            vs.bottom_camera.is_fetched = True

        # Check if detection is recent
        curr_time = rospy.Time.now()
        for d in detections:
            if (curr_time - d['time']) > rospy.Duration(nsecs=2e8):
                return True
        
        # Sort out lids
        lids = []
        for d in detections:
            if d['label'] == 'lid_empty':
                lids.append(
                    vs.bottom_camera.pixel_ray_plane_intersect(
                        d['center'],
                        (0, 0, translation.z - current_ping),
                        (0, 0, 1)
                    )    
                )   
        # if len(lids) == 1:
        #     if self.pr_dict['num_lid_detected'] == 0:
                
        #     elif self.pr_dict['num_lid_detected'] == 1:

        #     else:

        return True

    def execute(self, userdata):
        # Conduct linear search to find the bin
        mc.mov_control.update_tf()
        trans = mc.mov_control.trans.transform.translation
        direction = math.pi / 24
        mc.mov_control.set_focus_point((-129, 12, -129))
        outcome, detection_dict = au.forward_search(
            mc.mov_control, Point(trans.x, trans.y, trans.z - mc.mov_control.sonar_ping + 1.6), 20, mc.mov_control.euler[2] + 1.4, 120,
            # mc.mov_control, Point(trans.x + 2, trans.y + 10, trans.z - mc.mov_control.sonar_ping + 1.6), 100, direction + math.radians(120), 120,
            vs.bottom_camera, self.search_labels, 2, 0.35, 0.0001
        )
        # outcome, detection_dict = au.forward_search(
        #     mc.mov_control, Point(trans.x, trans.y, trans.z - mc.mov_control.sonar_ping + 1.6), 100, direction + math.radians(120), 120,
        #     vs.bottom_camera, ['lid_empty'], 2, 0.5, 0.0001
        # )

        # Checking outcomes
        print("Search finsihed with outcome: %s" % outcome)
        if outcome == "detected":
            # Prioritize lids
            if len(detection_dict['lid_empty']) > 0:
                # Add initial lid position estimate
                mc.mov_control.update_tf()
                position = vs.bottom_camera.pixel_ray_plane_intersect(
                    detection_dict['lid_empty'][-1]['center'],
                    (0, 0, mc.mov_control.trans.transform.translation.z - mc.mov_control.sonar_ping),
                    (0, 0, 1)
                )
                # Focus on the best lid
                self.pr_dict['num_lid_detected'] = 1
                self.pr_dict['lid_positions'][0] = position

                # Align the lid to get best results
                mc.mov_control.set_focus_point((25, 210, -129)) # add some bs focus
                outcome, point = au.bottom_aligning(
                    mc.mov_control, 0.2, 20,
                    vs.bottom_camera, 'lid_empty', 0.35, 0.001
                )
                print("Centering finished with outcome: %s" % outcome)
                rospy.sleep(5)

                # Conduct a circle search around detected lid
                if point is not None:
                    self.pr_dict['lid_positions'][0] = point
                    mc.mov_control.set_focus_point((82, -210, 2329))
                    point.z = trans.z - mc.mov_control.sonar_ping + 1.6
                    # Circular search with 100% confidence to eliminate all
                    outcome, detection_dict = au.circular_search(
                        mc.mov_control, point, 0.3, 12, mc.mov_control.euler[2], 120,
                        vs.bottom_camera, ['qual_gate'], 2, 1, 1, self.register_objects
                    )
                    print("Circle search finished with outcome: %s" % outcome)

                # Check if lid was a false flag
                elif outcome == "notfound":
                    self.pr_dict['num_lid_detected'] = 0
                    self.pr_dict['lid_positions'][0] = Point()
                    return 'notfound_bins'
                
                # Return with success
                self.pr_dict['done'] = True
                print(self.pr_dict)
                return "found_bins"
            
            else:
                pass
        
        # Check if bins were detected
        else:
            return 'notfound_bins'