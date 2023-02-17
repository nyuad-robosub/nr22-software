# Class for keeping track of progress
import json
from geometry_msgs.msg import Point 

class progress_tracker():
    def __init__(self):
        # Initialize here
        self.progress = {
            'check_submerged': {
                'done': False,
                'submerged': False
            },
            'coin_flip': {
                'done': False,
                'rotation_percent': 0.0,
                'rotation_direction': False,
                'start_yaw': 0.0,
                'detected_gate': False,
                'gate_center_yaw': 0.0
            },
            'pass_gate': {
                'done': False,
                'chosen_detection': "" #g_man or bootlegger
            },
            'bin_analyze': {
                'done': False,
                'num_lid_detected': 0, # number of lids detected, 6 including 2 lids and 4 images
                'lid_positions': [Point(), Point()], #initial lid detected
                'lid_types': ['unknown', 'unknown'], # 'gman', 'bootlegger'
                'num_img_detected': 0,
                'img_positions': [Point(), Point(), Point(), Point()], #initial lid detected
                'img_labels': ['unknown', 'unknown', 'unknown', 'unknown']
            }
        }
        self.last_known_position = Point(0, 0, 0)
    
    def get_first_unreached(self):
        """ Find the first (in dictionary) unreached state
        """
        for key in self.progress.keys():
            if not self.progress[key]['done']:
                return key
    
    def __str__(self):
        """ Print the dictionary
        """
        return json.dumps(self.progress, indent=4)

def init():
    global pr_track
    pr_track = progress_tracker()