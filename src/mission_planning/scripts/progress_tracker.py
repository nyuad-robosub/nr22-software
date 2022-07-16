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
            }
        }
        self.last_known_position = Point(0, 0, 0)
    
    def get_first_unreached(self):
        """ Find the first (in dictionary) unreached state
        """
        for key in self.progress.keys():
            if not self.progress[key]['done']:
                return key
    
    def __str__(self) -> str:
        """ Print the dictionary
        """
        return json.dumps(self.progress, indent=4)

def init():
    global pr_track
    pr_track = progress_tracker()