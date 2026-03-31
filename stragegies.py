from utility import *
from movement import *
from actions import *
from info import *

def collect_box(robot,motors, zone):
            markers = robot.camera.see()
            all_samples = sorted_boxes(markers)
            if TARGETED_SAMPLE == "ACIDS":
                targeted = all_samples[1]
                avoids = all_samples[2]
            else:
                targeted = all_samples[2]
                avoids = all_samples[1]




    

