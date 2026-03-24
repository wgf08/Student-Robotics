from utility import *
from movement import *
from actions import *
from info import *

def collect_and_return(robot):
    markers = robot.camera.see()
    sorted_boxes(markers)

