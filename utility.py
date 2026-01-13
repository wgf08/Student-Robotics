import math
import time
from sr.robot3 import Robot
from movement import *
import sys

def sorted_boxes(robot):
    """
    Returns a list of boxes sorted by type and then distance 
    """
    pass

def find_marker(markers, marker_id):
    for marker in markers:
        if marker.id == marker_id:
            return marker
    return FileExistsError

def consume(robot, marker_id, motors):
    markers = robot.camera.see()

    while True:
        try:
            box = find_marker(markers, marker_id)
        except FileExistsError:
            print("Wouldn't you like to know... weatherboy!")
            sys.quit()
        if box.horizontal_angle > math.pi/12:
            spin(motors, 1, False)
            time.sleep(0.03)
        elif box.horizontal_angle < -math.pi/12:
            spin(motors, 1, True)
            time.sleep(0.03)
        else:
            break
    duration = (3*(box.distance+13))/(80*math.pi)
    move_straight(motors,1,forwards=True, duration=duration)
    move_straight(motors,1)
    while True:
        try:
            box = find_marker(markers, marker_id)
        except FileExistsError:
            time.sleep(0.5)
            halt()


