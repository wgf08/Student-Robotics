from sr.robot3 import Robot
import time
robot = Robot()

def find_markers():
    cords = []
    markers = robot.camera.see()
    for marker in markers:
        id = marker.id
        dist = marker.position.distance
        horz_a = marker.position.horizontal_angle
        vert_a = marker.position.vertical_angle
        info = str(id) + ";" + str(dist) + ";" + str(horz_a) + ";" + str(vert_a)
        return info
