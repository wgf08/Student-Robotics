from movement import *

def lock_on(robot, motors):
    markers = robot.camera.see()

    if not markers:
        return False

    # Closest marker is usually best
    marker = min(markers, key=lambda m: m.distance)

    theta = marker.horizontal_angle

    # Move directly in that direction
    move_angle(motors, 1, theta)
    robot.sleep(3)
    halt(motors)

    return True