from utility import *
from movement import *
from info import *
import random

def wall_check(motors, robot):
    """
    Checks if a wall is extremely close, if it is it rotates the robot pi/2
    """
    distance_mm = robot.arduino.ultrasound_measure(2,3)
    if (distance_mm < 850) and (distance_mm != 0.0) :
        print(f'wall check {distance_mm}')
        move_straight(motors,-1)


def consume(robot, marker_id, motors):
    """
        Moves forward in order to collect a box on the ground given the id of the box
    """

    #Align Robot
    while True:
        markers = robot.camera.see()
        try:
            box = find_marker(markers, marker_id)
        except:
            return None 
        if box.horizontal_angle > math.pi/12:
            rotate_angle(motors, 0.22, 3*math.pi/2)
            robot.sleep(0.03)
        elif box.horizontal_angle < -math.pi/12:
            rotate_angle(motors, 0.22, math.pi/2)
            robot.sleep(0.03)
        else:
            break
    
    #Move Forward for at least estimate time
    duration = ((box.distance+13))/(80*math.pi) + 0.3
    move_straight(motors,1,forwards=True, duration=duration)
    move_straight(motors,0.2)

    #Continue until Box can no longer be seen
    while True:
        markers = robot.camera.see()
        try:
            box = find_marker(markers, marker_id)
        except:
            robot.sleep(0.1)
            halt()

def avoid(robot, marker_id, motors):

    direction = math.pi/2

    markers = robot.camera.see()

    if TARGETED_SAMPLE == "ACID":
        targeted = sorted_boxes(markers)[1]
    else:
        targeted = sorted_boxes(markers)[2]
    if not targeted:
        pass
    elif targeted[0].horizontal_angle < 0:
        direction = 3* math.pi/2
        
    while True:
        markers = robot.camera.see()
        try:
            box = find_marker(markers, marker_id)
        except:
            return None
        if -math.pi/8 < box.horizontal_angle < math.pi/8:
            rotate_angle(motors, 0.22, direction)
            robot.sleep(0.03)
        else:
            break

import math
import random
import time

# Distance threshold for considering a corner
CORNER_DIST_THRESHOLD = 1100  # adjust as needed
ANGLE_CORNER_THRESHOLD = 0.25  # radians, must be roughly straight-on

def return_to_zone(robot, zone, motors):
    wall_check(motors, robot)
    markers = robot.camera.see()

    try:
        m_id, d, ma = find_position(markers)
        print(f"Seeing marker: {m_id}, Distance: {d:.1f}, Angle: {ma:.2f} rad")
    except:
        print("No marker detected!")

        # Random chance to reverse 1 second
        if random.random() < 0.3:  # 30% chance
            print("Random reverse triggered for 1 second")
            move_straight(motors, -0.5)
            robot.sleep(1)
            halt(motors)

        # Rotate 45 degrees to search
        print("Rotating 45° to search for marker")
        rotate_angle(robot, math.pi / 1.5, motors, 1)
        return

    base_ids = ZONE_FIDUCIAL_MARKERS[zone]

    # Define middle markers on each side
    middle_markers = [2, 7, 12, 17]
    is_corner_marker = m_id not in middle_markers

    if m_id not in base_ids:
        target_markers = ZONE_FIDUCIAL_MARKERS[zone]

        # Distances clockwise and counter-clockwise to all target markers
        distances_cw = [(marker - m_id) % 20 for marker in target_markers]
        distances_ccw = [(m_id - marker) % 20 for marker in target_markers]

        min_cw = min(distances_cw)
        min_ccw = min(distances_ccw)

        direction = 'cw' if min_cw <= min_ccw else 'ccw'

        # Orientation adjustments
        ANGLE_THRESHOLD = 0.25  # radians
        if (ma < -ANGLE_THRESHOLD and direction == 'ccw') or (ma > ANGLE_THRESHOLD and direction == 'cw'):
            print("Rotating 180° to correct large misalignment")
            rotate_angle(robot, math.pi*1.3, motors, 1)
        elif abs(ma) <= ANGLE_THRESHOLD and d < 1500:
            print(f"Moving straight towards marker {m_id}, distance {d:.1f}, angle {ma:.2f}")
            move_straight(motors, power=0.5)
        else:
            print(f"Minor adjustment, moving slowly, angle {ma:.2f}, distance {d:.1f}")
            move_straight(motors, power=0.3)

        # Check if at a corner (any marker not the middle, roughly straight-on)
        at_turning_point = abs(ma) <= ANGLE_CORNER_THRESHOLD and d < CORNER_DIST_THRESHOLD

        if at_turning_point:
            print(f"At turning_point marker {m_id}, distance {d:.1f}, turning {direction}")
            angle = math.pi / 2 if direction == 'cw' else -math.pi / 2
            rotate_angle(robot, angle*1.8, motors, 1)
            robot.sleep(0.5)
            halt(motors)
            robot.sleep(0.5)
            return


    else:
        # Robot has reached the target zone
        halt(motors)
        robot.sleep(3)
        print(f"Reached target zone at marker {m_id}, distance {d:.1f}, angle {ma:.2f}")

        CENTERING_ANGLE_THRESHOLD = 0.2  # radians

        # First, center the marker in view
        if abs(ma) > CENTERING_ANGLE_THRESHOLD:
            print(f"Centering on marker {m_id}, angle {ma:.2f}")
            rotate_angle(robot, ma, motors, 1)
            halt(motors)
            robot.sleep(0.1)
            return  # re-evaluate after centering

        # Marker is centered → move forward
        move_straight(motors, 0.2)

        # Stop when near the wall
        while True:
            dist = robot.arduino.ultrasound_measure(2,3)
            print(f"Distance to wall: {dist:.1f}")
            if dist < 500:
                break
            robot.sleep(0.05)

        halt(motors)
        move_straight(motors, -0.5)
        robot.sleep(0.2)
        halt(motors)



def idle(motors):
    move_straight(motors,0.5,duration=0.3)


    

    
