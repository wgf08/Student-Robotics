from movement import *
from info import *
from utility import *
import random

def wall_check(motors, robot):
    """
    Checks if a wall is extremely close, if it is it rotates the robot pi/2
    """
    distance_mm = robot.arduino.ultrasound_measure(9, 10)
    if (distance_mm < 80) and (distance_mm != 0.0) :
        rotate_angle(math.pi, motors,1)


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
            return
        if box.horizontal_angle > math.pi/12:
            move_angle(motors, 0.22, 3*math.pi/2)
            time.sleep(0.03)
        elif box.horizontal_angle < -math.pi/12:
            move_angle(motors, 0.22, math.pi/2)
            time.sleep(0.03)
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
            time.sleep(0.1)
            halt(motors)

def avoid(robot, marker_id, motors):

    direction = math.pi/2

    markers = robot.camera.see()

    if TARGETED_SAMPLE == "ACID":
        targeted = sorted_boxes(markers)[1]
    else:
        targeted = sorted_boxes(markers)[2]
    if not targeted:
        return
    elif targeted[0].horizontal_angle < 0:
        direction = 3* math.pi/2
        
    while True:
        markers = robot.camera.see()
        try:
            box = find_marker(markers, marker_id)
        except:
            return
        if -math.pi/8 < box.horizontal_angle < math.pi/8:
            move_angle(motors, 0.22, direction)
            time.sleep(0.03)
        else:
            break

def return_to_zone(robot, zone, motors):

    markers = robot.camera.see()
    m_id, d, ma = find_position(markers)
    base_ids = ZONE_FIDUCIAL_MARKERS[zone]

    if m_id not in base_ids:
        target_markers = ZONE_FIDUCIAL_MARKERS[zone]
        # Compute di
        # stances going up and down to each marker
        distances_up = [(marker - m_id) % 20 for marker in target_markers]
        distances_down = [(m_id - marker) % 20 for marker in target_markers]
        
        # Find the minimum distance in each direction
        min_up = min(distances_up)
        min_down = min(distances_down)

        if min_up > min_down:
            dir = 'down'
        else:
            dir = 'up'

        if (ma < -math.pi/6 and dir == 'down') or (ma> math.pi/6 and dir == 'up'):
            rotate_angle(math.pi, motors, 1)
        elif (ma < -math.pi/6 and dir == 'up') or (ma> math.pi/6 and dir == 'down') or ( (-math.pi < ma < math.pi) and d>600):
            move_straight(motors,power=1)
        else:
            rotate_angle(math.pi/2, motors, 1)


    else:
        while True:
            markers = robot.camera.see()
            try:
                box = find_marker(markers, m_id)
            except:
                return
            if box.horizontal_angle > math.pi/12:
                move_angle(motors, 0.22, 3*math.pi/2)
                time.sleep(0.03)
            elif box.horizontal_angle < -math.pi/12:
                move_angle(motors, 0.22, math.pi/2)
                time.sleep(0.03)
            else:
                break
        move_straight(motors,1)
        while robot.arduino.ultrasound_measure(9,10):
            continue
        halt(motors)
        move_straight(forwards=False, power=1, duration= 1.5)
    #upon reaching base:

def idle(motors):
    spin(motors, 1, duration=0.5)
    move_straight(motors,0.5,duration=0.3)


    

    
