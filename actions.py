from movement import *
from info import *
from utility import *
import random
import time

def wall_check(motors, robot):
    """
    Checks if a wall is extremely close, if it is it rotates the robot pi/2
    """
    distance_mm = robot.arduino.ultrasound_measure(9, 10)
    if (distance_mm < 80) and (distance_mm != 0.0) :
        rotate_angle(math.pi, motors,1)


def consume(robot, marker_id, motors, method = 'indirect'):
    """
        Moves forward in order to collect a box on the ground given the id of the box
    """
    if method == 'indirect':
    #Align Robot
        while True:
            markers = robot.camera.see()
            try:
                box = find_marker(markers, marker_id)
                print('got box')
                print(box)
                y = sample_xyz(box)[0]
                print(y)
            except Exception as e:
                print(f'returned could not see box {e}')
                halt(motors)
                continue
                return 0
            if y > 150:
                move_angle(motors, 0.5, math.pi/2)
                time.sleep(0.03)
            elif y < -150:
                move_angle(motors, 0.5, 3*math.pi/2)
                time.sleep(0.03)
            else:
                break
        
        #Move Forward for at least estimate time
        x = sample_xyz(box)[1]
        duration = convert_dist_time(x)
        move_angle(motors,1,box.position.horizontal_angle)
        robot.sleep(duration)
        move_angle(motors,0.5,box.position.horiztonal_angle)

        #Continue until Box can no longer be seen
        while True:
            markers = robot.camera.see()
            try:
                box = find_marker(markers, marker_id)
            except:
                time.sleep(0.1)
                halt(motors)
                break
        return 1

    elif method == 'direct-ws':
        while True:
            try:
                box = find_marker(markers, marker_id)
                x, y, z = sample_xyz(box)
            except:
                return 0
            move_angle(motors, 1, box.position.horizontal_angle)
            if box.position.distance > 500:
                time.sleep(0.8)
                halt(motors)
                time.sleep(0.1)
            else:
                time.sleep(convert_dist_time(box.position.distance, 1))
                halt(motors)
                return 1 
    else:
        while True:
            try:
                box = find_marker(markers, marker_id)
                x, y, z = sample_xyz(box)
            except:
                halt(motors)
                try:
                    box = find_marker(markers, marker_id)
                    x, y, z = sample_xyz(box)
                    continue
                except:
                    return 0
            move_angle(motors, 0.5, box.position.horizontal_angle)
            if box.position.distance > 500:
                time.sleep(0.8)
            else:
                time.sleep(convert_dist_time(box.position.distance, 0.5))
                halt(motors)
                return 1 
        
                
    

def avoid(robot, marker_id, motors):

    while True:
        markers = robot.camera.see()
        try:
            box = find_marker(markers, marker_id)
            y = sample_xyz(box)[0]
        except:
            halt(motors)
            markers = robot.camera.see()
            if not markers:
                return
            else:
                continue
        print(y)
        if -450 < y < 450:
            move_angle(motors, 0.72, 3*math.pi/2 if y >= 0 else math.pi/2)
            time.sleep(0.03)
        else:
            continue

def return_to_zone(robot, zone, motors, direction = 'cw'):

    distance_mm = robot.arduino.ultrasound_measure(2,3)
    print(robot.arduino.ultrasound_measure(2,3))
    if distance_mm <350 and distance_mm!=0:
       move_straight(motors, power=-0.25)
       robot.sleep(0.5)
       rotate_angle(robot,0.4,motors,1)
       return (0.1, direction)


    base_ids = ZONE_FIDUCIAL_MARKERS[zone]
    YAW_RANGE = 0.7

    try:
        print('101')
        markers = sorted_boxes(robot.camera.see())[3]
        print('103')
        m = markers[0]
        x, y ,z = wall_xyz(m)
        print(y, m.id)
        a = set([m.id for m in sorted_boxes(robot.camera.see())[3]]) & set(base_ids)
        y2 = math.inf
        if a:
            y2 = wall_xyz(a)[0]
        print('108')
        if (not a) or (abs(y2) > 1100):
            print(not a)
            print((abs(y))> 1000)
            if m.id % 5 == 1 and m.position.distance > 1600:
                for box in markers: 
                    print('114')
                    if box.id%5 == 0: m = box
            target_markers = ZONE_FIDUCIAL_MARKERS[zone]
            distances_cw = [(marker - m.id) % 20 for marker in target_markers]
            distances_ccw = [(m.id - marker) % 20 for marker in target_markers]


            min_cw = min(distances_cw)
            min_ccw = min(distances_ccw)
            direction = 'cw' if min_cw <= min_ccw else 'ccw'
            curr_dir = 'cw' if m.orientation.yaw < 0 else 'acw'

            if curr_dir != direction and x < 100:
                print(curr_dir, direction, m, m.orientation.yaw)
                rotate_angle(robot,math.pi,motors,1)
                return (0.1,direction)

            angle = ((math.pi)/2+m.orientation.yaw)*2.5 if direction == 'cw' else ((math.pi)+m.orientation.yaw)*2.5

            if a:
                rotate_angle(robot,math.pi/1.3,motors,1,ac = True)
                return (0.2, direction)
            elif x < 900 or (x < 1600 and abs(m.orientation.yaw) > YAW_RANGE):
                print(f'1 {direction}')
                rotate_angle(robot,angle,motors,1,ac = False)
                move_straight(motors,0.4)
                return (0.8,direction)
            elif x>1600 and m.id%5 == 1 or 2 :
                print('3')
                rotate_angle(robot,m.position.horizontal_angle*1.4,motors, 1, m.position.horizontal_angle<0)
                move_straight(motors,0.4)
                return (0.4,direction)
            elif x > 1600 and abs(m.orientation.yaw) < YAW_RANGE:
                print('2 {x, m.id}')
                rotate_angle(robot,m.position.horizontal_angle*1.3,motors, 1, m.position.horizontal_angle<0)
                move_straight(motors,0.4)
                return (0.4,direction)
            else:
                print(f'Condition 4 executed. Stats \n M_ID = {m.id}   X: {x}   YAW: {m.orientation.yaw}')
                move_straight(motors,0.4)
                return (0.2,direction)
                

        else:
            
            print(y)
            distance_mm = robot.arduino.ultrasound_measure(2,3)
            marker = find_marker(robot.camera.see(), m.id)
            while marker is not None and marker.position.distance > 400:
                marker.position.distance
                marker = find_marker(robot.camera.see(), m.id)
                move_straight(motors,0.2)
            while distance_mm > 800 or distance_mm == 0:
                    distance_mm = robot.arduino.ultrasound_measure(2,3)
                    move_straight(motors,0.3)
                    continue
            

            halt(motors)
            move_straight(motors, power=-0.5)
            return (0.1, direction)

    except Exception as e:
        print(e)
        print('FAILURE - NOTHING SEEN')
        rand = random.random()
        if rand<=0.3:
            move_straight(motors, power=-0.2)
            robot.sleep(0.5)
            print('moving back?')
        rotate_angle(robot,math.pi/1.2,motors,1,'cw' if direction == 'acw' else 'acw' )
        return (0.1,direction)

def execute_timed_function(func, time_to_run):
    start_time = start_timer()
    curr_time = time.time()

    while curr_time - start_time > time_to_run:
        output = func()
        if output == 'Completed': return 1
        else: curr_time = time.time()
    return 0 

def return_loop(robot, zone, motors, direction = 'cw'):
    direction = 'cw'
    while True:
        rest, direction = return_to_zone(robot,2,motors,direction)
        robot.sleep(rest)
        halt(motors)
        robot.sleep(0.05)

def idle(motors):
    spin(motors, 1, duration=0.5)
    move_straight(motors,0.5,duration=0.3)


    

    
