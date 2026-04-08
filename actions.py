from movement import *
from info import *
from utility import *
import random
import time
from servo import whip

from async_movement import (
    init_queue, clear_queue, stay_vigilant, 
    add_T_update, add_R_update, 
    t_powerconverter, r_powerconverter
)

# ─────────────────────────────────────────────────────────────────────────────
# Navigation & Safety
# ─────────────────────────────────────────────────────────────────────────────

def wall_check(motors, robot):
    """
    Checks if a wall is extremely close, if it is it rotates the robot pi/2
    """
    distance_mm = robot.arduino.ultrasound_measure(9, 10)
    if (distance_mm < 80) and (distance_mm != 0.0) :
        rotate_angle(motors, math.pi/2, 1, True)


def consume(robot, marker_id, motors, method = 'indirect'):
    """
        Moves forward in order to collect a box on the ground given the id of the box
        INDIRECT - DEFAULT - Moves horizontally first until the box is within a central range. Known to work
        DIRECT-WS - NON DEFAULT - Moves at the current angle of the box, sotpping requently to adjust this angle, unreliable.
        OTHER - moves slowly towards target, unreliable.
    """

    if method == 'indirect':
    #Align Robot
        MAX_ALIGN_TRIES = 60  # ~1.8s at 30ms per loop before giving up
        align_tries = 0
        box = None
        while align_tries < MAX_ALIGN_TRIES:
            markers = robot.camera.see()
            box = find_marker(markers, marker_id)
            if box is None:
                halt(motors)
                align_tries += 1
                continue
            try:
                y = sample_xyz(box)[0]
                print(y)
            except Exception as e:
                print(f'returned could not see box {e}')
                halt(motors)
                align_tries += 1
                continue
            if y > 150:
                move_angle(motors, 0.5, math.pi/2)
                time.sleep(0.03)
            elif y < -150:
                move_angle(motors, 0.5, 3*math.pi/2)
                time.sleep(0.03)
            else:
                break
            align_tries += 1

        if box is None:
            halt(motors)
            return 0

        #Move Forward using accelerometer-assisted distance estimate
        x = sample_xyz(box)[1]
        move_distance(motors, 1, box.position.horizontal_angle, x, robot)
        move_angle(motors, 0.5, box.position.horizontal_angle)

        #Continue until Box can no longer be seen — use None check, not exception
        while True:
            markers = robot.camera.see()
            box = find_marker(markers, marker_id)
            if box is None:
                time.sleep(0.1)
                halt(motors)
                break
        return 1

    elif method == 'direct-ws':
        MAX_APPROACH_TRIES = 5
        for _ in range(MAX_APPROACH_TRIES):
            markers = robot.camera.see()
            box = find_marker(markers, marker_id)
            if box is None:
                return 0
            move_distance(motors, 1, box.position.horizontal_angle,
                          box.position.distance, robot)
            # Re-check — if the box is gone after the move, we collected it
            markers = robot.camera.see()
            if find_marker(markers, marker_id) is None:
                return 1
            # Still visible; loop to re-approach from updated position
        halt(motors)
        return 0
    else:
        while True:
            markers = robot.camera.see()
            box = find_marker(markers, marker_id)
            if box is None:
                halt(motors)
                # One retry
                markers = robot.camera.see()
                box = find_marker(markers, marker_id)
                if box is None:
                    return 0
            move_distance(motors, 0.5, box.position.horizontal_angle,
                          box.position.distance, robot)
            markers = robot.camera.see()
            if find_marker(markers, marker_id) is None:
                return 1
        
                
    

def avoid(robot, marker_id, motors):
    markers = robot.camera.see()
    box = find_marker(markers, marker_id)
    if not box: return

    x, y, _ = sample_xyz(box)
    side = 1 if x < 0 else -1
    # Convert mm to meters and split total distance (box distance + 500mm buffer) into 4 phases
    phase_dist = (y + 500) / 4000.0 
    
    init_queue()
    timeline = 0.0
    # The normal distribution path: Carve Out, Align, Merge In, Reset
    for r_val in [side * 0.125, -side * 0.125, -side * 0.125, side * 0.125]:
        dt, _ = t_powerconverter(0, phase_dist)
        dr, _ = r_powerconverter(r_val)
        
        add_T_update(0, phase_dist, timeline)
        add_R_update(r_val, timeline)
        timeline += max(dt, dr)

    end_time = time.time() + timeline + 0.1
    while time.time() < end_time:
        stay_vigilant(motors)
        time.sleep(0.01)
    clear_queue(motors)

def return_to_zone(robot, zone, motors, direction = 'cw'):

    distance_mm = robot.arduino.ultrasound_measure(2,3)
    print(robot.arduino.ultrasound_measure(2,3))
    if distance_mm <350 and distance_mm!=0:
       move_straight(motors, power=-0.25)
       robot.sleep(0.5)
       rotate_angle(motors, 0.4, 1, True)
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
            # Find the actual marker object for the closest matching wall marker
            all_wall_markers = sorted_boxes(robot.camera.see())[3]
            a_marker = next((wm for wm in all_wall_markers if wm.id in a), None)
            if a_marker is not None:
                y2 = wall_xyz(a_marker)[0]
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
            direction = 'cw' if min_cw <= min_ccw else 'acw'
            curr_dir = 'cw' if m.orientation.yaw < 0 else 'acw'

            if curr_dir != direction and x < 100:
                print(curr_dir, direction, m, m.orientation.yaw)
                rotate_angle(motors, math.pi, 1, True)
                return (0.1,direction)

            angle = ((math.pi)/2+m.orientation.yaw)*2.5 if direction == 'cw' else ((math.pi)+m.orientation.yaw)*2.5

            if a:
                rotate_angle(motors, math.pi/1.3, 1, False)
                return (0.2, direction)
            elif x < 900 or (x < 1600 and abs(m.orientation.yaw) > YAW_RANGE):
                print(f'1 {direction}')
                rotate_angle(motors, angle, 1, True)
                move_straight(motors,0.4)
                return (0.8,direction)
            elif x>1600 and (m.id%5 == 1 or m.id%5 == 2):
                print('3')
                rotate_angle(motors, abs(m.position.horizontal_angle)*1.4, 1, m.position.horizontal_angle > 0)
                move_straight(motors,0.4)
                return (0.4,direction)
            elif x > 1600 and abs(m.orientation.yaw) < YAW_RANGE:
                print(f'2 {x} {m.id}')
                rotate_angle(motors, abs(m.position.horizontal_angle)*1.3, 1, m.position.horizontal_angle > 0)
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
        rotate_angle(motors, math.pi/1.2, 1, direction == 'acw')
        return (0.1,direction)

def execute_timed_function(func, time_to_run):
    start_time = start_timer()
    curr_time = time.time()

    while curr_time - start_time < time_to_run:
        output = func()
        if output == 'Completed': return 1
        else: curr_time = time.time()
    return 0 

def return_loop(robot, zone, motors, direction = 'cw'):
    while True:
        rest, direction = return_to_zone(robot, zone, motors, direction)
        robot.sleep(rest)
        halt(motors)
        robot.sleep(0.05)
        
def idle(robot, motors):
    # Setup first run flag
    if not hasattr(idle, "first_run"):
        idle.first_run = True

    # --- 1. TARGET CHECK ---
    _, acids, bases, walls = sorted_boxes(robot.camera.see())
    if acids or bases:
        idle.first_run = False 
        return 

    # --- 2. FIRST RUN 360 SPIN ---
    if idle.first_run:
        idle.first_run = False
        init_queue()
        add_R_update(0.3, 0.0) 
        _execute_maneuver(robot, motors, 2.0)
        return

    init_queue()
    us_dist = robot.arduino.ultrasound_measure(2, 3)

    # --- 3. HEAD-ON CRASH (Ultrasound says blocked) ---
    if 0 < us_dist < 350:
        # Back up and turn hard. The NEXT loop will check if it's clear.
        add_T_update(0.0, -0.4, 0.0) 
        add_R_update(random.choice([-0.4, 0.4]), 0.0) 
        duration = 1.0

    # --- 4. GLANCING WALL (Camera sees a wall nearby) ---
    elif walls and walls[0].position.distance < 800:
        add_T_update(0.0, 0.5, 0.0) # Keep moving forward
        # Steer away from it
        if walls[0].position.horizontal_angle > 0:
            add_R_update(-0.3, 0.0) # Wall on right -> steer left
        else:
            add_R_update(0.3, 0.0)  # Wall on left -> steer right
        duration = 0.8

    # --- 5. OPEN WATER ---
    else:
        # Coast is clear. Fast sweeping forward arc.
        add_T_update(0.0, 0.6, 0.0)
        add_R_update(random.choice([-0.15, 0.15]), 0.0)
        duration = 1.0

    # --- EXECUTE ---
    _execute_maneuver(robot, motors, duration)


def _execute_maneuver(robot, motors, duration):
    """Executes queue, checks camera, and aborts instantly if target seen."""
    start = time.time()
    while time.time() - start < duration:
        _, acids, bases, _ = sorted_boxes(robot.camera.see())
        if acids or bases:
            clear_queue(motors)
            return
        stay_vigilant(motors)
        time.sleep(0.05)
    clear_queue(motors)

def dump(robot, target_zone, motors):
    """
    Drives into a target zone and deposits all carried samples.
    Homes in on the zone's wall markers and stops when the ultrasound
    detects we're close enough to the wall.
    """
    CLOSE_ENOUGH = 250  # mm
    TIMEOUT_S    = 20.0
    start        = time.time()

    while time.time() - start < TIMEOUT_S:
        # Check ultrasound first — stop if we're already close enough
        distance_mm = robot.arduino.ultrasound_measure(2, 3)
        if 0 < distance_mm < CLOSE_ENOUGH:
            halt(motors)
            return

        markers = robot.camera.see()
        zone_marker_ids = ZONE_FIDUCIAL_MARKERS[target_zone]
        zone_markers = [m for m in markers if m.id in zone_marker_ids]

        if not zone_markers:
            # Can't see the zone, spin slowly to find it
            spin(motors, 0.3, duration=0.2)
            continue

        # Home in on the nearest visible zone marker
        target = min(zone_markers, key=lambda m: m.position.distance)
        move_angle(motors, 0.6, target.position.horizontal_angle)
        time.sleep(0.05)

    halt(motors)


def autonomous_start_sequence_async(robot, motors, servo):
    # Tuck the arm in securely before moving

    # ==========================================================
    # PHASE 1: Collect + Orbit + Wall-Square to Platform
    # ==========================================================
    init_queue()
    t = 0.0

    # 1. Forward 300mm (eat first blue box)
    rt, _ = t_powerconverter(0.0, 0.3)
    add_T_update(0.0, 0.3, t)
    t += rt

    # 2. Forward 800mm (approach red box, leaving a 200mm buffer)
    rt, _ = t_powerconverter(0.0, 0.8)
    add_T_update(0.0, 0.8, t)
    t += rt

    # --- ORBIT AROUND RED ---
    
    # --- ORBIT AROUND RED (THE PURE CIRCLE METHOD) ---
    
    # 3. The Semi-Circle Orbit: Pure left translation + CW rotation
    # We move left 0.5m while rotating 180 degrees (0.5 rotations) CW.
    # This traces a perfect half-circle around the box on our right.
    rt_t, _ = t_powerconverter(-0.5, 0.0)
    rt_r, _ = r_powerconverter(0.5)  # 0.5 = 180 degrees CW
    add_T_update(-0.5, 0.0, t)
    add_R_update(0.5, t)
    t += max(rt_t, rt_r)

    # 4. The 180-Degree Correction Spin
    # We are now past the box, but facing perfectly backwards. 
    # Spin 180 degrees in place to face forward again.
    # (No T_update needed, just pure rotation).
    rt_r, _ = r_powerconverter(-0.5) # Spin 180 back around
    add_R_update(-0.5, t)
    t += rt_r

    # 5. Small forward bite (300mm) to secure the second blue box
    rt, _ = t_powerconverter(0.0, 0.3)
    add_T_update(0.0, 0.3, t)
    t += rt

    # 6. REVERSE into platform (Wall-Squaring)
    # We command -1.2m and add 0.5s of buffer time so the robot intentionally 
    # backs hard into the platform, flattening out any heading drift from the orbit.
    rt, _ = t_powerconverter(0.0, -1.2)
    add_T_update(0.0, -1.2, t)
    t += (rt + 0.5)

    # Execute Phase 1 timeline
    start = time.time()
    while time.time() - start < t + 0.2:
        stay_vigilant(motors)
        time.sleep(0.01)

    # Safety stop
    clear_queue(motors)
    time.sleep(0.1)

    # ==========================================================
    # PHASE 2: Whip
    # ==========================================================
    # Robot is physically flat against the platform. Perfect strike.
    whip(servo)
    time.sleep(0.3)

    # ==========================================================
    # PHASE 3: Detach, Turn, Crab, and Return to Base
    # ==========================================================
    init_queue()
    t = 0.0

    # 7. Move forward 500mm (detach from platform so turning is clear)
    rt, _ = t_powerconverter(0.0, 0.5)
    add_T_update(0.0, 0.5, t)
    t += rt

    # 8. TURN AROUND 180° to face home (0.5 rotations)
    rt, _ = r_powerconverter(0.5)
    add_R_update(0.5, t)
    t += rt
    
    # 9. CRAB RIGHT 500mm (Safely clears the red box from our new perspective)
    rt, _ = t_powerconverter(0.5, 0.0)
    add_T_update(0.5, 0.0, t)
    t += rt

    # 10. Drive straight back to base (~2100mm)
    rt, _ = t_powerconverter(0.0, 2.1)
    add_T_update(0.0, 2.1, t)
    t += rt

    # Execute Phase 3 timeline
    start = time.time()
    while time.time() - start < t + 0.2:
        stay_vigilant(motors)
        time.sleep(0.01)

    # Final safety stop
    clear_queue(motors)
    
