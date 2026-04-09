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
        move_distance(motors, 1, -box.position.horizontal_angle, x, robot)
        move_angle(motors, 0.45, -box.position.horizontal_angle)

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
            move_distance(motors, 1, -box.position.horizontal_angle,
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
            move_distance(motors, 0.5, -box.position.horizontal_angle,
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

def execute_timed_function(func, time_to_run):
    start_time = start_timer()
    curr_time = time.time()

    while curr_time - start_time < time_to_run:
        output = func()
        if output == 'Completed': return 1
        else: curr_time = time.time()
    return 0 


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

def return_loop(robot, zone, motors):
    import return_nav as ret
    ret.NavigateToZone(robot, motors, zone)

    if zone == robot.zone:
        # Leave deposited boxes behind and turn to face the arena
        move_straight(motors, 0.6, forwards=False, duration=0.4)
        halt(motors)
        time.sleep(0.1)
        rotate_angle(motors, math.pi / 2, 0.8, clockwise=False)


def dump_opportunistic(robot, target_zone, motors):
    """
    Opportunistic dump: only call this when you can already SEE the opponent's
    base markers in the current frame. Steers directly toward the nearest
    visible zone marker until the front ultrasound says we are close enough,
    then halts (boxes are deposited by proximity).

    Returns True if we successfully drove in, False if the zone markers were
    lost before we arrived (e.g. we drifted off-angle).

    DO NOT call this for deliberate navigation — use dump_navigate() instead.
    """
    CLOSE_ENOUGH = 250  # mm
    zone_marker_ids = ZONE_FIDUCIAL_MARKERS[target_zone]

    while True:
        distance_mm = robot.arduino.ultrasound_measure(2, 3)
        if 0 < distance_mm < CLOSE_ENOUGH:
            halt(motors)
            return True

        markers = robot.camera.see()
        zone_markers = [m for m in markers if m.id in zone_marker_ids]

        if not zone_markers:
            halt(motors)
            return False

        target = min(zone_markers, key=lambda m: m.position.distance)
        move_angle(motors, 0.6, -target.position.horizontal_angle)
        time.sleep(0.05)


def dump_navigate(robot, target_zone, motors):
    """
    Deliberate dump: navigates to target_zone using GPS pathfinding
    (return_nav.NavigateToZone), drives in until the front ultrasound confirms
    we are close enough, then backs out so the deposited boxes stay behind.

    Use this during the late-game steal/sabotage phase when we are deliberately
    routing to an opponent base regardless of whether it is visible right now.
    """
    import return_nav as ret

    CLOSE_ENOUGH = 250  # mm
    zone_marker_ids = ZONE_FIDUCIAL_MARKERS[target_zone]

    ret.NavigateToZone(robot, motors, target_zone)

    while True:
        distance_mm = robot.arduino.ultrasound_measure(2, 3)
        if 0 < distance_mm < CLOSE_ENOUGH:
            halt(motors)
            break

        markers = robot.camera.see()
        zone_markers = [m for m in markers if m.id in zone_marker_ids]

        if not zone_markers:
            move_straight(motors, 0.4, forwards=True, duration=0.2)
            break

        target = min(zone_markers, key=lambda m: m.position.distance)
        move_angle(motors, 0.5, -target.position.horizontal_angle)
        time.sleep(0.05)

    move_straight(motors, 0.6, forwards=False, duration=0.4)
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
    
