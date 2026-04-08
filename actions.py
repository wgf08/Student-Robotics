from movement import *
from info import *
from utility import *
from servo import whip  # FIX: whip is called in autonomous_start_sequence but was never imported
import random
import time

# ─────────────────────────────────────────────────────────────────────────────
# Navigation & Safety
# ─────────────────────────────────────────────────────────────────────────────

def wall_check(motors, robot):
    """
    Checks if a wall is extremely close, if it is it rotates the robot pi/2
    """
    distance_mm = robot.arduino.ultrasound_measure(9, 10)
    if (distance_mm < 80) and (distance_mm != 0.0):
        rotate_angle(motors, math.pi / 2, 1)


def consume(robot, marker_id, motors, method='indirect'):
    """
    Moves forward in order to collect a box on the ground given the id of the box.
    INDIRECT - DEFAULT - Moves horizontally first until the box is within a central range.
    DIRECT-WS - Moves at the current angle of the box, stopping frequently to readjust.
    OTHER     - Moves slowly towards target.
    """

    if method == 'indirect':
        # Align laterally
        while True:
            markers = robot.camera.see()
            try:
                box = find_marker(markers, marker_id)
                print('got box')
                print(box)
                y = sample_xyz(box)[0]   # lateral offset (cx)
                print(y)
            except Exception as e:
                print(f'returned could not see box {e}')
                halt(motors)
                continue
            if y > 150:
                move_angle(motors, 0.5, math.pi / 2)
                time.sleep(0.03)
            elif y < -150:
                move_angle(motors, 0.5, 3 * math.pi / 2)
                time.sleep(0.03)
            else:
                break

        # Move forward using accelerometer-assisted distance estimate
        x = sample_xyz(box)[1]   # forward distance (cy)
        move_distance(motors, 1, box.position.horizontal_angle, x, robot)
        move_angle(motors, 0.5, box.position.horizontal_angle)

        # Continue until the box can no longer be seen (i.e. it has been collected).
        # FIX: original used try/except here, but find_marker returns None — never raises.
        # The except block never fired so the loop was infinite and return 1 unreachable.
        while True:
            markers = robot.camera.see()
            box = find_marker(markers, marker_id)
            if box is None:
                time.sleep(0.1)
                halt(motors)
                break
        return 1

    elif method == 'direct-ws':
        while True:
            markers = robot.camera.see()
            box = find_marker(markers, marker_id)
            if box is None:
                return 0
            move_distance(motors, 1, box.position.horizontal_angle,
                          box.position.distance, robot)
            markers = robot.camera.see()
            if find_marker(markers, marker_id) is None:
                return 1
            # Still visible; loop to re-approach from updated position

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
    """
    Moves out the way of the box horizontally, and then drives forward to pass it.
    """
    while True:
        markers = robot.camera.see()
        box = find_marker(markers, marker_id)

        # FIX: original relied on sample_xyz(None) raising an exception and then checked
        # `if not markers` to decide whether to return. But wall markers (ids 0-19) are
        # always visible, so `not markers` was always False — the loop spun forever
        # whenever the target box disappeared. Now we exit cleanly when the target is gone.
        if box is None:
            halt(motors)
            return

        try:
            y = sample_xyz(box)[0]
        except Exception:
            halt(motors)
            continue

        print(y)
        if -450 < y < 450:
            move_angle(motors, 0.72, 3 * math.pi / 2 if y >= 0 else math.pi / 2)
            time.sleep(0.03)
        else:
            break  # FIX: was `continue` — never exited. Now breaks once laterally clear.

    # FIX: forward drive was entirely missing despite the docstring promising it.
    move_straight(motors, 0.5, forwards=True, duration=0.5)
    halt(motors)


def return_to_zone(robot, zone, motors, direction='cw'):

    distance_mm = robot.arduino.ultrasound_measure(2, 3)
    print(robot.arduino.ultrasound_measure(2, 3))
    if distance_mm < 350 and distance_mm != 0:
        move_straight(motors, power=-0.25)
        robot.sleep(0.5)
        rotate_angle(motors, 0.4, 1)  # FIX: was rotate_angle(robot, 0.4, motors, 1)
        return (0.1, direction)

    base_ids = ZONE_FIDUCIAL_MARKERS[zone]
    YAW_RANGE = 0.7

    try:
        print('101')
        markers = sorted_boxes(robot.camera.see())[3]
        print('103')
        m = markers[0]
        x, y, z = wall_xyz(m)
        print(y, m.id)

        # FIX: was `a = set([m.id for m in ...]) & set(base_ids)` — produced a set of
        # plain integers, then wall_xyz(a) was called on that set (TypeError).
        # Now we build a list of actual marker objects whose ids are in base_ids.
        all_wall_markers = sorted_boxes(robot.camera.see())[3]
        a = [marker for marker in all_wall_markers if marker.id in base_ids]

        y2 = math.inf
        if a:
            y2 = wall_xyz(a[0])[0]
        print('108')
        if (not a) or (abs(y2) > 1100):
            print(not a)
            print((abs(y)) > 1000)
            if m.id % 5 == 1 and m.position.distance > 1600:
                for box in markers:
                    print('114')
                    if box.id % 5 == 0: m = box
            target_markers = ZONE_FIDUCIAL_MARKERS[zone]
            distances_cw  = [(marker - m.id) % 20 for marker in target_markers]
            distances_ccw = [(m.id - marker) % 20 for marker in target_markers]

            min_cw  = min(distances_cw)
            min_ccw = min(distances_ccw)
            direction = 'cw' if min_cw <= min_ccw else 'ccw'

            # FIX: was 'acw' — but direction is set to 'ccw' above, so 'acw' != 'ccw'
            # was always True and caused spurious 180-degree rotations.
            curr_dir = 'cw' if m.orientation.yaw < 0 else 'ccw'

            if curr_dir != direction and x < 100:
                print(curr_dir, direction, m, m.orientation.yaw)
                rotate_angle(motors, math.pi, 1)  # FIX: was rotate_angle(robot, math.pi, motors, 1)
                return (0.1, direction)

            angle = ((math.pi) / 2 + m.orientation.yaw) * 2.5 if direction == 'cw' else ((math.pi) + m.orientation.yaw) * 2.5

            if a:
                # FIX: was rotate_angle(robot, math.pi/1.3, motors, 1, ac=True)
                # ac=True meant anticlockwise → clockwise=False
                rotate_angle(motors, math.pi / 1.3, 1, False)
                return (0.2, direction)
            elif x < 900 or (x < 1600 and abs(m.orientation.yaw) > YAW_RANGE):
                print(f'1 {direction}')
                rotate_angle(motors, abs(angle), 1, True)
                move_straight(motors, 0.4)
                return (0.8, direction)
            elif x > 1600 and (m.id % 5 == 1 or m.id % 5 == 2):
                # FIX: was `m.id % 5 == 1 or 2` — Python parses this as
                # `(m.id % 5 == 1) or 2`. Since bare `2` is always truthy the whole
                # condition was always True, making the next elif permanently dead code.
                print('3')
                rotate_angle(motors, abs(m.position.horizontal_angle) * 1.4, 1,
                             m.position.horizontal_angle > 0)
                move_straight(motors, 0.4)
                return (0.4, direction)
            elif x > 1600 and abs(m.orientation.yaw) < YAW_RANGE:
                print('2 {x, m.id}')
                rotate_angle(motors, abs(m.position.horizontal_angle) * 1.3, 1,
                             m.position.horizontal_angle > 0)
                move_straight(motors, 0.4)
                return (0.4, direction)
            else:
                print(f'Condition 4 executed. Stats \n M_ID = {m.id}   X: {x}   YAW: {m.orientation.yaw}')
                move_straight(motors, 0.4)
                return (0.2, direction)

        else:
            print(y)
            distance_mm = robot.arduino.ultrasound_measure(2, 3)
            marker = find_marker(robot.camera.see(), m.id)
            while marker is not None and marker.position.distance > 400:
                # FIX: `marker.position.distance` bare statement was dead code — removed.
                marker = find_marker(robot.camera.see(), m.id)
                move_straight(motors, 0.2)
            while distance_mm > 800 or distance_mm == 0:
                distance_mm = robot.arduino.ultrasound_measure(2, 3)
                move_straight(motors, 0.3)

            halt(motors)
            move_straight(motors, power=-0.5)
            return (0.1, direction)

    except Exception as e:
        print(e)
        print('FAILURE - NOTHING SEEN')
        rand = random.random()
        if rand <= 0.3:
            move_straight(motors, power=-0.2)
            robot.sleep(0.5)
            print('moving back?')
        # FIX: was rotate_angle(robot, math.pi/1.2, motors, 1, 'cw' if dir=='acw' else 'acw')
        # The 5th arg was always a truthy string. Intent: spin opposite to current direction.
        rotate_angle(motors, math.pi / 1.2, 1, direction != 'cw')
        return (0.1, direction)


def execute_timed_function(func, time_to_run):
    start_time = start_timer()
    curr_time = time.time()
    # FIX: was `>` — elapsed starts near 0 so the loop body never ran once.
    while curr_time - start_time < time_to_run:
        output = func()
        if output == 'Completed': return 1
        else: curr_time = time.time()
    return 0


def return_loop(robot, zone, motors, direction='cw'):
    direction = 'cw'
    while True:
        rest, direction = return_to_zone(robot, zone, motors, direction)
        robot.sleep(rest)
        halt(motors)
        robot.sleep(0.05)


def idle(robot, motors):
    # NOTE: This idle is not used in COMP mode — stragegies.idle is imported instead.
    # Kept here for DEV testing.

    _, acids, bases, _ = sorted_boxes(robot.camera.see())
    if acids or bases:
        return

    if random.random() < 0.10:
        rotate_angle(motors, math.pi * 2, 1)  # FIX: was rotate_angle(robot, math.pi*2, motors, 1)
        return

    front_space = robot.arduino.ultrasound_measure(2, 3)

    if 0 < front_space < 400:
        rotate_angle(motors, math.pi / 2, 1)  # FIX: was rotate_angle(robot, math.pi/2, motors, 1)
        left_space = robot.arduino.ultrasound_measure(2, 3)
        if 0 < left_space < 400:
            rotate_angle(motors, math.pi, 1)   # FIX: was rotate_angle(robot, math.pi, motors, 1)
        return

    move_straight(motors, power=0.4)
    robot.sleep(0.5)
    halt(motors)  # FIX: was move_straight(motors, power=0.0) which doesn't brake


def dump(robot, target_zone, motors):
    """
    Drives into a target zone and deposits all carried samples.
    """
    CLOSE_ENOUGH = 250  # mm

    while True:
        distance_mm = robot.arduino.ultrasound_measure(2, 3)
        if 0 < distance_mm < CLOSE_ENOUGH:
            halt(motors)
            return

        markers = robot.camera.see()
        zone_marker_ids = ZONE_FIDUCIAL_MARKERS[target_zone]
        zone_markers = [m for m in markers if m.id in zone_marker_ids]

        if not zone_markers:
            spin(motors, 0.3, duration=0.2)
            continue

        target = min(zone_markers, key=lambda m: m.position.distance)
        move_angle(motors, 0.6, target.position.horizontal_angle)
        time.sleep(0.05)


def autonomous_start_sequence(motors, servo):

    # 1. Eat 1st blue box (0.5s)
    move_straight(motors, 1.0, forwards=True, duration=0.5)

    # 2. Approach red box (1.33s)
    move_straight(motors, 1.0, forwards=True, duration=1.33)
    time.sleep(0.1)

    # 3. Strafe left to dodge red box (0.83s)
    move_angle(motors, 1.0, 3 * math.pi / 2)
    time.sleep(0.83)
    halt(motors)
    time.sleep(0.1)

    # 4. Pass the red box (1.67s)
    move_straight(motors, 1.0, forwards=True, duration=1.67)
    time.sleep(0.1)

    # 5. Strafe right to return to center line (0.83s)
    move_angle(motors, 1.0, math.pi / 2)
    time.sleep(0.83)
    halt(motors)
    time.sleep(0.1)

    # 6. Eat 2nd blue box (0.5s)
    move_straight(motors, 1.0, forwards=True, duration=0.5)
    time.sleep(0.1)

    # 7. Reverse to platform (1.67s)
    move_straight(motors, 1.0, forwards=False, duration=1.67)

    # 8. Whip the raised box
    whip(servo)

    # 9. Drive straight forward to detach from platform (0.5s)
    move_straight(motors, 1.0, forwards=True, duration=0.5)
    time.sleep(0.1)

    # 10. Spin 180 degrees to face the base (1.1s)
    spin(motors, 1.0, clockwise=True, duration=1.1)

    # 11. Crab right to clear the red box (0.83s)
    move_angle(motors, 1.0, math.pi / 2)
    time.sleep(0.83)
    halt(motors)
    time.sleep(0.1)

    # 12. Drive forward to return to base (3.5s)
    move_straight(motors, 1.0, forwards=True, duration=3.5)
