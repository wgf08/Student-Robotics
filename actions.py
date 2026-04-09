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
        move_distance(motors, 1, -box.position.horizontal_angle, x, robot)
        move_angle(motors, 0.5, -box.position.horizontal_angle)

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
            move_distance(motors, 1, -box.position.horizontal_angle,
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
            move_distance(motors, 0.5, -box.position.horizontal_angle,
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


def execute_timed_function(func, time_to_run):
    start_time = start_timer()
    curr_time = time.time()
    while curr_time - start_time < time_to_run:
        output = func()
        if output == 'Completed': return 1
        else: curr_time = time.time()
    return 0


def return_loop(robot, zone, motors):
    """
    Navigate back to target zone using GPS pathfinding (return_nav.NavigateToZone).

    NavigateToZone handles everything internally:
      • Rotates until ≥2 wall markers are visible (ScanForMarkers).
      • Triangulates position via GPS.
      • Plans an obstacle-avoiding path with Dijkstra.
      • Drives waypoints in short bursts, dead-reckoning between GPS fixes.
      • Re-plans automatically if the robot drifts off-track.

    If the target zone is our own base, we back up after arrival to leave any
    deposited boxes behind, then rotate pi/2 anticlockwise to face back out
    into the arena ready to collect again.
    """
    import return_nav as ret
    ret.NavigateToZone(robot, motors, zone)

    if zone == robot.zone:
        # Leave deposited boxes behind and turn to face the arena
        move_straight(motors, 0.6, forwards=False, duration=0.4)
        halt(motors)
        time.sleep(0.1)
        rotate_angle(motors, math.pi / 2, 0.8, clockwise=False)


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
            # Lost sight of the base — bail rather than driving blind
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

    # 1. GPS navigate to the zone centre
    ret.NavigateToZone(robot, motors, target_zone)

    # 2. Fine-approach: steer toward the nearest visible zone marker until close
    while True:
        distance_mm = robot.arduino.ultrasound_measure(2, 3)
        if 0 < distance_mm < CLOSE_ENOUGH:
            halt(motors)
            break

        markers = robot.camera.see()
        zone_markers = [m for m in markers if m.id in zone_marker_ids]

        if not zone_markers:
            # NavigateToZone got us close — nudge straight in
            move_straight(motors, 0.4, forwards=True, duration=0.2)
            break

        target = min(zone_markers, key=lambda m: m.position.distance)
        move_angle(motors, 0.5, -target.position.horizontal_angle)
        time.sleep(0.05)

    # 3. Back out so deposited boxes stay in the zone
    move_straight(motors, 0.6, forwards=False, duration=0.4)
    halt(motors)


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

    # 13. Back up slightly to leave collected boxes behind (0.4s)
    move_straight(motors, 0.6, forwards=False, duration=0.4)
    halt(motors)
    time.sleep(0.1)

    # 14. Rotate pi/2 anticlockwise so the robot faces out from base, leaving
    #     any deposited boxes in the zone behind it.
    rotate_angle(motors, math.pi / 2, 0.8, clockwise=False)
