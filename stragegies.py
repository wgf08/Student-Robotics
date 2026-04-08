from utility import *
from movement import *
from servo import whip, set_sweep, initialise_servo
from actions import consume, avoid, dump, return_loop
from info import *
import time
import math

# ─────────────────────────────────────────────────────────────────────────────
# Tunable constants
# ─────────────────────────────────────────────────────────────────────────────

HIGH_BOX_ANGLE_THRESHOLD = 0.15  # radians — vertical_angle above this = raised box
BLOCKER_ANGLE_TOLERANCE  = 0.25  # radians — half-cone in which a box counts as blocking
SEARCH_SPIN_POWER        = 0.3   # power when rotating to scan
SEARCH_SPIN_DURATION     = 0.4   # seconds per scan step

# ─────────────────────────────────────────────────────────────────────────────
# Score state — updated by strategies, readable from robot.py
# ─────────────────────────────────────────────────────────────────────────────

CURRENT_ROBOT_VALUE = 0  # net value of samples currently carried on the robot
CURRENT_BASE_VALUE  = 0  # cumulative net value deposited in our own base so far


# ─────────────────────────────────────────────────────────────────────────────
# Private helpers
# ─────────────────────────────────────────────────────────────────────────────

def _is_high(marker):
    """True if the marker is on a raised edge rather than the floor."""
    return marker.position.vertical_angle > HIGH_BOX_ANGLE_THRESHOLD


def _is_our_sample(marker):
    """True if this sample scores positively for us."""
    return sample_value(marker) == 1


def _is_blocking(candidate, target):
    """
    True if `candidate` sits between the robot and `target` —
    similar horizontal angle but closer distance.
    """
    if candidate.id == target.id:
        return False
    angle_diff = abs(
        candidate.position.horizontal_angle - target.position.horizontal_angle
    )
    return (
        angle_diff < BLOCKER_ANGLE_TOLERANCE
        and candidate.position.distance < target.position.distance
    )


def _bank_at_base(robot, motors):
    """
    Drive back to our zone, then update the score globals to reflect the deposit.
    Called any time we physically return home with boxes.
    """
    global CURRENT_ROBOT_VALUE, CURRENT_BASE_VALUE
    return_loop(robot, robot.zone, motors)
    CURRENT_BASE_VALUE  += CURRENT_ROBOT_VALUE
    CURRENT_ROBOT_VALUE  = 0


def _best_sabotage_zone(robot, zones):
    """
    Given a list of opponent zone indices, returns (zone, current_value,
    projected_value) for whichever zone would benefit most from us dumping
    there, or None if none are worth it.

    'Most benefit' = largest reduction in |value| after depositing
    THRESHOLD_CARRY boxes.
    """
    markers = robot.camera.see()
    best    = None

    for zone in zones:
        zone_marker_ids = ZONE_FIDUCIAL_MARKERS.get(zone, [])
        zone_markers    = [m for m in markers if m.id in zone_marker_ids]
        if not zone_markers:
            continue

        reference     = min(zone_markers, key=lambda m: m.position.distance)
        current_value = assess_base_value(reference, markers)
        projected     = current_value + THRESHOLD_CARRY  # +1 per box we carry
        improvement   = abs(current_value) - abs(projected)

        if improvement > 0:
            if best is None or improvement > best[3]:
                best = (zone, current_value, projected, improvement)

    if best is None:
        return None
    return best[0], best[1], best[2]


# ─────────────────────────────────────────────────────────────────────────────
# Strategy 1 — find_and_collect
# ─────────────────────────────────────────────────────────────────────────────

def find_and_collect(robot, motors, servo):
    """
    Single-tick collection decision. Targets the nearest visible sample
    regardless of type or height, then acts:

      • Wrong-type ground box blocking the target  → avoid() the blocker.
      • Target is raised (high vertical_angle)      → get_high_box().
      • Target is our type on the ground            → consume().
      • Target is wrong type on the ground          → avoid() it.

    Updates CURRENT_ROBOT_VALUE on a successful collect or whip.

    Returns: 'collected' | 'whipped' | 'avoided' | 'none'
    """
    global CURRENT_ROBOT_VALUE

    markers            = robot.camera.see()
    _, acids, bases, _ = sorted_boxes(markers)
    all_samples        = acids + bases

    if not all_samples:
        return 'none'

    # Nearest box is always the target — no priority filtering
    target      = min(all_samples, key=lambda m: m.position.distance)
    target_high = _is_high(target)
    target_ours = _is_our_sample(target)

    # Dodge any wrong-type ground boxes that sit between us and the target
    bad_ground = [
        m for m in all_samples if not _is_our_sample(m) and not _is_high(m)
    ]
    blockers = [m for m in bad_ground if _is_blocking(m, target)]

    if blockers:
        closest_blocker = min(blockers, key=lambda m: m.position.distance)
        avoid(robot, closest_blocker.id, motors)
        return 'avoided'

    # Act on the target
    if target_high:
        # Sweep the platform wall starting from this box — get_high_box is used
        # internally by platform_wall_sweep after the sweep is done.
        platform_wall_sweep(robot, motors, servo, target)
        return 'whipped'

    if target_ours:
        result = consume(robot, target.id, motors, 'direct-ws')
        if result == 1:
            CURRENT_ROBOT_VALUE += 1
            return 'collected'
        return 'none'

    # Target itself is wrong type on the ground — dodge it
    avoid(robot, target.id, motors)
    return 'avoided'


# ─────────────────────────────────────────────────────────────────────────────
# Strategy 2 — collect_until_threshold
# ─────────────────────────────────────────────────────────────────────────────

def collect_until_threshold(robot, motors, servo,
                            time_limit=None,
                            carry_limit=THRESHOLD_CARRY):
    """
    Loops find_and_collect until carry_limit boxes are collected OR time_limit
    seconds (from when this function is called) have elapsed. Then banks
    everything at our own base.

    If time_limit is None, only the carry limit applies.

    Args:
        time_limit  : float | None  — seconds budget; None = no time cap
        carry_limit : int           — stop collecting once this many are held
    """
    start   = time.time()
    carried = 0

    def _within_time():
        return time_limit is None or (time.time() - start) < time_limit

    while carried < carry_limit and _within_time():
        result = find_and_collect(robot, motors, servo)

        if result in ('collected', 'whipped'):
            carried += 1
        elif result == 'none':
            # Nothing in sight — rotate to widen the search
            spin(motors, SEARCH_SPIN_POWER, clockwise=True,
                 duration=SEARCH_SPIN_DURATION)
            halt(motors)

    _bank_at_base(robot, motors)


# ─────────────────────────────────────────────────────────────────────────────
# Strategy 3 — assess_and_sabotage
# ─────────────────────────────────────────────────────────────────────────────

def assess_and_sabotage(robot, motors, servo):
    """
    Scans all opponent zones, picks the one where depositing our current load
    would most reduce their base value (bring it closer to 0), and dumps there.

    Logic: each box we carry is worth +1 in scoring terms. Depositing at an
    opponent base shifts its value by +1 per box. This only helps us if their
    base currently has a negative value (loaded against us), since the deposit
    moves it toward 0, reducing their advantage.

    Clears CURRENT_ROBOT_VALUE on deposit (boxes are gone from the robot).

    Returns: True if sabotage was attempted, False if no zone was worth it.
    """
    global CURRENT_ROBOT_VALUE

    opponent_zones = [z for z in range(4) if z != robot.zone]
    best = _best_sabotage_zone(robot, opponent_zones)

    if best is None:
        return False

    target_zone, current_value, projected_value = best
    dump(robot, target_zone, motors)
    CURRENT_ROBOT_VALUE = 0  # boxes left on their base, not ours
    return True


# ─────────────────────────────────────────────────────────────────────────────
# Strategy 4 — collect_and_bank
# ─────────────────────────────────────────────────────────────────────────────

def collect_and_bank(robot, motors, servo,
                     time_limit=None,
                     carry_limit=THRESHOLD_CARRY):
    """
    Multi-trip sustained scoring loop. Unlike collect_until_threshold (one
    trip home), this repeatedly cycles:

        collect up to carry_limit boxes  →  bank at our base  →  repeat

    until time_limit seconds have elapsed (or indefinitely if None).

    This is the right strategy when there is still collection time left but
    you want points secured incrementally rather than risked in one run.

    Args:
        time_limit  : float | None — total seconds for the whole loop
        carry_limit : int          — boxes per banking trip
    """
    start = time.time()

    def _within_time():
        return time_limit is None or (time.time() - start) < time_limit

    while _within_time():
        carried = 0

        # Inner collection loop
        while carried < carry_limit and _within_time():
            result = find_and_collect(robot, motors, servo)
            if result in ('collected', 'whipped'):
                carried += 1
            elif result == 'none':
                spin(motors, SEARCH_SPIN_POWER, clockwise=True,
                     duration=SEARCH_SPIN_DURATION)
                halt(motors)

        # Bank whatever we have — even a partial load is worth securing
        if carried > 0:
            _bank_at_base(robot, motors)


# ─────────────────────────────────────────────────────────────────────────────
# Strategy 5 — platform_wall_sweep
# ─────────────────────────────────────────────────────────────────────────────

ULTRASOUND_TRIG     = 2
ULTRASOUND_ECHO     = 3
CORNER_THRESHOLD_MM = 300   # stop when the front sensor reads below this (corner reached)
SWEEP_POWER         = 0.4

# Alignment tolerances when approaching the platform (matches get_high_box)
_SWEEP_ALIGN_X_TOL = 200   # ±mm laterally
_SWEEP_ALIGN_Y_TOL = 400   # mm forward


def platform_wall_sweep(robot, motors, servo, marker):
    """
    Triggered when a high box is spotted. Aligns to the box using the same
    x/y geometry as get_high_box, then extends the servo arm to the middle
    (sweep) position and drives along the platform wall — knocking off any
    boxes the arm encounters — until the front ultrasound detects a corner
    (reading < CORNER_THRESHOLD_MM). At that point it whips the tail, then
    tries to collect any boxes that fell to the floor during the sweep.

    Args:
        marker : the high-box marker that triggered this strategy

    x = lateral offset  (positive → box is to the right)
    y = forward distance (how far straight ahead the box is)
    z = vertical height
    """
    global CURRENT_ROBOT_VALUE

    # ── 1. Align forward (y) — get close to the platform wall ────────────────
    while True:
        markers = robot.camera.see()
        m = find_marker(markers, marker.id)
        if m is None:
            halt(motors)
            break
        x, y, z = sample_xyz(m)
        if y <= _SWEEP_ALIGN_Y_TOL:
            halt(motors)
            break
        move_straight(motors, SWEEP_POWER, forwards=True)
        time.sleep(0.05)

    # ── 2. Centre laterally (x) — line up with the box along the wall ────────
    while True:
        markers = robot.camera.see()
        m = find_marker(markers, marker.id)
        if m is None:
            break
        x, y, z = sample_xyz(m)
        if abs(x) <= _SWEEP_ALIGN_X_TOL:
            halt(motors)
            break
        # x > 0 → box is right → strafe right; negative → strafe left
        direction = math.pi / 2 if x > 0 else 3 * math.pi / 2
        move_angle(motors, 0.3, direction)
        time.sleep(0.05)

    halt(motors)

    # ── 3. Extend arm to sweep position ──────────────────────────────────────
    set_sweep(servo)

    # ── 4. Drive along the wall until the corner is detected ─────────────────
    # The arm is now extended perpendicular to our direction of travel,
    # sweeping anything off the platform edge as we pass.
    # Strafe at math.pi/2 so the forward-facing ultrasound (pins 2,3) picks
    # up the end wall while we travel sideways along the platform.
    # Flip to 3*math.pi/2 if the robot moves in the wrong direction physically.
    move_angle(motors, SWEEP_POWER, math.pi / 2)

    while True:
        dist = robot.arduino.ultrasound_measure(ULTRASOUND_TRIG, ULTRASOUND_ECHO)
        if 0 < dist < CORNER_THRESHOLD_MM:
            halt(motors)
            break

    # ── 5. End of wall — whip the tail to clear any last boxes ───────────────
    whip(servo)
    time.sleep(0.3)  # let boxes settle

    # ── 6. Collect any boxes that fell during the sweep ───────────────────────
    # Re-scan the camera; pick up anything visible on the ground (not high).
    markers            = robot.camera.see()
    _, acids, bases, _ = sorted_boxes(markers)
    fallen             = [
        m for m in acids + bases
        if not _is_high(m) and _is_our_sample(m)
    ]

    for box in sorted(fallen, key=lambda m: m.position.distance):
        result = consume(robot, box.id, motors, 'direct-ws')
        if result == 1:
            CURRENT_ROBOT_VALUE += 1

    # Tuck arm back ready for normal operation
    initialise_servo(servo)


# ─────────────────────────────────────────────────────────────────────────────
# Idle — scan the arena and return the first actionable target
# ─────────────────────────────────────────────────────────────────────────────

# How many spin steps before giving up a full rotation (~15° per step)
_IDLE_MAX_STEPS = 24


def idle(robot, motors):
    """
    Slowly rotates the robot, scanning with the camera each step, and returns
    as soon as it finds something worth acting on.

    Return values:
        ('high',    marker)  — a raised box is visible; caller should use
                               platform_wall_sweep(robot, motors, servo, marker)
        ('collect', marker)  — a ground-level box of our type is visible;
                               caller should use find_and_collect(...)
        ('none',    None)    — completed a full rotation with nothing found

    Lives in strategies (not actions) because it needs _is_high and
    _is_our_sample which are defined here.
    """
    steps = 0

    while steps < _IDLE_MAX_STEPS:
        markers            = robot.camera.see()
        _, acids, bases, _ = sorted_boxes(markers)
        all_samples        = sorted(acids + bases,
                                    key=lambda m: m.position.distance)

        for m in all_samples:
            if _is_high(m):
                halt(motors)
                return ('high', m)
            if _is_our_sample(m):
                halt(motors)
                return ('collect', m)

        # Nothing useful this tick — rotate a small step and try again
        spin(motors, SEARCH_SPIN_POWER, clockwise=True,
             duration=SEARCH_SPIN_DURATION)
        halt(motors)
        steps += 1

    return ('none', None)


# ─────────────────────────────────────────────────────────────────────────────
# Strategy 6 — steal_from_base
# ─────────────────────────────────────────────────────────────────────────────

# How close to approach before scanning for stealable boxes (mm)

def steal_from_base(robot, motors, target_zone):
    """
    Navigates to an opponent's base using direct line-of-sight or the 
    return_to_zone function, then consumes any valuable ground samples.
    """
    global CURRENT_ROBOT_VALUE
    
    target_ids = ZONE_FIDUCIAL_MARKERS.get(target_zone, [])
    if not target_ids:
        return False

    markers = robot.camera.see()
    zone_markers = [m for m in markers if m.id in target_ids]

    # 1. Navigate to their base
    if zone_markers:
        # We can see the base! Go straight towards the nearest marker.
        target = min(zone_markers, key=lambda m: m.position.distance)
        move_angle(motors, 0.6, target.position.horizontal_angle)
        time.sleep(0.3)
        halt(motors)
    else:
        # Can't see it, use the return_to_zone function to navigate
        rest, _ = return_loop(robot, target_zone, motors)
        time.sleep(rest)
        halt(motors)

    # 2. Try to consume the boxes there
    markers = robot.camera.see()
    _, acids, bases, _ = sorted_boxes(markers)
    stolen = False

    # Additional logic: Only target boxes on the ground that score us points
    stealable = [m for m in (acids + bases) if not _is_high(m) and _is_our_sample(m)]
    
    for box in sorted(stealable, key=lambda m: m.position.distance):
        if consume(robot, box.id, motors, method='direct-ws') == 1:
            CURRENT_ROBOT_VALUE += 1
            stolen = True

    return stolen
