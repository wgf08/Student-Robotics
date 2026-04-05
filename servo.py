import time
from utility import *
from movement import *

# The SR servo API uses positions from -1 to 1
# For the MG996R 180°:
#   -1 = 0° (fully swept one way)
#    0 = 90° (centre)
#    1 = 180° (fully swept other way)

# Tune these values once the servo is physically mounted
RESTING_POSITION = -0.7  # Arm tucked back, ready to whip
WHIP_POSITION    =  1.0  # Arm fully extended for the whip
SWEEP_POSITION   =  0.0  # Arm at 90° — extended perpendicular as a side scoop
WHIP_DELAY       =  0.5  # Seconds to wait after whipping before returning

def setup_servo(robot):
    servo = robot.servo_board.servos[0]
    servo.set_duty_limits(500, 2500)
    return servo

def initialise_servo(servo):
    """
    Move the servo to the correct position for beginning a match.
    """
    servo.position = RESTING_POSITION
    time.sleep(0.5)  # Give it time to reach position before match starts

def set_sweep(servo):
    """
    Extend the arm to 90° so it acts as a side scoop during a wall sweep.
    Call this before starting platform_wall_sweep(); call initialise_servo()
    afterwards to tuck it back.
    """
    servo.position = SWEEP_POSITION
    time.sleep(0.3)  # Allow arm to reach position before moving

def whip(servo):
    """
    Moves the servo arm round in order to knock a sample off of the center 
    platform then once this is done, returns the servo to its original position.
    """
    
    # Swing the arm forward hard to knock the box
    servo.position = WHIP_POSITION
    time.sleep(WHIP_DELAY)  # Wait for arm to complete the swing

    # Return to resting position ready for next whip
    servo.position = RESTING_POSITION
    time.sleep(0.5)  # Wait to reach resting position before doing anything else


# Vertical angle above which a box is still considered raised/on-platform.
# Mirrors HIGH_BOX_ANGLE_THRESHOLD in strategies — keep in sync if you change it.
_HIGH_ANGLE = 0.15

# Lateral (x) and forward (y) tolerances for aligning to a high box (mm)
_ALIGN_X_TOL = 200   # centre within ±200 mm laterally
_ALIGN_Y_TOL = 400   # approach until within 400 mm forward

# Forward nudge used when the box can't be seen after whipping
_NUDGE_DURATION = 0.25  # seconds


def get_high_box(robot, servo, motors, marker_id):
    """
    Aligns to a raised box using x (lateral) and y (forward) from sample_xyz,
    whips the servo tail to knock it off, then tries to collect it from the floor.

    Alignment:
      y = forward distance  — move_straight forward while y > _ALIGN_Y_TOL
      x = lateral offset    — strafe left/right while abs(x) > _ALIGN_X_TOL
                              (positive x → box is to the right → strafe right)

    Post-whip collection:
      • If the marker is gone → nudge forward and look again.
      • If the marker is visible and no longer above the height threshold
        (i.e. it fell to the ground) → consume it normally.
      • If still showing as high → it may still be on the platform; give up
        rather than loop forever.
    """
    from actions import consume  # local import to avoid circular dependency

    # ── Alignment: forward ────────────────────────────────────────────────────
    while True:
        markers = robot.camera.see()
        marker  = find_marker(markers, marker_id)
        if marker is None:
            halt(motors)
            return
        x, y, z = sample_xyz(marker)
        if y <= _ALIGN_Y_TOL:
            break
        move_straight(motors, 0.5, forwards=True)
        time.sleep(0.05)

    halt(motors)

    # ── Alignment: lateral ────────────────────────────────────────────────────
    while True:
        markers = robot.camera.see()
        marker  = find_marker(markers, marker_id)
        if marker is None:
            halt(motors)
            return
        x, y, z = sample_xyz(marker)
        if abs(x) <= _ALIGN_X_TOL:
            break
        # x > 0 → box is to the right → strafe right (pi/2); left otherwise
        direction = math.pi / 2 if x > 0 else 3 * math.pi / 2
        move_angle(motors, 0.3, direction)
        time.sleep(0.05)

    halt(motors)

    # ── Whip ──────────────────────────────────────────────────────────────────
    whip(servo)
    time.sleep(0.2)  # brief pause for the box to settle after being knocked

    # ── Post-whip: collect if the box fell to the ground ─────────────────────
    markers = robot.camera.see()
    fallen  = find_marker(markers, marker_id)

    if fallen is None:
        # Can't see it — nudge forward in case it landed just ahead
        move_straight(motors, 0.4, forwards=True, duration=_NUDGE_DURATION)
        markers = robot.camera.see()
        fallen  = find_marker(markers, marker_id)

    if fallen is not None and fallen.position.vertical_angle <= _HIGH_ANGLE:
        # Box is on the ground — consume it
        consume(robot, marker_id, motors, 'direct-ws')
