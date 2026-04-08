from sr.robot3 import BRAKE, COAST, INPUT, OUTPUT, A0, A1
import math
import time
from info import *

def halt(motors):
    motors[0].power = BRAKE
    motors[1].power = BRAKE
    motors[2].power = BRAKE

def coast(motors):
    motors[0].power = COAST
    motors[1].power = COAST
    motors[2].power = COAST

def move_straight(motors, power, forwards=True, duration=None):
    if not forwards: power = -power
    motors[0].power = power
    motors[1].power = 0
    motors[2].power = -power
    if duration is None:
        return
    else:
        time.sleep(duration)
        halt(motors)
        return

def spin(motors, power, clockwise=True, duration=None):
    if not clockwise: power = -power
    motors[0].power = power
    motors[1].power = power
    motors[2].power = power
    if duration:
        time.sleep(duration)
        halt(motors)
        return
    else:
        return

def move_angle(motors, power, theta, rotation=0):
    """
    ANGLE IN RADIANS
    Moves at a requested angle and speed, with optional rotation.
    rotation: -1 (full anticlockwise) to 1 (full clockwise)
    """
    theta = (theta + math.pi) % (2 * math.pi)

    wheel_angles = [60, 180, 300]

    vals = [
        math.sin(theta - math.radians(a)) + rotation
        for a in wheel_angles
    ]

    # Always normalise so power behaves consistently at all angles
    max_mag = max(abs(v) for v in vals)
    if max_mag > 0:
        vals = [v / max_mag for v in vals]

    motors[0].power = power * vals[0]
    motors[1].power = power * vals[1]
    motors[2].power = power * vals[2]

def rotate_angle(motors, theta, power, clockwise=True):
    spin(motors, power, clockwise)
    tpr = 2.2
    rtime = (theta / (2 * math.pi)) * tpr
    time.sleep(rtime)
    halt(motors)


# ─────────────────────────────────────────────────────────────────────────────
# Accelerometer-assisted distance movement
# ─────────────────────────────────────────────────────────────────────────────

# FIX: removed the duplicate integer pin definitions (0, 1) that appeared before
# the correct A0/A1 constants. The A0/A1 import is now at the top of the file.
ACCEL_PIN_X      = A0      # forward axis of the accelerometer
ACCEL_PIN_Y      = A1      # lateral axis (set None if unavailable)

# ADXL335 on 5 V: 0 g = ~512 counts, ~61 counts/g.
# ADXL335 on 3.3 V: 0 g = ~512 counts, ~102 counts/g.  Tune on your robot.
ACCEL_ZERO_G     = 512     # ADC reading at exactly 0 g
ACCEL_SCALE      = 61      # ADC counts per g

# Estimated max speed mm/s at power=1 (0.6 m/s)
KINEM_SPEED_MM_S = 600.0

# Hard timeout — halt no matter what after this many seconds.
MOVE_TIMEOUT_S   = 10.0


def _read_accel_g(robot):
    """
    Read both accelerometer axes, return (ax_g, ay_g).
    ay_g is 0.0 when ACCEL_PIN_Y is None.
    """
    robot.arduino.pins[ACCEL_PIN_X].mode = INPUT
    if ACCEL_PIN_Y is not None:
        robot.arduino.pins[ACCEL_PIN_Y].mode = INPUT

    raw_x = robot.arduino.pins[ACCEL_PIN_X].analog_read()
    ax_g  = (raw_x - ACCEL_ZERO_G) / float(ACCEL_SCALE)

    if ACCEL_PIN_Y is not None:
        raw_y = robot.arduino.pins[ACCEL_PIN_Y].analog_read()
        ay_g  = (raw_y - ACCEL_ZERO_G) / float(ACCEL_SCALE)
    else:
        ay_g  = 0.0

    return ax_g, ay_g


def move_distance(motors, power, theta, target_mm, robot):
    """
    Move at angle `theta` (radians) and motor `power` until the blended
    distance estimate reaches `target_mm`, then halt.

    Uses a fixed weighted average:
      - 30% Accelerometer (double integration)  — noisy on small robots
      - 70% Kinematic estimate (based on 0.6 m/s) — stable baseline
    FIX: comment previously claimed 70% accel / 30% kinem, but the code
    always computed 0.3 * pos_accel + 0.7 * pos_kinem. Comment corrected.
    """
    G_TO_MM_S2 = 9806.65

    # Project accelerometer axes onto travel direction
    proj_x = math.cos(theta)
    proj_y = math.sin(theta)

    move_angle(motors, power, theta)

    t_prev    = time.time()
    t_start   = t_prev
    vel_accel = 0.0
    pos_accel = 0.0
    pos_kinem = 0.0

    while True:
        t_now = time.time()
        dt    = t_now - t_prev
        t_prev = t_now

        if (t_now - t_start) > MOVE_TIMEOUT_S:
            break

        # ── 1. Kinematic estimate (70%) ──
        pos_kinem += KINEM_SPEED_MM_S * power * dt

        # ── 2. Accelerometer estimate (30%) ──
        ax_g, ay_g    = _read_accel_g(robot)
        a_along_g     = ax_g * proj_x + ay_g * proj_y
        a_along_mm_s2 = a_along_g * G_TO_MM_S2

        vel_accel += a_along_mm_s2 * dt
        pos_accel += vel_accel * dt

        # ── Blending: 30% accel, 70% kinem ──
        pos_blend = (0.3 * pos_accel) + (0.7 * pos_kinem)

        if pos_blend >= target_mm:
            break

    halt(motors)
