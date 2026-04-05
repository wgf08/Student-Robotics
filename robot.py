# ─────────────────────────────────────────────────────────────────────────────
# robot.py — entry point
# ─────────────────────────────────────────────────────────────────────────────

from sr.robot3 import Robot, Colour, LED_A, LED_B, LED_C, COMP, DEV
from movement import *
from utility import *
from info import *
from actions import *
from stragegies import (
    find_and_collect,
    collect_until_threshold,
    assess_and_sabotage,
    collect_and_bank,
    platform_wall_sweep,
    CURRENT_BASE_VALUE,
    CURRENT_ROBOT_VALUE,
)
import stragegies
import time

# ─────────────────────────────────────────────────────────────────────────────
# Game constants
# ─────────────────────────────────────────────────────────────────────────────

GAME_LENGTH      = 180   # seconds
# Time (seconds elapsed) at which we switch strategy phases
PHASE_2_START    = 70    # switch from aggressive collection → sabotage window
PHASE_3_START    = 130   # switch from sabotage window      → rapid bank cycling
PHASE_4_START    = 160   # switch from bank cycling         → head home and stay

# ─────────────────────────────────────────────────────────────────────────────
# Setup
# ─────────────────────────────────────────────────────────────────────────────

start_time = start_timer()

robot  = Robot()
zone   = robot.zone

mb1    = robot.motor_boards[srlnums[0]]
mb2    = robot.motor_boards[srlnums[1]]
sb     = robot.servo_board

motor1 = mb2.motors[0]
motor2 = mb1.motors[0]
motor3 = mb1.motors[1]
motors = [motor1, motor2, motor3]

servo  = setup_servo(robot)
initialise_servo(servo)

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def elapsed():
    """Seconds since the match began."""
    return time.time() - start_time

def time_remaining():
    """Seconds left in the match."""
    return GAME_LENGTH - elapsed()

def log(msg):
    """Lightweight status print with elapsed time."""
    print(f"[{elapsed():.1f}s] {msg}  |  robot={stragegies.CURRENT_ROBOT_VALUE}  base={stragegies.CURRENT_BASE_VALUE}")

# ─────────────────────────────────────────────────────────────────────────────
# DEV mode — individual test routines, unchanged
# ─────────────────────────────────────────────────────────────────────────────

if robot.mode == DEV:
    code = 2

    if code == 0:
        # Rotation test
        while True:
            for i in range(6):
                rotate_angle(motors, math.pi / 2, 1, True)
                robot.sleep(1)

    elif code == 1:
        # consume() test on first visible marker
        while True:
            markers = robot.camera.see()
            try:
                m = markers[0]
            except IndexError:
                continue
            consume(robot, m.id, motors, 'direct-ws')

    elif code == 2:
        # avoid() test on first visible marker
        while True:
            markers = robot.camera.see()
            try:
                m = markers[0]
            except IndexError:
                continue
            avoid(robot, m.id, motors)

    elif code == 3:
        # Spin test
        while True:
            rotate_angle(motors, math.pi, 1, True)
            time.sleep(2)

    elif code == 4:
        # Strategy test — single find_and_collect tick, looped
        while True:
            result = find_and_collect(robot, motors, servo)
            log(f"find_and_collect → {result}")

# ─────────────────────────────────────────────────────────────────────────────
# COMP mode — time-phased strategy
# ─────────────────────────────────────────────────────────────────────────────

else:
    # ── Phase 1: Aggressive collection (0 → PHASE_2_START) ───────────────────
    # Grab as many boxes as possible and bank them before switching focus.
    log("Phase 1 — platform wall sweep")
    platform_wall_sweep(robot, motors, servo)

    log("Phase 1 — aggressive collection")
    phase_1_budget = PHASE_2_START - elapsed()
    collect_until_threshold(
        robot, motors, servo,
        time_limit=max(0, phase_1_budget - 3),  # 3s buffer for return trip
        carry_limit=THRESHOLD_CARRY,
    )

    # ── Phase 2: Sabotage window (PHASE_2_START → PHASE_3_START) ─────────────
    # Check every opponent zone; dump at the most damaging one if it is worth
    # it. If no sabotage opportunity exists, keep collecting instead.
    log("Phase 2 — sabotage window")
    if stragegies.CURRENT_ROBOT_VALUE >= THRESHOLD_CARRY:
        did_sabotage = assess_and_sabotage(robot, motors, servo)
        log(f"sabotage attempted → {did_sabotage}")

    # Use whatever time is left in phase 2 to keep collecting
    phase_2_remaining = PHASE_3_START - elapsed()
    if phase_2_remaining > 10:
        log("Phase 2 — continued collection")
        collect_until_threshold(
            robot, motors, servo,
            time_limit=max(0, phase_2_remaining - 3),
            carry_limit=THRESHOLD_CARRY,
        )

    # ── Phase 3: Rapid bank cycling (PHASE_3_START → PHASE_4_START) ──────────
    # Short trips: collect a small load, bank it, repeat. Keeps scoring even
    # if the field is sparse and ensures no carried boxes are lost to time.
    log("Phase 3 — rapid bank cycling")
    phase_3_budget = PHASE_4_START - elapsed()
    if phase_3_budget > 5:
        collect_and_bank(
            robot, motors, servo,
            time_limit=max(0, phase_3_budget - 3),
            carry_limit=max(1, THRESHOLD_CARRY - 1),  # smaller loads = faster trips
        )

    # ── Phase 4: Return home and hold (PHASE_4_START → end) ──────────────────
    # Get into our zone and stay there. Any boxes still on the robot at this
    # point count when deposited in our base.
    log("Phase 4 — returning home")
    return_loop(robot, zone, motors)
    halt(motors)
    log("Done — parked in base")
