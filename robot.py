# ─────────────────────────────────────────────────────────────────────────────
# robot.py — entry point
# ─────────────────────────────────────────────────────────────────────────────

from sr.robot3 import Robot, Colour, LED_A, LED_B, LED_C, COMP, DEV
from movement import *
from utility import *
from info import *
from actions import consume, avoid, dump, return_loop
from servo import setup_servo, initialise_servo          # must be explicit — not in actions
from stragegies import (
    find_and_collect,
    collect_until_threshold,
    assess_and_sabotage,
    collect_and_bank,
    platform_wall_sweep,
    idle,
)
import stragegies
import time

# ─────────────────────────────────────────────────────────────────────────────
# Game constants
# ─────────────────────────────────────────────────────────────────────────────

GAME_LENGTH   = 180   # seconds
PHASE_2_START = 70    # elapsed s: aggressive collection → sabotage window
PHASE_3_START = 130   # elapsed s: sabotage window      → rapid bank cycling
PHASE_4_START = 160   # elapsed s: rapid cycling        → return home

# ─────────────────────────────────────────────────────────────────────────────
# Hardware setup
# ─────────────────────────────────────────────────────────────────────────────

start_time = start_timer()

robot  = Robot()
zone   = robot.zone

mb1    = robot.motor_boards[srlnums[0]]
mb2    = robot.motor_boards[srlnums[1]]

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

def log(msg):
    """Status print with elapsed time and live score state."""
    print(
        f"[{elapsed():.1f}s] {msg}"
        f"  |  robot={stragegies.CURRENT_ROBOT_VALUE}"
        f"  base={stragegies.CURRENT_BASE_VALUE}"
    )

# ─────────────────────────────────────────────────────────────────────────────
# DEV mode — individual test routines
# ─────────────────────────────────────────────────────────────────────────────

if robot.mode == DEV:
    code = 2

    if code == 0:
        # Rotation calibration test
        while True:
            for _ in range(6):
                rotate_angle(motors, math.pi / 2, 1, True)
                robot.sleep(1)

    elif code == 1:
        # consume() on first visible marker
        while True:
            markers = robot.camera.see()
            try:
                m = markers[0]
            except IndexError:
                continue
            consume(robot, m.id, motors, 'direct-ws')

    elif code == 2:
        # avoid() on first visible marker
        while True:
            markers = robot.camera.see()
            try:
                m = markers[0]
            except IndexError:
                continue
            avoid(robot, m.id, motors)

    elif code == 3:
        # Spin calibration test
        while True:
            rotate_angle(motors, math.pi, 1, True)
            time.sleep(2)

    elif code == 4:
        # find_and_collect loop with logging
        while True:
            result = find_and_collect(robot, motors, servo)
            log(f"find_and_collect → {result}")

    elif code == 5:
        # idle scan loop with logging
        while True:
            action, marker = idle(robot, motors)
            log(f"idle → {action}  marker={marker.id if marker else None}")

# ─────────────────────────────────────────────────────────────────────────────
# COMP mode — time-phased strategy
# ─────────────────────────────────────────────────────────────────────────────

else:
    # ── Phase 1: Platform sweep + aggressive collection (0 → PHASE_2_START) ──
    # Open by scanning for a raised box. If one is found, sweep the platform
    # wall to knock off the most boxes before the field fills with opponents.
    # Then collect until the carry limit or time budget is reached.
    log("Phase 1 — scanning for platform boxes")
    action, marker = idle(robot, motors)

    if action == 'high':
        log("Phase 1 — platform wall sweep")
        platform_wall_sweep(robot, motors, servo, marker)

    log("Phase 1 — aggressive collection")
    phase_1_budget = PHASE_2_START - elapsed()
    collect_until_threshold(
        robot, motors, servo,
        time_limit=max(0, phase_1_budget - 3),   # 3 s buffer for return trip
        carry_limit=THRESHOLD_CARRY,
    )

    # ── Phase 2: Sabotage window (PHASE_2_START → PHASE_3_START) ─────────────
    # If we are carrying a full load and an opponent base looks profitable to
    # neutralise, dump there. Otherwise keep collecting.
    log("Phase 2 — sabotage window")
    if stragegies.CURRENT_ROBOT_VALUE >= THRESHOLD_CARRY:
        did_sabotage = assess_and_sabotage(robot, motors, servo)
        log(f"Phase 2 — sabotage {'executed' if did_sabotage else 'skipped (no benefit)'}")

    phase_2_remaining = PHASE_3_START - elapsed()
    if phase_2_remaining > 10:
        log("Phase 2 — continued collection")
        collect_until_threshold(
            robot, motors, servo,
            time_limit=max(0, phase_2_remaining - 3),
            carry_limit=THRESHOLD_CARRY,
        )

    # ── Phase 3: Rapid bank cycling (PHASE_3_START → PHASE_4_START) ──────────
    # Shorter trips: collect a smaller load, bank it, repeat. Keeps score
    # ticking and ensures no boxes are lost if time runs short.
    log("Phase 3 — rapid bank cycling")
    phase_3_budget = PHASE_4_START - elapsed()
    if phase_3_budget > 5:
        collect_and_bank(
            robot, motors, servo,
            time_limit=max(0, phase_3_budget - 3),
            carry_limit=max(1, THRESHOLD_CARRY - 1),
        )

    # ── Phase 4: Return home and hold (PHASE_4_START → end) ──────────────────
    log("Phase 4 — returning home")
    return_loop(robot, zone, motors)
    halt(motors)
    log("Parked in base")
