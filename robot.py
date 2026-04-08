# ─────────────────────────────────────────────────────────────────────────────
# robot.py — entry point
# ─────────────────────────────────────────────────────────────────────────────

from sr.robot3 import Robot, Colour, LED_A, LED_B, LED_C, COMP, DEV
from movement import *
from utility import *
from info import *
from actions import consume, avoid, dump, return_loop, autonomous_start_sequence
from servo import setup_servo, initialise_servo

from strategies import (
    find_and_collect,
    collect_until_threshold,
    assess_and_sabotage,
    collect_and_bank,
    platform_wall_sweep,
    steal_from_base,
    idle,
)
import time

# ─────────────────────────────────────────────────────────────────────────────
# Game constants
# ─────────────────────────────────────────────────────────────────────────────

GAME_LENGTH     = 180   # seconds
STEAL_THRESHOLD = 120   # elapsed seconds before we start stealing from others

# ─────────────────────────────────────────────────────────────────────────────
# Hardware setup
# ─────────────────────────────────────────────────────────────────────────────

CURRENT_ROBOT_VALUE = 0
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
    """Returns elapsed time in seconds since start"""
    return time.time() - start_time

def log(msg):
    print(f"[{elapsed():.1f}s] {msg}")

# ─────────────────────────────────────────────────────────────────────────────
# Main Match Routine
# ─────────────────────────────────────────────────────────────────────────────

log("Match started!")

# ── 1. Preprogrammed Start ──────────────────────────────────────────────────
log("Executing preprogrammed start sequence")
# Ensure this function still exists in your new actions.py file
autonomous_start_sequence(motors, servo)

# ── 2. Collect & Bank Phase ─────────────────────────────────────────────────
log("Phase 1: Collecting and banking boxes")
while elapsed() < STEAL_THRESHOLD:
    time_remaining_in_phase = STEAL_THRESHOLD - elapsed()
    
    collect_until_threshold(
        robot, 
        motors, 
        servo, 
        time_limit=max(0, time_remaining_in_phase), 
        carry_limit=THRESHOLD_CARRY
    )

# ── 3. Sabotage & Steal Phase ───────────────────────────────────────────────
log("Phase 2: Time threshold reached, beginning theft")
while elapsed() < GAME_LENGTH:
    for target_zone in range(4):
        # Don't steal from our own base and make sure there's time left
        if target_zone != zone and elapsed() < GAME_LENGTH:
            log(f"Attempting to steal from opponent zone {target_zone}")
            steal_from_base(robot, motors, target_zone)
            
            _bank_budget = GAME_LENGTH - elapsed()
            # FIX: Updated to reference the correctly spelled module
            if CURRENT_ROBOT_VALUE > 0 and _bank_budget > 10:
                log(f"Banking stolen boxes back at base!")
                return_loop(robot, robot.zone, motors)
                CURRENT_ROBOT_VALUE = 0

log("Match ended. Stopping motors.")
halt(motors)