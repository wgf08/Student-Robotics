# ─────────────────────────────────────────────────────────────────────────────
# robot.py — entry point
# ─────────────────────────────────────────────────────────────────────────────
 
from sr.robot3 import Robot, Colour, LED_A, LED_B, LED_C, COMP, DEV
from movement import *
from utility import *
from info import *
from actions import consume, avoid, dump_opportunistic, dump_navigate, return_loop, autonomous_start_sequence_async
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
import strategies
import time
 
# ─────────────────────────────────────────────────────────────────────────────
# Game constants
# ─────────────────────────────────────────────────────────────────────────────
 
GAME_LENGTH     = 180   # seconds
STEAL_THRESHOLD = 120   # elapsed seconds before we start stealing from others
 
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
    """Returns elapsed time in seconds since start."""
    return time.time() - start_time
 
def log(msg):
    print(f"[{elapsed():.1f}s] {msg}")
 
def opponent_zones_by_value():
    """
    Returns opponent zone indices sorted by OPPONENT_BASE_VALUES descending —
    richest base first, so we always steal from wherever has the most boxes.
    """
    opponent_zones = [z for z in range(4) if z != zone]
    return sorted(
        opponent_zones,
        key=lambda z: strategies.OPPONENT_BASE_VALUES.get(z, 0),
        reverse=True
    )
 
# ─────────────────────────────────────────────────────────────────────────────
# Main Match Routine
# ─────────────────────────────────────────────────────────────────────────────
 
log("Match started!")
 
# ── 1. Preprogrammed Start ──────────────────────────────────────────────────
log("Executing preprogrammed start sequence")
autonomous_start_sequence_async(robot, motors, servo)
 
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
    # Sort by highest known value each iteration — intel updates as we play
    for target_zone in opponent_zones_by_value():
        if elapsed() >= GAME_LENGTH:
            break
 
        log(f"Stealing from zone {target_zone} "
            f"(estimated value: {strategies.OPPONENT_BASE_VALUES.get(target_zone, 0)})")
        steal_from_base(robot, motors, target_zone)
 
        _bank_budget = GAME_LENGTH - elapsed()
        if strategies.CURRENT_ROBOT_VALUE > 0 and _bank_budget > 10:
            log(f"Banking {strategies.CURRENT_ROBOT_VALUE} stolen box(es) at base!")
            return_loop(robot, robot.zone, motors)
            strategies.CURRENT_ROBOT_VALUE = 0
 
log("Match ended. Stopping motors.")
halt(motors)