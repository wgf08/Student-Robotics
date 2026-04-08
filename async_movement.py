import time
import math

# Global state for the async movement queue
starttime = time.time()
update_times = {} 
motor_sums = [0.0, 0.0, 0.0] 

# ─────────────────────────────────────────────────────────────────────────────
# Queue Management
# ─────────────────────────────────────────────────────────────────────────────

def init_queue():
    """Resets the queue and timer. Call this at the start of the match."""
    global starttime, update_times, motor_sums
    starttime = time.time()
    update_times.clear()
    motor_sums = [0.0, 0.0, 0.0]

def clear_queue(motors):
    """Emergency stop: clears all upcoming movements and halts the robot."""
    global update_times, motor_sums
    update_times.clear()
    motor_sums = [0.0, 0.0, 0.0]
    for m in motors:
        m.power = 0

def stay_vigilant(motors):
    """
    Must be called constantly in your main loop! 
    Checks if any scheduled movements are due and applies them.
    """
    current_time = time.time()
    
    # Find all timestamps in the past or present (fixes the exact-time bug)
    due_times = [t for t in update_times.keys() if t <= current_time]
    
    if not due_times:
        return # Nothing to update right now

    # Sort them to apply chronological updates correctly
    for t in sorted(due_times):
        commands = update_times[t]
        for power_levels in commands:
            _update_power(power_levels, motors)
        del update_times[t] # Remove from queue once executed

def _update_power(power_levels, motors):
    """Internal function to apply vector addition to the motors."""
    global motor_sums
    for i in range(3):
        motor_sums[i] += power_levels[i]

    # Normalize so no motor gets a power > 1.0 or < -1.0
    max_sum = max(abs(s) for s in motor_sums)
    if max_sum == 0: 
        max_sum = 1.0
    
    # Apply to physical motors
    for i in range(3):
        motors[i].power = motor_sums[i] / max_sum

# ─────────────────────────────────────────────────────────────────────────────
# Kinematic Converters
# ─────────────────────────────────────────────────────────────────────────────

def r_powerconverter(rotations):
    rots_per_sec = 1/2.2 
    tolerance = 0.1 

    add_motor_levels = [1.0, 1.0, 1.0] if rotations > 0 else [-1.0, -1.0, -1.0]

    predicted_levels = [motor_sums[i] + add_motor_levels[i] for i in range(3)]
    div_factor = max(abs(p) for p in predicted_levels) or 1.0
    
    norm_levels = [p / div_factor for p in predicted_levels]
    average_power = sum(abs(n) for n in norm_levels) / 3.0

    average_rotations = rots_per_sec * average_power
    run_time = abs(rotations) / (average_rotations if average_rotations > 0 else 1)
    run_time += run_time * tolerance

    return run_time, add_motor_levels

def t_powerconverter(x_distance, y_distance):
    y_direction = [-1.0, 0.0, 1.0]
    x_direction = [0.866, -1.0, 0.866]
    speed = 0.6
    tolerance = 0.1 

    # Fix: List comprehension for vector math
    x_levels = [x * x_distance for x in x_direction]
    y_levels = [y * y_distance for y in y_direction]

    add_motor_levels = [x_levels[i] + y_levels[i] for i in range(3)]
    
    max_add = max(abs(m) for m in add_motor_levels) or 1.0
    add_motor_levels = [m / max_add for m in add_motor_levels]

    predicted_levels = [motor_sums[i] + add_motor_levels[i] for i in range(3)]
    div_factor = max(abs(p) for p in predicted_levels) or 1.0
    
    norm_levels = [p / div_factor for p in predicted_levels]
    average_power = sum(abs(n) for n in norm_levels) / 3.0
    
    distance = math.sqrt(x_distance**2 + y_distance**2)
    average_speed = speed * average_power
    run_time = distance / (average_speed if average_speed > 0 else 1)
    run_time += run_time * tolerance

    return run_time, add_motor_levels

# ─────────────────────────────────────────────────────────────────────────────
# Schedulers
# ─────────────────────────────────────────────────────────────────────────────

def add_T_update(x_distance, y_distance, rel_start_time):
    run_time, power_levels = t_powerconverter(x_distance, y_distance)
    abs_start = starttime + rel_start_time
    abs_end = abs_start + run_time

    # Append to the list of commands at that timestamp
    update_times.setdefault(abs_start, []).append(power_levels)
    
    # To stop, we add the inverse of the vector
    stop_levels = [-p for p in power_levels]
    update_times.setdefault(abs_end, []).append(stop_levels)

def add_R_update(rotations_num, rel_start_time):
    run_time, power_levels = r_powerconverter(rotations_num)
    abs_start = starttime + rel_start_time
    abs_end = abs_start + run_time

    update_times.setdefault(abs_start, []).append(power_levels)
    
    stop_levels = [-p for p in power_levels]
    update_times.setdefault(abs_end, []).append(stop_levels)