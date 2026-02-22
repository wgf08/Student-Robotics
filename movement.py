from sr.robot3 import BRAKE, COAST
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

def move_straight(motors, power, forwards = True, duration = None):
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

def spin(motors, power, clockwise = True, duration = None):
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
    ANGLE IN RADIANS DICKHEADS
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

def rotate_angle(motors, theta, power, clockwise = True):
    spin(motors,power,clockwise)
    time.sleep((theta/360)*2.2)
    halt(motors)