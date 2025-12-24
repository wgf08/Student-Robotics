import math
import time
from sr.robot3 import Robot

def move_straight(motors,power, forwards = True):
    if not forwards: power = -power
    motors[1].power(-power)
    motors[2].power(0)
    motors[3].power(power)

def spin(motors, power, clockwise = True):
    if not clockwise: power = -power
    motors[1].power(power)
    motors[2].power(power)
    motors[3].power(power)

def halt(motors):
    motors[1].power(0)
    motors[2].power(0)
    motors[3].power(0)

def move_angle(motors, power, theta):
    vals = [
        math.cos(theta - 2*math.pi/3),
        math.cos(theta),
        math.cos(theta + 2*math.pi/3)
    ]

    max_mag = max(abs(v) for v in vals)
    vals = [v / max_mag for v in vals]

    motors[1].power(power * vals[0])
    motors[2].power(power * vals[1])
    motors[3].power(power * vals[2])

def rotate_angle(theta, motors, power, clockwise = True):
    spin(motors,power,clockwise)
    time.sleep((theta/360)*2.2)
    halt(motors)