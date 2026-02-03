from movement import *

"""
TESTS IF ROBOT CAN MOVE IN SUGGESTED WAY
"""

def dance(motors, power, duration):
    move_straight(motors,power, duration=duration)
    print(f'Im dancing!')
    halt(motors)
    spin(motors,power,duration)
    halt()