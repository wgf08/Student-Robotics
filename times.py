#from sr.robot3 import Robot, Colour, LED_A, LED_B, LED_C
import time

def start_timer():
    return time.time()

def time_left(start_time):
    time_elapsed = start_time-time.time()
    return 150-time_elapsed

