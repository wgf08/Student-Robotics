#imports
from sr.robot3 import Robot, Colour, LED_A, LED_B, LED_C
from times import *
from movement import *
import math

#setup variables
TARGETED_BASE_COLOUR = "BLUE"
THRESHOLD_TIME = 20
start_time = start_timer()
srlnums = ["srABC1","srXYZ1"]

#setup robot and motors
robot = Robot()
mb1 = robot.motor_boards[srlnums[0]]
mb2 = robot.motor_boards[srlnums[1]]
motor1 = mb1.motors[0]
motor2 = mb1.motors[1]
motor3 = mb2.motors[0]
motors = [motor1,motor2,motor3]

def loop():
    move_straight(motors,1)
    time.sleep(5)
    halt(motors)
    time.sleep(2)
    move_angle(motors,1,(math.pi)/2)
    time.sleep(5)
    halt(motors)
    time.sleep(2)
    spin(motors,1)