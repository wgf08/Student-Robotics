#imports
from sr.robot3 import Robot, Colour, LED_A, LED_B, LED_C
from movement import *
from utility import *
from info import *
from tests import *

#setup
TESTING = True
start_time = start_timer()

#setup robot and motors
robot = Robot()
zone = set_zone(robot)
mb1 = robot.motor_boards[srlnums[0]]
mb2 = robot.motor_boards[srlnums[1]]
motor1 = mb2.motors[0]
motor2 = mb1.motors[1]
motor3 = mb1.motors[0]
motors = [motor1,motor2,motor3]

CURRENT_BASE_VALUE = 0
CURRENT_ROBOT_VALUE = 0

if TESTING:
    dance(motors,0.5,5)
else:
    pass
    



