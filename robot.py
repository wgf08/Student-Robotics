#imports
from sr.robot3 import Robot, Colour, LED_A, LED_B, LED_C, COMP, DEV
from movement import *
from utility import *
from info import *
from tests import *
from actions import *

start_time = start_timer()

#setup robot and motors
robot = Robot()
zone = robot.zone

mb1 = robot.motor_boards[srlnums[0]]
mb2 = robot.motor_boards[srlnums[1]]
sb = robot.servo_board

motor1 = mb2.motors[0]
motor2 = mb1.motors[0]
motor3 = mb1.motors[1]
motors = [motor1,motor2,motor3]

CURRENT_BASE_VALUE = 0
CURRENT_ROBOT_VALUE = 0

if robot.mode == DEV:
    code = 2
    if code == 0:
        while True:
            for i in range(6):
                angle = 60*(i+1)
                rotate_angle(motors,math.pi/2,1,True)
                robot.sleep(1)
    if code == 1:
        while True:
            markers = robot.camera.see()
            try:
                m = markers[0]
            except:
                continue
            consume(robot,m.id,motors,'direct-ws')
    if code == 2:
        while True:
            markers = robot.camera.see()
            try:
                m = markers[0]
            except:
                continue
            avoid(robot, m.id, motors)
    if code == 3:
        while True:
            rotate_angle(motors,math.pi,1,True)
            time.sleep(2)
                
else:
    pass