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
zone = set_zone(robot)[0]
mb1 = robot.motor_boards[srlnums[0]]
mb2 = robot.motor_boards[srlnums[1]]
motor1 = mb2.motors[0]
motor2 = mb1.motors[0]
motor3 = mb1.motors[1]
motors = [motor1,motor2,motor3]

CURRENT_BASE_VALUE = 0
CURRENT_ROBOT_VALUE = 0

if robot.mode == DEV:
    print('DEV MODE ON, BEGGINING MOVE TEST')
    move_test(motors,0.5,5)
    print('TASK COMPLETED, FOLLOW_TESST IN 2 SECONDS')
    robot.sleep(2)

    """
    while not follow_test():
        pass
    print('TASK COMPLETED, CONSUMPTION IN 2 SECONDS')
    robot.sleep(2)

    markers = robot.see()
    sample = sorted_boxes(markers)[2][0]
    consume(robot, sample, motors)
    """

    """
    while True:
        spin(motors,1,True)

    spin(motors, 1, True, 1)
    """

else:
    pass
    



