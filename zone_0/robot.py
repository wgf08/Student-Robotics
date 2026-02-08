from sr.robot3 import Robot
from actions import *

robot = Robot()

motors = [robot.motor_board.motors[0],robot.motor_board.motors[1] ]


while True:
    return_to_zone(robot, 2, motors)
    robot.sleep(2)
    halt(motors)





