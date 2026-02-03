import movement
import time
from sr.robot3 import  Robot

#set up robot so we can control it
robot = Robot()

srlnums = ["srABC1", "srXYZ1"]
mb1 = robot.motor_boards[srlnums[0]]
mb2 = robot.motor_boards[srlnums[1]]
motor1 = mb1.motors[0]
motor2 = mb1.motors[1]
motor3 = mb2.motors[0]
motors = [motor1, motor2, motor3]

movement.move_straight(motors, 1)
is_close = False

while is_close == False:
    distance_mm = robot.arduino.ultrasound_measure(9, 10)
    if distance_mm != 0 and distance_mm <= 90:
        movement.halt(motors)
        is_close = True
