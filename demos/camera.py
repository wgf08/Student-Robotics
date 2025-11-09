# Import everything we will need
from sr.robot3 import Robot, Colour, LED_A, LED_B, LED_C


# Setup the robot so that we can control it
robot = Robot()
while True:
    robot.camera.see()