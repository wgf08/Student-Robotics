from sr.robot3 import Robot

# Setup the robot so that we can control it
robot = Robot()
motor_board = robot.motor_boards

motor_board.motors[0].power = 1
motor_board.motors[1].power = 1

while True:
    #get all fiducial markers
    markers = robot.camera.see()
    for marker in markers:
        if marker.position.distance <= 100:
            motor_board.motors[0].power = 0
            motor_board.motors[1].power = 0
        else:
            pass
