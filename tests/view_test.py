""" 
TESTS WHAT THE ROBOT IS CURRENTLY SEEING
USED FOR PURPOSES ON ROBOT.LAN TO CHECK FOV OF ROBOT
"""

def fov(robot):
    while True: robot.camera.see()