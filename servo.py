import time
from utility import *
from movement import *

# The SR servo API uses positions from -1 to 1
# For the MG996R 180°:
#   -1 = 0° (fully swept one way)
#    0 = 90° (centre)
#    1 = 180° (fully swept other way)

# Tune these values once the servo is physically mounted
RESTING_POSITION = -0.7   # Arm tucked back, ready to whip
WHIP_POSITION = 1       # Arm fully extended after the whip
WHIP_DELAY = 0.5         # Seconds to wait after whipping before returning

def setup_servo(robot):
    servo = robot.servo_board.servos[0]
    servo.set_duty_limits(500, 2500)
    return servo

def initialise_servo(servo):
    """
    Move the servo to the correct position for beginning a match.
    """
    servo.position = RESTING_POSITION
    time.sleep(0.5)  # Give it time to reach position before match starts

def whip(servo):
    """
    Moves the servo arm round in order to knock a sample off of the center 
    platform then once this is done, returns the servo to its original position.
    """
    
    # Swing the arm forward hard to knock the box
    servo.position = WHIP_POSITION
    time.sleep(WHIP_DELAY)  # Wait for arm to complete the swing

    # Return to resting position ready for next whip
    servo.position = RESTING_POSITION
    time.sleep(0.5)  # Wait to reach resting position before doing anything else

def get_high_box(servo, motors, marker_id):
    """
    Positions itself next to a marker that is raised above the ground and moves 
    into a position at which it thinks it can successfully whip its tail to 
    collect the box.
    """
    marker = find_marker(marker_id)
    x, y, z = sample_xyz(marker)
    while y > 600:
        move_straight(motors,1,True)
        try:
            marker = find_marker(marker_id)
            x, y, z = sample_xyz(marker)
        except:
            break

    while abs(x) > 500:
        move_angle(motors, 0.3, math.pi if y > 0 else 3*math.pi/2)
        try:
            marker = find_marker(marker_id)
            x, y, z = sample_xyz(marker)
        except:
            break
    whip(servo)
    return
    
    