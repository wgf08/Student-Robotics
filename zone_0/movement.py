from sr.robot3 import BRAKE, COAST
import math
def move_straight(motors, power):
    motors[0].power = power
    motors[1].power = power

def halt(motors):
    motors[0].power = 0
    motors[1].power = 0

def rotate_angle(robot, angle, motors, power, ac=False):
    if not ac:
        motors[0].power = power
        motors[1].power = -power
        duration = abs(angle / 15)
        robot.sleep(duration)

        halt(motors)
    else:
        'rotating left'
        motors[0].power = -power
        motors[1].power = power
        duration = abs(angle / 15)
        robot.sleep(duration)

        halt(motors)

def move_at_angle(motors, theta, speed=0.5):
    """
    Move the robot in the direction theta (radians) relative to its current heading.
    Only works with 2-wheel differential drive.
    """
    # Forward/backward component
    v_forward = math.cos(theta) * speed
    # Rotational component
    v_rotate = math.sin(theta) * speed

    # Set motor powers
    left_power = v_forward - v_rotate
    right_power = v_forward + v_rotate

    # Clamp to [-1, 1] if necessary
    left_power = max(min(left_power, 1), -1)
    right_power = max(min(right_power, 1), -1)

    motors[0].power = left_power
    motors[1].power = right_power


