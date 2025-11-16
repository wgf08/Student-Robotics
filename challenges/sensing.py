from sr.robot3 import Robot, Colour, LED_A, LED_B, LED_C


# Setup the robot so that we can control it
robot = Robot()

distance_mm = robot.arduino.ultrasound_measure(9, 10)

# Set each LED to a different colour
robot.kch.leds[LED_A].colour = Colour.OFF

curr_state = "RED"

while True:
    distance_mm = robot.arduino.ultrasound_measure(9, 10)
    if curr_state == "RED" and distance_mm < 200 and distance_mm !=0:
        robot.kch.leds[LED_B].colour = Colour.BLUE
        curr_state = "BLUE"
    else:
        if distance_mm > 200 or distance_mm == 0:
            robot.kch.leds[LED_B].colour = Colour.RED
            curr_state = "RED"
        else:
            continue
                