from sr.robot3 import Robot, Colour, LED_A, LED_B, LED_C
import math

robot = Robot()

while True:
    Marker = robot.camera.see()[0]
    print("I can see marker ", Marker.id)

    Distance = Marker.position.distance
    Angle = Marker.position.horizontal_angle * 180 / math.pi

    ColourA = Colour.OFF
    ColourB = Colour.OFF
    ColourC = Colour.OFF

    # Comment out 19 - 24 if you are doing Angle Testing

    if(Distance > 1500):
        ColourB = Colour.RED
    elif(Distance < 300):
        ColourB = Colour.GREEN
    else:
        ColourB = Colour.BLUE

    # Comment out 27 - 32 if you're doing Distance Testing
    if(Angle < -15):
        ColourA = Colour.BLUE
    elif(Angle > 15):
        ColourC = Colour.BLUE
    else:
        ColourB = Colour.RED

    robot.kch.leds[LED_A].colour = ColourA
    robot.kch.leds[LED_B].colour = ColourB
    robot.kch.leds[LED_C].colour = ColourC