#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
import math as m
import time

                                       

###### Initialising the Hardware       

ev3 = EV3Brick()

#Motors
motorA = Motor(Port.A)
motorD = Motor(Port.D)
steerMotor = Motor(Port.B)

#Sensors
distanceSensor = UltrasonicSensor(Port.S3)
lightSensor = ColorSensor(Port.S4)


#Defining some expected constants
normalBrightness = lightSensor.reflection() #Either by meassuring in a nother helper function or just hard code this magic numbers
normalDistance = 30

maxRot = 70
maxLight = 70
maxDistance = 100




###### Defining some helper functions
       
def driveForward(motorA, motorD, speed = 100):
    motorA.run(speed)
    motorD.run(-speed)

def getAngleInPosition(steerMotor):
    ##########
    #To Do ###
    ##########
    steerMotor.run_target(90,0)
    return 0.0
    ##########


#Possible transition functions
def distanceToSteering(distance):
    pass

def lightToSteering(light):
    pass


#Function to evaluate how much to steer according to which part of the track the bot is
#@requires everything being initialized the right way
#@ensures motor is not changed

#@returns returns a value in the domain [-70 70] which should be the angle for the wheels (and the raw sensor data)
def getSteeringValue(lightSensor, distanceSensor, oldLight, oldDistance, normalBrightness, normalDistance, rot):
    light = lightSensor.reflection()
    distance = distanceSensor.distance()

    ######################
    ###   T O    D O   ###
    ######################
    #This should convert the steering value to a domain [-70 70] or [0 140] and the last step is to shift it by 70 degree
    #Simple formula is if sensor gets data from [a b] equaly spaced (otherwise interpolate it with a polynom) 140/(b-a) and shift by 70 + a
    #Following is my first guess
    lightToSteering = 1.5 
    distanceToSteering = 1
    shiftLight = 45
    shiftDistance = 0

    ######################

    #Checking for what the current sensor in the map is
    #Alternatively you could also just let one sensor for example the light sensor be dominant (easy implementation)
    #Maybe this is better done seperatly - Try it!
    #mode: 0 - light, 1 - distance, 2 - both
    if distance > maxDistance:
        mode = 0
    elif light > maxLight & distance > 20: #TODO Change -20- : Add the appropriate value for which distance the light should be the color of the tunnel
        mode = 1
    else:
        mode = 2 
    
    
    
    #Could be done in one step but I'm thinking about changing the (findModeTask) somewhere else
    if mode == 0:
        steering = lightToSteering * (normalBrightness - light) - shiftLight
        change = lightToSteering * abs(light-oldLight)
    elif mode == 1:
        steering = distanceToSteering * (normalDistance - distance) - shiftDistance
        change = distanceToSteering * abs(distance-oldDistance)
    elif mode == 2:
        lightSteering = lightToSteering * (normalBrightness - light) - shiftLight
        distanceSteering = distanceToSteering * (normalDistance - distance) - shiftDistance
        steering = 1/2 *( lightSteering + distanceSteering)
        change = 1/2 * (lightToSteering * abs(light-oldLight) + distanceToSteering * abs(distance-oldDistance))

    #Final calculation

    #If the change is high don't steer as strong
    changeFactor = 1/(1 + change) #TODO
    limitFactor = (1 - rot/maxRot) #TODO
    #If the rotation is already high enough or to high dont change to strong
    steering = limitFactor * changeFactor * (steering - rot)

    #If we use steerMotor.run_angle: We don't need
    steering = steering + rot
    #This line of code

    return steering, light, distance
    

    #We could also use a lightToSteering etc. function like:
    #steering = 150*m.asin((0.008*(normalBrightness - currentBrightness))) 
    #Again this is best testet with the robot

    #Another thing to implement is to use old data to calculate the derivative
    
    



#####################################################################
#####     M A I N     P R O G R A M     #############################
#####################################################################

driveForward(motorA, motorD)
light = lightSensor.reflection()
distance = distanceSensor.distance()
angleError = getAngleInPosition()

while True:
    
    rot = steerMotor.angle() - angleError

    #To return light and distance is a convention to use it in derivative and also maybe somewhere later
    steering, light, distance = getSteeringValue(lightSensor, distanceSensor, light, distance, normalBrightness, normalDistance,rot)

    
    if (-maxRot < rot & rot < maxRot):
        steerMotor.run_target(90, steering)
        #try steerMotor.run_angle -angleError and change getSteeringValue
    wait(7)
